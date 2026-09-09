// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! Control-endpoint bookkeeping: the in-flight request table, the tag
//! allocator, and the reserved-tag entropy sink.

use crate::proto;
use kernel::prelude::*;

const MAX_INFLIGHT: usize = 8;

const POOL_LEN: usize = (proto::TAG_POOL_LAST - proto::TAG_POOL_FIRST + 1) as usize;

#[derive(Clone, Copy)]
struct Slot {
    used: bool,
    tag: u8,
    reply: Option<proto::ControlReply>,
}

impl Slot {
    const FREE: Slot = Slot {
        used: false,
        tag: 0,
        reply: None,
    };
}

struct EntropySink {
    value: Option<u32>,
    msg1: u32,
    unclaimed: u32,
    busy: bool,
}

pub(crate) enum Delivery {
    Entropy,
    Matched,
    Unmatched,
}

pub(crate) struct ControlState {
    slots: [Slot; MAX_INFLIGHT],
    next_tag: u8,
    retired: [u64; 4],
    retired_count: u32,
    // SEP-initiated messages (type != 0x01).
    unsolicited: u32,
    entropy: EntropySink,
    // After the persistent-state exchange the SEP parks this endpoint; refuse rather than time out.
    closed: bool,
}

impl ControlState {
    pub(crate) fn new() -> Self {
        ControlState {
            slots: [Slot::FREE; MAX_INFLIGHT],
            next_tag: proto::TAG_POOL_FIRST,
            retired: [0; 4],
            retired_count: 0,
            unsolicited: 0,
            entropy: EntropySink {
                value: None,
                msg1: 0,
                unclaimed: 0,
                busy: false,
            },
            closed: false,
        }
    }

    fn is_retired(&self, tag: u8) -> bool {
        self.retired[(tag >> 6) as usize] & (1u64 << (tag & 0x3f)) != 0
    }

    fn retire(&mut self, tag: u8) {
        if !self.is_retired(tag) {
            self.retired[(tag >> 6) as usize] |= 1u64 << (tag & 0x3f);
            self.retired_count += 1;
        }
    }

    fn tag_in_flight(&self, tag: u8) -> bool {
        self.slots.iter().any(|s| s.used && s.tag == tag)
    }

    pub(crate) fn alloc(&mut self) -> Result<(usize, u8)> {
        if self.closed {
            return Err(EPIPE);
        }
        let idx = self.slots.iter().position(|s| !s.used).ok_or(EBUSY)?;

        for _ in 0..POOL_LEN {
            let tag = self.next_tag;
            self.next_tag = if tag >= proto::TAG_POOL_LAST {
                proto::TAG_POOL_FIRST
            } else {
                tag + 1
            };
            if !self.tag_in_flight(tag) && !self.is_retired(tag) {
                self.slots[idx] = Slot {
                    used: true,
                    tag,
                    reply: None,
                };
                return Ok((idx, tag));
            }
        }
        Err(EBUSY)
    }

    pub(crate) fn take_reply(&mut self, idx: usize) -> Option<proto::ControlReply> {
        self.slots[idx].reply.take()
    }

    pub(crate) fn release(&mut self, idx: usize) {
        self.slots[idx] = Slot::FREE;
    }

    // Retire the tag so a late reply cannot be mistaken for a later request's answer.
    pub(crate) fn abandon(&mut self, idx: usize) -> u8 {
        let tag = self.slots[idx].tag;
        self.retire(tag);
        self.slots[idx] = Slot::FREE;
        tag
    }

    pub(crate) fn retired_count(&self) -> u32 {
        self.retired_count
    }

    pub(crate) fn deliver(&mut self, reply: proto::ControlReply) -> Delivery {
        // The reserved entropy tag is claimed first, outstanding request or not.
        if reply.tag == proto::TAG_ENTROPY {
            if self.entropy.value.is_some() {
                self.entropy.unclaimed = self.entropy.unclaimed.wrapping_add(1);
            }
            self.entropy.value = Some(reply.data_lo);
            self.entropy.msg1 = reply.msg1;
            return Delivery::Entropy;
        }

        if let Some(slot) = self
            .slots
            .iter_mut()
            .find(|s| s.used && s.tag == reply.tag && s.reply.is_none())
        {
            slot.reply = Some(reply);
            return Delivery::Matched;
        }

        // No match: drop it, never hand it to another waiter.
        Delivery::Unmatched
    }

    pub(crate) fn note_unsolicited(&mut self) -> u32 {
        self.unsolicited = self.unsolicited.wrapping_add(1);
        self.unsolicited
    }

    pub(crate) fn entropy_begin(&mut self) -> Result<()> {
        if self.closed {
            return Err(EPIPE);
        }
        if self.entropy.busy {
            return Err(EBUSY);
        }
        self.entropy.busy = true;
        if self.entropy.value.take().is_some() {
            self.entropy.unclaimed = self.entropy.unclaimed.wrapping_add(1);
        }
        Ok(())
    }

    pub(crate) fn entropy_take(&mut self) -> Option<(u32, u32)> {
        self.entropy.value.take().map(|v| (v, self.entropy.msg1))
    }

    pub(crate) fn entropy_end(&mut self) {
        self.entropy.busy = false;
    }

}
