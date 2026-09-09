// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
//! SEP key store (SKS, endpoint `0x12`): key-bag and key-management request/reply
//! framing, DER-imaged request builders, and lock-state control.

#![allow(dead_code)]

use super::*;
use kernel::prelude::*;
use kernel::soc::apple::mailbox::Message;
use crate::proto::*;

impl SepData {
    pub(crate) fn on_sks(&self, msg: Message) {
        let r = crate::sks::decode_sks(&msg);
        let mut probe = self.sks_probe.lock();
        if probe.active {
            if probe.captured.len() < SKS_MAX_CAPTURE {
                let _ = probe.captured.push(msg, GFP_KERNEL);
            }
            drop(probe);
            self.sks_wq.notify_all();
        } else {
            probe.unsolicited = probe.unsolicited.wrapping_add(1);
            let matched = probe
                .abandoned
                .iter()
                .flatten()
                .find(|a| a.selector == r.selector && a.seq == r.seq)
                .copied();
            drop(probe);
            if let Some(a) = matched {
                self.sks_wedged.store(0, Relaxed);
                dev_warn!(self.dev, "sks: late answer to {} after {} ms; wedge lifted\n", a.label, a.waited_ms);
            }
        }
    }

    fn sks_note_abandoned(&self, selector: u8, seq: u8, label: &'static CStr, waited_ms: u64) {
        let mut probe = self.sks_probe.lock();
        let slot = probe.abandoned_next;
        probe.abandoned[slot] = Some(Abandoned {
            selector,
            seq,
            label,
            waited_ms,
        });
        probe.abandoned_next = (slot + 1) % SKS_MAX_ABANDONED;
    }

    fn sks_read_buffer(&self, inbound: bool, len: usize) -> Result<KVec<u8>> {
        let guard = self.ool_sks.lock();
        let pair = guard.as_ref().ok_or(ENODEV)?;
        let src = if inbound {
            &pair.inbound
        } else {
            &pair.outbound
        };
        if len > pair.allocated {
            return Err(EINVAL);
        }
        let mut out = KVec::new();
        // SAFETY: the enclave writes this buffer then signals the mailbox, so it is
        // quiescent when we read it; the driver is the only other accessor.
        out.extend_from_slice(unsafe { &src.as_ref()[..len] }, GFP_KERNEL)?;
        Ok(out)
    }

    fn sks_zero_buffers(&self) -> Result<()> {
        let mut guard = self.ool_sks.lock();
        let pair: &mut Option<OolPair> = &mut guard;
        let buffers = pair.as_mut().ok_or(ENODEV)?;
        // SAFETY: the enclave touches these only between a request word and its
        // reply, and none is outstanding here; the driver is the only other accessor.
        unsafe {
            buffers.inbound.as_mut()[..buffers.allocated].fill(0);
            buffers.outbound.as_mut()[..buffers.allocated].fill(0);
        }
        Ok(())
    }

    pub(crate) fn ool_registered(&self, slot: &Mutex<Option<OolPair>>) -> bool {
        slot.lock().as_ref().is_some_and(|b| b.registered)
    }

    pub(crate) fn sks_next_seq(&self) -> crate::sks::Sequence {
        let n = self.sks_seq.load(Relaxed);
        self.sks_seq.store(n.wrapping_add(1), Relaxed);
        crate::sks::Sequence::from_counter(n as u8)
    }

    fn sks_exchange(
        &self,
        label: &'static CStr,
        msg: Message,
        img: &image::RequestImage,
    ) -> Option<SksOutcome> {
        self.sks_exchange_patient(label, msg, img, 0, true)
    }

    fn sks_exchange_patient(
        &self,
        label: &'static CStr,
        msg: Message,
        img: &image::RequestImage,
        floor_ms: time::Msecs,
        condemn: bool,
    ) -> Option<SksOutcome> {
        if !self.ool_registered(&self.ool_sks) {
            return None;
        }

        if self.sks_wedged.load(Relaxed) != 0 {
            return None;
        }

        if self.sks_zero_buffers().is_err() {
            return None;
        }
        if self.ool_write(&self.ool_sks, 0, img.as_slice()).is_err() {
            return None;
        }

        let sized_ms = sks_timeout_for(img.len());
        let timeout_ms = if sized_ms < floor_ms {
            floor_ms
        } else {
            sized_ms
        };
        self.sks_arm(label);
        if self.send(msg).is_err() {
            self.sks_disarm();
            return None;
        }
        let started_ns = crate::shim::boottime_ns();

        let sent = crate::sks::decode_sks(&msg);
        let mut examined = 0usize;
        let mut set_aside = 0u32;
        let correlating = loop {
            self.sks_wait(timeout_ms, examined + 1);

            let next = {
                let probe = self.sks_probe.lock();
                let m = probe.captured.get(examined).copied();
                if m.is_some() {
                    examined += 1;
                }
                m
            };

            let Some(candidate) = next else { break None };

            let decoded = crate::sks::decode_sks(&candidate);
            if decoded.seq == sent.seq && decoded.selector == sent.selector {
                break Some(candidate);
            }

            set_aside = set_aside.saturating_add(1);
            if set_aside >= SKS_MAX_SET_ASIDE {
                break None;
            }
        };

        let leftover = self.sks_disarm();
        let correlating = match correlating {
            Some(m) => Some(m),
            None => leftover.iter().skip(examined).copied().find(|m| {
                let d = crate::sks::decode_sks(m);
                d.seq == sent.seq && d.selector == sent.selector
            }),
        };

        let Some(raw) = correlating else {
            let waited_ms = crate::shim::boottime_ns().saturating_sub(started_ns) / 1_000_000;
            self.sks_note_abandoned(sent.selector, sent.seq, label, waited_ms);
            if condemn {
                self.sks_wedged.store(1, Relaxed);
            }
            dev_err!(self.dev, "sks: {} got no reply after {} ms\n", label, waited_ms);
            return None;
        };

        let reply = crate::sks::decode_sks(&raw);
        // reply.status is signed here on (e.g. -13, not 0xf3).

        let mut response = Secret::empty();
        if reply.response_size > 0 {
            if let Ok(bytes) = self.sks_read_buffer(false, reply.response_size as usize) {
                response = Secret(bytes);
            }
        }

        Some(SksOutcome { reply, response })
    }

    pub(crate) fn sks_report_response<'a>(
        &self,
        _label: &CStr,
        out: &'a SksOutcome,
    ) -> Option<&'a [u8]> {
        if out.response.is_empty() {
            return None;
        }

        let parsed = match image::parse_response(&out.response) {
            Ok(p) => p,
            Err(_) => {
                return None;
            }
        };
        Some(parsed.body)
    }

    pub(crate) fn sks_timestamp_us(&self) -> u64 {
        crate::shim::boottime_ns() / 1000
    }

    pub(crate) fn sks_image_len(&self, img: &image::RequestImage) -> Result<crate::sks::ImageLen> {
        let declared_in = sks_declared_sizes().0;
        if img.len() > declared_in {
            return Err(ENOSPC);
        }
        crate::sks::ImageLen::of(img.len()).ok_or(EINVAL)
    }

    fn sks_seal(&self, op: &crate::sks::SksOp, body: &image::Body) -> Result<SksRequest> {
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), body)?;
        let len = self.sks_image_len(&img)?;
        Ok(SksRequest {
            name: op.name(),
            msg: crate::sks::encode_sks_read(op, self.sks_next_seq(), len),
            img,
        })
    }

    /// `0x4d` get capabilities.
    fn sks_req_get_capabilities(&self) -> Result<SksRequest> {
        let op = crate::sks::sks_get_capabilities();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(1)?;
        body.put_blob(&[])?;
        self.sks_seal(&op, &body)
    }

    /// `0x0d` designate.
    fn sks_req_designate(&self, d: &crate::sks::Designation, secret: &[u8]) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(crate::sks::SKS_DESIGNATE_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(d.source().value())?;
        body.put_i32(d.user().special_handle().value())?;
        body.put_blob(secret)?;
        // Flags is a u64, not u32; a u32 leaves the body short -> enclave answers -13.
        body.put_u64(crate::sks::SKS_DESIGNATE_FLAGS)?;

        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_designate(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_DESIGNATE_NAME,
            msg,
            img,
        })
    }

    /// `0x05` unload the source handle.
    pub(crate) fn sks_req_unload_keybag(&self, handle: crate::sks::KeyBagHandle) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_unload_keybag(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_UNLOAD_NAME,
            msg,
            img,
        })
    }

    /// `0x06` UUID read, against the special handle.
    pub(crate) fn sks_req_copy_uuid_special(&self, special: crate::sks::SpecialHandle) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag_uuid();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        self.sks_seal(&op, &body)
    }

    pub(crate) fn sks_req_unlock_special(
        &self,
        special: crate::sks::SpecialHandle,
        secret: &[u8],
        healthy: Healthy,
    ) -> Result<SksRequest> {
        let Healthy(()) = healthy;
        let mut body = image::Body::new();
        body.put_u32(SKS_LOCK_STATE_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        body.put_i32(LockState::Unlocked.wire())?;
        body.put_blob(secret)?;
        body.put_u64(SKS_LOCK_STATE_FLAGS)?;
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_change_lock_state(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_LOCK_STATE_NAME,
            msg,
            img,
        })
    }

    /// `0x06` copy key-bag UUID.
    fn sks_req_copy_keybag_uuid(&self, handle: crate::sks::KeyBagHandle) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag_uuid();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        self.sks_seal(&op, &body)
    }

    /// `0x02` copy the designated biometric identity bag.
    fn sks_req_copy_keybag_special(&self, handle: crate::sks::SpecialHandle) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        self.sks_seal(&op, &body)
    }

    /// `0x03` load key bag.
    fn sks_req_load_keybag(&self, wrapped: &[u8]) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_blob(wrapped)?;

        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_load(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_LOAD_NAME,
            msg,
            img,
        })
    }

    pub(crate) fn sks_send(&self, req: Result<SksRequest>) -> Option<SksOutcome> {
        match req {
            Ok(r) => self.sks_exchange(r.name, r.msg, &r.img),
            Err(_) => None,
        }
    }

    pub(crate) fn sks_designate_user_keybag(&self, handle: crate::sks::KeyBagHandle, secret: &[u8]) {
        let Some(user) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) else {
            return;
        };

        let designation = crate::sks::Designation::new(handle, user);

        let Some(out) = self.sks_send(self.sks_req_designate(&designation, secret)) else {
            dev_warn!(self.dev, "sks: DESIGNATE_KEYBAG did not complete; enrolment will refuse\n");
            return;
        };
        let Some(body) =
            self.sks_report_response(crate::sks::SKS_DESIGNATE_NAME, &out)
        else {
            return;
        };

        if body.len() != crate::sks::SKS_DESIGNATE_REPLY_LEN {
            return;
        }
        let variant = u32::from_le_bytes([body[0], body[1], body[2], body[3]]);
        if variant != crate::sks::SKS_DESIGNATE_VARIANT {
            return;
        }

        self.keybag_designated.store(true, Relaxed);

        self.sks_remember_enrolment_material(
            designation.user().special_handle(),
            secret,
        );
    }

    fn sks_remember_enrolment_material(
        &self,
        special: crate::sks::SpecialHandle,
        secret: &[u8],
    ) {
        let mut copy = KVec::new();
        if copy.extend_from_slice(secret, GFP_KERNEL).is_err() {
            return;
        }
        *self.enrol_material.lock() = Some(EnrolMaterial {
            special,
            secret: Secret(copy),
        });
    }

    /// `0x04` change lock state (v1).
    pub(crate) fn sks_req_change_lock_state(
        &self,
        handle: crate::sks::KeyBagHandle,
        state: LockState,
        secret: &[u8],
        healthy: Healthy,
    ) -> Result<SksRequest> {
        let Healthy(()) = healthy;
        let mut body = image::Body::new();
        body.put_u32(SKS_LOCK_STATE_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        body.put_i32(state.wire())?;
        body.put_blob(secret)?;
        // Trailing u64 goes after the blob (`0x18` puts it before).
        body.put_u64(SKS_LOCK_STATE_FLAGS)?;
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_change_lock_state(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_LOCK_STATE_NAME,
            msg,
            img,
        })
    }

    /// `0x21` verify-secret (variant 1).
    pub(crate) fn sks_req_verify_secret(
        &self,
        special: crate::sks::SpecialHandle,
        secret: &[u8],
        acm_context: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN],
    ) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(crate::sks::SKS_VERIFY_SECRET_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        body.put_blob(secret)?;
        body.put_blob(acm_context)?;
        body.put_u64(0)?;
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_verify_secret(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_VERIFY_SECRET_NAME,
            msg,
            img,
        })
    }

    pub(crate) fn sks_step(
        &self,
        label: &CStr,
        build: impl FnOnce(Healthy) -> Result<SksRequest>,
    ) -> Option<SksOutcome> {
        let healthy = self.sks_health_check(label)?;
        let out = self.sks_send(build(healthy))?;
        if out.reply.status != 0 {
            return None;
        }
        Some(out)
    }

    pub(crate) fn sks_health_check(&self, why: &CStr) -> Option<Healthy> {
        let Some(out) = self.sks_send(self.sks_req_get_capabilities()) else {
            dev_err!(self.dev, "sks: health check ({}) got no reply; endpoint gone for this boot\n", why);
            return None;
        };
        if out.reply.status != 0 {
            dev_err!(self.dev, "sks: health check ({}) returned status {}\n", why, out.reply.status);
            return None;
        }
        Some(Healthy(()))
    }

    pub(crate) fn sks_recover(
        &self,
        stored: &keybag::StoredKeyBag,
    ) -> Option<(crate::sks::KeyBagHandle, [u8; keybag::UUID_LEN])> {
        let out = self.sks_send(self.sks_req_load_keybag(stored.wrapped()))?;
        let body = self.sks_report_response(crate::sks::SKS_LOAD_NAME, &out)?;

        if out.reply.status != 0 {
            dev_warn!(
                self.dev,
                "sks: LOAD_KEYBAG mailbox status {}; no handle\n",
                out.reply.status
            );
            return None;
        }
        if body.len() != SKS_LOAD_REPLY_LEN {
            return None;
        }
        let status = i32::from_le_bytes([body[0], body[1], body[2], body[3]]);
        let handle = i32::from_le_bytes([body[4], body[5], body[6], body[7]]);
        if status != 0 || handle < 0 {
            return None;
        }
        let handle = crate::sks::KeyBagHandle::from_load_reply(handle);

        let uuid = self.sks_read_uuid(handle)?;

        if uuid == *stored.uuid() {
            Some((handle, uuid))
        } else {
            dev_err!(
                self.dev,
                "sks: load succeeded but bag identity mismatch: stored UUID {}, loaded UUID {}\n",
                Hex(stored.uuid()),
                Hex(&uuid)
            );
            None
        }
    }

    pub(crate) fn resnapshot_identity_keybag(&self) -> bool {
        let special = {
            let material = self.enrol_material.lock();
            let Some(material) = material.as_ref() else {
                return false;
            };
            material.special
        };

        let Some(out) = self.sks_send(self.sks_req_copy_keybag_special(special)) else {
            return false;
        };
        let Some(wrapped) = self.wrapped_from_copy_reply(&out, c"the designated identity bag")
        else {
            return false;
        };

        let Some(uuid) = self
            .sks_send(self.sks_req_copy_uuid_special(special))
            .and_then(|out| self.sks_uuid_from_reply(&out))
        else {
            return false;
        };

        match keybag::replace_wrapped(keybag::Slot::Identity, &wrapped, &uuid) {
            Ok(()) => true,
            Err(e) => {
                dev_err!(
                    self.dev,
                    "enrol: could not commit the identity-bag snapshot ({:?}); may not survive reboot\n",
                    e
                );
                false
            }
        }
    }

    fn sks_read_uuid(&self, handle: crate::sks::KeyBagHandle) -> Option<[u8; keybag::UUID_LEN]> {
        let out = self.sks_send(self.sks_req_copy_keybag_uuid(handle))?;
        self.sks_uuid_from_reply(&out)
    }

    pub(crate) fn sks_uuid_from_reply(&self, out: &SksOutcome) -> Option<[u8; keybag::UUID_LEN]> {
        let body = self.sks_report_response(c"COPY_KEYBAG_UUID", out)?;

        if out.reply.status != 0 || image::operation_status(body).unwrap_or(-1) != 0 {
            return None;
        }
        let (blob, _) = image::read_blob(body, 4)?;
        if blob.len() != keybag::UUID_LEN {
            return None;
        }
        let mut uuid = [0u8; keybag::UUID_LEN];
        uuid.copy_from_slice(blob);
        Some(uuid)
    }

    pub(crate) fn enable_sks(&self) -> Result<()> {
        self.register_ool(&self.ool_sks)
    }

    pub(crate) fn sks_ready(&self) -> bool {
        if self.ool_registered(&self.ool_sks) {
            return true;
        }
        if !self.endpoint_present(proto::EP_SKS) {
            dev_err!(
                self.dev,
                "sks: endpoint 0x{:02x} (key store) was not advertised on this boot; keybag and ref-key operations cannot run\n",
                proto::EP_SKS
            );
            return false;
        }
        if let Err(e) = self.enable_sks() {
            dev_err!(self.dev, "sks: could not register out-of-line buffers ({:?})\n", e);
            return false;
        }
        true
    }

    fn sks_arm(&self, label: &'static CStr) {
        let mut probe = self.sks_probe.lock();
        probe.captured.clear();
        probe.label = Some(label);
        probe.active = true;
    }

    fn sks_disarm(&self) -> KVec<Message> {
        let mut probe = self.sks_probe.lock();
        probe.active = false;
        probe.label = None;
        core::mem::take(&mut probe.captured)
    }

    fn sks_wait(&self, ms: time::Msecs, until: usize) {
        let mut remaining = time::msecs_to_jiffies(ms);
        let mut guard = self.sks_probe.lock();
        loop {
            if guard.captured.len() >= until || remaining == 0 {
                return;
            }
            match self
                .sks_wq
                .wait_interruptible_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }
}

const SKS_SELECTOR_MAX: u8 = 0x5f;

pub(crate) const SKS_REPLY_BIT: u8 = 0x80;

const OP_SKS_REWRAP_FORBIDDEN: u8 = 0x0f;

pub(crate) struct SksOp {
    opcode: u8,
    name: &'static CStr,
}

impl SksOp {
    pub(crate) fn opcode(&self) -> u8 {
        self.opcode
    }
    pub(crate) fn name(&self) -> &'static CStr {
        self.name
    }
}

const OP_SKS_DEVICE_STATE: u8 = 0x19;
pub(crate) fn sks_get_device_state() -> SksOp {
    SksOp {
        opcode: OP_SKS_DEVICE_STATE,
        name: c"GET_DEVICE_STATE",
    }
}

const OP_SKS_GET_CONFIGURATION: u8 = 0x23;

const OP_SKS_SET_CONFIGURATION: u8 = 0x24;
static_assert!(OP_SKS_SET_CONFIGURATION != OP_SKS_GET_CONFIGURATION);

const OP_SKS_NEW_PFK: u8 = 0x10;
static_assert!(OP_SKS_NEW_PFK != 0x0f);
static_assert!(OP_SKS_NEW_PFK != 0x09);

// FileVault seal order: 0x42 KEK, then 0x40 VEK, then 0x41 install; 0x47 clear forbidden
pub(crate) const OP_SKS_FV_NEW_VEK: u8 = 0x40;
pub(crate) const OP_SKS_FV_UNWRAP_VEK: u8 = 0x41;
pub(crate) const OP_SKS_FV_NEW_KEK: u8 = 0x42;

const OP_SKS_GENERIC_OPERATION: u8 = 0x1a;
const OP_SKS_PERFORM_OPERATION: u8 = 0x22;
const OP_SKS_IDENTITY_OPERATION: u8 = 0x51;
pub(crate) const SKS_PERFORM_OP_NAME: &CStr = c"PERFORM_OPERATION";
static_assert!(OP_SKS_PERFORM_OPERATION != OP_SKS_GET_CONFIGURATION);
static_assert!(OP_SKS_PERFORM_OPERATION != OP_SKS_SET_CONFIGURATION);

pub(crate) fn encode_sks_perform_operation(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_PERFORM_OPERATION, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_fv(selector: u8, seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(selector, seq.value(), len.value());
    Some(msg)
}
const OP_SKS_LAST_USER_OPERATION: u8 = 0x53;
static_assert!(OP_SKS_LAST_USER_OPERATION != 0x56);

const OP_SKS_CAPABILITIES: u8 = 0x4d;
pub(crate) fn sks_get_capabilities() -> SksOp {
    SksOp {
        opcode: OP_SKS_CAPABILITIES,
        name: c"GET_CAPABILITIES",
    }
}

const OP_SKS_COPY_UUID: u8 = 0x06;
pub(crate) fn sks_copy_keybag_uuid() -> SksOp {
    SksOp {
        opcode: OP_SKS_COPY_UUID,
        name: c"COPY_KEYBAG_UUID",
    }
}

const OP_SKS_COPY_KEYBAG: u8 = 0x02;
pub(crate) fn sks_copy_keybag() -> SksOp {
    SksOp {
        opcode: OP_SKS_COPY_KEYBAG,
        name: c"COPY_KEYBAG",
    }
}

const OP_SKS_CREATE_KEYBAG: u8 = 0x01;

const OP_SKS_LOAD_KEYBAG: u8 = 0x03;

pub(crate) const SKS_LOAD_NAME: &CStr = c"LOAD_KEYBAG";

const OP_SKS_CHANGE_LOCK_STATE: u8 = 0x04;

pub(crate) const SKS_LOCK_STATE_NAME: &CStr = c"CHANGE_LOCK_STATE";

const OP_SKS_TOKEN_CREATE: u8 = 0x1c;

pub(crate) const SKS_TOKEN_CREATE_NAME: &CStr = c"AUTH_TOKEN_CREATE";

const OP_SKS_TOKEN_VERIFY: u8 = 0x1d;

static_assert!(OP_SKS_TOKEN_VERIFY == OP_SKS_TOKEN_CREATE + 1);

#[derive(Clone, Copy)]
pub(crate) struct NewDeviceState(i32);

impl NewDeviceState {
    pub(crate) const UNLOCKED: NewDeviceState = NewDeviceState(0);

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}
static_assert!(NewDeviceState::UNLOCKED.value() == 0);

const OP_SKS_DEVICE_STATE_TRANSITION: u8 = 0x18;
static_assert!(OP_SKS_DEVICE_STATE_TRANSITION + 1 == 0x19);

pub(crate) const SKS_DEVICE_STATE_REPLY_LEN: usize = 20;
static_assert!(SKS_DEVICE_STATE_REPLY_LEN == 4 + 8 + 8);

const OP_SKS_VERIFY_SECRET: u8 = 0x21;
pub(crate) const SKS_VERIFY_SECRET_NAME: &CStr = c"VERIFY_SECRET";
pub(crate) const SKS_VERIFY_SECRET_VARIANT: u32 = 1;

pub(crate) fn encode_sks_verify_secret(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_VERIFY_SECRET, seq.value(), len.value());
    Some(msg)
}

pub(crate) const SKS_VERIFY_SECRET_REPLY_LEN: usize = 12;
static_assert!(SKS_VERIFY_SECRET_REPLY_LEN == 4 + 8);

const SKS_SELECTOR_BITS: u8 = 0x7f;

static_assert!(SKS_SELECTOR_MAX <= SKS_SELECTOR_BITS);

pub(crate) const SKS_CLIENT_ID: u64 = u64::from_be_bytes(*b"LINUXSKS");
static_assert!(SKS_CLIENT_ID == 0x4c49_4e55_5853_4b53);

pub(crate) const SKS_CLIENT_ID_SEALING: u64 = u64::from_be_bytes(*b"LINUXSLS");
static_assert!(SKS_CLIENT_ID_SEALING == 0x4c49_4e55_5853_4c53);
static_assert!(SKS_CLIENT_ID_SEALING != SKS_CLIENT_ID);

#[derive(Clone, Copy)]
pub(crate) struct KeyBagHandle(i32);

impl KeyBagHandle {

    pub(crate) const fn from_load_reply(v: i32) -> KeyBagHandle {
        KeyBagHandle(v)
    }

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}

const OP_SKS_DESIGNATE_KEYBAG: u8 = 0x0d;

pub(crate) const SKS_DESIGNATE_VARIANT: u32 = 1;

pub(crate) const SKS_DESIGNATE_VARIANT_GENERIC: u32 = 0;

static_assert!(SKS_DESIGNATE_VARIANT == 1);
static_assert!(SKS_DESIGNATE_VARIANT_GENERIC == 0);
static_assert!(SKS_DESIGNATE_VARIANT != SKS_DESIGNATE_VARIANT_GENERIC);

pub(crate) const SKS_CREATE_VARIANT_IDENTITY: u32 = 5;
static_assert!(SKS_CREATE_VARIANT_IDENTITY != 1);

pub(crate) const SKS_IDENTITY_UUID_LEN: usize = 16;
static_assert!(SKS_IDENTITY_UUID_LEN == crate::sbio::IDENTITY_UUID_LEN);

pub(crate) const SKS_IDENTITY_USER_ID: i32 = 1000;

pub(crate) const SKS_DESIGNATE_USER_MIN: i32 = 10;
static_assert!(SKS_DESIGNATE_USER_MIN > 0);

#[derive(Clone, Copy)]
pub(crate) struct DesignateUser(i32);

impl DesignateUser {
    pub(crate) fn new(value: i32) -> Option<DesignateUser> {
        if value >= SKS_DESIGNATE_USER_MIN {
            Some(DesignateUser(value))
        } else {
            None
        }
    }

    pub(crate) const fn special_handle(&self) -> SpecialHandle {
        SpecialHandle(-self.0)
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SpecialHandle(i32);

impl SpecialHandle {
}

impl SpecialHandle {
    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}

pub(crate) const SKS_AUTH_TOKEN_LEN: usize = 16;

pub(crate) struct AuthToken([u8; SKS_AUTH_TOKEN_LEN]);

impl AuthToken {
    pub(crate) fn from_reply(body: &[u8]) -> Option<AuthToken> {
        if body.len() < 8 {
            return None;
        }
        let status = i32::from_le_bytes([body[0], body[1], body[2], body[3]]);
        if status != 0 {
            return None;
        }
        let len = u32::from_le_bytes([body[4], body[5], body[6], body[7]]) as usize;
        if len != SKS_AUTH_TOKEN_LEN || body.len() < 8 + len {
            return None;
        }
        let mut out = [0u8; SKS_AUTH_TOKEN_LEN];
        out.copy_from_slice(&body[8..8 + len]);
        Some(AuthToken(out))
    }

}

impl Drop for AuthToken {
    fn drop(&mut self) {
        for b in self.0.iter_mut() {
            // SAFETY: a valid, uniquely borrowed byte; volatile so the wipe is not elided.
            unsafe { core::ptr::write_volatile(b, 0) };
        }
    }
}

pub(crate) struct Designation {
    source: KeyBagHandle,
    user: DesignateUser,
}

impl Designation {
    pub(crate) fn new(source: KeyBagHandle, user: DesignateUser) -> Designation {
        Designation { source, user }
    }

    pub(crate) const fn source(&self) -> KeyBagHandle {
        self.source
    }

    pub(crate) const fn user(&self) -> DesignateUser {
        self.user
    }
}

pub(crate) const SKS_DESIGNATE_REPLY_LEN: usize = 4;

pub(crate) const SKS_DESIGNATE_FLAGS: u64 = 0;
pub(crate) const SKS_DESIGNATE_FLAGS_DEVICE_KEYBAG: u64 = 0x100;
static_assert!(SKS_DESIGNATE_FLAGS == 0);
static_assert!(SKS_DESIGNATE_FLAGS != SKS_DESIGNATE_FLAGS_DEVICE_KEYBAG);

// keybag_flags sits at +0x64 in a 0x01 body
pub(crate) struct CreateFlags(u32);

pub(crate) const SKS_KEYBAG_FLAG_MAX: u32 = 0xff;
static_assert!((SKS_KEYBAG_FLAG_MAX as u64) < SKS_DESIGNATE_FLAGS_DEVICE_KEYBAG);

impl CreateFlags {
    pub(crate) const fn new(bits: u32) -> Option<CreateFlags> {
        if bits <= SKS_KEYBAG_FLAG_MAX {
            Some(CreateFlags(bits))
        } else {
            None
        }
    }

    pub(crate) const fn none() -> CreateFlags {
        CreateFlags(0)
    }

    pub(crate) const fn value(&self) -> u32 {
        self.0
    }
}

static_assert!(CreateFlags::new(SKS_DESIGNATE_FLAGS_DEVICE_KEYBAG as u32).is_none());
static_assert!(CreateFlags::new(0x100).is_none());
static_assert!(CreateFlags::new(0x101).is_none());
static_assert!(CreateFlags::new(0x1ff).is_none());
static_assert!(CreateFlags::new(u32::MAX).is_none());
static_assert!(CreateFlags::new(0x80).is_some());
static_assert!(CreateFlags::none().value() == 0);

// device keybag = 0x0d with a u64 flags of 0x100 at +0x78

pub(crate) fn encode_sks_designate(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_DESIGNATE_KEYBAG, seq.value(), len.value());
    Some(msg)
}

pub(crate) const SKS_DESIGNATE_NAME: &CStr = c"DESIGNATE_KEYBAG";

const OP_SKS_WRAP: u8 = 0x08;
const OP_SKS_UNWRAP: u8 = 0x45;

// 0x08/0x45 carry no keybag-handle field

const OP_SKS_CHANGE_SECRET: u8 = 0x07;
const OP_SKS_DRAIN_BACKUP_KEYS: u8 = 0x17;
const OP_SKS_ESCROW_ENABLE: u8 = 0x29;
const OP_SKS_ESCROW_CREATE: u8 = 0x13;
const OP_SKS_ESCROW_PERSIST: u8 = 0x2b;
// 0x13 reply is status only, no handle

const OP_SKS_PUBLIC_BACKUP_HANDLE: u8 = 0x2d;
const OP_SKS_UNLOAD_PUBLIC_BACKUP: u8 = 0x2e;

static_assert!(SKS_DESIGNATE_FLAGS_DEVICE_KEYBAG == 0x100);
static_assert!(SKS_DESIGNATE_FLAGS != SKS_DESIGNATE_FLAGS_DEVICE_KEYBAG);

const OP_SKS_MAKE_BACKUP_BAG: u8 = 0x11;
const OP_SKS_SET_BACKUP_BAG: u8 = 0x0e;
const OP_SKS_BACKUP_WRAP: u8 = 0x54;
const OP_SKS_BACKUP_UNWRAP: u8 = 0x55;

pub(crate) const SKS_SET_BACKUP_NAME: &CStr = c"SET_BACKUP_BAG";

pub(crate) fn encode_sks_set_backup_bag(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_SET_BACKUP_BAG, seq.value(), len.value());
    Some(msg)
}

const OP_SKS_SET_ENV: u8 = 0x2a;

pub(crate) const SKS_CLIENT_ID_ALT: u64 = u64::from_be_bytes(*b"LINUXSKT");
static_assert!(SKS_CLIENT_ID_ALT != SKS_CLIENT_ID);
static_assert!(SKS_CLIENT_ID_ALT != SKS_CLIENT_ID_SEALING);
const fn client_id_byte_distance(a: u64, b: u64) -> u32 {
    let (x, y) = (a.to_be_bytes(), b.to_be_bytes());
    let mut differing = 0;
    let mut i = 0;
    while i < 8 {
        if x[i] != y[i] {
            differing += 1;
        }
        i += 1;
    }
    differing
}
static_assert!(client_id_byte_distance(SKS_CLIENT_ID, SKS_CLIENT_ID_ALT) == 1);

const OP_SKS_UNLOAD_KEYBAG: u8 = 0x05;
pub(crate) const SKS_UNLOAD_NAME: &CStr = c"UNLOAD_KEYBAG";

pub(crate) fn encode_sks_unload_keybag(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_UNLOAD_KEYBAG, seq.value(), len.value());
    Some(msg)
}

#[derive(Clone, Copy)]
pub(crate) struct Sequence(u8);

const SEQ_MIN: u8 = 0x60;

impl Sequence {
    pub(crate) const fn from_counter(n: u8) -> Sequence {
        Sequence(SEQ_MIN | (n & !SEQ_MIN))
    }

    pub(crate) const fn value(&self) -> u8 {
        self.0
    }
}

pub(crate) const SEQ_SPACE: usize = 64;

pub(crate) const SEQ_FIRST_REUSE: u8 = 32;

static_assert!(Sequence::from_counter(0).value() != Sequence::from_counter(1).value());
static_assert!(
    Sequence::from_counter(0).value() == Sequence::from_counter(SEQ_FIRST_REUSE).value()
);
static_assert!(SEQ_SPACE == 64);

const fn every_sequence_is_above_the_selector_range() -> bool {
    let mut n = 0u8;
    loop {
        if Sequence::from_counter(n).value() <= SKS_SELECTOR_MAX {
            return false;
        }
        if n == 0xff {
            return true;
        }
        n += 1;
    }
}
static_assert!(every_sequence_is_above_the_selector_range());

#[derive(Clone, Copy)]
pub(crate) struct ImageLen(u16);

impl ImageLen {
    pub(crate) fn of(len: usize) -> Option<ImageLen> {
        if len < crate::image::HEADER_WIRE {
            return None;
        }
        match u16::try_from(len) {
            Ok(v) => Some(ImageLen(v)),
            Err(_) => None,
        }
    }

    pub(crate) const fn value(&self) -> u16 {
        self.0
    }
}

const fn encode_sks_raw(selector: u8, seq: u8, len: u16) -> Message {
    Message {
        msg0: (EP_SKS as u64)
            | ((selector as u64) << MSG_TAG_SHIFT)
            | ((seq as u64) << MSG_TYPE_SHIFT)
            | ((len as u64) << 48),
        msg1: 0,
    }
}

pub(crate) fn encode_sks_read(op: &SksOp, seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(op.opcode(), seq.value(), len.value())
}

pub(crate) fn encode_sks_load(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_LOAD_KEYBAG, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_change_lock_state(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_CHANGE_LOCK_STATE, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_token_create(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_TOKEN_CREATE, seq.value(), len.value());
    Some(msg)
}

pub(crate) struct SksReply {
    pub(crate) selector: u8,
    pub(crate) seq: u8,
    pub(crate) status: i8,
    pub(crate) flags: u16,
    pub(crate) response_size: u16,
}

pub(crate) fn decode_sks(msg: &Message) -> SksReply {
    let b = msg.msg0.to_le_bytes();
    SksReply {
        selector: b[1] & !SKS_REPLY_BIT,
        seq: b[2],
        status: b[3] as i8,
        flags: u16::from_le_bytes([b[4], b[5]]),
        response_size: u16::from_le_bytes([b[6], b[7]]),
    }
}

pub(crate) const SKS_STATUS_MALFORMED: i8 = -13;
static_assert!(SKS_STATUS_MALFORMED as u8 == 0xf3);

pub(crate) const SKS_STATUS_REFUSED: i8 = -19;
static_assert!(SKS_STATUS_REFUSED != SKS_STATUS_MALFORMED);
static_assert!(SKS_STATUS_REFUSED != 0);
