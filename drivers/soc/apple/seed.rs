// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! Importing the enclave's existing records.

use crate::shim;
use crate::store::{Key, Store};
use kernel::prelude::*;

pub(crate) const SEED_PATH: &CStr = c"/var/lib/apple-sep-seed.bin";

const MAGIC: [u8; 4] = *b"AXRT";
const HEADER_LEN: usize = 8;
const RECORD_HEADER_LEN: usize = 1 + 16 + 4;

const MIN_RECORD_LEN: u32 = 1;
const MAX_RECORD_LEN: u32 = 0x8000;

const MAX_SEED_BYTES: u64 = 1 << 20;

const MAX_RECORDS: u32 = 64;

const TYPE_ROOT_LOW: u8 = 1;
const TYPE_ROOT_HIGH: u8 = 2;
const TYPE_SESSION_HIGH: u8 = 4;

pub(crate) struct Imported {
    pub(crate) records: usize,
    pub(crate) roots: usize,
    pub(crate) sessions: usize,
    pub(crate) bytes: usize,
}

fn le32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn read_seed() -> Result<KVec<u8>> {
    let file = shim::StoreFile::open_readonly(SEED_PATH)?;
    let size = file.size()?;
    if size < HEADER_LEN as u64 || size > MAX_SEED_BYTES {
        return Err(EINVAL);
    }
    let mut buf = KVec::with_capacity(size as usize, GFP_KERNEL)?;
    buf.resize(size as usize, 0, GFP_KERNEL)?;
    file.read_exact(0, &mut buf)?;
    Ok(buf)
}

pub(crate) fn import(store: &mut Store) -> Result<Imported> {
    let buf = read_seed()?;

    if buf[..4] != MAGIC {
        return Err(EINVAL);
    }
    let count = le32(&buf, 4);
    if count == 0 || count > MAX_RECORDS {
        return Err(EINVAL);
    }

    let mut offsets: KVec<(u8, [u8; 16], usize, usize)> = KVec::new();
    let mut pos = HEADER_LEN;
    for _ in 0..count {
        if pos + RECORD_HEADER_LEN > buf.len() {
            return Err(EINVAL);
        }
        let kind = buf[pos];
        let mut uuid = [0u8; 16];
        uuid.copy_from_slice(&buf[pos + 1..pos + 17]);
        let len = le32(&buf, pos + 17);
        pos += RECORD_HEADER_LEN;

        if !(TYPE_ROOT_LOW..=TYPE_SESSION_HIGH).contains(&kind) {
            return Err(EINVAL);
        }
        if !(MIN_RECORD_LEN..=MAX_RECORD_LEN).contains(&len) {
            return Err(EINVAL);
        }
        // Root records carry an all-zero UUID; anything else would be unreachable.
        let is_root = kind == TYPE_ROOT_LOW || kind == TYPE_ROOT_HIGH;
        if is_root && uuid != [0u8; 16] {
            return Err(EINVAL);
        }
        let len = len as usize;
        if pos + len > buf.len() {
            return Err(EINVAL);
        }
        offsets.push((kind, uuid, pos, len), GFP_KERNEL)?;
        pos += len;
    }

    // Writes are keyed, so re-running after a crash simply replaces.
    let mut result = Imported {
        records: 0,
        roots: 0,
        sessions: 0,
        bytes: 0,
    };
    for (kind, uuid, at, len) in offsets {
        store.write(&Key::new(kind, uuid), &buf[at..at + len])?;
        result.records += 1;
        result.bytes += len;
        if kind == TYPE_ROOT_LOW || kind == TYPE_ROOT_HIGH {
            result.roots += 1;
        } else {
            result.sessions += 1;
        }
    }

    // Mark consumed last: a crash above leaves the flag clear and the import reruns.
    store.mark_seeded()?;
    Ok(result)
}
