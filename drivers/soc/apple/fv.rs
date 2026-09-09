// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
//! Native FileVault key hierarchy: device-bound volume-key provisioning,
//! reverse-engineered and kept as capability. Not wired to a caller.
#![allow(dead_code)]

use crate::{image, proto, shim};
use crate::{LockState, SepData, SksRequest};
use kernel::prelude::*;

impl SepData {
    const FV_LABEL: &'static [u8; 16] = b"AppleSEPvek00001";
    const FV_PKH: &'static [u8; 16] = b"AppleSEP-KEK-001";
    const FV_PARAM_LEN: usize = 0x130;

    fn fv_param(&self) -> Result<KVec<u8>> {
        let mut p: KVec<u8> = KVec::new();
        p.resize(Self::FV_PARAM_LEN, 0u8, GFP_KERNEL)?;
        p[0x10..0x20].copy_from_slice(Self::FV_LABEL);
        Ok(p)
    }

    fn sks_req_fv(&self, selector: u8, body: &image::Body) -> Result<SksRequest> {
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_fv(selector, self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_PERFORM_OP_NAME,
            msg,
            img,
        })
    }

    fn fv_send(
        &self,
        selector: u8,
        body: image::Body,
        _what: &str,
    ) -> Option<(i32, Option<i32>, Option<KVec<u8>>)> {
        let out = self.sks_send(self.sks_req_fv(selector, &body))?;
        let mailbox: i32 = out.reply.status.into();
        let (mut opst, mut blob) = (None, None);
        if mailbox == 0 {
            if let Some(rbody) = self.sks_report_response(crate::sks::SKS_PERFORM_OP_NAME, &out) {
                let mut f = proto::FieldCursor::new(rbody);
                opst = f.i32();
                if let Some(b) = f.blob() {
                    let mut owned: KVec<u8> = KVec::new();
                    owned.extend_from_slice(b, GFP_KERNEL).ok()?;
                    blob = Some(owned);
                }
            }
        }
        Some((mailbox, opst, blob))
    }

    /// `0x42` mint the device-bound key-encryption key.
    fn fv_new_kek(&self, handle: u64, param: &[u8]) -> Option<KVec<u8>> {
        let mut b = image::Body::new();
        b.put_u32(0).ok()?;
        b.put_u64(handle).ok()?;
        b.put_blob(param).ok()?;
        b.put_u32(0).ok()?;
        b.put_blob(&[]).ok()?;
        b.put_blob(Self::FV_PKH).ok()?;
        let (mb, op, blob) = self.fv_send(crate::sks::OP_SKS_FV_NEW_KEK, b, "new_kek")?;
        (mb == 0 && op == Some(0)).then_some(())?;
        blob
    }

    /// `0x40` mint the wrapped, device-bound volume key.
    fn fv_new_vek(&self, handle: u64, param: &[u8]) -> Option<KVec<u8>> {
        let mut b = image::Body::new();
        b.put_u32(0).ok()?;
        b.put_u64(handle).ok()?;
        b.put_blob(param).ok()?;
        b.put_blob(&[]).ok()?;
        b.put_blob(&[]).ok()?;
        b.put_blob(Self::FV_PKH).ok()?;
        let (mb, op, blob) = self.fv_send(crate::sks::OP_SKS_FV_NEW_VEK, b, "new_vek")?;
        (mb == 0 && op == Some(0)).then_some(())?;
        blob
    }

    /// `0x41` install the volume key into our collection (empty KEK slot = self-derive).
    fn fv_unwrap_vek(&self, handle: u64, param: &[u8], wrapped_vek: &[u8]) -> Option<KVec<u8>> {
        let mut b = image::Body::new();
        b.put_u32(0).ok()?;
        b.put_u64(handle).ok()?;
        b.put_blob(param).ok()?;
        b.put_u32(0).ok()?;
        b.put_blob(&[]).ok()?;
        b.put_blob(&[]).ok()?;
        b.put_blob(wrapped_vek).ok()?;
        b.put_blob(&[]).ok()?;
        let (mb, op, blob) = self.fv_send(crate::sks::OP_SKS_FV_UNWRAP_VEK, b, "unwrap_vek")?;
        (mb == 0 && op == Some(0)).then_some(())?;
        blob.or_else(|| Some(KVec::new()))
    }

    fn fv_dump(path: &CStr, data: &[u8]) {
        if let Ok(f) = shim::StoreFile::open(path) {
            let _ = f.write_all(0, data);
            let _ = f.sync();
        }
    }

    fn fv_load(path: &CStr) -> Option<KVec<u8>> {
        let f = shim::StoreFile::open_readonly(path).ok()?;
        let sz = f.size().unwrap_or(0);
        if !(1..=8192).contains(&sz) {
            return None;
        }
        let mut buf: KVec<u8> = KVec::new();
        buf.resize(sz as usize, 0u8, GFP_KERNEL).ok()?;
        f.read_exact(0, &mut buf).ok()?;
        Some(buf)
    }

    /// Provisions the device-bound FileVault key hierarchy and installs the volume
    /// key. The destructive clear (`0x47`) is never emitted; the installed VEK is
    /// consumed by the storage inline-AES engine (ANS), so on its own this seals
    /// nothing on Linux.
    pub(crate) fn fv_provision(&self, handle: crate::sks::KeyBagHandle, secret: &[u8]) -> Option<u64> {
        const KEK_PATH: &CStr = c"/var/lib/apple-sep-fv-kek.bin";
        const VEK_PATH: &CStr = c"/var/lib/apple-sep-fv-vek.bin";

        if let Some(healthy) = self.sks_health_check(c"fv provision") {
            let _ = self.sks_send(self.sks_req_change_lock_state(
                handle,
                LockState::Unlocked,
                secret,
                healthy,
            ));
        }
        let param = self.fv_param().ok()?;
        let h = handle.value() as u64;

        let (_kek, vek) = match (Self::fv_load(KEK_PATH), Self::fv_load(VEK_PATH)) {
            (Some(kek), Some(vek)) => (kek, vek),
            _ => {
                let kek = self.fv_new_kek(h, &param)?;
                let vek = self.fv_new_vek(h, &param)?;
                Self::fv_dump(KEK_PATH, &kek);
                Self::fv_dump(VEK_PATH, &vek);
                (kek, vek)
            }
        };

        let installed = self.fv_unwrap_vek(h, &param, &vek)?;
        let vek_handle = if installed.len() >= 4 {
            u64::from(u32::from_le_bytes([installed[0], installed[1], installed[2], installed[3]]))
        } else {
            0
        };
        Some(vek_handle)
    }
}
