// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple SEP driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings,
    container_of,
    device,
    dma,
    error::to_result,
    ffi::{
        c_int,
        c_ulong,
        c_void, //
    },
    io::poll::read_poll_timeout,
    module_platform_driver,
    new_condvar,
    new_mutex,
    of,
    platform,
    prelude::*,
    soc::apple::mailbox::{
        MailCallback,
        Mailbox,
        Message, //
    },
    sync::{
        aref::ARef,
        Arc,
        CondVar,
        CondVarTimeoutResult,
        Mutex, //
    },
    time::{
        msecs_to_jiffies,
        Delta, //
    },
    types::{
        ForeignOwnable,
        Opaque, //
    },
    workqueue::{
        self,
        impl_has_work,
        new_work,
        Work,
        WorkItem, //
    }, //
};

const SHMEM_SIZE: usize = 0x30000;
const MSG_BOOT_TZ0: u64 = 0x5;
const MSG_BOOT_IMG4: u64 = 0x6;
const MSG_SET_SHMEM: u64 = 0x18;
const MSG_BOOT_TZ0_ACK1: u64 = 0x69;
const MSG_BOOT_TZ0_ACK2: u64 = 0xD2;
const MSG_BOOT_IMG4_ACK: u64 = 0x6A;
const MSG_ADVERTISE_EP: u64 = 0;
const MSG_CONTROL_SLEEP: u64 = 0x0C;
const MSG_CONTROL_OS_ACTIVE: u64 = 0x15;
const EP_CONTROL: u64 = 0;
const EP_DISCOVER: u64 = 0xFD;
const EP_SHMEM: u64 = 0xFE;
const EP_BOOT: u64 = 0xFF;

/// SLEEP parameter selecting a suspend to RAM (as opposed to hibernation).
const SLEEP_TYPE_SUSPEND: u64 = 2;
/// Time the coprocessor has to answer a control message.
const CONTROL_REPLY_TIMEOUT_MS: u32 = 15000;
/// Time the coprocessor has to gate its own power domain after SLEEP.
const POWER_GATE_TIMEOUT_MS: i64 = 5000;

const MSG_TYPE_SHIFT: u32 = 16;
const MSG_TYPE_MASK: u64 = 0xFF;
const MSG_TAG_SHIFT: u32 = 8;
const MSG_TAG_MASK: u64 = 0xFF;
const MSG_PARAM_SHIFT: u32 = 24;
const MSG_PARAM_MASK: u64 = 0xFF;

const MSG_EP_MASK: u64 = 0xFF;
const MSG_DATA_SHIFT: u32 = 32;

const IOVA_SHIFT: u32 = 0xC;

const ASC_CPU_CONTROL: usize = 0x44;
const ASC_CPU_CONTROL_RUN: u32 = 1 << 4;

const PS_ACTUAL_SHIFT: u32 = 4;
const PS_ACTUAL_MASK: u32 = 0xF;

type ShMem = dma::Coherent<[u8]>;

fn align_up(v: usize, a: usize) -> usize {
    (v + a - 1) & !(a - 1)
}

fn memcpy_to_iomem(iomem: &mut ShMem, off: usize, src: &[u8]) -> Result<()> {
    // SAFETY:
    // as_slice_mut() checks that off and src.len() are whithin iomem's limits.
    // memcpy_to_iomem is only called from within probe() ansuring there are no
    // concurrent read and write accesses to the same region while the slice is
    // alive per as_slice_mut()'s requiremnts.
    unsafe {
        let target = &mut iomem.as_mut()[off..off + src.len()];
        target.copy_from_slice(src);
    }
    Ok(())
}

fn build_shmem(dev: &platform::Device<device::Core>) -> Result<ShMem> {
    let fwnode = dev.as_ref().fwnode().ok_or(EIO)?;
    let mut iomem = dma::Coherent::<u8>::zeroed_slice(dev.as_ref(), SHMEM_SIZE, GFP_KERNEL)?;

    let panic_offset = 0x4000;
    let panic_size = 0x8000;
    memcpy_to_iomem(&mut iomem, panic_offset, &1u32.to_le_bytes())?;

    let lpol_offset = panic_offset + panic_size;
    let lpol_prop_name = c"local-policy-manifest";
    let lpol_prop_size = fwnode.property_count_elem::<u8>(lpol_prop_name)?;
    let lpol = fwnode
        .property_read_array_vec(lpol_prop_name, lpol_prop_size)?
        .required_by(dev.as_ref())?;
    memcpy_to_iomem(
        &mut iomem,
        lpol_offset,
        &(lpol_prop_size as u32).to_le_bytes(),
    )?;
    memcpy_to_iomem(&mut iomem, lpol_offset + 4, &lpol)?;
    let lpol_size = align_up(lpol_prop_size + 4, 0x4000);

    let ibot_offset = lpol_offset + lpol_size;
    let ibot_prop_name = c"iboot-manifest";
    let ibot_prop_size = fwnode.property_count_elem::<u8>(ibot_prop_name)?;
    let ibot = fwnode
        .property_read_array_vec(ibot_prop_name, ibot_prop_size)?
        .required_by(dev.as_ref())?;
    memcpy_to_iomem(
        &mut iomem,
        ibot_offset,
        &(ibot_prop_size as u32).to_le_bytes(),
    )?;
    memcpy_to_iomem(&mut iomem, ibot_offset + 4, &ibot)?;
    let ibot_size = align_up(ibot_prop_size + 4, 0x4000);

    memcpy_to_iomem(&mut iomem, 0, b"CNIP")?;
    memcpy_to_iomem(&mut iomem, 4, &(panic_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&mut iomem, 8, &(panic_offset as u32).to_le_bytes())?;

    memcpy_to_iomem(&mut iomem, 16, b"OPLA")?;
    memcpy_to_iomem(&mut iomem, 16 + 4, &(lpol_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&mut iomem, 16 + 8, &(lpol_offset as u32).to_le_bytes())?;

    memcpy_to_iomem(&mut iomem, 32, b"IPIS")?;
    memcpy_to_iomem(&mut iomem, 32 + 4, &(ibot_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&mut iomem, 32 + 8, &(ibot_offset as u32).to_le_bytes())?;

    memcpy_to_iomem(&mut iomem, 48, b"llun")?;
    Ok(iomem)
}

#[pin_data]
struct SepReceiveWork {
    data: Arc<SepData>,
    msg: Message,
    #[pin]
    work: Work<SepReceiveWork>,
}

impl_has_work! {
    impl HasWork<Self, 0> for SepReceiveWork { self.work }
}

impl SepReceiveWork {
    fn new(data: Arc<SepData>, msg: Message) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(SepReceiveWork {
                data,
                msg,
                work <- new_work!("SepReceiveWork::work"),
            }),
            GFP_ATOMIC,
        )
    }
}

impl WorkItem for SepReceiveWork {
    type Pointer = Arc<SepReceiveWork>;

    fn run(this: Arc<SepReceiveWork>) {
        this.data.process_message(this.msg);
    }
}

struct FwRegionParams {
    addr: u64,
    size: usize,
}

/// Returns the physical address of the power state register of the
/// coprocessor's power domain. The domain node's `reg` is an offset into the
/// register block of its parent power manager node.
fn ps_reg_addr(dev: &device::Device) -> Result<bindings::phys_addr_t> {
    // SAFETY: `dev` is a valid device; its `of_node` is either NULL or valid.
    let np = unsafe { (*dev.as_raw()).of_node };
    if np.is_null() {
        return Err(ENOENT);
    }
    // SAFETY: `np` is valid. A non-NULL result carries a reference.
    let ps = unsafe { bindings::of_parse_phandle(np, c"power-domains".as_char_ptr(), 0) };
    if ps.is_null() {
        return Err(ENOENT);
    }
    let mut offset: u32 = 0;
    // SAFETY: `ps` is valid and `offset` is valid for writes.
    let ret =
        unsafe { bindings::of_property_read_u32_index(ps, c"reg".as_char_ptr(), 0, &mut offset) };
    // SAFETY: `ps` is valid; the parent is returned with its own reference.
    let pmgr = unsafe { bindings::of_get_parent(ps) };
    // SAFETY: Drops the reference taken by `of_parse_phandle`.
    unsafe { bindings::of_node_put(ps) };
    if pmgr.is_null() {
        return Err(ENOENT);
    }
    let mut res = bindings::resource::default();
    // SAFETY: `pmgr` is valid and `res` is valid for writes.
    let ret2 = unsafe { bindings::of_address_to_resource(pmgr, 0, &mut res) };
    // SAFETY: Drops the reference taken by `of_get_parent`.
    unsafe { bindings::of_node_put(pmgr) };
    to_result(ret)?;
    to_result(ret2)?;
    Ok(res.start + bindings::phys_addr_t::from(offset))
}

/// Registers used to stop and start the coprocessor and to observe its power
/// state across system sleep.
struct SleepRegs {
    asc: *mut c_void,
    ps: *mut c_void,
}

impl SleepRegs {
    fn new(pdev: &platform::Device<device::Core>) -> Result<Self> {
        let asc_start = pdev.resource_by_index(0).ok_or(ENOENT)?.start();
        let ps_start = ps_reg_addr(pdev.as_ref())?;
        // SAFETY: Maps the coprocessor control block described by the device's
        // first register range. The mailbox driver maps another part of the
        // same block, so the range is not requested exclusively.
        let asc = unsafe { bindings::ioremap_np(asc_start, ASC_CPU_CONTROL + 4) };
        if asc.is_null() {
            return Err(ENOMEM);
        }
        // SAFETY: Maps the single power state register of the coprocessor's
        // power domain. It is only read.
        let ps = unsafe { bindings::ioremap_np(ps_start, 4) };
        if ps.is_null() {
            // SAFETY: `asc` was mapped above.
            unsafe { bindings::iounmap(asc) };
            return Err(ENOMEM);
        }
        Ok(Self { asc, ps })
    }

    fn set_cpu_running(&self, run: bool) {
        let reg = self.asc.cast::<u8>().wrapping_add(ASC_CPU_CONTROL).cast::<c_void>();
        // SAFETY: `reg` lies within the mapping created in `new`.
        unsafe {
            let val = bindings::readl(reg);
            let val = if run {
                val | ASC_CPU_CONTROL_RUN
            } else {
                val & !ASC_CPU_CONTROL_RUN
            };
            bindings::writel(val, reg);
        }
    }

    /// Returns the power level the domain is currently in; 0 is power-gated.
    fn ps_actual(&self) -> u32 {
        // SAFETY: `ps` is the mapping created in `new`.
        (unsafe { bindings::readl(self.ps) } >> PS_ACTUAL_SHIFT) & PS_ACTUAL_MASK
    }
}

impl Drop for SleepRegs {
    fn drop(&mut self) {
        // SAFETY: Both mappings were created in `new` and are no longer used.
        unsafe {
            bindings::iounmap(self.ps);
            bindings::iounmap(self.asc);
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SepState {
    /// Boot handshake sent, waiting for the OS to report itself active.
    Booting,
    /// The OS is running and accepts control messages.
    Active,
    /// SLEEP was acknowledged and the CPU stopped.
    Asleep,
}

struct PmState {
    state: SepState,
    /// Tag of the last control message sent; never 0.
    tag: u8,
    /// Tag of the control message awaiting its reply.
    pending: Option<u8>,
    /// Reply to the pending control message.
    reply: Option<u64>,
}

#[pin_data]
struct SepData {
    dev: ARef<device::Device>,
    #[pin]
    mbox: Mutex<Option<Mailbox<SepData>>>,
    shmem: ShMem,
    region_params: FwRegionParams,
    /// DMA address of the firmware region, mapped on the first boot and kept
    /// for every later one.
    #[pin]
    fw_iova: Mutex<Option<bindings::dma_addr_t>>,
    #[pin]
    pm: Mutex<PmState>,
    #[pin]
    pm_cv: CondVar,
    regs: Option<SleepRegs>,
}

impl SepData {
    fn new(
        dev: &platform::Device<device::Core>,
        region_params: FwRegionParams,
        regs: Option<SleepRegs>,
    ) -> Result<Arc<SepData>> {
        Arc::pin_init(
            try_pin_init!(SepData {
                shmem: build_shmem(dev)?,
                dev: ARef::<device::Device>::from(dev.as_ref()),
                mbox <- new_mutex!(None),
                region_params,
                fw_iova <- new_mutex!(None),
                pm <- new_mutex!(PmState {
                    state: SepState::Booting,
                    tag: 0,
                    pending: None,
                    reply: None,
                }),
                pm_cv <- new_condvar!(),
                regs,
            }),
            GFP_KERNEL,
        )
    }
    fn start(&self) -> Result<()> {
        self.mbox.lock().as_ref().unwrap().send(
            Message {
                msg0: EP_BOOT | (MSG_BOOT_TZ0 << MSG_TYPE_SHIFT),
                msg1: 0,
            },
            false,
        )
    }
    fn map_fw(&self) -> Result<bindings::dma_addr_t> {
        let mut iova = self.fw_iova.lock();
        if let Some(addr) = *iova {
            return Ok(addr);
        }
        // SAFETY: Maps the reserved firmware region for the device.
        let addr = unsafe {
            bindings::dma_map_resource(
                self.dev.as_raw(),
                self.region_params.addr,
                self.region_params.size,
                bindings::dma_data_direction_DMA_TO_DEVICE,
                0,
            )
        };
        // SAFETY: `addr` was returned by `dma_map_resource` for this device.
        if unsafe { bindings::dma_mapping_error(self.dev.as_raw(), addr) } != 0 {
            dev_err!(self.dev, "Failed to map firmware");
            return Err(ENOMEM);
        }
        *iova = Some(addr);
        Ok(addr)
    }
    fn load_fw_and_shmem(&self) -> Result<()> {
        let fw_addr = self.map_fw()? >> IOVA_SHIFT;
        let guard = self.mbox.lock();
        let mbox = guard.as_ref().unwrap();
        mbox.send(
            Message {
                msg0: EP_BOOT | (MSG_BOOT_IMG4 << MSG_TYPE_SHIFT) | (fw_addr << MSG_DATA_SHIFT),
                msg1: 0,
            },
            false,
        )?;
        let shm_addr = self.shmem.dma_handle() >> IOVA_SHIFT;
        mbox.send(
            Message {
                msg0: EP_SHMEM | (MSG_SET_SHMEM << MSG_TYPE_SHIFT) | (shm_addr << MSG_DATA_SHIFT),
                msg1: 0,
            },
            false,
        )?;
        Ok(())
    }
    fn process_boot_msg(&self, msg: Message) {
        let ty = (msg.msg0 >> MSG_TYPE_SHIFT) & MSG_TYPE_MASK;
        match ty {
            MSG_BOOT_TZ0_ACK1 => {}
            MSG_BOOT_TZ0_ACK2 => {
                let res = self.load_fw_and_shmem();
                if let Err(e) = res {
                    dev_err!(self.dev, "Unable to load firmware: {:?}", e);
                }
            }
            MSG_BOOT_IMG4_ACK => {}
            _ => {
                dev_err!(self.dev, "Unknown boot message type: {}", ty);
            }
        }
    }
    fn process_discover_msg(&self, msg: Message) {
        let ty = (msg.msg0 >> MSG_TYPE_SHIFT) & MSG_TYPE_MASK;
        //let data = (msg.msg0 >> MSG_DATA_SHIFT) as u32;
        //let param = (msg.msg0 >> MSG_PARAM_SHIFT) & MSG_PARAM_MASK;
        match ty {
            MSG_ADVERTISE_EP => {
                /*dev_info!(
                    self.dev,
                    "Got endpoint {:?} at {}",
                    core::str::from_utf8(&data.to_be_bytes()),
                    param
                );*/
            }
            _ => {
                //dev_warn!(self.dev, "Unknown discovery message type: {}", ty);
            }
        }
    }
    fn process_control_msg(&self, msg: Message) {
        let ty = (msg.msg0 >> MSG_TYPE_SHIFT) & MSG_TYPE_MASK;
        let tag = ((msg.msg0 >> MSG_TAG_SHIFT) & MSG_TAG_MASK) as u8;
        let mut pm = self.pm.lock();
        if ty == MSG_CONTROL_OS_ACTIVE && pm.state == SepState::Booting {
            pm.state = SepState::Active;
        }
        if pm.pending == Some(tag) {
            pm.pending = None;
            pm.reply = Some(msg.msg0);
        }
        drop(pm);
        self.pm_cv.notify_all();
    }
    fn process_message(&self, msg: Message) {
        let ep = msg.msg0 & MSG_EP_MASK;
        match ep {
            EP_BOOT => self.process_boot_msg(msg),
            EP_DISCOVER => self.process_discover_msg(msg),
            EP_CONTROL => self.process_control_msg(msg),
            _ => {} // dev_warn!(self.dev, "Message from unknown endpoint: {}", ep),
        }
    }
    /// Puts the coprocessor to sleep: once its OS is up, SLEEP is sent on the
    /// control endpoint and must be acknowledged with a zero result. The CPU is
    /// then stopped and the coprocessor gates its own power domain.
    fn sleep(&self, regs: &SleepRegs) -> Result {
        let mut pm = self.pm.lock();
        let mut left = msecs_to_jiffies(CONTROL_REPLY_TIMEOUT_MS);
        while pm.state == SepState::Booting {
            match self.pm_cv.wait_timeout(&mut pm, left) {
                CondVarTimeoutResult::Woken { jiffies } => left = jiffies,
                _ => break,
            }
        }
        match pm.state {
            SepState::Active => {}
            SepState::Asleep => return Ok(()),
            SepState::Booting => {
                dev_warn!(self.dev, "Not running, leaving it awake for sleep\n");
                return Err(EBUSY);
            }
        }
        pm.tag = if pm.tag == u8::MAX { 1 } else { pm.tag + 1 };
        let tag = pm.tag;
        pm.pending = Some(tag);
        pm.reply = None;
        drop(pm);

        let sent = self.mbox.lock().as_ref().ok_or(ENODEV).and_then(|mbox| {
            mbox.send(
                Message {
                    msg0: EP_CONTROL
                        | (u64::from(tag) << MSG_TAG_SHIFT)
                        | (MSG_CONTROL_SLEEP << MSG_TYPE_SHIFT)
                        | (SLEEP_TYPE_SUSPEND << MSG_PARAM_SHIFT),
                    msg1: 0,
                },
                false,
            )
        });

        let mut pm = self.pm.lock();
        let mut left = msecs_to_jiffies(CONTROL_REPLY_TIMEOUT_MS);
        while sent.is_ok() && pm.reply.is_none() {
            match self.pm_cv.wait_timeout(&mut pm, left) {
                CondVarTimeoutResult::Woken { jiffies } => left = jiffies,
                _ => break,
            }
        }
        pm.pending = None;
        let reply = pm.reply.take();
        sent?;
        let Some(reply) = reply else {
            dev_err!(self.dev, "SLEEP reply timeout\n");
            return Err(ETIMEDOUT);
        };
        let result = (reply >> MSG_PARAM_SHIFT) & MSG_PARAM_MASK;
        if result != 0 {
            dev_err!(self.dev, "SLEEP failed with result {}\n", result);
            return Err(EIO);
        }
        pm.state = SepState::Asleep;
        drop(pm);

        regs.set_cpu_running(false);
        read_poll_timeout(
            || Ok(regs.ps_actual()),
            |level: &u32| *level == 0,
            Delta::from_millis(1),
            Delta::from_millis(POWER_GATE_TIMEOUT_MS),
        )
        .map(|_| ())
        .inspect_err(|_| {
            dev_err!(
                self.dev,
                "Failed to power-gate (level {})\n",
                regs.ps_actual()
            );
        })
    }
    /// Restarts the coprocessor after sleep. Waking from sleep goes through
    /// its boot ROM again, so the CPU is started and the full boot handshake
    /// is repeated with the firmware and shared memory already set up.
    fn wake(&self, regs: &SleepRegs) -> Result {
        {
            let mut pm = self.pm.lock();
            if pm.state != SepState::Asleep {
                return Ok(());
            }
            pm.state = SepState::Booting;
        }
        regs.set_cpu_running(true);
        self.start()
    }
    fn remove(&self) {
        *self.mbox.lock() = None;
        if let Some(iova) = *self.fw_iova.lock() {
            // SAFETY: `iova` was mapped in `map_fw` with these parameters.
            unsafe {
                bindings::dma_unmap_resource(
                    self.dev.as_raw(),
                    iova,
                    self.region_params.size,
                    bindings::dma_data_direction_DMA_TO_DEVICE,
                    0,
                );
            }
        }
    }
}

impl MailCallback for SepData {
    type Data = Arc<SepData>;
    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, msg: Message) {
        let work = SepReceiveWork::new(data.into(), msg);
        if let Ok(work) = work {
            let res = workqueue::system().enqueue(work);
            if res.is_err() {
                dev_err!(
                    data.dev,
                    "Unable to schedule work item for message {}",
                    msg.msg0
                );
            }
        } else {
            dev_err!(
                data.dev,
                "Unable to allocate work item for message {}",
                msg.msg0
            );
        }
    }
}

unsafe impl Send for SepData {}
unsafe impl Sync for SepData {}

#[pin_data(PinnedDrop)]
struct SleepNotifier {
    data: Arc<SepData>,
    /// Initialized last: registering it makes the callback live.
    #[pin]
    nb: Opaque<bindings::notifier_block>,
}

impl SleepNotifier {
    fn new(data: Arc<SepData>) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            data,
            nb <- Opaque::try_ffi_init(|slot: *mut bindings::notifier_block| {
                // SAFETY: `slot` is valid for writes and pinned for the life of
                // `Self`; it is unregistered in `drop` before it goes away.
                unsafe {
                    slot.write(bindings::notifier_block {
                        notifier_call: Some(Self::callback),
                        ..Default::default()
                    });
                    to_result(bindings::register_pm_notifier(slot))
                }
            }),
        })
    }

    /// # Safety
    ///
    /// Only called by the PM notifier chain, with `nb` pointing at the `nb`
    /// field of a registered `SleepNotifier`.
    unsafe extern "C" fn callback(
        nb: *mut bindings::notifier_block,
        action: c_ulong,
        _data: *mut c_void,
    ) -> c_int {
        // SAFETY: `nb` is the `nb` field of a registered, pinned `SleepNotifier`.
        let this =
            unsafe { &*container_of!(nb.cast::<Opaque<bindings::notifier_block>>(), Self, nb) };
        let Some(regs) = this.data.regs.as_ref() else {
            return bindings::NOTIFY_DONE as c_int;
        };
        match action as u32 {
            bindings::PM_SUSPEND_PREPARE => {
                if let Err(e) = this.data.sleep(regs) {
                    dev_warn!(this.data.dev, "Failed to enter sleep: {:?}\n", e);
                }
            }
            bindings::PM_POST_SUSPEND => {
                if let Err(e) = this.data.wake(regs) {
                    dev_err!(this.data.dev, "Failed to wake: {:?}\n", e);
                }
            }
            _ => return bindings::NOTIFY_DONE as c_int,
        }
        bindings::NOTIFY_OK as c_int
    }
}

// SAFETY: The notifier block is only touched by the PM notifier chain, which
// serializes its callers; `SepData` is `Send` and `Sync`.
unsafe impl Send for SleepNotifier {}
// SAFETY: See `Send`.
unsafe impl Sync for SleepNotifier {}

#[pinned_drop]
impl PinnedDrop for SleepNotifier {
    fn drop(self: Pin<&mut Self>) {
        // SAFETY: `nb` was registered in `new`.
        unsafe { bindings::unregister_pm_notifier(self.nb.get()) };
    }
}

struct SepDriver {
    sleep: Option<Pin<KBox<SleepNotifier>>>,
    data: Arc<SepData>,
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c"apple,sep"), ())]
);

impl platform::Driver for SepDriver {
    type IdInfo = ();

    const OF_ID_TABLE: Option<of::IdTable<()>> = Some(&OF_TABLE);

    fn probe(
        pdev: &platform::Device<device::Core>,
        _info: Option<&()>,
    ) -> impl PinInit<Self, Error> {
        let of = pdev.as_ref().of_node().ok_or(EIO)?;
        let res = of.reserved_mem_region_to_resource_byname(c"sepfw")?;
        let dev: &device::Device = pdev.as_ref();
        let regs = SleepRegs::new(pdev)
            .inspect_err(|e| dev_info!(dev, "No system sleep support: {:?}\n", e))
            .ok();
        let data = SepData::new(
            pdev,
            FwRegionParams {
                addr: res.start(),
                size: res.size().try_into()?,
            },
            regs,
        )?;
        *data.mbox.lock() = Some(Mailbox::new_byname(pdev.as_ref(), c"mbox", data.clone())?);
        data.start()?;
        let sleep = if data.regs.is_some() {
            Some(KBox::pin_init(SleepNotifier::new(data.clone()), GFP_KERNEL)?)
        } else {
            None
        };
        Ok(Self { sleep, data })
    }
}

impl Drop for SepDriver {
    fn drop(&mut self) {
        // No sleep transition may touch the coprocessor once unbinding starts.
        self.sleep = None;
        self.data.remove();
    }
}

module_platform_driver! {
    type: SepDriver,
    name: "apple_sep",
    description: "Secure enclave processor stub driver",
    license: "Dual MIT/GPL",
}
