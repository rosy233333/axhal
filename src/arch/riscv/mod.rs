#[macro_use]
mod macros;

mod context;

pub use self::context::{GeneralRegisters, TaskContext, TrapFrame};
use memory_addr::{PhysAddr, VirtAddr};
use riscv::asm;
use riscv::register::{satp, sstatus};

/// Allows the current CPU to respond to interrupts.
#[inline]
pub fn enable_irqs() {
    unsafe { sstatus::set_sie() }
}

/// Makes the current CPU to ignore interrupts.
#[inline]
pub fn disable_irqs() {
    unsafe { sstatus::clear_sie() }
}

/// Returns whether the current CPU is allowed to respond to interrupts.
#[inline]
pub fn irqs_enabled() -> bool {
    sstatus::read().sie()
}

/// Relaxes the current CPU and waits for interrupts.
///
/// It must be called with interrupts enabled, otherwise it will never return.
#[inline]
pub fn wait_for_irqs() {
    unsafe { riscv::asm::wfi() }
}

/// Halt the current CPU.
#[inline]
pub fn halt() {
    disable_irqs();
    unsafe { riscv::asm::wfi() } // should never return
}

/// Reads the register that stores the current page table root.
///
/// Returns the physical address of the page table root.
#[inline]
pub fn read_page_table_root() -> PhysAddr {
    PhysAddr::from(satp::read().ppn() << 12)
}

/// Writes the register to update the current page table root.
///
/// # Safety
///
/// This function is unsafe as it changes the virtual memory address space.
pub unsafe fn write_page_table_root(root_paddr: PhysAddr) {
    let old_root = read_page_table_root();
    trace!("set page table root: {:#x} => {:#x}", old_root, root_paddr);
    if old_root != root_paddr {
        satp::set(satp::Mode::Sv39, 0, root_paddr.as_usize() >> 12);
        asm::sfence_vma_all();
    }
}
pub use self::write_page_table_root as write_page_table_root0;

/// Flushes the TLB.
///
/// If `vaddr` is [`None`], flushes the entire TLB. Otherwise, flushes the TLB
/// entry that maps the given virtual address.
#[inline]
pub fn flush_tlb(vaddr: Option<VirtAddr>) {
    unsafe {
        if let Some(vaddr) = vaddr {
            asm::sfence_vma(0, vaddr.as_usize())
        } else {
            asm::sfence_vma_all();
        }
    }
}

/// Reads the thread pointer of the current CPU.
///
/// It is used to implement TLS (Thread Local Storage).
#[inline]
pub fn read_thread_pointer() -> usize {
    let tp;
    unsafe { core::arch::asm!("mv {}, tp", out(reg) tp) };
    tp
}

/// Writes the thread pointer of the current CPU.
///
/// It is used to implement TLS (Thread Local Storage).
///
/// # Safety
///
/// This function is unsafe as it changes the CPU states.
#[inline]
pub unsafe fn write_thread_pointer(tp: usize) {
    core::arch::asm!("mv tp, {}", in(reg) tp)
}

include_asm_marcos!();

#[cfg(feature = "signal")]
core::arch::global_asm!(include_str!("signal.S"));

#[cfg(feature = "monolithic")]
#[no_mangle]
/// To handle the first time into the user space
///
/// 1. push the given trap frame into the kernel stack
/// 2. go into the user space
///
/// args:
///
/// 1. kernel_sp: the top of the kernel stack
///
/// 2. frame_base: the address of the trap frame which will be pushed into the kernel stack
pub fn first_into_user(kernel_sp: usize, frame_base: usize) {
    // Make sure that all csr registers are stored before enable the interrupt

    disable_irqs();
    flush_tlb(None);
    info!("trap: {:x?}", kernel_sp);
    let trap_frame_size = core::mem::size_of::<TrapFrame>();
    let kernel_base = kernel_sp - trap_frame_size;
    unsafe {
        core::arch::asm!(
            r"
            mv      sp, {frame_base}
            .short  0x2432                      // fld fs0,264(sp)
            .short  0x24d2                      // fld fs1,272(sp)
            mv      t1, {kernel_base}
            LDR     t0, sp, 2
            STR     gp, t1, 2
            mv      gp, t0
            LDR     t0, sp, 3
            STR     tp, t1, 3                   // save supervisor tp. Note that it is stored on the kernel stack rather than in sp, in which case the ID of the currently running CPU should be stored
            mv      tp, t0                      // tp: now it stores the TLS pointer to the corresponding thread
            csrw    sscratch, {kernel_sp}       // put supervisor sp to scratch
            LDR     t0, sp, 31
            LDR     t1, sp, 32
            csrw    sepc, t0
            csrw    sstatus, t1
            POP_GENERAL_REGS
            LDR     sp, sp, 1
            sret
        ",
            frame_base = in(reg) frame_base,
            kernel_sp = in(reg) kernel_sp,
            kernel_base = in(reg) kernel_base,
        );
    };
}
