//! This shows how to write text to uart0.
//! You can see the output with `espflash` if you provide the `--monitor` option

#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::{Event, Gpio9, Input, PullDown},
    interrupt,
    peripherals::{self, Peripherals, TIMG0},
    prelude::*,
    riscv,
    timer::{self, Timer0, TimerGroup},
    Rtc,
    Uart,
    IO,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    timer0.start(1u64.secs());

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut button = io.pins.gpio9.into_pull_down_input();
    button.listen_with_options(Event::FallingEdge, false, true, false);

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));

    interrupt::enable(
        peripherals::Interrupt::GPIO_NMI,
        interrupt::Priority::Priority3,
    )
    .unwrap();

    unsafe {
        riscv::interrupt::enable();
    }

    let mut mode = Mode::Interruptable;
    loop {
        writeln!(uart0, "Calling some_fn(.., {mode:?})").unwrap();
        some_fn(&mut timer0, 7, mode);
        writeln!(uart0, "Processing complete!").unwrap();
        mode = match mode {
            Mode::CriticalSection => Mode::Interruptable,
            Mode::Interruptable => Mode::CriticalSection,
        }
    }
}

static BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn GPIO_NMI(frame: &mut esp32c3_hal::trapframe::TrapFrame) {
    critical_section::with(|cs| {
        esp_println::println!("GPIO interrupt");
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

        // TODO: probably not on this end, but maybe on the monitor end?
        // ```
        // 0x42001c7a (frame #0)
        // 0x42001c7a - critical_section::release
        //     at /home/seth/.cargo/registry/src/github.com-1ecc6299db9ec823/critical-section-1.1.1/src/lib.rs:197
        // 0x42001c1e (frame #1)
        // 0x42001c1e - stack_walk::deepest_fn
        //     at /home/seth/Code/src/github.com/rustbox/esp-hal/esp32c3-hal/examples/stack_walk.rs:149
        // ```
        // this omits a (logical) frame (/line number?), because
        // critical_section::release is inline'd
        // i.e. would be nice to see, like:
        // ```
        //     at /home/seth/.cargo/registry/src/github.com-1ecc6299db9ec823/critical-section-1.1.1/src/lib.rs:197 (inlined)
        //     at /home/seth/Code/src/github.com/rustbox/esp-hal/esp32c3-hal/examples/stack_walk.rs:155
        // ```

        let mut fp = frame.s0; // AKA fp
        let mut index = 0;
        loop {
            let address = unsafe {
                let addr = (fp as *const usize).offset(-1).read_volatile(); // RA/PC
                fp = (fp as *const usize).offset(-2).read_volatile(); // next FP
                addr
            };

            // TODO: if address == main? or _start_rust? or if we've exceeded the top of the
            // stack?
            if address == 0 {
                break;
            }

            // see: esp_backtrace::is_valid_ram_address
            if (fp & 0xF) != 0 || !(0x3FC8_0000..=0x3FCE_0000).contains(&fp) {
                break;
            }

            println!("0x{address:x} (frame #{index})");
            index += 1;
        }
    });
}

type T = timer::Timer<Timer0<TIMG0>>;

#[derive(Debug, Clone, Copy)]
enum Mode {
    CriticalSection,
    Interruptable,
}

#[inline(never)]
fn some_fn(t: &mut T, n: u8, m: Mode) -> u8 {
    other_fn(t, n, m) + 1
}

#[inline(never)]
fn other_fn(t: &mut T, n: u8, m: Mode) -> u8 {
    deeper_fn(t, n, m) + 1
}

#[inline(never)]
fn deeper_fn(t: &mut T, n: u8, m: Mode) -> u8 {
    deepest_fn(t, n, m) + 1
}

#[inline(never)]
fn deepest_fn(t: &mut T, n: u8, m: Mode) -> u8 {
    use Mode::*;
    if n == 0 {
        let r = match m {
            CriticalSection => critical_section::with(|_| {
                block!(t.wait()).unwrap();
                2
            }),
            Interruptable => {
                block!(t.wait()).unwrap();
                1
            }
        };
        return r;
    }
    deepest_fn(t, n - 1, m) + 1
}
