#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;
extern crate bare_test;

#[bare_test::tests]
mod tests {
    use log::info;

    use bare_test::{
        GetIrqConfig,
        globals::{PlatformInfoKind, global_val},
        irq::{IrqInfo, IrqParam},
        mem::iomap,
        println,
    };

    use pl011::{Pl011Uart, mutex::Mutex};

    static PL011_UART: Mutex<Option<Pl011Uart>> = Mutex::new(None);

    #[test]
    fn it_works() {
        info!("This is a test log message.");
        let a = 2;
        let b = 2;
        assert_eq!(a + b, 4);
        println!("test passed!");
    }

    #[test]
    fn test_uart() {
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let dbt = fdt.get();
        let node = dbt.find_compatible(&["arm,pl011"]).next().unwrap();
        let uart_regs = node.reg().unwrap().next().unwrap();
        let irq_info = node.irq_info().unwrap();
        println!("irq info {irq_info:?}");

        let cfg = irq_info.cfgs[0].clone();
        let base = uart_regs.address;

        let mut mmio = iomap((base as usize).into(), uart_regs.size.unwrap());

        let mut pl011_uart = Pl011Uart::new(unsafe { mmio.as_mut() });
        pl011_uart.init();

        {
            let mut uart = PL011_UART.lock();
            *uart = Some(pl011_uart);
        }

        // 注册中断处理
        IrqParam {
            intc: irq_info.irq_parent,
            cfg,
        }
        .register_builder(|_| {
            unsafe {
                PL011_UART.force_use().as_mut().unwrap().handle_interrupt();
            }
            bare_test::irq::IrqHandleResult::Handled
        })
        .register();

        spin_on::spin_on(async {
            let mut uart = PL011_UART.lock();
            let pl011_uart = uart.as_mut().unwrap();

            pl011_uart
                .write_bytes(b"hello,phytium async uart\r\n")
                .await;
            println!("irq count {}\r\n", pl011_uart.get_tx_irq_count());
        });
    }
}
