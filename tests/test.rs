#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;
extern crate bare_test;

#[bare_test::tests]
mod tests {
    use log::info;

    use bare_test::{
        globals::{PlatformInfoKind, global_val},
        mem::iomap,
        println,
    };

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
        let uart_regs = dbt
            .find_compatible(&["arm,pl011"])
            .next()
            .unwrap()
            .reg()
            .unwrap()
            .next()
            .unwrap();
        let base = uart_regs.address;
        
        // 映射到虚拟地址
        let mut mmio = iomap((base as usize).into(), uart_regs.size.unwrap());
        let mut pl011_uart = pl011::Pl011Uart::new(unsafe { mmio.as_mut() });

        pl011_uart.init();
        pl011_uart.write_byte_poll('A' as u8);
        
        println!("\r\n");
        
    }
}
