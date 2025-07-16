use core::ptr::NonNull;
use futures::task::AtomicWaker;
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

/*
 * UART寄存器映射（参考芯片手册表5-63）：
 * 0x000  UARTDR      数据寄存器
 * 0x004  UARTRSR/ECR 接收状态/错误清除寄存器
 * 0x018  UARTFR      标志寄存器
 * 0x020  UARTILPR    低功耗计数寄存器
 * 0x024  UARTIBRD    波特率整数值配置寄存器
 * 0x028  UARTFBRD    波特率小数值配置寄存器
 * 0x02C  UARTLCR_H   线控寄存器
 * 0x030  UARTCR      控制寄存器
 * 0x034  UARTIFLS    FIFO阈值选择寄存器
 * 0x038  UARTIMSC    中断屏蔽选择/清除寄存器
 * 0x03C  UARTRIS     中断状态寄存器
 * 0x040  UARTMIS     中断屏蔽状态寄存器
 * 0x044  UARTICR     中断清除寄存器
 * 0x048  UARTDMACR   DMA控制寄存器
 */
register_structs! {
    Pl011UartRegs {
        (0x00 => dr: ReadWrite<u32, DR::Register>),
        (0x04 => rsr: ReadWrite<u32, RSR::Register>),
        (0x08 => _reserved0),
        (0x18 => fr: ReadOnly<u32, FR::Register>),
        (0x1c => _reserved1),
        (0x20 => ilpr: ReadWrite<u32, ILPR::Register>),
        (0x24 => ibrd: ReadWrite<u32, IBRD::Register>),
        (0x28 => fbrd: ReadWrite<u32, FBRD::Register>),
        (0x2c => lcr_h: ReadWrite<u32, LCRH::Register>),
        (0x30 => cr: ReadWrite<u32, CR::Register>),
        (0x34 => ifls: ReadWrite<u32, IFLS::Register>),
        (0x38 => imsc: ReadWrite<u32, IMSC::Register>),
        (0x3c => ris: ReadOnly<u32, RIS::Register>),
        (0x40 => mis: ReadOnly<u32, MIS::Register>),
        (0x44 => icr: WriteOnly<u32, ICR::Register>),
        (0x48 => dmacr: ReadWrite<u32, DMACR::Register>),
        (0x4c => @END),
    }
}

register_bitfields![u32,
    DR [
        DATA OFFSET(0) NUMBITS(8),   // 7:0  数据 [RW]
        FE   OFFSET(8) NUMBITS(1),   // 8    帧错误 [RW]
        PE   OFFSET(9) NUMBITS(1),   // 9    奇偶校验错误 [RW]
        BE   OFFSET(10) NUMBITS(1),  // 10   突发错误 [RW]
        OE   OFFSET(11) NUMBITS(1),  // 11   溢出错误 [RW]
        // 31:12 reserved
    ],
    RSR [
        FE OFFSET(0) NUMBITS(1),        // 0    帧错误 [RW]
        PE OFFSET(1) NUMBITS(1),        // 1    奇偶校验错误 [RW]
        BE OFFSET(2) NUMBITS(1),        // 2    突发错误 [RW]
        OE OFFSET(3) NUMBITS(1),        // 3    溢出错误 [RW]
        // 31:4 reserved
    ],
    FR [
        CTS  OFFSET(0)  NUMBITS(1),   // 0    清除发送 [RO]
        DSR  OFFSET(1)  NUMBITS(1),   // 1    数据准备完成 [RO]
        DCD  OFFSET(2)  NUMBITS(1),   // 2    载波被检测 [RO]
        BUSY OFFSET(3)  NUMBITS(1),   // 3    UART繁忙 [RO]
        RXFE OFFSET(4)  NUMBITS(1),   // 4    接收FIFO为空 [RO]
        TXFF OFFSET(5)  NUMBITS(1),   // 5    发送FIFO已满 [RO]
        RXFF OFFSET(6)  NUMBITS(1),   // 6    接收FIFO已满 [RO]
        TXFE OFFSET(7)  NUMBITS(1),   // 7    发送FIFO为空 [RO]
        RI   OFFSET(8)  NUMBITS(1),   // 8    Ring指示信号 [RO]
        // 31:9 reserved
    ],
    ILPR [
        ILPDVSR OFFSET(0) NUMBITS(8),   // 7:0  低功耗计数器 [RW]
        // 31:8 reserved
    ],
    IBRD [
        DIVINT OFFSET(0) NUMBITS(16),   // 15:0 波特率计算因子的整数值 [RW]
        // 31:16 reserved
    ],
    FBRD [
        DIVFRAC OFFSET(0) NUMBITS(6),   // 5:0 波特率计算因子的分数值 [RW]
        // 31:6 reserved
    ],
    LCRH [
        BRK  OFFSET(0) NUMBITS(1) [],   // 0    发送中断命令 [RW]
        PEN  OFFSET(1) NUMBITS(1) [],   // 1    奇偶校验使能 [RW]
        EPS  OFFSET(2) NUMBITS(1) [],   // 2    奇偶类型选择 [RW]
        STP2 OFFSET(3) NUMBITS(1) [],   // 3    两个停止位选择 [RW]
        FEN  OFFSET(4) NUMBITS(1) [],   // 4    FIFO使能 [RW]
        WLEN OFFSET(5) NUMBITS(2) [     // 6:5  数据长度 [RW]
            len5 = 0,                   // 00  5位
            len6 = 1,                   // 01  6位
            len7 = 2,                   // 10  7位
            len8 = 3                    // 11  8位
        ],
        SPS  OFFSET(7) NUMBITS(1) [],   // 7    高偶校验位 [RW]
        // 31:8 reserved
    ],
    CR [
        UARTEN  OFFSET(0)  NUMBITS(1),   // 0    UART使能 [RW]
        SIREN   OFFSET(1)  NUMBITS(1),   // 1    SIR使能位 [RW]
        SIRLP   OFFSET(2)  NUMBITS(1),   // 2    SIR低功耗IrDA模式 [RW]
        // 7:3 reserved
        TXE     OFFSET(8)  NUMBITS(1),   // 8    发送使能 [RW]
        RXE     OFFSET(9)  NUMBITS(1),   // 9    接收使能 [RW]
        DTR     OFFSET(10) NUMBITS(1),   // 10   数据终端准备 [RW]
        RTS     OFFSET(11) NUMBITS(1),   // 11   请求发送 [RW]
        Out1    OFFSET(12) NUMBITS(1),   // 12   Out1 [RW]
        Out2    OFFSET(13) NUMBITS(1),   // 13   Out2 [RW]
        RTSEN   OFFSET(14) NUMBITS(1),   // 14   RTS硬件流控 [RW]
        CTSEN   OFFSET(15) NUMBITS(1),   // 15   CTS硬件流控 [RW]
        // 31:16 reserved
    ],
    IFLS [
        TXIFLSEL OFFSET(0) NUMBITS(3) [ // 2:0  发送FIFO中断阈值选择 [RW]
            tx1_8 = 0,                  // b000 发送FIFO ≤ 1/8 full
            tx1_4 = 1,                  // b001 发送FIFO ≤ 1/4 full
            tx1_2 = 2,                  // b010 发送FIFO ≤ 1/2 full
            tx3_4 = 3,                  // b011 发送FIFO ≤ 3/4 full
            tx7_8 = 4                   // b100 发送FIFO ≤ 7/8 full
            // b101-b111 保留
        ],
        RXIFLSEL OFFSET(3) NUMBITS(3) [ // 5:3  接收FIFO中断阈值选择 [RW]
            rx1_8 = 0,                  // b000 接收FIFO ≥ 1/8 full
            rx1_4 = 1,                  // b001 接收FIFO ≥ 1/4 full
            rx1_2 = 2,                  // b010 接收FIFO ≥ 1/2 full
            rx3_4 = 3,                  // b011 接收FIFO ≥ 3/4 full
            rx7_8 = 4                   // b100 接收FIFO ≥ 7/8 full
            // b101-b111 保留
        ]
        // 31:6 reserved
    ],
    IMSC [
        RIMIM   OFFSET(0)  NUMBITS(1),   // 0    nUARTRI中断屏蔽 [RW]
        CTSMIM  OFFSET(1)  NUMBITS(1),   // 1    nUARTCTS中断屏蔽 [RW]
        DCDMIM  OFFSET(2)  NUMBITS(1),   // 2    nUARTDCD中断屏蔽 [RW]
        DSRMIM  OFFSET(3)  NUMBITS(1),   // 3    nUARTDSR中断屏蔽 [RW]
        RXIM    OFFSET(4)  NUMBITS(1),   // 4    接收中断屏蔽 [RW]
        TXIM    OFFSET(5)  NUMBITS(1),   // 5    发送中断屏蔽 [RW]
        RTIM    OFFSET(6)  NUMBITS(1),   // 6    超时中断屏蔽 [RW]
        FEIM    OFFSET(7)  NUMBITS(1),   // 7    帧错误中断屏蔽 [RW]
        PEIM    OFFSET(8)  NUMBITS(1),   // 8    奇偶校验错误中断屏蔽 [RW]
        BEIM    OFFSET(9)  NUMBITS(1),   // 9    突发错误中断屏蔽 [RW]
        OEIM    OFFSET(10) NUMBITS(1),   // 10   溢出错误中断屏蔽 [RW]
        // 31:11 reserved
    ],
    RIS [
        RIRMIS   OFFSET(0)  NUMBITS(1),   // 0    nUARTRI调制解调中断状态 [RO]
        CTSRMIS  OFFSET(1)  NUMBITS(1),   // 1    nUARTCTS调制解调中断状态 [RO]
        DCDRMIS  OFFSET(2)  NUMBITS(1),   // 2    nUARTDCD调制解调中断状态 [RO]
        DSRRMIS  OFFSET(3)  NUMBITS(1),   // 3    nUARTDSR调制解调中断状态 [RO]
        RXRIS    OFFSET(4)  NUMBITS(1),   // 4    接收中断状态 [RO]
        TXRIS    OFFSET(5)  NUMBITS(1),   // 5    发送中断状态 [RO]
        RTRIS    OFFSET(6)  NUMBITS(1),   // 6    超时中断状态 [RO]
        FERIS    OFFSET(7)  NUMBITS(1),   // 7    帧错误中断状态 [RO]
        PERIS    OFFSET(8)  NUMBITS(1),   // 8    奇偶校验错误中断状态 [RO]
        BERIS    OFFSET(9)  NUMBITS(1),   // 9    突发错误中断状态 [RO]
        OERIS    OFFSET(10) NUMBITS(1),   // 10   溢出错误中断状态 [RO]
        // 31:11 reserved
    ],
    MIS [
        RIMMIS   OFFSET(0)  NUMBITS(1),   // 0    nUARTRI调制解调屏蔽中断状态 [RO]
        CTSMMIS  OFFSET(1)  NUMBITS(1),   // 1    nUARTCTS调制解调屏蔽中断状态 [RO]
        DCDMMIS  OFFSET(2)  NUMBITS(1),   // 2    nUARTDCD调制解调屏蔽中断状态 [RO]
        DSRMMIS  OFFSET(3)  NUMBITS(1),   // 3    nUARTDSR调制解调屏蔽中断状态 [RO]
        RXMIS    OFFSET(4)  NUMBITS(1),   // 4    接收屏蔽中断状态 [RO]
        TXMIS    OFFSET(5)  NUMBITS(1),   // 5    发送屏蔽中断状态 [RO]
        RTMIS    OFFSET(6)  NUMBITS(1),   // 6    超时屏蔽中断状态 [RO]
        FEMIS    OFFSET(7)  NUMBITS(1),   // 7    帧错误屏蔽中断状态 [RO]
        PEMIS    OFFSET(8)  NUMBITS(1),   // 8    奇偶校验错误屏蔽中断状态 [RO]
        BEMIS    OFFSET(9)  NUMBITS(1),   // 9    突发错误屏蔽中断状态 [RO]
        OEMIS    OFFSET(10) NUMBITS(1),   // 10   溢出错误屏蔽中断状态 [RO]
        // 31:11 reserved
    ],
    ICR [
        RIMIC   OFFSET(0)  NUMBITS(1),   // 0    清除nUARTRI中断 [WO]
        CTSMIC  OFFSET(1)  NUMBITS(1),   // 1    清除nUARTCTS中断 [WO]
        DCDMIC  OFFSET(2)  NUMBITS(1),   // 2    清除nUARTDCD中断 [WO]
        DSRMIC  OFFSET(3)  NUMBITS(1),   // 3    清除nUARTDSR中断 [WO]
        RXIC    OFFSET(4)  NUMBITS(1),   // 4    清除接收中断 [WO]
        TXIC    OFFSET(5)  NUMBITS(1),   // 5    清除发送中断 [WO]
        RTIC    OFFSET(6)  NUMBITS(1),   // 6    清除超时中断 [WO]
        FEIC    OFFSET(7)  NUMBITS(1),   // 7    清除帧错误中断 [WO]
        PEIC    OFFSET(8)  NUMBITS(1),   // 8    清除奇偶校验错误中断 [WO]
        BEIC    OFFSET(9)  NUMBITS(1),   // 9    清除突发错误中断 [WO]
        OEIC    OFFSET(10) NUMBITS(1),   // 10   清除溢出错误中断 [WO]
        // 31:11 reserved
    ],
    DMACR [
        TXDMAE   OFFSET(1) NUMBITS(1),   // 1    发送FIFO的DMA启用 [RW]
        RXDMAE   OFFSET(0) NUMBITS(1),   // 0    接收FIFO的DMA启用 [RW]
        DMAONERR OFFSET(2) NUMBITS(1),   // 2    DMA错误 [RW]
        // 31:3 reserved
    ]
];

pub struct Pl011Uart {
    base: NonNull<Pl011UartRegs>,
    waker: AtomicWaker,
    tx_irq_cnt: usize,
    rx_irq_cnt: usize,
}

unsafe impl Send for Pl011Uart {}
unsafe impl Sync for Pl011Uart {}

impl Pl011Uart {
    pub const fn new(base: *mut u8) -> Self {
        Self {
            base: NonNull::new(base).unwrap().cast(),
            waker: AtomicWaker::new(),
            rx_irq_cnt: 0,
            tx_irq_cnt: 0,
        }
    }
    const fn regs(&self) -> &Pl011UartRegs {
        unsafe { self.base.as_ref() }
    }

    // 设置波特率
    pub fn set_baudrate(&mut self, baudrate: u32) {
        // 波特率计算公式：BaudRate = Fref / (16 * (DIVINT + DIVFRAC))
        // UART 控制器（非 MIO 配置） 主时钟为 100MHz
        let freq = 100_000_000;
        let divint = (freq / (16 * baudrate)) as u32;
        let divfrac =
            (((freq % (16 * baudrate)) * 64 + (16 * baudrate / 2)) / (16 * baudrate)) as u32;

        self.regs().ibrd.write(IBRD::DIVINT.val(divint));
        self.regs().fbrd.write(FBRD::DIVFRAC.val(divfrac));
    }

    pub fn init(&mut self) {
        // 先失能UART
        self.regs().cr.modify(CR::UARTEN::CLEAR);
        // 设置波特率为115200
        self.set_baudrate(115200);
        // 设置数据位为8位，停止位为1位，无奇偶校验，使能FIFO
        self.regs()
            .lcr_h
            .write(LCRH::WLEN::len8 + LCRH::STP2::CLEAR + LCRH::PEN::CLEAR + LCRH::FEN::SET);
        // 使能发送接收中断
        self.regs().imsc.write(IMSC::TXIM::SET + IMSC::RXIM::SET);
        // 设置发送FIFO中断阈值为3/4, 接收FIFO中断阈值为1/2
        self.regs()
            .ifls
            .write(IFLS::TXIFLSEL::tx3_4 + IFLS::RXIFLSEL::rx1_2);
        // 使能UART, 发送与接收
        self.regs()
            .cr
            .modify(CR::UARTEN::SET + CR::TXE::SET + CR::RXE::SET);
    }

    pub fn enable(&mut self) {
        // 使能UART
        self.regs().cr.modify(CR::UARTEN::SET);
    }

    pub fn disable(&mut self) {
        // 失能UART
        self.regs().cr.modify(CR::UARTEN::CLEAR);
    }

    pub fn read_byte_poll(&self) -> u8 {
        // 如果接收FIFO为空则返回0
        if self.regs().fr.is_set(FR::RXFE) {
            return 0x00;
        }
        // 读取数据寄存器
        (self.regs().dr.read(DR::DATA) & 0xFF) as u8
    }

    pub fn write_byte_poll(&self, byte: u8) {
        // 等待发送FIFO不满
        while self.regs().fr.is_set(FR::TXFF) {}
        // 写入数据寄存器
        self.regs().dr.write(DR::DATA.val(byte as u32));
    }

    pub fn write_bytes<'a>(&'a mut self, b: &'a [u8]) -> impl Future<Output = usize> + 'a {
        WriteFuture {
            uart: self,
            bytes: b,
            n: 0,
        }
    }

    pub fn handle_interrupt(&mut self) {
        // 如果发送FIFO不满, 继续向FIFO写入数据
        if !self.regs().fr.is_set(FR::TXFF) {
            self.waker.wake();
            self.tx_irq_cnt += 1;
        }

        // 如果接收FIFO满, 从FIFO读取数据
        if self.regs().fr.is_set(FR::RXFF) {
            self.rx_irq_cnt += 1;
        }

        // 清除中断
        self.regs().icr.write(ICR::RXIC::SET + ICR::TXIC::SET);
    }

    pub fn get_tx_irq_count(&self) -> usize {
        self.tx_irq_cnt
    }

    pub fn get_rx_irq_count(&self) -> usize {
        self.rx_irq_cnt
    }
}

pub struct WriteFuture<'a> {
    uart: &'a Pl011Uart,
    bytes: &'a [u8],
    n: usize,
}

impl<'a> Future for WriteFuture<'a> {
    type Output = usize;
    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let this = self.get_mut();
        loop {
            if this.n >= this.bytes.len() {
                return core::task::Poll::Ready(this.n);
            }
            if this.uart.regs().fr.is_set(FR::TXFF) {
                // not ready to send
                this.uart.waker.register(cx.waker());
                return core::task::Poll::Pending;
            }
            let b = this.bytes[this.n];
            this.uart.regs().dr.write(DR::DATA.val(b as u32));
            this.n += 1;
        }
    }
}
