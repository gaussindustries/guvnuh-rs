#![no_std]

pub mod ll;
// Does not work at the moment
// use heapless::Vec;

// use stm32h7xx_hal::{
//     dma::{
//         dma::{DmaConfig, StreamsTuple, Stream0 as DmaStream0, Stream1 as DmaStream1},
//         ConstDBTransfer, DBTransfer, MemoryToPeripheral, PeripheralToMemory, Transfer,
//     },
//     gpio::Pin,
//     pac::{self, DMA1, SPI1},
//     prelude::*,
//     rcc::rec,
//     spi::{self, Disabled, Enabled, Spi},
// };

// const MAX_DMA_LEN: usize = 256; // adjust to your frame size

// pub struct Ads131m04 {
//     // Keep SPI disabled so it implements TargetAddress for DMA
//     spi: Option<Spi<SPI1, Disabled, u8>>,
//     drdy: Pin<'B', 15>,
//     rx_stream: Option<DmaStream0<DMA1>>,
//     tx_stream: Option<DmaStream1<DMA1>>,
// }

// impl Ads131m04 {
//     pub fn new<P: stm32h7xx_hal::spi::Pins<SPI1>>(
//         spi1: SPI1,
//         pins: P,
//         drdy: Pin<'B', 15>,
//         clocks: &stm32h7xx_hal::rcc::CoreClocks,
//         spi1_rec: rec::Spi1,
//         dma1: DMA1,
//         dma1_rec: rec::Dma1,
//     ) -> Self {
//         // Build SPI enabled, then disable to get Spi<Disabled> (required by DMA TargetAddress)
//         let spi_en: Spi<SPI1, Enabled, u8> = spi1.spi(
//             pins,
//             spi::Mode {
//                 phase: spi::Phase::CaptureOnFirstTransition,
//                 polarity: spi::Polarity::IdleLow,
//             },
//             1.MHz(),
//             spi1_rec,
//             clocks,
//         );
//         let spi_dis: Spi<SPI1, Disabled, u8> = spi_en.disable();

//         // Correct way to destructure a tuple-struct:
//         let streams = StreamsTuple::new(dma1, dma1_rec);
//         let StreamsTuple(rx_s, tx_s, _s2, _s3, _s4, _s5, _s6, _s7) = streams;

//         Self {
//             spi: Some(spi_dis),
//             drdy,
//             rx_stream: Some(rx_s),
//             tx_stream: Some(tx_s),
//         }
//     }

//     /// Blocking full-duplex-like burst using two DMA transfers (RX then TX).
//     /// Uses owned heapless buffers to satisfy DMA's StableDeref + lifetime.
//     pub fn xfer_dma_blocking(
// 		&mut self,
// 		rx: &'static mut [u8], // WriteBuffer
// 		tx: &'static [u8],     // ReadBuffer
// 	) -> Result<(), ()> {
//         assert_eq!(rx.len(), tx.len());
//         assert!(rx.len() <= MAX_DMA_LEN);
//         assert!(tx.len() <= MAX_DMA_LEN);

//         // --- Build owned DMA buffers ---
//         let mut tx_buf: Vec<u8, MAX_DMA_LEN> = Vec::new();
//         tx_buf.extend_from_slice(tx).map_err(|_| ())?;

//         let mut rx_buf: Vec<u8, MAX_DMA_LEN> = Vec::new();
//         rx_buf.resize(rx.len(), 0).map_err(|_| ())?;

//         // --- Basic DMA configs ---
//         let cfg_rx = DmaConfig::default()
//             .memory_increment(true)
//             .peripheral_increment(false)
//             .transfer_complete_interrupt(true);

//         let cfg_tx = DmaConfig::default()
//             .memory_increment(true)
//             .peripheral_increment(false)
//             .transfer_complete_interrupt(true);

//         // Pull SPI/streams out of self (move into transfers)
//         let rx_stream = self.rx_stream.take().ok_or(())?;
//         let tx_stream = self.tx_stream.take().ok_or(())?;
//         let spi = self.spi.take().ok_or(())?;

//         // ---------- RX: Peripheral -> Memory ----------
//         let mut rx_xfer: Transfer<
//             DmaStream0<DMA1>,
//             Spi<SPI1, Disabled, u8>,
//             PeripheralToMemory,
//             Vec<u8, MAX_DMA_LEN>,
//             DBTransfer,
//         > = Transfer::init(rx_stream, spi, rx_buf, None, cfg_rx);

//         rx_xfer.start(|_spi| {
//             // If you need to toggle CR2 DMA bits directly, do it here via _spi.regs()
//             // The HAL handles most setup when the transfer starts.
//         });

//         // Get SPI back before TX
//         let (rx_stream, spi_back, rx_buf_done, _dbuf) = rx_xfer.free();

//         // ---------- TX: Memory -> Peripheral ----------
//         let mut tx_xfer: Transfer<
//             DmaStream1<DMA1>,
//             Spi<SPI1, Disabled, u8>,
//             MemoryToPeripheral,
//             Vec<u8, MAX_DMA_LEN>,
//             ConstDBTransfer,
//         > = Transfer::init_const(tx_stream, spi_back, tx_buf, None, cfg_tx);

//         tx_xfer.start(|_spi| {});

//         // Poll to completion (or use interrupts)
//         while !tx_xfer.get_transfer_complete_flag() {}
//         tx_xfer.clear_transfer_complete_interrupt();

//         let (tx_stream, spi_back2, _tx_buf_done, _dbuf2) = tx_xfer.free();

//         // Copy bytes from owned RX buffer back to caller’s slice
//         rx[..rx_buf_done.len()].copy_from_slice(rx_buf_done.as_slice());

//         // Put resources back
//         self.rx_stream = Some(rx_stream);
//         self.tx_stream = Some(tx_stream);
//         self.spi = Some(spi_back2);

//         Ok(())
//     }

//     pub fn read_samples(&mut self, rx: &mut [u8], tx: &[u8]) -> Result<(), ()> {
//         self.xfer_dma_blocking(rx, tx)
//     }
// }
