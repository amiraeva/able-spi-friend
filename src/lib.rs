#![no_std]

use feather_m0::{
    clock::{GenericClockController, Tc4Tc5Clock},
    gpio::{Floating, Input, OpenDrain, Output, Pa12, Pa21, Pa6 as D8, Pb10, Pb11, PfD, Port},
    pac::{PM, SERCOM4, TC5},
    prelude::*,
    sercom::{SPIMaster4, Sercom4Pad0, Sercom4Pad2, Sercom4Pad3},
    time::Microseconds,
    timer::TimerCounter5,
};

use arraydeque::{ArrayDeque, Wrapping};
use core::{cmp::min, iter};
use nb::block;

type Sck = Pb11<Input<Floating>>;
type Mosi = Pb10<Input<Floating>>;
type Miso = Pa12<Input<Floating>>;
type Irq = Pa21<Input<Floating>>;

type Spi = SPIMaster4<Sercom4Pad0<Pa12<PfD>>, Sercom4Pad2<Pb10<PfD>>, Sercom4Pad3<Pb11<PfD>>>;

pub struct BleUart {
    spi: Spi,
    cs: ChipSelect,
    tc5: TimerCounter5,
    irq: Irq,
    deque: ArrayDeque<[u8; 128], Wrapping>,
}

impl BleUart {
    const BUS_SPEED: u32 = 4_000_000;
    const SPI_CS_DELAY: Microseconds = Microseconds(50);
    const BLE_MAX_TIMEOUTS: usize = 10; // this * CS_DELAY = total timeout

    const SPI_IGNORED_BYTE: u8 = 0xFE;
    const SPI_OVERREAD_BYTE: u8 = 0xFF;
    const SPI_RESPONSE_BYTE: u8 = 0x20;
    const SPI_ERROR_BYTE: u8 = 0x80;

    const COMMAND_ID: u8 = 0x10;

    const SDEP_MAX_PACKET_SIZE: usize = 16;
    const SDEP_UARTRX_CMD: u16 = 0x0A02;

    pub fn new(
        sck: Sck,
        mosi: Mosi,
        miso: Miso,
        sercom4: SERCOM4,
        d8: D8<Input<Floating>>,
        irq: Irq,
        tc5: TC5,
        tc45: &Tc4Tc5Clock,
        clocks: &mut GenericClockController,
        pm: &mut PM,
        port: &mut Port,
    ) -> Self {
        let cs = ChipSelect::new(d8, port);
        let tc5 = TimerCounter5::tc5_(tc45, tc5, pm);

        let spi = feather_m0::spi_master(
            clocks,
            Self::BUS_SPEED.hz(),
            sercom4,
            pm,
            sck,
            mosi,
            miso,
            port,
        );

        let deque = ArrayDeque::new();

        let mut ble = Self {
            spi,
            cs,
            tc5,
            irq,
            deque,
        };

        ble.send_init_pattern();

        ble
    }

    pub fn write(&mut self, buf: &[u8]) {
        const SDEP_UARTTX_CMD: u16 = 0x0A01;

        let chunks = buf.chunks(Self::SDEP_MAX_PACKET_SIZE);

        let more_data_flags = iter::repeat(true)
            // more data as long as there are chunks - 1
            .take(chunks.len())
            // here is the -1 bit
            .skip(1)
            // functionally equivilant to setting the last flag to false
            .chain(iter::once(false));

        for (chunk, more_data) in chunks.zip(more_data_flags) {
            self.send_packet(SDEP_UARTTX_CMD, chunk, more_data);
        }
    }

    pub fn read<'a>(&mut self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        let deq_len = self.deque.len();

        if deq_len == 0 {
            self.pull_into_fifo();
            return None;
        }

        let len = min(buf.len(), deq_len);

        buf.iter_mut()
            .zip(self.deque.drain(..len))
            .for_each(|(b, deq)| *b = deq);

        Some(&buf[..len])
    }

    // poll periodically to pull RX data into fifo buffer
    pub fn pull_into_fifo(&mut self) {
        // need both together or else crash??
        self.send_packet(Self::SDEP_UARTRX_CMD, &[], false);
        self.get_responses();
    }

    fn send_init_pattern(&mut self) {
        const SDEP_INIT_CMD: u16 = 0xBEEF;
        self.send_packet(SDEP_INIT_CMD, &[], false);
    }

    fn send_packet(&mut self, command: u16, buf: &[u8], more_data: bool) {
        self.spi.enable();
        self.cs.enable();

        let rdy = iter::repeat_with(|| self.is_ready())
            .take(Self::BLE_MAX_TIMEOUTS)
            .any(|rdy| rdy == true);

        if !rdy {
            // panic!();
            return;
        };

        let [low, high] = command.to_le_bytes();
        spi_transfer(&mut self.spi, low as u8);
        spi_transfer(&mut self.spi, high as u8);

        let md_bit = (more_data as u8) << 7;
        let buf_len = min(buf.len(), Self::SDEP_MAX_PACKET_SIZE);

        let length = buf_len as u8 | md_bit;
        spi_transfer(&mut self.spi, length);

        let _ = self.spi.write(&buf[..buf_len]);

        self.cs.disable();
        self.spi.disable();
    }

    fn get_responses(&mut self) {
        while self.deque_remaining() > Self::SDEP_MAX_PACKET_SIZE {
            if let Some(more) = self.get_packet() {
                if !more {
                    return;
                }
                // self.send_packet(Self::SDEP_UARTRX_CMD, &[], false);
                self.tc5.start(Microseconds(50));
                let _ = block!(self.tc5.wait());
            }
        }
    }

    fn get_packet(&mut self) -> Option<bool> {
        const DUMMY_CMD: u8 = 0xFF;

        if !self.irq_ready() {
            return None;
        }

        self.spi.enable();
        self.cs.enable();

        let spi = &mut self.spi;
        let tc5 = &mut self.tc5;
        let cs = &mut self.cs;

        let mut spi_xfer = || spi_transfer(spi, DUMMY_CMD);

        let _ = iter::repeat_with(&mut spi_xfer)
            .take(Self::BLE_MAX_TIMEOUTS)
            .find_map(|msg| handle_initial_msgs(msg, cs, tc5))?;

        let cmd_id = u16::from_le_bytes([spi_xfer(), spi_xfer()]);

        if cmd_id != Self::SDEP_UARTRX_CMD {
            return None;
        };

        let len_tmp = spi_xfer();

        let more_data = (len_tmp >> 7) != 0;
        let payload_len = (len_tmp & 0x1F) as usize;

        if payload_len > Self::SDEP_MAX_PACKET_SIZE {
            return None;
        };

        let mut scratch = [0xFF; Self::SDEP_MAX_PACKET_SIZE];
        let buf = &mut scratch[..payload_len];

        let data = self.spi.transfer(buf).ok()?;

        self.deque.extend_back(data.iter().copied());

        self.cs.disable();
        self.spi.disable();

        Some(more_data)
    }

    fn irq_ready(&mut self) -> bool {
        const SPI_IRQ_DELAY: Microseconds = Microseconds(500);

        self.tc5.start(SPI_IRQ_DELAY);

        while self.irq.is_high().unwrap() {
            if self.tc5.wait().is_ok() {
                return false;
            }
        }

        true
    }

    fn is_ready(&mut self) -> bool {
        let byte = spi_transfer(&mut self.spi, Self::COMMAND_ID);

        if byte != Self::SPI_IGNORED_BYTE {
            return true;
        }

        cycle_cs(&mut self.cs, &mut self.tc5);

        false
    }

    fn deque_remaining(&self) -> usize {
        self.deque.capacity() - self.deque.len()
    }
}

struct ChipSelect {
    d8: D8<Output<OpenDrain>>,
}

impl ChipSelect {
    #[inline]
    fn new(d8: D8<Input<Floating>>, port: &mut Port) -> Self {
        let mut d8 = d8.into_open_drain_output(port);
        let _ = d8.set_high(); // de-assert CS by default
        Self { d8 }
    }

    #[inline]
    fn disable(&mut self) {
        let _ = self.d8.set_high();
    }

    #[inline]
    fn enable(&mut self) {
        let _ = self.d8.set_low();
    }
}

impl ufmt_write::uWrite for BleUart {
    type Error = core::convert::Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write(s.as_bytes());
        Ok(())
    }
}

fn spi_transfer(spi: &mut Spi, send: u8) -> u8 {
    block!(spi.send(send)).unwrap();
    let recv = block!(spi.read()).unwrap();
    recv
}

fn cycle_cs(cs: &mut ChipSelect, tc5: &mut TimerCounter5) {
    cs.disable();

    tc5.start(BleUart::SPI_CS_DELAY);
    let _ = block!(tc5.wait());

    cs.enable();
}

fn handle_initial_msgs(msg: u8, cs: &mut ChipSelect, tc5: &mut TimerCounter5) -> Option<u8> {
    match msg {
        BleUart::SPI_RESPONSE_BYTE => Some(msg),
        BleUart::SPI_IGNORED_BYTE | BleUart::SPI_OVERREAD_BYTE => {
            cycle_cs(cs, tc5);
            None
        },
        BleUart::SPI_ERROR_BYTE | _ => None
    }
}
