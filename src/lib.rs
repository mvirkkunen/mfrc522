//! A platform agnostic driver to interface the MFRC522 (RFID reader/writer)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! You'll find an example for the Raspeberry Pi in the `examples` directory. You should find an
//! example for ARM Cortex-M microcontrollers on the [`blue-pill`] repository. If that branch is
//! gone, check the master branch.
//!
//! [`blue-pill`]: https://github.com/japaric/blue-pill/tree/singletons/examples
//!
//! # References
//!
//! - [Identification cards - Contactless integrated circuit(s) cards - Proximity cards - Part 3:
//! Initialization and anticollision][1]
//! - [MFRC522 data sheet][2]
//!
//! [1]: http://wg8.de/wg8n1496_17n3613_Ballot_FCD14443-3.pdf
//! [2]: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf

#![allow(dead_code)]
#![deny(missing_docs)]
//#![deny(warnings)]
#![no_std]

extern crate embedded_hal as hal;
extern crate generic_array;
#[macro_use]
extern crate nb;

use core::convert::TryInto;
use core::mem;

use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

#[allow(deprecated)]
use hal::digital::OutputPin;
use hal::blocking::spi;
use hal::serial;

pub mod interface;
use interface::*;

mod picc;

/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Wrong Block Character Check (BCC)
    Bcc,
    /// FIFO buffer overflow
    BufferOverflow,
    /// Collision
    Collision,
    /// Wrong CRC
    Crc,
    /// Incomplete RX frame
    IncompleteFrame,
    /// Internal temperature sensor detects overheating
    Overheating,
    /// Parity check failed
    Parity,
    /// Error during MFAuthent operation
    Protocol,
    /// Interface error
    Interface(E),
    /// Timeout
    Timeout,
    /// ???
    Wr,
    /// The requested PICC was not found
    NotFound,
    /// The UID returned by the PICC was in an invalid format
    UidFormat,
}

/// MFRC522 driver
pub struct Mfrc522<IF> {
    iface: IF,
}

const ERR_IRQ: u8 = 1 << 1;
const IDLE_IRQ: u8 = 1 << 4;
const RX_IRQ: u8 = 1 << 5;
const TIMER_IRQ: u8 = 1 << 0;

const CRC_IRQ: u8 = 1 << 2;

impl<SPI, NSS> Mfrc522<SpiInterface<SPI, NSS>> {
    /// Creates a new driver using the provided SPI peripheral and NSS pin.
    #[allow(deprecated)]
    pub fn new_spi<E>(spi: SPI, nss: NSS)
        -> Result<Mfrc522<SpiInterface<SPI, NSS>>, E>
    where
        SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
        NSS: OutputPin
    {
        Mfrc522::new(SpiInterface::new(spi, nss))
    }
}

impl<W, R> Mfrc522<SerialInterface<W, R>> {
    /// Creates a new driver using the provided serial read/write halves.
    pub fn new_serial<E>(write: W, read: R)
        -> Result<Mfrc522<SerialInterface<W, R>>, E>
    where
        W: serial::Write<u8, Error = E>,
        R: serial::Read<u8, Error = E>,
    {
        Mfrc522::new(SerialInterface::new(write, read))
    }
}

impl<E, IF> Mfrc522<IF>
where
    IF: Mfrc522Interface<Error = E>
{
    /// Creates a new driver with the provided interface
    pub fn new(iface: IF) -> Result<Self, E> {
        let mut mfrc522 = Mfrc522 { iface };

        // soft reset
        mfrc522.command(Command::SoftReset)?;

        while mfrc522.iface.read(Register::Command)? & (1 << 4) != 0 {}

        // configure timer to operate at 10 KHz.
        // f_timer = 13.56 MHz / (2 + TPrescaler + 2)
        mfrc522.iface.write(Register::Demod, 0x4d | (1 << 4))?;
        mfrc522.iface.write(Register::TMode, 0x0 | (1 << 7) | 0b10)?;
        mfrc522.iface.write(Register::TPrescaler, 165)?;

        // configure timer for a 5 ms timeout
        mfrc522.iface.write(Register::ReloadL, 50)?;

        // forces 100% ASK modulation
        // NOTE my tags don't work without this ...
        mfrc522.iface.write(Register::TxAsk, 1 << 6)?;

        // set preset value for the CRC co-processor to 0x6363
        // in accordance to section 6.2.4 of ISO/IEC FCD 14443-3
        mfrc522.iface.write(Register::Mode, (0x3f & (!0b11)) | 0b01)?;

        // enable the antenna
        mfrc522.iface.write(Register::TxControl, 0x80 | 0b11)?;

        Ok(mfrc522)
    }

    /// Sends a REQuest type A to nearby PICCs
    pub fn reqa<'b>(&mut self) -> Result<AtqA, Error<E>> {
        self.transceive_bit_oriented(&[picc::REQA], 7, 0)
            .map(|bytes| AtqA { bytes })
    }

    /// Selects an idle PICC
    // TODO anticollision loop
    pub fn select(&mut self, atqa: &AtqA) -> Result<PiccInfo, Error<E>> {
        let _ = atqa;

        let mut bytes = [0u8; 10];
        let mut index = 0;

        for &cl in &[picc::SEL_CL1, picc::SEL_CL2, picc::SEL_CL3] {
            let rx = self.transceive_bit_oriented::<U5>(&[cl, 0x20], 0, 0)?;

            let uid_part: [u8; 4] = rx[..4].try_into().unwrap();

            let rx_bcc = rx[4];
            if calculate_bcc(&uid_part) != rx_bcc {
                return Err(Error::Bcc);
            }

            let sak = self.select_cl(cl, uid_part)?;

            if sak.complete {
                bytes[index..index+4].copy_from_slice(&uid_part);
                index += 4;

                return Ok(PiccInfo {
                    uid: Uid::try_new(&bytes[..index]).unwrap(),
                    compliant: sak.compliant,
                })
            } else {
                bytes[index..index+3].copy_from_slice(&uid_part[1..]);
                index += 3;
            }
        }

        Err(Error::UidFormat)
    }

    /// Select an idle PICC with the provided UID
    pub fn select_uid(&mut self, atqa: &AtqA, uid: Uid) -> Result<PiccInfo, Error<E>> {
        let _ = atqa;

        let bytes = uid.bytes();
        let mut index = 0;

        for &cl in &[picc::SEL_CL1, picc::SEL_CL2, picc::SEL_CL3] {
            let mut uid_part: [u8; 4] = unsafe { mem::uninitialized() };
            if bytes.len() - index == 4 {
                uid_part.copy_from_slice(&bytes[index..]);
                index += 4;
            } else {
                uid_part[0] = picc::CT;
                uid_part[1..].copy_from_slice(&bytes[index..index+3]);
                index += 3;
            }

            let sak = self.select_cl(cl, uid_part)?;

            if index == bytes.len() {
                if !sak.complete {
                    return Err(Error::NotFound);
                }

                return Ok(PiccInfo {
                    uid,
                    compliant: sak.compliant,
                });
            }
        }

        Err(Error::NotFound)
    }

    fn select_cl(&mut self, cl: u8, uid_part: [u8; 4]) -> Result<Sak, Error<E>> {
        let mut tx: [u8; 9] = unsafe { mem::uninitialized() };
        tx[0] = cl;
        tx[1] = 0x70;
        tx[2..6].copy_from_slice(&uid_part);
        tx[6] = calculate_bcc(&uid_part);

        let rx = self.transceive::<U3>(&mut tx)?;

        Ok(Sak::new(rx[0]))
    }

    /// Returns the version of the MFRC522
    pub fn version(&mut self) -> Result<u8, E> {
        self.iface.read(Register::Version)
    }

    fn calculate_crc(&mut self, data: &[u8]) -> Result<[u8; 2], Error<E>> {
        // stop any ongoing command
        self.command(Command::Idle).map_err(Error::Interface)?;

        // clear the CRC_IRQ interrupt flag
        self.iface.write(Register::DivIrq, 1 << 2).map_err(Error::Interface)?;

        // flush FIFO buffer
        self.flush_fifo_buffer().map_err(Error::Interface)?;

        // write data to transmit to the FIFO buffer
        self.iface.write_many(Register::FifoData, data)
            .map_err(Error::Interface)?;

        self.command(Command::CalcCRC).map_err(Error::Interface)?;

        // TODO timeout when connection to the MFRC522 is lost
        // wait for CRC to complete
        let mut irq;
        loop {
            irq = self.iface.read(Register::DivIrq).map_err(Error::Interface)?;

            if irq & CRC_IRQ != 0 {
                self.command(Command::Idle).map_err(Error::Interface)?;
                let crc = [
                    self.iface.read(Register::CrcResultL).map_err(Error::Interface)?,
                    self.iface.read(Register::CrcResultH).map_err(Error::Interface)?,
                ];

                break Ok(crc);
            }
        }
    }

    fn check_error_register(&mut self) -> Result<(), Error<E>> {
        const PROTOCOL_ERR: u8 = 1 << 0;
        const PARITY_ERR: u8 = 1 << 1;
        const CRC_ERR: u8 = 1 << 2;
        const COLL_ERR: u8 = 1 << 3;
        const BUFFER_OVFL: u8 = 1 << 4;
        const TEMP_ERR: u8 = 1 << 6;
        const WR_ERR: u8 = 1 << 7;

        let err = self.iface.read(Register::Error).map_err(Error::Interface)?;

        if err & PROTOCOL_ERR != 0 {
            Err(Error::Protocol)
        } else if err & PARITY_ERR != 0 {
            Err(Error::Parity)
        } else if err & CRC_ERR != 0 {
            Err(Error::Crc)
        } else if err & COLL_ERR != 0 {
            Err(Error::Collision)
        } else if err & BUFFER_OVFL != 0 {
            Err(Error::BufferOverflow)
        } else if err & TEMP_ERR != 0 {
            Err(Error::Overheating)
        } else if err & WR_ERR != 0 {
            Err(Error::Wr)
        } else {
            Ok(())
        }
    }

    fn command(&mut self, command: Command) -> Result<(), E> {
        self.iface.write(Register::Command, command.value())
    }

    fn flush_fifo_buffer(&mut self) -> Result<(), E> {
        self.iface.write(Register::FifoLevel, 1 << 7)
    }

    /// Sends a request to the PICC and returns the response. This method automatically calculates
    /// and adds the CRC to the last two bytes of tx_buffer, so you must reserve two bytes of space
    /// for it. The respone CRC is automatically verified.
    ///
    /// # Arguments
    ///
    /// tx_buffer:    Bytes to send to PICC. Must contain two additional bytes of space at the end
    ///               for the CRC.
    fn transceive<RX>(
        &mut self,
        tx_buffer: &mut [u8],
    ) -> Result<GenericArray<u8, RX>, Error<E>>
    where
        RX: ArrayLength<u8>,
    {
        let tx_crc_index = tx_buffer.len() - 2;

        let tx_crc = self.calculate_crc(&tx_buffer[..tx_crc_index])?;
        tx_buffer[tx_crc_index..].copy_from_slice(&tx_crc);

        let rx = self.transceive_bit_oriented::<RX>(tx_buffer, 0, 0)?;

        if rx.len() < 3 {
            return Err(Error::IncompleteFrame);
        }

        let rx_crc_index = rx.len() - 2;

        let computed_rx_crc = self.calculate_crc(&rx[..rx_crc_index])?;
        if &rx[rx_crc_index..] != &computed_rx_crc {
            return Err(Error::Crc);
        }

        Ok(rx)
    }

    /// Sends a request to the PICC and returns the response. Allows bit-oriented frames.
    ///
    /// # Arguments
    ///
    /// tx_buffer:    bytes to send to PICC
    /// tx_last_bits: number of valid bits in the last byte of tx_buffer for bit-oriented frames
    ///               0 = not bit-oriented frame
    /// rx_align:     bit position to place first bit of received bit-oriented frame
    ///               0 = not bit-oriented frame
    fn transceive_bit_oriented<RX>(
        &mut self,
        tx_buffer: &[u8],
        tx_last_bits: u8,
        rx_align: u8,
    ) -> Result<GenericArray<u8, RX>, Error<E>>
    where
        RX: ArrayLength<u8>,
    {
        // stop any ongoing command
        self.command(Command::Idle).map_err(Error::Interface)?;

        // clear all interrupt flags
        self.iface.write(Register::ComIrq, 0x7f).map_err(Error::Interface)?;

        // flush FIFO buffer
        self.flush_fifo_buffer().map_err(Error::Interface)?;

        // write data to transmit to the FIFO buffer
        self.iface.write_many(Register::FifoData, tx_buffer)
            .map_err(Error::Interface)?;

        // signal command
        self.command(Command::Transceive).map_err(Error::Interface)?;

        let bit_framing = (1 << 7) | ((rx_align & 0x7) << 4) | (tx_last_bits & 0x7);

        // configure short frame and start transmission
        self.iface.write(Register::BitFraming, bit_framing).map_err(Error::Interface)?;

        // TODO timeout when connection to the MFRC522 is lost (?)
        // wait for transmission + reception to complete
        let mut irq;
        loop {
            irq = self.iface.read(Register::ComIrq).map_err(Error::Interface)?;

            if irq & (RX_IRQ | ERR_IRQ | IDLE_IRQ) != 0 {
                break;
            } else if irq & TIMER_IRQ != 0 {
                return Err(Error::Timeout);
            }
        }

        // check for any outstanding error
        if irq & ERR_IRQ != 0 {
            self.check_error_register()?;
        }

        // grab RX data
        let mut rx_buffer: GenericArray<u8, RX> = unsafe { mem::uninitialized() };

        {
            let rx_buffer: &mut [u8] = &mut rx_buffer;

            let received_bytes = self.iface.read(Register::FifoLevel).map_err(Error::Interface)?;

            if received_bytes as usize != rx_buffer.len() {
                return Err(Error::IncompleteFrame);
            }

            self.iface.read_many(Register::FifoData, rx_buffer)
                .map_err(Error::Interface)?;
        }

        Ok(rx_buffer)
    }
}

#[derive(Clone, Copy)]
enum Command {
    Idle,
    Mem,
    GenerateRandomID,
    CalcCRC,
    Transmit,
    NoCmdChange,
    Receive,
    Transceive,
    MFAuthent,
    SoftReset,
}

impl Command {
    fn value(&self) -> u8 {
        match *self {
            Command::Idle => 0b0000,
            Command::Mem => 0b0001,
            Command::GenerateRandomID => 0b0010,
            Command::CalcCRC => 0b0011,
            Command::Transmit => 0b0100,
            Command::NoCmdChange => 0b0111,
            Command::Receive => 0b1000,
            Command::Transceive => 0b1100,
            Command::MFAuthent => 0b1110,
            Command::SoftReset => 0b1111,
        }
    }
}

#[derive(Clone, Copy)]
#[allow(missing_docs)]
pub enum Register {
    BitFraming = 0x0d,
    Coll = 0x0e,
    ComIrq = 0x04,
    Command = 0x01,
    CrcResultH = 0x21,
    CrcResultL = 0x22,
    Demod = 0x19,
    DivIrq = 0x05,
    Error = 0x06,
    FifoData = 0x09,
    FifoLevel = 0x0a,
    ModWidth = 0x24,
    Mode = 0x11,
    ReloadH = 0x2c,
    ReloadL = 0x2d,
    RxMode = 0x13,
    TCountValH = 0x2e,
    TCountValL = 0x2f,
    TMode = 0x2a,
    TPrescaler = 0x2b,
    TxAsk = 0x15,
    TxControl = 0x14,
    TxMode = 0x12,
    Version = 0x37,
}

/// Answer To reQuest A
pub struct AtqA {
    bytes: GenericArray<u8, U2>,
}

/// Information about a scanned PICC
#[derive(Debug)]
pub struct PiccInfo {
    uid: Uid,
    compliant: bool,
}

impl PiccInfo {
    fn new(uid: Uid, compliant: bool) -> Self {
        Self { uid, compliant }
    }

    /// Gets the tag UID
    pub fn uid(&self) -> &Uid {
        &self.uid
    }

    /// Is the PICC compliant with ISO/IEC 14443-4?
    pub fn is_compliant(&self) -> bool {
        self.compliant
    }
}

/// PICC UID
#[derive(Copy, Clone)]
pub struct Uid {
    bytes: [u8; 10],
    len: u8,
}

impl Uid {
    /// Creates a new Uid with the provided bytes. The length of bytes must be 4, 7 or 10.
    pub fn try_new(bytes: &[u8]) -> Result<Uid, ()> {
        match bytes.len() {
            4 | 7 | 10 => {
                Ok(Self {
                    bytes: {
                        let mut b: [u8; 10] = unsafe { mem::uninitialized() };
                        b[..bytes.len()].copy_from_slice(bytes);
                        b
                    },
                    len: bytes.len() as u8,
                })
            },
            _ => Err(()),
        }
    }

    /// The bytes of the UID
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
}

impl core::fmt::Debug for Uid {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> Result<(), core::fmt::Error> {
        write!(f, "{:02x?}", self.bytes())
    }
}

struct Sak {
    pub complete: bool,
    pub compliant: bool,
}

impl Sak {
    fn new(sak: u8) -> Self {
        Self {
            complete: sak & (1 << 2) == 0,
            compliant: sak & (1 << 5) == 0,
        }
    }
}

fn calculate_bcc(bytes: &[u8; 4]) -> u8 {
    return bytes[0] ^ bytes[1] ^ bytes[2] ^ bytes[3];
}
