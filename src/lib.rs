#![no_std]

use arrayvec::ArrayVec;
use embedded_hal_async::i2c::I2c;

pub struct LonganLabsI2CCan<I: I2c> {
    interface: I,
    address: u8,
}

impl<I: I2c> LonganLabsI2CCan<I> {
    const DEFAULT_ADDRESS: u8 = 0x25;

    pub fn new(interface: I) -> Self {
        Self {
            interface,
            address: Self::DEFAULT_ADDRESS,
        }
    }

    pub fn new_with_address(interface: I, address: u8) -> Self {
        Self { interface, address }
    }

    /// Sets the i2c address
    pub async fn set_addres(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        self.interface.write(self.address, &[0x01, value]).await?;
        self.address = value;
        Ok(())
    }

    /// The I2C CAN Bus Module can store up to 16 CAN Frames.
    /// The number of CAN Frames currently stored in the device can be read with this function.
    /// If the number of CAN frames stored in the device exceeds 16,
    /// the new CAN frame will overwrite the ones that were not read in time.
    pub async fn available_frames(&mut self) -> Result<u8, Error<I::Error>> {
        let mut result = [0; 1];
        self.interface.write(self.address, &[0x02]).await?;
        self.interface.read(self.address, &mut result).await?;
        Ok(result[0])
    }

    /// Set the baud rate of the CAN bus
    pub async fn set_can_baud_rate(&mut self, value: BaudRate) -> Result<(), Error<I::Error>> {
        self.interface
            .write(self.address, &[0x03, value as u8])
            .await?;

        Ok(())
    }

    /// Send a CAN frame on the bus
    pub async fn send_frame(&mut self, frame: CanFrame) -> Result<(), Error<I::Error>> {
        let mut buffer = [0; 17];

        buffer[0] = 0x30; // Register address
        buffer[1..5].copy_from_slice(&frame.id.to_be_bytes());
        buffer[5] = frame.extended_id as u8;
        buffer[6] = frame.remote_transmission_request as u8;
        buffer[7] = frame.data.len() as u8;
        buffer[8..][..frame.data.len()].copy_from_slice(&frame.data);
        buffer[16] = Self::make_checksum(&buffer[1..16]);

        self.interface.write(self.address, &buffer).await?;

        Ok(())
    }

    /// Get a CAN frame if one is available
    pub async fn try_receive_frame(&mut self) -> Result<Option<CanFrame>, Error<I::Error>> {
        if self.available_frames().await? == 0 {
            return Ok(None);
        }

        let mut buffer = [0; 16];

        self.interface.write(self.address, &[0x40]).await?;
        self.interface.read(self.address, &mut buffer).await?;

        // Check the checksum
        let checksum = Self::make_checksum(&buffer[0..15]);

        if buffer[15] != checksum {
            return Err(Error::InvalidChecksum);
        }

        let id = u32::from_be_bytes(buffer[0..4].try_into().unwrap());
        let extended_id = buffer[4] > 0;
        let remote_transmission_request = buffer[5] > 0;
        let data_len = buffer[6] as usize;

        if data_len > 8 {
            return Err(Error::DataTooLarge);
        }

        let data = ArrayVec::try_from(&buffer[7..][..data_len]).unwrap();

        Ok(Some(CanFrame {
            id,
            extended_id,
            remote_transmission_request,
            data,
        }))
    }

    /// Keep polling until a CAN frame is received
    pub async fn receive_frame(&mut self) -> Result<CanFrame, Error<I::Error>> {
        loop {
            if let Some(frame) = self.try_receive_frame().await? {
                return Ok(frame);
            }
        }
    }

    // TODO: Create functions for the filters and masks

    fn make_checksum(data: &[u8]) -> u8 {
        let mut sum: u32 = data.iter().map(|byte| *byte as u32).sum();

        if sum > 0xFF {
            sum = !sum;
            sum += 1;
        }

        (sum & 0xFF) as u8
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub extended_id: bool,
    pub remote_transmission_request: bool,
    pub data: ArrayVec<u8, 8>,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum BaudRate {
    /// 5 kbps
    K5 = 1,
    /// 10 kbps
    K10 = 2,
    /// 20 kbps
    K20 = 3,
    /// 25 kbps
    K25 = 4,
    /// 31.2 kbps
    K31_2 = 5,
    /// 33 kbps
    K33 = 6,
    /// 40 kbps
    K40 = 7,
    /// 50 kbps
    K50 = 8,
    /// 80 kbps
    K80 = 9,
    /// 83.3 kbps
    K83_3 = 10,
    /// 95 kbps
    K95 = 11,
    /// 100 kbps
    K100 = 12,
    /// 125 kbps
    K125 = 13,
    /// 200 kbps
    K200 = 14,
    /// 250 kbps
    K250 = 15,
    /// 500 kbps
    K500 = 16,
    /// 666 kbps
    K666 = 17,
    /// 1000 kbps (1 mbps)
    K1000 = 18,
}

#[derive(Debug)]
pub enum Error<IE: embedded_hal_async::i2c::Error> {
    InterfaceError(IE),
    InvalidChecksum,
    DataTooLarge
}

impl<IE: embedded_hal_async::i2c::Error> From<IE> for Error<IE> {
    fn from(e: IE) -> Self {
        Self::InterfaceError(e)
    }
}
