// SPDX-License-Identifier: GPL-3.0-or-later

mod spi_flash;
mod usart_probe;
mod display;
mod lcd;
mod touchscreen;
mod display_spi;

use spi_flash::{SpiFlashConfig, SpiFlash};
use usart_probe::{UsartProbeConfig, UsartProbe};
use display::{DisplayConfig, Display};
use lcd::{LcdConfig, Lcd};
use touchscreen::{TouchscreenConfig, Touchscreen};

use std::fmt::Pointer;
use std::{rc::Rc, cell::RefCell};
use serde::Deserialize;
use anyhow::Result;

use crate::config;
use crate::peripherals::gpio::Pin;
use crate::{system::System, framebuffers::Framebuffers, peripherals::gpio::GpioPorts};

use self::display_spi::DisplaySpi;
use self::display_spi::DisplaySpiConfig;

#[derive(Debug, Deserialize, Default)]
pub struct ExtDevicesConfig {
    pub spi_flash: Option<Vec<SpiFlashConfig>>,
    pub usart_probe: Option<Vec<UsartProbeConfig>>,
    pub display: Option<Vec<DisplayConfig>>,
    pub display_spi: Option<Vec<DisplaySpiConfig>>,
    pub lcd: Option<Vec<LcdConfig>>,
    pub touchscreen: Option<Vec<TouchscreenConfig>>,
}

pub struct ExtDevices {
    pub spi_flashes: Vec<Rc<RefCell<SpiFlash>>>,
    pub usart_probes: Vec<Rc<RefCell<UsartProbe>>>,
    pub displays: Vec<Rc<RefCell<Display>>>,
    pub displays_spi: Vec<Rc<RefCell<DisplaySpi>>>,
    pub lcds: Vec<Rc<RefCell<Lcd>>>,
    pub touchscreens: Vec<Rc<RefCell<Touchscreen>>>,
}

impl ExtDevices {
    pub fn find_serial_device(&self, peri_name: &str) -> Option<Rc<RefCell<dyn ExtDevice<(), u8>>>> {
        self.spi_flashes.iter()
            .filter(|d| d.borrow().config.peripheral == peri_name)
            .next()
            .map(|d| d.clone() as Rc<RefCell<dyn ExtDevice<(), u8>>>)
        .or_else(||
        self.usart_probes.iter()
            .filter(|d| d.borrow().config.peripheral == peri_name)
            .next()
            .map(|d| d.clone() as Rc<RefCell<dyn ExtDevice<(), u8>>>)
       )
        .or_else(||
        self.lcds.iter()
            .filter(|d| d.borrow().config.peripheral == peri_name)
            .next()
            .map(|d| d.clone() as Rc<RefCell<dyn ExtDevice<(), u8>>>)
       )
        .or_else(||
        self.touchscreens.iter()
            .filter(|d| d.borrow().config.peripheral == peri_name)
            .next()
            .map(|d| d.clone() as Rc<RefCell<dyn ExtDevice<(), u8>>>)
       )        
       .or_else(||
        self.displays_spi.iter()
            .filter(|d| d.borrow().config.peripheral == peri_name)
            .next()
            .map(|d| d.clone() as Rc<RefCell<dyn ExtDevice<(), u8>>>)
       )
    }

    pub fn find_mem_device(&self, peri_name: &str) -> Option<Rc<RefCell<dyn ExtDevice<u32, u32>>>> {
        self.displays.iter()
            .filter(|d| d.borrow().config.peripheral == peri_name)
            .next()
            .map(|d| d.clone() as Rc<RefCell<dyn ExtDevice<u32, u32>>>)
    }
}

impl ExtDevicesConfig {
    pub fn into_ext_devices(self, gpio: &mut GpioPorts, framebuffers: &Framebuffers) -> Result<ExtDevices> {
        let spi_flashes = self.spi_flash.unwrap_or_default().into_iter()
            .map(|config| SpiFlash::new(config).map(RefCell::new).map(Rc::new))
            .collect::<Result<_>>()?;

        let usart_probes = self.usart_probe.unwrap_or_default().into_iter()
            .map(|config| UsartProbe::new(config).map(RefCell::new).map(Rc::new))
            .collect::<Result<_>>()?;

        let displays = self.display.unwrap_or_default().into_iter()
            .map(|config| Display::new(config, framebuffers).map(RefCell::new).map(Rc::new))
            .collect::<Result<_>>()?;
        let mut cmd_ping : Option<String> = None;
        let displays_spi : Vec<Rc<RefCell<DisplaySpi>>> = self.display_spi.unwrap_or_default().into_iter()
            .map(|config| {
                cmd_ping = config.cmd_pin.clone();
                DisplaySpi::new(config, gpio, framebuffers).map(RefCell::new).map(Rc::new)
            })
            .collect::<Result<_>>()?;

        for v in &displays_spi{
            if let Some(vv) = cmd_ping.clone()  {
                let cmd_pin = Pin::from_str(&vv);
                let s = v.clone();
                gpio.add_write_callback(cmd_pin, move |sys, v| {
                    s.borrow_mut().write_cmd(sys, v);
                });
            }
        }

        let lcds = self.lcd.unwrap_or_default().into_iter()
            .map(|config| Lcd::new(config, framebuffers).map(RefCell::new).map(Rc::new))
            .collect::<Result<_>>()?;

        let touchscreens = self.touchscreen.unwrap_or_default().into_iter()
            .map(|config| Touchscreen::new(config, gpio, framebuffers).map(RefCell::new).map(Rc::new))
            .collect::<Result<_>>()?;

        Ok(ExtDevices { spi_flashes, usart_probes, displays, lcds, touchscreens, displays_spi })
    }
}

///////////////////////////////////////////////////////////////////////////////////////

pub trait ExtDevice<A, T> {
    /// Should returns "{peri_name} {ext_device_name}"
    fn connect_peripheral<'a>(&mut self, peri_name: &str) -> String;
    fn read(&mut self, sys: &System, addr: A) -> T;
    fn write(&mut self, sys: &System, addr: A, v: T);
}
