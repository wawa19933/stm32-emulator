// SPDX-License-Identifier: GPL-3.0-or-later

use std::{convert::TryFrom, collections::VecDeque, rc::Rc, cell::RefCell};

use anyhow::Result;
use serde::Deserialize;

use crate::{system::System, util::{Rect, Point}, framebuffers::{Framebuffer, Framebuffers, RGB565}, peripherals::gpio::{Pin, GpioPorts}};
use super::{ExtDevice, display::{DisplayConfig, ReplyConfig}};


#[derive(Debug, Deserialize)]
pub struct DisplaySpiConfig {
    pub peripheral: String,
    pub swap_bytes: Option<bool>,
    pub replies: Option<Vec<ReplyConfig>>,
    pub framebuffer: String,
    pub cmd_pin: Option<String>,
}

pub struct DisplaySpi {
    pub config: DisplaySpiConfig,
    name: String,
    draw_region: Rect,
    cmd: Option<(Command, Vec<u8>)>,
    reply: VecDeque<u16>,
    previous_pix: Option<u8>,
    drawing: bool,
    current_position: Point,
    width: u16,
    height: u16,
    cmd_pin: bool,
    framebuffer: Rc<RefCell<dyn Framebuffer<RGB565>>>,
}

impl DisplaySpi {
    
    pub fn new(config: DisplaySpiConfig, gpio: &mut GpioPorts, framebuffers: &Framebuffers) -> Result<Self> {
        let framebuffer = framebuffers.get(&config.framebuffer)?;
        let width = framebuffer.borrow().get_config().width;
        let height = framebuffer.borrow().get_config().height;

        let config_cmd = config.cmd_pin.clone();

        // let self_ = Rc::new(RefCell::new(Self {
        //     name: "?".to_string(), // This is filled out on connect_peripheral()
        //     draw_region: Rect { left: 0, top: 0, right: width-1, bottom: height-1 },
        //     cmd: None,
        //     reply: Default::default(),
        //     drawing: false,
        //     current_position: Point::default(),
        //     width, height,
        //     framebuffer: framebuffer.clone(),
        //     config,
        //     previous_pix: None,
        //     cmd_pin: false,
        // }));

        // let s = self_.clone();

        // if let Some(ref cmd_pin_cfg) = config_cmd {
        //     let cmd_pin = Pin::from_str(cmd_pin_cfg);
        //     gpio.add_write_callback(cmd_pin, move |sys, v| {
        //         s.borrow_mut().write_cmd(sys, v);
        //      });
        // }


        let novo = Self {
            name: "?".to_string(), // This is filled out on connect_peripheral()
            draw_region: Rect { left: 0, top: 0, right: width-1, bottom: height-1 },
            cmd: None,
            reply: Default::default(),
            drawing: false,
            current_position: Point::default(),
            width, height,
            framebuffer: framebuffer.clone(),
            config,
            previous_pix: None,
            cmd_pin: false,
        };

        // let s = Rc::new(RefCell::new(novo));

        // let s = self_.clone();

        // if let Some(ref cmd_pin_cfg) = config_cmd {
        //     let cmd_pin = Pin::from_str(cmd_pin_cfg);
        //     gpio.add_write_callback(cmd_pin, move |sys, v| {
        //         s.borrow_mut().write_cmd(sys, v);
        //      });
        // }

        Ok(novo)
    }

    pub fn write_cmd(&mut self, _sys: &System, value: bool) {
        self.cmd_pin = value;
    }

    fn get_mode(&self) -> Mode {
        if self.cmd_pin {
            Mode::Data
        }else{
            Mode::Cmd
        }
    }

    #[inline]
    fn get_framebuffer_pixel_index(&mut self, x: u16, y: u16) -> usize {
        let x = x.min(self.width-1) as usize;
        let y = y.min(self.height-1) as usize;
        x + y * self.width as usize
    }

    fn draw_pixel(&mut self, c: u16) {
        let c = if self.config.swap_bytes.unwrap_or_default() {
            c.swap_bytes()
        } else {
            c
        };

        let Point { mut x, mut y } = self.current_position;
        let i = self.get_framebuffer_pixel_index(x, y);
        let mut coiso = self.framebuffer.borrow_mut();

        coiso.get_pixels()[i] = c;


        x += 1;
        if x > self.draw_region.right {
            x = self.draw_region.left;
            y += 1;

            if y > self.draw_region.bottom {
                y = self.draw_region.top;
                self.drawing = false;
            }
        }

        self.current_position = Point { x, y };
    }

    fn finish_cmd(&mut self) {
        self.drawing = false;
    }
}

impl ExtDevice<(), u8> for DisplaySpi {
    fn connect_peripheral(&mut self, peri_name: &str) -> String {
        self.name = format!("{} display", peri_name);
        self.name.clone()
    }

    fn read(&mut self, _sys: &System, addr: ()) -> u8 {
        // let mode = Mode::from_addr(self.config.cmd_addr_bit, addr);

        // let v = match mode {
        //     Mode::Cmd => {
        //         0
        //     }
        //     Mode::Data => {
        //         self.reply.pop_front().unwrap_or_default()
        //     }
        // };

        // trace!("{} READ {:?} -> {:02x}", self.name, mode, v);

        2
    }

    fn write(&mut self, _sys: &System, _addr: (), v: u8) {
        let mode = self.get_mode();

        trace!("{} WRITE {:?} value=0x{:04x}", self.name, mode, v);

        match mode {
            Mode::Cmd => {
                self.finish_cmd();
                if let Some(cmd) = Command::try_from(v).ok() {
                    // We are receiving a new command
                    if !self.try_process_command(cmd, &[]) {
                        self.cmd = Some((cmd, vec![]));
                    }
                } else if v != 0xff && v != 0x00 {
                    debug!("{} unknown cmd={:02x}", self.name, v);
                }
                // self.cmd = Some((value as u8, vec![]));
            }
            Mode::Data => {
                if self.drawing {
                    if let Some(pix) = self.previous_pix{
                        let pixel = ((pix as u16) << 8) | v as u16;
                        self.draw_pixel(pixel);
                        self.previous_pix = None;
                    }else{
                        self.previous_pix = Some(v);
                    }
                }else{
                    if let Some((cmd, mut args)) = self.cmd.take() {
                        // We are collecting a command argument
                        args.push(v);
                        if !self.try_process_command(cmd, &args) {
                            self.cmd = Some((cmd, args));
                        }
                    } 
                }
            }
        }
    }
}


#[derive(Debug, PartialEq, Eq)]
enum Mode {
    Cmd,
    Data,
}

#[derive(Clone, Copy, Debug, num_enum::TryFromPrimitive)]
#[repr(u8)]
enum Command {
    SetHoriRegion = 0x2A,
    SetVertRegion = 0x2B,
    Draw = 0x2C,
}

impl DisplaySpi {
    /// Return some reply when the command is processed.
    /// None when command arguments are incomplete.
    fn try_process_command(&mut self, cmd: Command, args: &[u8]) -> bool{
        match (cmd, args) {
            (Command::SetHoriRegion, [a,b,c,d] ) => {
                let left  = ((*a as u16) << 8) | *b as u16;
                let right = ((*c as u16) << 8) | *d as u16;

                self.draw_region.left = left;
                self.draw_region.right = right;
                debug!("{} cmd={:?} left={} right={}", self.name, cmd, left, right);
                true

            },
            (Command::SetVertRegion, [a,b,c,d]) => {
                let top  = ((*a as u16) << 8) | *b as u16;
                let bottom = ((*c as u16) << 8) | *d as u16;
                self.draw_region.top = top;
                self.draw_region.bottom = bottom;
                debug!("{} cmd={:?} top={} bottom={}", self.name, cmd, top, bottom);
                true

            },
            (Command::Draw, []) => {
                self.drawing = true;
                self.current_position = Point {
                    x: self.draw_region.left,
                    y: self.draw_region.top,
                };
                debug!("{} cmd={:?}", self.name, cmd);
                true
            },
            _ => {
                false

            },
        }

    }
}

// fn handle_cmd(&mut self) {
//     if let Some((cmd, args)) = self.cmd.take() {
//         match (Command::try_from(cmd).ok(), args.len()) {
//             (Some(cmd @ Command::SetHoriRegion), 4) => {
//                 let left  = (args[0] << 8) | args[1];
//                 let right = (args[2] << 8) | args[3];
//                 self.draw_region.left = left;
//                 self.draw_region.right = right;
//                 debug!("{} cmd={:?} left={} right={}", self.name, cmd, left, right);
//             }
//             (Some(cmd @ Command::SetVertRegion), 4) => {
//                 let top    = (args[0] << 8) | args[1];
//                 let bottom = (args[2] << 8) | args[3];
//                 self.draw_region.top = top;
//                 self.draw_region.bottom = bottom;
//                 debug!("{} cmd={:?} top={} bottom={}", self.name, cmd, top, bottom);
//             }
//             (Some(cmd @ Command::Draw), 0) => {
//                 self.drawing = true;
//                 self.current_position = Point {
//                     x: self.draw_region.left,
//                     y: self.draw_region.top,
//                 };
//                 debug!("{} cmd={:?}", self.name, cmd);
//             }
//             _ => {
//                 // If we need to reply to a read, put it there.
//                 if let Some(replies) = self.config.replies.as_ref() {
//                     if let Some(reply) = replies.iter().find(|r| r.cmd == cmd) {
//                         self.reply = reply.data.iter().cloned().collect();
//                         debug!("{} cmd={:02x?} reply={:02x?}", self.name, cmd, reply.data);
//                         return;
//                     }
//                 }

//                 // Not the right time to consume, put it back
//                 self.cmd = Some((cmd, args));
//             }
//         }
//     }
// }


impl Mode {
    fn from_addr(data_addr_bit: u32, offset: u32) -> Mode {
        if offset & data_addr_bit != 0 {
            Mode::Data
        } else {
            Mode::Cmd
        }
    }
}
