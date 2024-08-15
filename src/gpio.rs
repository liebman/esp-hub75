use core::convert::Infallible;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Dimensions;
use embedded_graphics::geometry::Point;
use embedded_graphics::geometry::Size;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Pixel;
use embedded_hal::digital::OutputPin;

use crate::Color;

/// Frame buffer for a Hub75 display
pub struct FrameBuffer<const COL: usize, const ROW: usize> {
    data: [[Color; COL]; ROW],
}

impl<const COL: usize, const ROW: usize> Default for FrameBuffer<COL, ROW> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const COL: usize, const ROW: usize> FrameBuffer<COL, ROW> {
    /// Create a new frame buffer
    pub const fn new() -> Self {
        Self {
            data: [[Color::BLACK; COL]; ROW],
        }
    }

    /// Clear the frame buffer to black.
    pub fn clear(&mut self) {
        for row in self.data.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = Color::BLACK;
            }
        }
    }

    /// Set a pixel in the frame buffer
    pub fn set_pixel(&mut self, x: usize, y: usize, color: Color) {
        self.data[y][x] = color;
    }

    /// Get a pixel from the frame buffer
    pub fn get_pixel(&self, x: usize, y: usize) -> Color {
        self.data[y][x]
    }
    pub fn get_row(&self, y: usize) -> &[Color; COL] {
        &self.data[y]
    }
}

/// Implement embedded_graphics::Dimensions for FrameBuffer
impl<const COL: usize, const ROW: usize> Dimensions for FrameBuffer<COL, ROW> {
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        Rectangle::new(Point::zero(), Size::new(COL as u32, ROW as u32))
    }
}

/// Implement embedded_graphics::DrawTarget for FrameBuffer
impl<const COL: usize, const ROW: usize> DrawTarget for FrameBuffer<COL, ROW> {
    type Color = Color;

    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::prelude::Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            if p.x < 0 || p.x as usize >= COL || p.y < 0 || p.y as usize >= ROW {
                continue;
            }
            self.set_pixel(p.x as usize, p.y as usize, c);
        }

        Ok(())
    }
}

pub struct Hub75<
    A: OutputPin,
    B: OutputPin,
    C: OutputPin,
    D: OutputPin,
    E: OutputPin,
    R1: OutputPin,
    G1: OutputPin,
    B1: OutputPin,
    R2: OutputPin,
    G2: OutputPin,
    B2: OutputPin,
    CLK: OutputPin,
    LAT: OutputPin,
    OE: OutputPin,
    const MATRIX_COLS: usize,
    const MATRIX_ROWS: usize,
> {
    a: A,
    b: B,
    c: C,
    d: D,
    e: E,
    r1: R1,
    g1: G1,
    b1: B1,
    r2: R2,
    g2: G2,
    b2: B2,
    clk: CLK,
    lat: LAT,
    oe: OE,
}

impl<
        A: OutputPin,
        B: OutputPin,
        C: OutputPin,
        D: OutputPin,
        E: OutputPin,
        R1: OutputPin,
        G1: OutputPin,
        B1: OutputPin,
        R2: OutputPin,
        G2: OutputPin,
        B2: OutputPin,
        CLK: OutputPin,
        LAT: OutputPin,
        OE: OutputPin,
        const MATRIX_COLS: usize,
        const MATRIX_ROWS: usize,
    > Hub75<A, B, C, D, E, R1, G1, B1, R2, G2, B2, CLK, LAT, OE, MATRIX_COLS, MATRIX_ROWS>
{
    const XROWS: usize = MATRIX_ROWS / 2;
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        a: A,
        b: B,
        c: C,
        d: D,
        e: E,
        r1: R1,
        g1: G1,
        b1: B1,
        r2: R2,
        g2: G2,
        b2: B2,
        clk: CLK,
        lat: LAT,
        oe: OE,
    ) -> Self {
        Self {
            a,
            b,
            c,
            d,
            e,
            r1,
            g1,
            b1,
            r2,
            g2,
            b2,
            clk,
            lat,
            oe,
        }
    }

    const BRIGHTNESS_BITS: u8 = 3;
    const BRIGHTNESS_STEP: u8 = 1 << (8 - Self::BRIGHTNESS_BITS);

    #[esp_hal_procmacros::ram]
    fn render_row(
        &mut self,
        row: usize,
        data0: &[Color; MATRIX_COLS],
        data1: &[Color; MATRIX_COLS],
        brightness: u8,
    ) {
        for i in 0..MATRIX_COLS {
            self.clock_in(
                data0[i].r() > brightness,
                data0[i].g() > brightness,
                data0[i].b() > brightness,
                data1[i].r() > brightness,
                data1[i].g() > brightness,
                data1[i].b() > brightness,
            );
        }
        self.blank(true);
        self.set_address(row as u8);
        self.latch(0);
        self.blank(false);
    }

    #[esp_hal_procmacros::ram]
    pub fn set_address(&mut self, addr: u8) {
        // info!("Setting address: {}", addr);
        self.a.set_state(((addr & 1) != 0).into()).unwrap();
        self.b.set_state(((addr & 2) != 0).into()).unwrap();
        self.c.set_state(((addr & 4) != 0).into()).unwrap();
        self.d.set_state(((addr & 8) != 0).into()).unwrap();
        self.e.set_state(((addr & 16) != 0).into()).unwrap();
    }

    #[esp_hal_procmacros::ram]
    pub fn clock_in(&mut self, r1: bool, g1: bool, b1: bool, r2: bool, g2: bool, b2: bool) {
        self.r1.set_state(r1.into()).unwrap();
        self.g1.set_state(g1.into()).unwrap();
        self.b1.set_state(b1.into()).unwrap();
        self.r2.set_state(r2.into()).unwrap();
        self.g2.set_state(g2.into()).unwrap();
        self.b2.set_state(b2.into()).unwrap();
        self.clk.set_high().unwrap();
        self.clk.set_low().unwrap();
    }

    #[esp_hal_procmacros::ram]
    pub fn clock(&mut self) {
        self.clk.set_high().unwrap();
        self.clk.set_low().unwrap();
    }

    #[esp_hal_procmacros::ram]
    pub fn latch(&mut self, clks: u8) {
        self.lat.set_high().unwrap();
        for _ in 0..clks {
            self.clock();
        }
        self.lat.set_low().unwrap();
    }

    #[esp_hal_procmacros::ram]
    pub fn blank(&mut self, state: bool) {
        self.oe.set_state(state.into()).unwrap();
    }

    #[esp_hal_procmacros::ram]
    pub fn display(&mut self, framebuffer: &FrameBuffer<MATRIX_COLS, MATRIX_ROWS>) {
        self.blank(false);
        for i in 0..Self::XROWS {
            let row0 = &framebuffer.data[i];
            let row1 = &framebuffer.data[i + Self::XROWS];
            let brightness_count = ((1 << Self::BRIGHTNESS_BITS as u16) - 1) as u8;
            for brightness in 0..brightness_count {
                let brightness = brightness.saturating_mul(Self::BRIGHTNESS_STEP);
                self.render_row(i, row0, row1, brightness);
            }
        }
        self.blank(true);
    }
}
