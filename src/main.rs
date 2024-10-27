#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, Ordering},
};

use atomic_enum::atomic_enum;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoFont, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    gpio::{self, AnyPin, GpioPin, Input},
    i2c::I2c,
    interrupt::Priority,
    peripherals::{Interrupt, ADC2, I2C0},
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
    Blocking,
};
use fugit::HertzU32;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

// TODO: Can these type arguments be avoided here? and avoid hardcoding display size
type DisplayType<'a> = Ssd1306<
    I2CInterface<I2c<'a, I2C0, Blocking>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

const LOW_JOYSTICK: u16 = 100;
const MID_JOYSTICK: u16 = 1600;
const HIGH_JOYSTICK: u16 = 3500;

static PLAYER_DIRECTION: AtomicDirection = AtomicDirection::new(Direction::Right);
static PROCESSED_DIRECTION: AtomicDirection = AtomicDirection::new(Direction::Right);
static CONFIRM_BOOL: AtomicBool = AtomicBool::new(false);

static G_BUTTON: Mutex<RefCell<Option<Input<AnyPin>>>> = Mutex::new(RefCell::new(None));

#[atomic_enum]
#[derive(PartialEq)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
}

impl Direction {
    pub fn from_urx_ury(urx: u16, ury: u16) -> Option<Self> {
        // Here Up is towards the wires connecting the joypad
        // If the joystick is in a corner, return the most extreme direction
        if urx < LOW_JOYSTICK && ury < LOW_JOYSTICK {
            if MID_JOYSTICK - urx < MID_JOYSTICK - ury {
                return Some(Direction::Up);
            } else {
                return Some(Direction::Right);
            }
        } else if urx > HIGH_JOYSTICK && ury < LOW_JOYSTICK {
            if urx - MID_JOYSTICK < MID_JOYSTICK - ury {
                return Some(Direction::Down);
            } else {
                return Some(Direction::Right);
            }
        } else if urx < LOW_JOYSTICK && ury > HIGH_JOYSTICK {
            if MID_JOYSTICK - urx < ury - MID_JOYSTICK {
                return Some(Direction::Up);
            } else {
                return Some(Direction::Left);
            }
        } else if urx > HIGH_JOYSTICK && ury > HIGH_JOYSTICK {
            if urx - MID_JOYSTICK < ury - MID_JOYSTICK {
                return Some(Direction::Down);
            } else {
                return Some(Direction::Left);
            }
        } else if urx < LOW_JOYSTICK {
            return Some(Direction::Up);
        } else if urx > HIGH_JOYSTICK {
            return Some(Direction::Down);
        } else if ury < LOW_JOYSTICK {
            return Some(Direction::Right);
        } else if ury > HIGH_JOYSTICK {
            return Some(Direction::Left);
        }
        None
    }

    pub fn opposite(&self) -> Self {
        match self {
            Direction::Up => Direction::Down,
            Direction::Down => Direction::Up,
            Direction::Left => Direction::Right,
            Direction::Right => Direction::Left,
        }
    }
}

pub enum GameState {
    StartScreen,
    Playing,
    GameOver,
    Won,
}

pub struct Player {
    pub head: Point,
    pub tail: [Option<Point>; 32],
    pub length: u8,
    pub direction: &'static AtomicDirection,
}

impl Player {
    pub fn new() -> Self {
        Player {
            head: Point::new(0, 0),
            tail: [None; 32],
            length: 1,
            direction: &PLAYER_DIRECTION,
        }
    }

    pub fn shift_position(p: Point, direction: &AtomicDirection, size: Size) -> Point {
        let direction = direction.load(Ordering::Relaxed);
        match direction {
            Direction::Up => Point::new(p.x, (p.y - 1).rem_euclid(size.height as i32)),
            Direction::Down => Point::new(p.x, (p.y + 1).rem_euclid(size.height as i32)),
            Direction::Left => Point::new((p.x - 1).rem_euclid(size.width as i32), p.y),
            Direction::Right => Point::new((p.x + 1).rem_euclid(size.width as i32), p.y),
        }
    }

    pub fn next_pos_is_food(&self, food_pos: Point, board_size: Size) -> bool {
        Self::shift_position(self.head, self.direction, board_size) == food_pos
    }

    pub fn head_in_tail(&self) -> bool {
        self.tail.contains(&Some(self.head))
    }

    pub fn move_player(&mut self, size: Size) {
        for i in (1..self.length).rev() {
            if self.tail[i as usize].is_none() {
                continue;
            }
            self.tail[i as usize] = self.tail[i as usize - 1];
        }

        if self.tail[0].is_some() {
            self.tail[0] = Some(self.head);
        }
        let new_head = Self::shift_position(self.head, self.direction, size);
        self.head = new_head;
        // if head in tail then end
    }

    pub fn add_head(&mut self, new_head: Point) {
        for i in (0..self.length).rev() {
            self.tail[i as usize + 1] = self.tail[i as usize];
        }
        self.tail[0] = Some(self.head);
        self.head = new_head;
    }
}

impl Default for Player {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Game {
    pub state: GameState,
    pub pixel_scale: u8,
    pub title_font: MonoFont<'static>,
    pub score: u16,
    pub score_pos: Point,
    pub top_left: Point,
    pub size: Size,
    pub food_pos: Option<Point>,
    pub player: Player,
    pub style_on: PrimitiveStyle<BinaryColor>,
    pub style_off: PrimitiveStyle<BinaryColor>,
    pub rng: Rng,
}

impl Game {
    pub fn new(rng: Rng, pixel_scale: u8, title_font: MonoFont<'static>) -> Self {
        Game {
            score: 0,
            player: Player::new(),
            // Physical start of the game board in pixels
            top_left: Point::new(
                0,
                (title_font.character_size.height + (3 * pixel_scale as u32)) as i32,
            ),
            // Logical size - divided by pixel scale
            size: Size::new(
                128 / pixel_scale as u32,
                ((64 - title_font.character_size.height) / pixel_scale as u32) - 3,
            ),
            food_pos: None,
            score_pos: Point::new((title_font.character_size.width * 8) as i32, 0),
            pixel_scale,
            title_font,
            style_on: PrimitiveStyle::with_stroke(BinaryColor::On, 1),
            style_off: PrimitiveStyle::with_fill(BinaryColor::Off),
            rng,
            state: GameState::StartScreen,
        }
    }

    pub fn reset(&mut self) {
        PLAYER_DIRECTION.store(Direction::Right, Ordering::Relaxed);
        self.score = 0;
        self.player = Player::new();
        self.food_pos = None;
    }

    fn draw_start_screen(&mut self, display: &mut DisplayType) {
        self.clear_screen(display);
        let text_style = MonoTextStyleBuilder::new()
            .font(&self.title_font)
            .text_color(BinaryColor::On)
            .build();
        Text::with_baseline(
            "Click stick\nto start! ",
            Point::zero(),
            text_style,
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        display.flush().unwrap();
    }

    fn draw_death_screen(&mut self, display: &mut DisplayType) {
        self.clear_screen(display);
        let text_style = MonoTextStyleBuilder::new()
            .font(&self.title_font)
            .text_color(BinaryColor::On)
            .build();
        Text::with_baseline(
            "You died!\nClick stick\nto restart! ",
            Point::zero(),
            text_style,
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        display.flush().unwrap();
    }

    fn clear_screen(&mut self, display: &mut DisplayType) {
        display.clear_buffer();
        display.flush().unwrap();
    }

    fn draw_win_screen(&mut self, display: &mut DisplayType) {
        self.clear_screen(display);
        let text_style = MonoTextStyleBuilder::new()
            .font(&self.title_font)
            .text_color(BinaryColor::On)
            .build();
        Text::with_baseline(
            "You won!\nClick stick\nto restart! ",
            Point::zero(),
            text_style,
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        display.flush().unwrap();
    }

    fn start_game(&mut self, display: &mut DisplayType) {
        self.clear_screen(display);
        // esp_println::println!("Size: {:?}", self.size);

        let text_style = MonoTextStyleBuilder::new()
            .font(&self.title_font)
            .text_color(BinaryColor::On)
            .build();
        Text::with_baseline("Score: ", Point::zero(), text_style, Baseline::Top)
            .draw(display)
            .unwrap();

        Text::with_baseline("0", self.score_pos, text_style, Baseline::Top)
            .draw(display)
            .unwrap();

        let line_start_y =
            (self.title_font.character_size.height + (self.pixel_scale as u32 * 2)) as i32;
        Line::new(Point::new(0, line_start_y), Point::new(128, line_start_y))
            .into_styled(self.style_on)
            .draw(display)
            .unwrap();

        self.draw_player(display, self.style_on);
        self.place_food(display);

        display.flush().unwrap();
    }

    pub fn draw_player(&self, display: &mut DisplayType, style: PrimitiveStyle<BinaryColor>) {
        let head_pos = self.top_left + (self.player.head * self.pixel_scale as i32);
        Rectangle::new(
            head_pos,
            Size::new(self.pixel_scale as u32, self.pixel_scale as u32),
        )
        .into_styled(style)
        .draw(display)
        .unwrap();

        for tail_pos in self.player.tail.iter().filter_map(|x| *x) {
            let tail_pos = self.top_left + tail_pos * (self.pixel_scale as i32);

            Rectangle::new(
                tail_pos,
                Size::new(self.pixel_scale as u32, self.pixel_scale as u32),
            )
            .into_styled(style)
            .draw(display)
            .unwrap();
        }
    }

    pub fn place_food(&mut self, display: &mut DisplayType) {
        loop {
            let food_pos = self.get_random_board_position();
            if food_pos == self.player.head || self.player.tail.contains(&Some(food_pos)) {
                continue;
            }
            self.food_pos = Some(food_pos);
            break;
        }

        // Physical pixel position
        let food_pos = self.top_left + (self.food_pos.unwrap() * self.pixel_scale as i32);
        Rectangle::new(
            food_pos,
            Size::new(self.pixel_scale as u32, self.pixel_scale as u32),
        )
        .into_styled(self.style_on)
        .draw(display)
        .unwrap();
    }

    pub fn get_random_board_position(&mut self) -> Point {
        let x = self.rng.random() % self.size.width;
        let y = self.rng.random() % self.size.height;
        Point::new(x as i32, y as i32)
    }

    pub fn process_frame(&mut self, display: &mut DisplayType) {
        match self.state {
            GameState::StartScreen => {
                if CONFIRM_BOOL.swap(false, Ordering::Relaxed) {
                    self.state = GameState::Playing;
                    self.start_game(display);
                }
            }
            GameState::Playing => {
                // esp_println::println!("Head: {:?}, Tail: {:?}", self.player.head, self.player.tail);
                self.draw_player(display, self.style_off);
                // display.flush().unwrap();

                if let Some(food_pos) = self.food_pos {
                    if self.player.next_pos_is_food(food_pos, self.size) {
                        self.player.length += 1;
                        self.score += 1;
                        self.player.add_head(self.food_pos.take().unwrap());

                        self.place_food(display);

                        // Update Score
                        Rectangle::new(
                            self.score_pos,
                            Size::new(
                                self.title_font.character_size.width * 4,
                                self.title_font.character_size.height,
                            ),
                        )
                        .into_styled(self.style_off)
                        .draw(display)
                        .unwrap();

                        // display.flush().unwrap();

                        let mut buffer = itoa::Buffer::new();
                        let printed = buffer.format(self.score);

                        let text_style = MonoTextStyleBuilder::new()
                            .font(&self.title_font)
                            .text_color(BinaryColor::On)
                            .build();
                        Text::with_baseline(printed, self.score_pos, text_style, Baseline::Top)
                            .draw(display)
                            .unwrap();
                        display.flush().unwrap();
                        return;
                    }
                }
                self.player.move_player(self.size);
                PROCESSED_DIRECTION.store(self.player.direction.load(Ordering::Relaxed), Ordering::Relaxed);

                self.draw_player(display, self.style_on);
                if self.player.head_in_tail() {
                    self.state = GameState::GameOver;
                    CONFIRM_BOOL.store(false, Ordering::Relaxed);
                    self.draw_death_screen(display);
                    return;
                }
                if self.player.length >= 16 {
                    self.state = GameState::Won;
                    CONFIRM_BOOL.store(false, Ordering::Relaxed);
                    self.draw_win_screen(display);
                    return;
                }
                display.flush().unwrap();
            }
            GameState::GameOver => {
                if CONFIRM_BOOL.swap(false, Ordering::Relaxed) {
                    self.state = GameState::Playing;
                    self.reset();
                    self.start_game(display);
                }
            }
            GameState::Won => {
                if CONFIRM_BOOL.swap(false, Ordering::Relaxed) {
                    self.state = GameState::Playing;
                    self.reset();
                    self.start_game(display);
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn check_joypad(urx: GpioPin<25>, ury: GpioPin<15>, adc_peripheral: ADC2) {
    let mut adc2_config = esp_hal::analog::adc::AdcConfig::new();
    let mut urx_pin =
        adc2_config.enable_pin(urx, esp_hal::analog::adc::Attenuation::Attenuation0dB);
    let mut ury_pin =
        adc2_config.enable_pin(ury, esp_hal::analog::adc::Attenuation::Attenuation0dB);
    let mut adc2 = esp_hal::analog::adc::Adc::new(adc_peripheral, adc2_config);

    loop {
        let urx_pin_value: u16 =
            esp_hal::prelude::nb::block!(adc2.read_oneshot(&mut urx_pin)).unwrap();
        let ury_pin_value: u16 =
            esp_hal::prelude::nb::block!(adc2.read_oneshot(&mut ury_pin)).unwrap();
        // esp_println::println!(
        //     "URX Value: {}, URY: Value: {}, Direction: {:?}",
        //     urx_pin_value,
        //     ury_pin_value,
        //     Direction::from_urx_ury(urx_pin_value, ury_pin_value),
        // );

        if let Some(direction) = Direction::from_urx_ury(urx_pin_value, ury_pin_value) {
            // We use processed direction to avoid being able to double back if we change direction twice before the next frame
            let current_direction = PROCESSED_DIRECTION.load(Ordering::Relaxed);
            if direction != current_direction && direction != current_direction.opposite() {
                PLAYER_DIRECTION.store(direction, Ordering::Relaxed);
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let io = esp_hal::gpio::Io::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_println::println!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // SSD1306 OLED display
    let sda = io.pins.gpio21;
    let scl = io.pins.gpio22;

    let i2c = esp_hal::i2c::I2c::new(peripherals.I2C0, sda, scl, HertzU32::kHz(100));

    // Scan for I2C devices
    // for addr in 1..=127 {
    //     esp_println::println!("Scanning Address {}", addr as u8);

    //     // Scan Address
    //     let res = i2c.read(addr as u8, &mut [0]);

    //     // Check and Print Result
    //     match res {
    //         Ok(_) => esp_println::println!("Device Found at Address {}", addr as u8),
    //         Err(_) => esp_println::println!("No Device Found"),
    //     }
    // }

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);

    let mut switch = gpio::Input::new(io.pins.gpio14, esp_hal::gpio::Pull::Up);

    switch.listen(gpio::Event::FallingEdge);
    esp_hal::interrupt::enable(Interrupt::GPIO, Priority::Priority3).unwrap();
    unsafe {
        // esp_hal::xtensa_lx::interrupt::enable(); https://github.com/esp-rs/esp-hal/discussions/922
        esp_hal::interrupt::bind_interrupt(Interrupt::GPIO, GPIO_HANDLER.handler());
    }

    critical_section::with(|cs| G_BUTTON.borrow_ref_mut(cs).replace(switch));

    let mut game = Game::new(rng, 4, FONT_8X13);

    game.draw_start_screen(&mut display);

    // Task to set input at faster rate than framerate
    let urx = io.pins.gpio25;
    let ury = io.pins.gpio15;
    let adc2_peripheral = peripherals.ADC2;
    spawner.spawn(check_joypad(urx, ury, adc2_peripheral)).ok();

    loop {
        game.process_frame(&mut display);
        Timer::after(Duration::from_millis(300)).await;
    }
}

#[handler(priority = Priority::Priority3)]
fn GPIO_HANDLER() {
    CONFIRM_BOOL.store(true, Ordering::Relaxed);

    critical_section::with(|cs| {
        G_BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
