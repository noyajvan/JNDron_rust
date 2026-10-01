//! Status-LED logic, ported from `led.cpp` / `updateLED()`.
//!
//! The original firmware drove a single WS2812 pixel at brightness 5/255.
//! `updateLED` is a pure function of the bridge state plus the clock, so it
//! lives here and returns the colour + brightness that the firmware should
//! push to the pixel.

use crate::consts::*;

/// Result of [`update_led`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedOut {
    /// 0xRRGGBB.
    pub color: u32,
    /// 0..=255, matching `Adafruit_NeoPixel::setBrightness`.
    pub brightness: u8,
}

impl LedOut {
    pub const OFF: LedOut = LedOut {
        color: LED_OFF,
        brightness: 5,
    };
}

/// Everything `updateLED()` reads.
#[derive(Debug, Clone, Copy)]
pub struct LedInput {
    pub mdfly: i32,
    pub heartbeat_received: bool,
    pub has_server: bool,
    pub wifi_on: bool,
    pub has_wifi: bool,
    pub state: State,
    pub cal_completion_pct: u8,
}

/// The default pixel brightness the firmware uses (`pixels.setBrightness(5)`).
pub const BASE_BRIGHTNESS: u8 = 5;

/// `setLed(c)`: full base brightness, given colour.
pub fn solid(color: u32) -> LedOut {
    LedOut {
        color,
        brightness: BASE_BRIGHTNESS,
    }
}

/// `setLedBreathing(color, period_ms)`.
pub fn breathing(color: u32, period_ms: u32, now: u32) -> LedOut {
    if period_ms == 0 {
        return solid(color);
    }
    let phase = (now % period_ms) as f32 / period_ms as f32 * 2.0 * std::f32::consts::PI;
    let wave = (phase - std::f32::consts::FRAC_PI_2).sin();
    let b = (wave + 1.0) / 2.0;
    let b = 0.4 + b * 0.6;
    LedOut {
        color,
        brightness: (b * BASE_BRIGHTNESS as f32) as u8,
    }
}

/// Pure port of `updateLED()`.
pub fn update_led(now: u32, i: &LedInput) -> LedOut {
    // Flight loop disabled: LED is dark.
    if i.mdfly == MDFLY_STOP {
        return LedOut::OFF;
    }

    if !i.heartbeat_received {
        if !i.has_server {
            if !i.wifi_on || !i.has_wifi {
                // 250 ms white blink every 1.5 s.
                return if now % 1_500 < 250 {
                    solid(LED_WHITE)
                } else {
                    LedOut::OFF
                };
            }
            // 500 ms white blink every 1 s.
            return if now % 1_000 < 500 {
                solid(LED_WHITE)
            } else {
                LedOut::OFF
            };
        }
        return solid(LED_WHITE);
    }

    match i.state {
        State::InitMavlink => breathing(LED_BLUE, 2_000, now),
        State::MagOk => {
            if now % 1_000 < 500 {
                solid(LED_BLUE)
            } else {
                LedOut::OFF
            }
        }
        State::Calibration => {
            // Faster breathing as calibration progresses; floor at 200 ms.
            let period = 2_000u32
                .saturating_sub(i.cal_completion_pct as u32 * 18)
                .max(200);
            breathing(LED_CHERRY_D, period, now)
        }
        State::CalibrationEnd => LedOut::OFF,
        State::NoArm => {
            if now % 1_000 < 500 {
                solid(LED_YELLOW)
            } else {
                LedOut::OFF
            }
        }
        State::Arming => {
            if now % 1_000 < 500 {
                solid(LED_GREEN)
            } else {
                LedOut::OFF
            }
        }
        State::Mission => solid(LED_GREEN),
        State::RelayControl => {
            if now % 1_000 < 500 {
                solid(LED_RED)
            } else {
                LedOut::OFF
            }
        }
        _ => LedOut::OFF,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> LedInput {
        LedInput {
            mdfly: 0,
            heartbeat_received: true,
            has_server: true,
            wifi_on: true,
            has_wifi: true,
            state: State::Mission,
            cal_completion_pct: 0,
        }
    }

    #[test]
    fn stopped_is_dark() {
        let mut i = base();
        i.mdfly = MDFLY_STOP;
        assert_eq!(update_led(0, &i), LedOut::OFF);
    }

    #[test]
    fn no_heartbeat_blinks_white() {
        let mut i = base();
        i.heartbeat_received = false;
        i.has_server = false;
        i.has_wifi = false;
        assert_eq!(update_led(100, &i).color, LED_WHITE);
        assert_eq!(update_led(400, &i).color, LED_OFF);
    }

    #[test]
    fn mission_is_solid_green() {
        assert_eq!(update_led(1234, &base()).color, LED_GREEN);
    }

    #[test]
    fn calibration_breathing_speeds_up() {
        let mut i = base();
        i.state = State::Calibration;
        i.cal_completion_pct = 0;
        let slow = breathing(LED_CHERRY_D, 2000, 500);
        assert_eq!(update_led(500, &i), slow);

        i.cal_completion_pct = 100;
        let fast = breathing(LED_CHERRY_D, 200, 500);
        assert_eq!(update_led(500, &i), fast);
    }

    #[test]
    fn relay_control_blinks_red() {
        let mut i = base();
        i.state = State::RelayControl;
        assert_eq!(update_led(0, &i).color, LED_RED);
        assert_eq!(update_led(700, &i).color, LED_OFF);
    }
}
