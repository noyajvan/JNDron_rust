//! Crash / hard-landing detection, ported from `checkCrashDetection()` in
//! `mavlink_util.cpp`.
//!
//! The detector is gated on "the drone actually flew": it only arms itself in
//! `MISSION`, while armed, and after climbing more than 1 m above the mission
//! base altitude. Timers are reset whenever the drone moves again, so a
//! "fall then fly again" is not reported as a crash.

use crate::consts::State;

/// Why a crash was declared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrashKind {
    /// Stuck in an obstacle: stable altitude, zero speed, high throttle.
    Net,
    /// Violent rotation / tumble.
    Tumble,
    /// Descent faster than 5 m/s.
    RapidDescent,
}

impl CrashKind {
    pub fn label(self) -> &'static str {
        match self {
            CrashKind::Net => "Net",
            CrashKind::Tumble => "Tumble",
            CrashKind::RapidDescent => "RapidDescent",
        }
    }
}

/// Snapshot of the values `checkCrashDetection` reads.
#[derive(Debug, Clone, Copy)]
pub struct CrashInput {
    pub state: State,
    pub is_armed: bool,
    pub vfr_alt: f32,
    /// `-1.0` until the first valid VFR_HUD fixes the base altitude.
    pub mission_base_alt: f32,
    pub ground_speed: f32,
    pub throttle: u16,
    pub climb: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
}

/// Persistent detector state (the `static` locals of the C++ function).
#[derive(Debug, Clone)]
pub struct CrashDetector {
    was_flying: bool,
    last_stable_alt: f32,
    stuck_timer: u32,
    gyro_timer: u32,
    descend_timer: u32,
}

impl Default for CrashDetector {
    fn default() -> Self {
        CrashDetector {
            was_flying: false,
            last_stable_alt: 0.0,
            stuck_timer: 0,
            gyro_timer: 0,
            descend_timer: 0,
        }
    }
}

/// Thresholds. Kept as named constants so the behaviour is auditable against
/// the pilot guide (see `firmware/GUIDE.md`).
pub const STUCK_ALT_EPS_M: f32 = 0.15;
pub const STUCK_SPEED_EPS: f32 = 0.15;
pub const STUCK_TIME_MS: u32 = 3_000;
pub const STUCK_THROTTLE_MIN: u16 = 45;
/// 4500 raw gyro units/s ~= 274 deg/s.
pub const TUMBLE_GYRO_LIMIT: f32 = 4_500.0;
pub const TUMBLE_TIME_MS: u32 = 150;
pub const DESCENT_LIMIT_MPS: f32 = -5.0;
pub const DESCENT_TIME_MS: u32 = 500;
/// The drone must have climbed at least this far above base to count as flown.
pub const FLY_ALT_M: f32 = 1.0;

impl CrashDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Run one detection step. Returns the crash kind the moment it triggers.
    pub fn update(&mut self, now: u32, i: &CrashInput) -> Option<CrashKind> {
        // Gate: only in MISSION and armed.
        if i.state != State::Mission || !i.is_armed {
            self.was_flying = false;
            self.stuck_timer = 0;
            self.gyro_timer = 0;
            self.descend_timer = 0;
            self.last_stable_alt = i.vfr_alt;
            return None;
        }

        if i.mission_base_alt >= 0.0 && i.vfr_alt - i.mission_base_alt > FLY_ALT_M {
            self.was_flying = true;
        }
        if !self.was_flying {
            return None;
        }

        let mut kind = None;

        // 1. NET - stuck against an obstacle.
        if (i.vfr_alt - self.last_stable_alt).abs() < STUCK_ALT_EPS_M
            && i.ground_speed < STUCK_SPEED_EPS
        {
            if self.stuck_timer == 0 {
                self.stuck_timer = now;
            }
            if now.wrapping_sub(self.stuck_timer) > STUCK_TIME_MS && i.throttle > STUCK_THROTTLE_MIN
            {
                kind = Some(CrashKind::Net);
            }
        } else {
            self.stuck_timer = 0;
            self.last_stable_alt = i.vfr_alt;
        }

        // 2. TUMBLE - violent rotation.
        if i.gyro_x.abs() > TUMBLE_GYRO_LIMIT || i.gyro_y.abs() > TUMBLE_GYRO_LIMIT {
            if self.gyro_timer == 0 {
                self.gyro_timer = now;
            }
            if now.wrapping_sub(self.gyro_timer) > TUMBLE_TIME_MS {
                kind = kind.or(Some(CrashKind::Tumble));
            }
        } else {
            self.gyro_timer = 0;
        }

        // 3. RAPID DESCENT.
        if i.climb < DESCENT_LIMIT_MPS {
            if self.descend_timer == 0 {
                self.descend_timer = now;
            }
            if now.wrapping_sub(self.descend_timer) > DESCENT_TIME_MS {
                kind = kind.or(Some(CrashKind::RapidDescent));
            }
        } else {
            self.descend_timer = 0;
        }

        kind
    }

    pub fn was_flying(&self) -> bool {
        self.was_flying
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input() -> CrashInput {
        CrashInput {
            state: State::Mission,
            is_armed: true,
            vfr_alt: 10.0,
            mission_base_alt: 0.0,
            ground_speed: 5.0,
            throttle: 50,
            climb: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
        }
    }

    #[test]
    fn requires_flight_above_1m() {
        let mut d = CrashDetector::new();
        let mut i = input();
        i.vfr_alt = 0.5; // never climbed above base + 1 m
        i.climb = -10.0; // a "descent" on the ground
        for t in 0..2000 {
            assert_eq!(d.update(t * 10, &i), None);
        }
        assert!(!d.was_flying());
    }

    #[test]
    fn rapid_descent_triggers_after_500ms() {
        let mut d = CrashDetector::new();
        let mut i = input();
        // first tick establishes "was flying"
        assert_eq!(d.update(0, &i), None);
        i.vfr_alt = 9.0;
        i.climb = -6.0;
        assert_eq!(d.update(100, &i), None);
        assert_eq!(d.update(400, &i), None); // < 500 ms
        assert_eq!(d.update(700, &i), Some(CrashKind::RapidDescent));
    }

    #[test]
    fn tumble_triggers_after_150ms() {
        let mut d = CrashDetector::new();
        let mut i = input();
        assert_eq!(d.update(0, &i), None);
        i.gyro_x = 5000.0;
        assert_eq!(d.update(10, &i), None);
        assert_eq!(d.update(200, &i), Some(CrashKind::Tumble));
    }

    #[test]
    fn net_requires_high_throttle() {
        let mut d = CrashDetector::new();
        let mut i = input();
        i.ground_speed = 0.0;
        assert_eq!(d.update(0, &i), None);
        i.throttle = 30; // too low
        assert_eq!(d.update(4000, &i), None);
        i.throttle = 60;
        assert_eq!(d.update(8000, &i), Some(CrashKind::Net));
    }

    #[test]
    fn movement_resets_timers_so_recovery_is_not_a_crash() {
        let mut d = CrashDetector::new();
        let mut i = input();
        assert_eq!(d.update(0, &i), None);
        i.climb = -6.0;
        assert_eq!(d.update(100, &i), None);
        i.climb = 0.0; // recovered
        assert_eq!(d.update(400, &i), None);
        i.climb = -6.0;
        assert_eq!(d.update(500, &i), None); // timer restarted
        assert_eq!(d.update(1100, &i), Some(CrashKind::RapidDescent));
    }

    #[test]
    fn disarmed_or_not_in_mission_disables_detection() {
        let mut d = CrashDetector::new();
        let mut i = input();
        i.climb = -9.0;
        i.state = State::Arming;
        for t in 0..100 {
            assert_eq!(d.update(t * 100, &i), None);
        }
    }
}
