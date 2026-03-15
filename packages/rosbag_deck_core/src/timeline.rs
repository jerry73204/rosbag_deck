use std::time::{Duration, Instant};

use crate::types::{LoopMode, PlaybackMode, PlaybackState};

/// Maps wall-clock time to bag time for playback pacing.
///
/// In real-time mode, messages are delivered at their recorded rate (adjusted
/// by speed). In best-effort mode, no timing is applied.
pub struct VirtualTimeline {
    bag_anchor_ns: i64,
    wall_anchor: Instant,
    speed: f64,
    mode: PlaybackMode,
    state: PlaybackState,
    segment_id: u64,
    loop_mode: LoopMode,
    loop_iteration: u64,
    bag_start_ns: i64,
    bag_end_ns: i64,
}

impl VirtualTimeline {
    pub fn new(bag_start_ns: i64, bag_end_ns: i64) -> Self {
        Self {
            bag_anchor_ns: bag_start_ns,
            wall_anchor: Instant::now(),
            speed: 1.0,
            mode: PlaybackMode::RealTime,
            state: PlaybackState::Stopped,
            segment_id: 0,
            loop_mode: LoopMode::Off,
            loop_iteration: 0,
            bag_start_ns,
            bag_end_ns,
        }
    }

    /// Current playback state.
    pub fn state(&self) -> PlaybackState {
        self.state
    }

    /// Current segment ID (incremented on seek/step).
    pub fn segment_id(&self) -> u64 {
        self.segment_id
    }

    /// Current playback speed.
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Current playback mode.
    pub fn mode(&self) -> PlaybackMode {
        self.mode
    }

    /// Current loop mode.
    pub fn loop_mode(&self) -> LoopMode {
        self.loop_mode
    }

    /// Current loop iteration (0 on first pass, incremented on each wrap).
    pub fn loop_iteration(&self) -> u64 {
        self.loop_iteration
    }

    /// Whether looping is enabled (convenience: loop_mode != Off).
    pub fn looping(&self) -> bool {
        self.loop_mode != LoopMode::Off
    }

    /// Bag start timestamp.
    pub fn bag_start_ns(&self) -> i64 {
        self.bag_start_ns
    }

    /// Bag end timestamp.
    pub fn bag_end_ns(&self) -> i64 {
        self.bag_end_ns
    }

    /// Start or resume playback.
    pub fn play(&mut self) {
        self.state = PlaybackState::Playing;
        self.wall_anchor = Instant::now();
    }

    /// Pause playback, preserving the current bag time anchor.
    pub fn pause(&mut self) {
        if self.state == PlaybackState::Playing {
            // Snapshot the current bag time so we can resume from it.
            self.bag_anchor_ns = self.current_bag_time_ns();
            self.state = PlaybackState::Paused;
        }
    }

    /// Stop playback.
    pub fn stop(&mut self) {
        self.state = PlaybackState::Stopped;
    }

    /// Toggle between playing and paused.
    pub fn toggle_play_pause(&mut self) {
        match self.state {
            PlaybackState::Playing => self.pause(),
            PlaybackState::Paused | PlaybackState::Stopped => self.play(),
        }
    }

    /// Seek to a bag timestamp. Increments segment_id and resets loop iteration.
    pub fn seek(&mut self, bag_ns: i64) {
        self.bag_anchor_ns = bag_ns;
        self.wall_anchor = Instant::now();
        self.segment_id += 1;
        self.loop_iteration = 0;
    }

    /// Advance segment ID (e.g., on step). Returns the new segment ID.
    pub fn advance_segment(&mut self) -> u64 {
        self.segment_id += 1;
        self.segment_id
    }

    /// Set playback speed (e.g., 0.5, 1.0, 2.0).
    pub fn set_speed(&mut self, speed: f64) {
        // Snapshot current position before changing speed.
        self.bag_anchor_ns = self.current_bag_time_ns();
        self.wall_anchor = Instant::now();
        self.speed = speed;
    }

    /// Set playback mode.
    pub fn set_mode(&mut self, mode: PlaybackMode) {
        self.mode = mode;
    }

    /// Set loop mode.
    pub fn set_loop_mode(&mut self, mode: LoopMode) {
        self.loop_mode = mode;
        if mode == LoopMode::Off {
            self.loop_iteration = 0;
        }
    }

    /// Set looping (convenience: maps bool to Restart/Off).
    pub fn set_looping(&mut self, looping: bool) {
        self.set_loop_mode(if looping {
            LoopMode::Restart
        } else {
            LoopMode::Off
        });
    }

    /// Bag duration in nanoseconds.
    pub fn bag_duration_ns(&self) -> i64 {
        self.bag_end_ns - self.bag_start_ns
    }

    /// Compute the current bag time based on elapsed wall-clock time.
    fn current_bag_time_ns(&self) -> i64 {
        let elapsed = self.wall_anchor.elapsed();
        let elapsed_ns = (elapsed.as_nanos() as f64 * self.speed) as i64;
        self.bag_anchor_ns + elapsed_ns
    }

    /// How long to wait before delivering a message at `msg_timestamp_ns`.
    ///
    /// Returns `None` in best-effort mode (deliver immediately) or if the
    /// message time has already passed.
    pub fn delay_until(&self, msg_timestamp_ns: i64) -> Option<Duration> {
        if self.mode == PlaybackMode::BestEffort {
            return None;
        }

        let current = self.current_bag_time_ns();
        let diff_ns = msg_timestamp_ns - current;

        if diff_ns <= 0 {
            return None;
        }

        // Wall-clock wait = bag_diff / speed
        let wall_ns = diff_ns as f64 / self.speed;
        if wall_ns <= 0.0 {
            return None;
        }

        Some(Duration::from_nanos(wall_ns as u64))
    }

    /// Check if a bag timestamp is past the end. If looping (Restart or
    /// Monotonic), wraps to start and returns the wrapped timestamp.
    /// Returns `None` if not past end.
    pub fn check_end(&mut self, timestamp_ns: i64) -> Option<i64> {
        if timestamp_ns > self.bag_end_ns {
            if self.looping() {
                self.loop_iteration += 1;
                // Use internal seek logic without resetting loop_iteration.
                self.bag_anchor_ns = self.bag_start_ns;
                self.wall_anchor = Instant::now();
                self.segment_id += 1;
                Some(self.bag_start_ns)
            } else {
                self.stop();
                None
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_realtime_delay() {
        let mut tl = VirtualTimeline::new(0, 10_000_000_000);
        tl.play();

        // Message 100ms from now in bag time — should have ~100ms delay.
        let delay = tl.delay_until(tl.bag_anchor_ns + 100_000_000);
        assert!(delay.is_some());
        let d = delay.unwrap();
        // Should be roughly 100ms (allow some margin for elapsed time).
        assert!(d.as_millis() >= 80 && d.as_millis() <= 120);
    }

    #[test]
    fn test_speed_2x() {
        let mut tl = VirtualTimeline::new(0, 10_000_000_000);
        tl.set_speed(2.0);
        tl.play();

        // Message 100ms ahead in bag time — at 2x, wall wait should be ~50ms.
        let delay = tl.delay_until(tl.bag_anchor_ns + 100_000_000);
        assert!(delay.is_some());
        let d = delay.unwrap();
        assert!(d.as_millis() >= 30 && d.as_millis() <= 70);
    }

    #[test]
    fn test_best_effort_no_delay() {
        let mut tl = VirtualTimeline::new(0, 10_000_000_000);
        tl.set_mode(PlaybackMode::BestEffort);
        tl.play();

        assert!(tl.delay_until(999_999_999_999).is_none());
    }

    #[test]
    fn test_pause_resume_adjusts_anchor() {
        let mut tl = VirtualTimeline::new(0, 10_000_000_000);
        tl.play();

        // Let some wall time pass.
        thread::sleep(Duration::from_millis(50));
        tl.pause();

        // Sleep during pause — should not affect timing.
        thread::sleep(Duration::from_millis(100));
        tl.play();

        // A message 50ms ahead from the resume point should have ~50ms delay.
        let delay = tl.delay_until(tl.bag_anchor_ns + 50_000_000);
        assert!(delay.is_some());
        let d = delay.unwrap();
        assert!(d.as_millis() <= 70);
    }

    #[test]
    fn test_segment_id_increments_on_seek() {
        let mut tl = VirtualTimeline::new(0, 10_000_000_000);
        assert_eq!(tl.segment_id(), 0);

        tl.seek(5_000_000_000);
        assert_eq!(tl.segment_id(), 1);

        tl.seek(1_000_000_000);
        assert_eq!(tl.segment_id(), 2);

        tl.advance_segment();
        assert_eq!(tl.segment_id(), 3);
    }

    #[test]
    fn test_loop_wraps_to_start() {
        let mut tl = VirtualTimeline::new(1000, 5000);
        tl.set_looping(true);
        tl.play();

        // Timestamp past end should wrap.
        let wrapped = tl.check_end(6000);
        assert_eq!(wrapped, Some(1000));
        assert_eq!(tl.state(), PlaybackState::Playing);
    }

    #[test]
    fn test_loop_iteration_increments() {
        let mut tl = VirtualTimeline::new(1000, 5000);
        tl.set_loop_mode(LoopMode::Monotonic);
        tl.play();

        assert_eq!(tl.loop_iteration(), 0);

        tl.check_end(6000);
        assert_eq!(tl.loop_iteration(), 1);

        tl.check_end(6000);
        assert_eq!(tl.loop_iteration(), 2);
    }

    #[test]
    fn test_seek_resets_loop_iteration() {
        let mut tl = VirtualTimeline::new(1000, 5000);
        tl.set_loop_mode(LoopMode::Monotonic);
        tl.play();

        tl.check_end(6000);
        assert_eq!(tl.loop_iteration(), 1);

        tl.seek(2000);
        assert_eq!(tl.loop_iteration(), 0);
    }

    #[test]
    fn test_bag_duration_ns() {
        let tl = VirtualTimeline::new(1000, 5000);
        assert_eq!(tl.bag_duration_ns(), 4000);
    }

    #[test]
    fn test_loop_mode_off_stops_at_end() {
        let mut tl = VirtualTimeline::new(1000, 5000);
        tl.set_loop_mode(LoopMode::Off);
        tl.play();

        let wrapped = tl.check_end(6000);
        assert_eq!(wrapped, None);
        assert_eq!(tl.state(), PlaybackState::Stopped);
    }

    #[test]
    fn test_looping_convenience() {
        let mut tl = VirtualTimeline::new(0, 1000);
        assert!(!tl.looping());

        tl.set_loop_mode(LoopMode::Restart);
        assert!(tl.looping());

        tl.set_loop_mode(LoopMode::Monotonic);
        assert!(tl.looping());

        tl.set_loop_mode(LoopMode::Off);
        assert!(!tl.looping());
    }
}
