mod ui;

use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

use anyhow::Result;
use crossterm::{
    cursor,
    event::{self, Event, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{self, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::prelude::*;

use rosbag_deck_core::{PlaybackMode, PlaybackState, TimedMessage};

use crate::play::{self, PlayOpts};

/// Maximum number of messages to keep in the log for display.
const MESSAGE_LOG_CAPACITY: usize = 200;

/// TUI application state.
pub struct App {
    pub deck: rosbag_deck_core::Deck,
    pub message_log: VecDeque<LogEntry>,
    pub should_quit: bool,
    pub seek_input: Option<String>,
    pub status_message: Option<String>,
}

/// A single entry in the message log.
pub struct LogEntry {
    pub timestamp_ns: i64,
    pub topic: String,
    pub data_len: usize,
}

impl From<&TimedMessage> for LogEntry {
    fn from(msg: &TimedMessage) -> Self {
        Self {
            timestamp_ns: msg.message.timestamp_ns,
            topic: msg.message.topic.clone(),
            data_len: msg.message.data.len(),
        }
    }
}

impl App {
    fn new(opts: &PlayOpts) -> Result<Self> {
        let deck = play::open_deck(opts)?;
        Ok(Self {
            deck,
            message_log: VecDeque::with_capacity(MESSAGE_LOG_CAPACITY),
            should_quit: false,
            seek_input: None,
            status_message: None,
        })
    }

    fn tick(&mut self) {
        if self.deck.state() != PlaybackState::Playing {
            return;
        }

        match self.deck.next_message() {
            Ok(Some(timed)) => {
                self.push_log(&timed);
            }
            Ok(None) => {}
            Err(e) => {
                self.status_message = Some(format!("Error: {e}"));
            }
        }
    }

    fn push_log(&mut self, msg: &TimedMessage) {
        if self.message_log.len() >= MESSAGE_LOG_CAPACITY {
            self.message_log.pop_front();
        }
        self.message_log.push_back(LogEntry::from(msg));
    }

    fn handle_key(&mut self, key: KeyEvent) {
        // If in seek input mode, handle typing.
        if let Some(ref mut input) = self.seek_input {
            match key.code {
                KeyCode::Enter => {
                    let input = input.clone();
                    self.seek_input = None;
                    if let Some(ns) = parse_seek_input(&input, self.deck.metadata().start_time_ns) {
                        self.deck.seek_to_time(ns);
                        self.status_message = None;
                    } else {
                        self.status_message =
                            Some("Invalid seek time. Use seconds (e.g., 10.5) or MM:SS".into());
                    }
                }
                KeyCode::Esc => {
                    self.seek_input = None;
                }
                KeyCode::Backspace => {
                    input.pop();
                }
                KeyCode::Char(c) if c.is_ascii_digit() || c == '.' || c == ':' => {
                    input.push(c);
                }
                _ => {}
            }
            return;
        }

        match key.code {
            KeyCode::Char('q') | KeyCode::Char('Q') => self.should_quit = true,
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                self.should_quit = true;
            }
            KeyCode::Char(' ') => self.deck.toggle_play_pause(),
            KeyCode::Char('s') | KeyCode::Char('S') => self.deck.stop(),
            KeyCode::Right | KeyCode::Char('l') => {
                let _ = self.deck.step_forward();
            }
            KeyCode::Left | KeyCode::Char('h') => {
                let _ = self.deck.step_backward();
            }
            KeyCode::Char('+') | KeyCode::Char('=') => {
                let new_speed = next_speed_up(self.deck.speed());
                self.deck.set_speed(new_speed);
            }
            KeyCode::Char('-') | KeyCode::Char('_') => {
                let new_speed = next_speed_down(self.deck.speed());
                self.deck.set_speed(new_speed);
            }
            KeyCode::Char('g') | KeyCode::Char('G') => {
                self.seek_input = Some(String::new());
                self.status_message = Some("Seek to (seconds or MM:SS):".into());
            }
            KeyCode::Char('o') | KeyCode::Char('O') => {
                self.deck.set_looping(!self.deck.looping());
            }
            KeyCode::Home => {
                self.deck.seek_to_ratio(0.0);
            }
            KeyCode::End => {
                self.deck.seek_to_ratio(1.0);
            }
            _ => {}
        }
    }

    /// Playback progress as a ratio 0.0 .. 1.0.
    pub fn progress(&self) -> f64 {
        let meta = self.deck.metadata();
        let range = meta.end_time_ns - meta.start_time_ns;
        if range <= 0 {
            return 0.0;
        }
        let pos = self.deck.cursor_ns() - meta.start_time_ns;
        (pos as f64 / range as f64).clamp(0.0, 1.0)
    }

    /// Current position relative to bag start, in nanoseconds.
    pub fn position_ns(&self) -> i64 {
        self.deck.cursor_ns() - self.deck.metadata().start_time_ns
    }

    /// Total duration in nanoseconds.
    pub fn duration_ns(&self) -> i64 {
        self.deck.metadata().end_time_ns - self.deck.metadata().start_time_ns
    }
}

/// Parse seek input. Supports seconds ("10.5") or "MM:SS" format.
/// Returns absolute bag timestamp in nanoseconds.
fn parse_seek_input(input: &str, bag_start_ns: i64) -> Option<i64> {
    if input.contains(':') {
        let parts: Vec<&str> = input.split(':').collect();
        if parts.len() == 2 {
            let mins: f64 = parts[0].parse().ok()?;
            let secs: f64 = parts[1].parse().ok()?;
            let total_secs = mins * 60.0 + secs;
            return Some(bag_start_ns + (total_secs * 1e9) as i64);
        }
        None
    } else {
        let secs: f64 = input.parse().ok()?;
        Some(bag_start_ns + (secs * 1e9) as i64)
    }
}

/// Speed step presets.
const SPEED_PRESETS: &[f64] = &[0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0];

fn next_speed_up(current: f64) -> f64 {
    for &s in SPEED_PRESETS {
        if s > current + 0.01 {
            return s;
        }
    }
    current
}

fn next_speed_down(current: f64) -> f64 {
    for &s in SPEED_PRESETS.iter().rev() {
        if s < current - 0.01 {
            return s;
        }
    }
    current
}

/// Run the TUI.
pub fn run(opts: &PlayOpts) -> Result<()> {
    terminal::enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    execute!(stdout, EnterAlternateScreen, cursor::Hide)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new(opts)?;
    app.deck.set_mode(PlaybackMode::BestEffort);

    let tick_rate = Duration::from_millis(16); // ~60 fps
    let result = run_loop(&mut terminal, &mut app, tick_rate);

    // Restore terminal.
    terminal::disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen, cursor::Show)?;

    result
}

fn run_loop(
    terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    app: &mut App,
    tick_rate: Duration,
) -> Result<()> {
    let mut last_tick = Instant::now();

    loop {
        terminal.draw(|frame| ui::draw(frame, app))?;

        let timeout = tick_rate.saturating_sub(last_tick.elapsed());
        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                app.handle_key(key);
            }
        }

        if last_tick.elapsed() >= tick_rate {
            app.tick();
            last_tick = Instant::now();
        }

        if app.should_quit {
            return Ok(());
        }
    }
}
