use ratatui::{
    prelude::*,
    widgets::{Block, Borders, Gauge, Paragraph, Row, Table},
};

use rosbag_deck::PlaybackState;

use super::App;

pub fn draw(frame: &mut Frame, app: &App) {
    let area = frame.area();

    let chunks = Layout::vertical([
        Constraint::Length(3), // header
        Constraint::Length(3), // transport + timeline
        Constraint::Min(5),    // topics / messages
        Constraint::Length(2), // help bar
    ])
    .split(area);

    draw_header(frame, app, chunks[0]);
    draw_transport(frame, app, chunks[1]);
    draw_messages(frame, app, chunks[2]);
    draw_help(frame, app, chunks[3]);
}

fn draw_header(frame: &mut Frame, app: &App, area: Rect) {
    let meta = app.deck.metadata();
    let dur = meta.duration;
    let secs = dur.as_secs();

    let text = format!(
        " {}  |  {}  |  {:02}:{:02}:{:02}  |  {} messages  |  {} topics",
        meta.storage_identifier,
        if app.deck.looping() { "Loop" } else { "Once" },
        secs / 3600,
        (secs % 3600) / 60,
        secs % 60,
        meta.message_count,
        meta.topics.len(),
    );

    let block = Block::default()
        .title(" RosBag Deck ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));

    let paragraph = Paragraph::new(text).block(block);
    frame.render_widget(paragraph, area);
}

fn draw_transport(frame: &mut Frame, app: &App, area: Rect) {
    let state_icon = match app.deck.state() {
        PlaybackState::Playing => "▶ Playing ",
        PlaybackState::Paused => "⏸ Paused  ",
        PlaybackState::Stopped => "⏹ Stopped ",
    };

    let pos = format_duration_ns(app.position_ns().max(0));
    let dur = format_duration_ns(app.duration_ns().max(0));
    let speed = app.deck.speed();

    let label = format!("{state_icon} {pos} / {dur}   {speed:.1}x");

    let gauge = Gauge::default()
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        )
        .gauge_style(Style::default().fg(Color::Cyan).bg(Color::DarkGray))
        .ratio(app.progress())
        .label(Span::styled(label, Style::default().fg(Color::White)));

    frame.render_widget(gauge, area);
}

fn draw_messages(frame: &mut Frame, app: &App, area: Rect) {
    let block = Block::default()
        .title(" Messages ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::DarkGray));

    if app.message_log.is_empty() {
        let hint = if app.deck.state() == PlaybackState::Stopped {
            "Press Space to start playback"
        } else {
            "Waiting for messages..."
        };
        let paragraph = Paragraph::new(hint)
            .block(block)
            .style(Style::default().fg(Color::DarkGray));
        frame.render_widget(paragraph, area);
        return;
    }

    let inner_height = area.height.saturating_sub(2) as usize; // borders
    let start = app.message_log.len().saturating_sub(inner_height);
    let bag_start_ns = app.deck.metadata().start_time_ns;

    let rows: Vec<Row> = app
        .message_log
        .iter()
        .skip(start)
        .map(|entry| {
            let rel_ns = entry.timestamp_ns - bag_start_ns;
            let time_str = format_duration_ns(rel_ns.max(0));
            let size = format_bytes(entry.data_len);
            Row::new(vec![time_str, entry.topic.clone(), size])
        })
        .collect();

    let widths = [
        Constraint::Length(12),
        Constraint::Min(20),
        Constraint::Length(10),
    ];
    let table = Table::new(rows, widths)
        .header(
            Row::new(vec!["Time", "Topic", "Size"])
                .style(Style::default().fg(Color::Yellow))
                .bottom_margin(0),
        )
        .block(block);

    frame.render_widget(table, area);
}

fn draw_help(frame: &mut Frame, app: &App, area: Rect) {
    let text = if let Some(ref input) = app.seek_input {
        format!("Seek to: {input}_ (Enter to confirm, Esc to cancel)")
    } else if let Some(ref msg) = app.status_message {
        msg.clone()
    } else {
        "Space:Play/Pause  s:Stop  ←→:Step  ±:Speed  g:Seek  o:Loop  Home/End  q:Quit".into()
    };

    let paragraph = Paragraph::new(text).style(Style::default().fg(Color::DarkGray));
    frame.render_widget(paragraph, area);
}

fn format_duration_ns(ns: i64) -> String {
    let total_secs = ns / 1_000_000_000;
    let millis = (ns % 1_000_000_000) / 1_000_000;
    let mins = total_secs / 60;
    let secs = total_secs % 60;

    if mins >= 60 {
        let hours = mins / 60;
        let mins = mins % 60;
        format!("{hours:02}:{mins:02}:{secs:02}.{millis:03}")
    } else {
        format!("{mins:02}:{secs:02}.{millis:03}")
    }
}

fn format_bytes(bytes: usize) -> String {
    if bytes >= 1024 * 1024 {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    } else if bytes >= 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else {
        format!("{bytes} B")
    }
}
