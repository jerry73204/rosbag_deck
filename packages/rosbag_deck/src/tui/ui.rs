use ratatui::{
    prelude::*,
    widgets::{Block, Borders, Clear, Gauge, List, ListItem, Paragraph, Row, Table},
};

use tracing::Level;

use rosbag_deck_core::{LoopMode, PlaybackState};

use super::App;

pub fn draw(frame: &mut Frame, app: &App) {
    let area = frame.area();

    let chunks = Layout::vertical([
        Constraint::Length(3),                                  // header
        Constraint::Length(3),                                  // transport + timeline
        Constraint::Min(5),                                     // messages
        Constraint::Length(log_panel_height(app, area.height)), // logs
        Constraint::Length(2),                                  // help bar
    ])
    .split(area);

    draw_header(frame, app, chunks[0]);
    draw_transport(frame, app, chunks[1]);
    draw_messages(frame, app, chunks[2]);
    draw_logs(frame, app, chunks[3]);
    draw_help(frame, app, chunks[4]);

    // Topic panel overlay.
    if let Some(ref panel) = app.topic_panel {
        draw_topic_panel(frame, panel, area);
    }
}

/// Compute log panel height: 0 if empty, otherwise up to 30% of terminal height
/// (min 4 lines including border).
fn log_panel_height(app: &App, terminal_height: u16) -> u16 {
    if app.trace_log.is_empty() {
        return 0;
    }
    let max_h = (terminal_height as f64 * 0.3).clamp(4.0, 15.0) as u16;
    // +2 for borders
    let needed = (app.trace_log.len() as u16 + 2).min(max_h);
    needed.max(4)
}

fn draw_header(frame: &mut Frame, app: &App, area: Rect) {
    let meta = app.deck.metadata();
    let dur = meta.duration;
    let secs = dur.as_secs();

    let iteration = app.deck.loop_iteration();
    let loop_label = match app.deck.loop_mode() {
        LoopMode::Off => "Once".to_string(),
        LoopMode::Restart if iteration > 0 => format!("Loop #{}", iteration + 1),
        LoopMode::Restart => "Loop".to_string(),
        LoopMode::Monotonic if iteration > 0 => format!("Loop+Mono #{}", iteration + 1),
        LoopMode::Monotonic => "Loop+Mono".to_string(),
    };

    let pub_label = if app.deck.is_publishing() { "Pub" } else { "" };

    let text = if pub_label.is_empty() {
        format!(
            " {}  |  {}  |  {:02}:{:02}:{:02}  |  {} messages  |  {} topics",
            meta.storage_identifier,
            loop_label,
            secs / 3600,
            (secs % 3600) / 60,
            secs % 60,
            meta.message_count,
            meta.topics.len(),
        )
    } else {
        format!(
            " {}  |  {}  |  {}  |  {:02}:{:02}:{:02}  |  {} messages  |  {} topics",
            meta.storage_identifier,
            loop_label,
            pub_label,
            secs / 3600,
            (secs % 3600) / 60,
            secs % 60,
            meta.message_count,
            meta.topics.len(),
        )
    };

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

fn draw_logs(frame: &mut Frame, app: &App, area: Rect) {
    if app.trace_log.is_empty() || area.height == 0 {
        return;
    }

    let block = Block::default()
        .title(" Logs ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::DarkGray));

    let inner_height = area.height.saturating_sub(2) as usize;
    let start = app.trace_log.len().saturating_sub(inner_height);

    let items: Vec<ListItem> = app
        .trace_log
        .iter()
        .skip(start)
        .map(|event| {
            let (prefix, color) = match event.level {
                Level::ERROR => ("ERROR", Color::Red),
                Level::WARN => ("WARN ", Color::Yellow),
                Level::INFO => ("INFO ", Color::Green),
                Level::DEBUG => ("DEBUG", Color::Blue),
                Level::TRACE => ("TRACE", Color::DarkGray),
            };
            ListItem::new(Line::from(vec![
                Span::styled(format!("{prefix} "), Style::default().fg(color)),
                Span::styled(&event.message, Style::default().fg(color)),
            ]))
        })
        .collect();

    let list = List::new(items).block(block);
    frame.render_widget(list, area);
}

fn draw_help(frame: &mut Frame, app: &App, area: Rect) {
    let text = if app.topic_panel.is_some() {
        "↑↓:Navigate  Space:Toggle  a:All  n:None  /:Search  Enter:Apply  Esc:Cancel".into()
    } else if let Some(ref input) = app.seek_input {
        format!("Seek to: {input}_ (Enter to confirm, Esc to cancel)")
    } else if let Some(ref msg) = app.status_message {
        msg.clone()
    } else {
        "Space:Play/Pause  s:Stop  ←→:Step  ±:Speed  g:Seek  o:Loop  p:Pub  Q:QoS  t:Topics  q:Quit"
            .into()
    };

    let paragraph = Paragraph::new(text).style(Style::default().fg(Color::DarkGray));
    frame.render_widget(paragraph, area);
}

fn draw_topic_panel(frame: &mut Frame, panel: &super::TopicPanel, area: Rect) {
    // Center the panel, taking up to 60% width and 80% height.
    let panel_w = (area.width as f64 * 0.6).clamp(30.0, 60.0) as u16;
    let panel_h = (area.height as f64 * 0.8).clamp(10.0, 40.0) as u16;
    let x = area.x + (area.width.saturating_sub(panel_w)) / 2;
    let y = area.y + (area.height.saturating_sub(panel_h)) / 2;
    let panel_area = Rect::new(x, y, panel_w.min(area.width), panel_h.min(area.height));

    frame.render_widget(Clear, panel_area);

    let visible = panel.visible_topics();

    let title = if let Some(ref search) = panel.search {
        format!(" Topics (/{search}) ")
    } else {
        format!(" Topics ({}/{}) ", panel.selected.len(), panel.topics.len())
    };

    let block = Block::default()
        .title(title)
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Yellow));

    let inner = block.inner(panel_area);

    // Scroll so cursor is visible.
    let inner_height = inner.height as usize;
    let scroll_offset = if panel.cursor >= inner_height {
        panel.cursor - inner_height + 1
    } else {
        0
    };

    let items: Vec<ListItem> = visible
        .iter()
        .enumerate()
        .skip(scroll_offset)
        .take(inner_height)
        .map(|(i, &topic)| {
            let check = if panel.selected.contains(topic) {
                "[x]"
            } else {
                "[ ]"
            };
            let style = if i == panel.cursor {
                Style::default().fg(Color::Black).bg(Color::Cyan)
            } else {
                Style::default()
            };
            ListItem::new(format!("{check} {topic}")).style(style)
        })
        .collect();

    let list = List::new(items).block(block);
    frame.render_widget(list, panel_area);
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
