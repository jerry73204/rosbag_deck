//! A tracing subscriber that captures log events into a channel
//! for display in the TUI, instead of writing to stderr.

use std::sync::mpsc;

use tracing::{field::Visit, Level, Subscriber};
use tracing_subscriber::{
    layer::{Context, SubscriberExt},
    registry::LookupSpan,
    util::SubscriberInitExt,
    EnvFilter, Layer,
};

/// A captured log entry with its severity level.
pub struct LogEvent {
    pub level: Level,
    pub message: String,
}

/// A tracing Layer that sends events to a channel.
struct ChannelLayer {
    tx: mpsc::Sender<LogEvent>,
}

impl<S: Subscriber + for<'a> LookupSpan<'a>> Layer<S> for ChannelLayer {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, S>) {
        let level = *event.metadata().level();
        let target = event.metadata().target();

        let mut visitor = MessageVisitor::default();
        event.record(&mut visitor);

        let message = if let Some(msg) = visitor.message {
            if visitor.fields.is_empty() {
                format!("{target}: {msg}")
            } else {
                format!("{target}: {msg} {}", visitor.fields.join(" "))
            }
        } else if !visitor.fields.is_empty() {
            format!("{target}: {}", visitor.fields.join(" "))
        } else {
            target.to_string()
        };

        let _ = self.tx.send(LogEvent { level, message });
    }
}

/// Extracts the message and key=value fields from a tracing event.
#[derive(Default)]
struct MessageVisitor {
    message: Option<String>,
    fields: Vec<String>,
}

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.message = Some(format!("{value:?}"));
        } else {
            self.fields.push(format!("{}={:?}", field.name(), value));
        }
    }

    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        if field.name() == "message" {
            self.message = Some(value.to_string());
        } else {
            self.fields.push(format!("{}={}", field.name(), value));
        }
    }
}

/// Install a tracing subscriber that sends structured events to a channel.
/// Returns the receiver for draining in the TUI tick loop.
pub fn init() -> mpsc::Receiver<LogEvent> {
    let (tx, rx) = mpsc::channel();

    let layer = ChannelLayer { tx };

    tracing_subscriber::registry()
        .with(EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")))
        .with(layer)
        .init();

    rx
}
