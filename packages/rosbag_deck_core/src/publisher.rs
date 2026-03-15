use std::collections::HashMap;

use crate::{types::QosPreset, Result};

/// A single topic publisher that can publish serialized CDR data.
pub trait TopicPublisher: Send {
    fn publish(&self, data: &[u8]) -> Result<()>;
}

/// Backend that creates publishers and manages the ROS 2 node.
pub trait PublisherBackend: Send {
    /// Create a publisher for a topic.
    fn create_publisher(
        &mut self,
        topic: &str,
        type_name: &str,
        qos_depth: usize,
        reliable: bool,
    ) -> Result<Box<dyn TopicPublisher>>;

    /// Non-blocking spin for DDS discovery.
    fn spin_some(&self);

    /// Shut down the node and all publishers.
    fn shutdown(&mut self);
}

/// Manages per-topic publishers, lazily creating them on first message.
pub struct PublisherManager {
    backend: Box<dyn PublisherBackend>,
    publishers: HashMap<String, Box<dyn TopicPublisher>>,
    qos_depth: usize,
    qos_preset: QosPreset,
    enabled: bool,
}

impl PublisherManager {
    /// Create a new publisher manager with the given backend.
    pub fn new(
        backend: Box<dyn PublisherBackend>,
        qos_depth: usize,
        qos_preset: QosPreset,
    ) -> Self {
        Self {
            backend,
            publishers: HashMap::new(),
            qos_depth,
            qos_preset,
            enabled: true,
        }
    }

    /// Whether publishing is currently enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enable or disable publishing.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Number of publishers created so far.
    pub fn publisher_count(&self) -> usize {
        self.publishers.len()
    }

    /// Current QoS preset.
    pub fn qos_preset(&self) -> QosPreset {
        self.qos_preset
    }

    /// Change the QoS preset. Clears all publishers so they are
    /// lazily recreated with the new QoS on the next message.
    pub fn set_qos_preset(&mut self, preset: QosPreset) {
        if self.qos_preset != preset {
            self.qos_preset = preset;
            self.publishers.clear();
        }
    }

    /// Ensure a publisher exists for the topic and publish data.
    pub fn ensure_and_publish(&mut self, topic: &str, type_name: &str, data: &[u8]) {
        if !self.enabled {
            return;
        }

        if !self.publishers.contains_key(topic) {
            let reliable = self.qos_preset.is_reliable(type_name);
            match self
                .backend
                .create_publisher(topic, type_name, self.qos_depth, reliable)
            {
                Ok(pub_handle) => {
                    self.publishers.insert(topic.to_string(), pub_handle);
                }
                Err(e) => {
                    tracing::warn!(%topic, %e, "failed to create publisher");
                    return;
                }
            }
        }

        if let Some(publisher) = self.publishers.get(topic) {
            if let Err(e) = publisher.publish(data) {
                tracing::warn!(%topic, %e, "publish failed");
            }
        }
    }

    /// Non-blocking spin for DDS discovery.
    pub fn spin_some(&self) {
        self.backend.spin_some();
    }

    /// Shut down the backend.
    pub fn shutdown(&mut self) {
        self.publishers.clear();
        self.backend.shutdown();
    }
}
