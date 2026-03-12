use std::collections::{HashMap, HashSet};

use crate::types::{BagMetadata, TopicInfo};

/// Topic-to-type registry with optional topic filtering.
///
/// Built from merged `BagMetadata` at open time. Used by the playback
/// loop to filter messages by topic.
pub struct MessageTypeRegistry {
    topics: HashMap<String, TopicInfo>,
    filter: Option<HashSet<String>>,
}

impl MessageTypeRegistry {
    /// Build from bag metadata. Merges topics across all bags.
    pub fn new(all_metadata: &[BagMetadata]) -> Self {
        let mut topics = HashMap::new();
        for meta in all_metadata {
            for topic in &meta.topics {
                topics
                    .entry(topic.name.clone())
                    .or_insert_with(|| topic.clone());
            }
        }
        Self {
            topics,
            filter: None,
        }
    }

    /// Check if a topic is accepted by the current filter.
    /// If no filter is set, all topics are accepted.
    pub fn is_accepted(&self, topic: &str) -> bool {
        match &self.filter {
            None => true,
            Some(allowed) => allowed.contains(topic),
        }
    }

    /// Set the topic filter. `None` means accept all topics.
    pub fn set_filter(&mut self, filter: Option<HashSet<String>>) {
        self.filter = filter;
    }

    /// Look up topic info by name.
    pub fn topic_info(&self, name: &str) -> Option<&TopicInfo> {
        self.topics.get(name)
    }

    /// All known topic names.
    pub fn topic_names(&self) -> impl Iterator<Item = &str> {
        self.topics.keys().map(|s| s.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn make_metadata_with_topics(topics: &[(&str, &str)]) -> BagMetadata {
        BagMetadata {
            topics: topics
                .iter()
                .map(|(name, type_name)| TopicInfo {
                    name: name.to_string(),
                    type_name: type_name.to_string(),
                    serialization_format: "cdr".to_string(),
                    message_count: 100,
                })
                .collect(),
            message_count: topics.len() as u64 * 100,
            duration: Duration::from_secs(10),
            start_time_ns: 0,
            end_time_ns: 10_000_000_000,
            storage_identifier: "sqlite3".to_string(),
        }
    }

    #[test]
    fn test_build_from_metadata() {
        let meta = make_metadata_with_topics(&[
            ("/camera", "sensor_msgs/Image"),
            ("/imu", "sensor_msgs/Imu"),
        ]);
        let reg = MessageTypeRegistry::new(&[meta]);

        assert!(reg.topic_info("/camera").is_some());
        assert!(reg.topic_info("/imu").is_some());
        assert!(reg.topic_info("/nonexistent").is_none());
    }

    #[test]
    fn test_filter_accepts_matching() {
        let meta = make_metadata_with_topics(&[
            ("/camera", "sensor_msgs/Image"),
            ("/imu", "sensor_msgs/Imu"),
        ]);
        let mut reg = MessageTypeRegistry::new(&[meta]);
        reg.set_filter(Some(HashSet::from(["/camera".to_string()])));

        assert!(reg.is_accepted("/camera"));
    }

    #[test]
    fn test_filter_rejects_non_matching() {
        let meta = make_metadata_with_topics(&[
            ("/camera", "sensor_msgs/Image"),
            ("/imu", "sensor_msgs/Imu"),
        ]);
        let mut reg = MessageTypeRegistry::new(&[meta]);
        reg.set_filter(Some(HashSet::from(["/camera".to_string()])));

        assert!(!reg.is_accepted("/imu"));
    }

    #[test]
    fn test_no_filter_accepts_all() {
        let meta = make_metadata_with_topics(&[
            ("/camera", "sensor_msgs/Image"),
            ("/imu", "sensor_msgs/Imu"),
        ]);
        let reg = MessageTypeRegistry::new(&[meta]);

        assert!(reg.is_accepted("/camera"));
        assert!(reg.is_accepted("/imu"));
        assert!(reg.is_accepted("/anything"));
    }
}
