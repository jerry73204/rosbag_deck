use std::{collections::BTreeMap, sync::Mutex};

use crate::types::BagMetadata;

/// A coarse-grained anchor point derived from bag metadata at open time.
/// Milestones are permanent and never evicted.
#[derive(Debug, Clone)]
pub struct Milestone {
    pub timestamp_ns: i64,
    pub bag_id: u16,
}

/// An entry in the visited index, recorded as messages flow through playback.
#[derive(Debug, Clone)]
pub struct VisitedEntry {
    pub timestamp_ns: i64,
    pub topic: String,
    pub bag_id: u16,
    pub data_size: u32,
}

/// Time range for a single bag.
#[derive(Debug, Clone)]
struct BagTimeRange {
    start_ns: i64,
    end_ns: i64,
    bag_id: u16,
}

/// Two-tier sparse index: permanent milestones + bounded visited entries.
///
/// Milestones are built from `BagMetadata` at open time (O(1), no I/O).
/// Visited entries are populated lazily during playback and evicted when
/// the configured limit is reached.
pub struct IndexManager {
    milestones: Vec<Milestone>,
    bag_time_ranges: Vec<BagTimeRange>,
    visited: Mutex<VisitedIndex>,
    visited_limit: usize,
}

struct VisitedIndex {
    entries: BTreeMap<i64, VisitedEntry>,
}

impl IndexManager {
    /// Build an `IndexManager` from bag metadata. O(1) — no message iteration.
    pub fn new(all_metadata: &[BagMetadata], milestone_count: usize, visited_limit: usize) -> Self {
        let mut milestones = Vec::new();
        let mut bag_time_ranges = Vec::new();

        for (bag_id, meta) in all_metadata.iter().enumerate() {
            let bag_id = bag_id as u16;
            bag_time_ranges.push(BagTimeRange {
                start_ns: meta.start_time_ns,
                end_ns: meta.end_time_ns,
                bag_id,
            });

            if milestone_count > 0 && meta.start_time_ns < meta.end_time_ns {
                let range = meta.end_time_ns - meta.start_time_ns;
                for i in 0..milestone_count {
                    let ts = meta.start_time_ns + (range * i as i64) / milestone_count as i64;
                    milestones.push(Milestone {
                        timestamp_ns: ts,
                        bag_id,
                    });
                }
            }
        }

        milestones.sort_by_key(|m| m.timestamp_ns);

        Self {
            milestones,
            bag_time_ranges,
            visited: Mutex::new(VisitedIndex {
                entries: BTreeMap::new(),
            }),
            visited_limit,
        }
    }

    /// Return all milestones.
    pub fn milestones(&self) -> &[Milestone] {
        &self.milestones
    }

    /// Return all bag IDs.
    pub fn all_bag_ids(&self) -> Vec<u16> {
        self.bag_time_ranges.iter().map(|r| r.bag_id).collect()
    }

    /// Return the bag IDs whose time range contains the given timestamp.
    pub fn bags_containing(&self, timestamp_ns: i64) -> Vec<u16> {
        self.bag_time_ranges
            .iter()
            .filter(|r| timestamp_ns >= r.start_ns && timestamp_ns <= r.end_ns)
            .map(|r| r.bag_id)
            .collect()
    }

    /// Look up an exact timestamp in the visited index.
    pub fn lookup(&self, timestamp_ns: i64) -> Option<VisitedEntry> {
        let visited = self.visited.lock().unwrap();
        visited.entries.get(&timestamp_ns).cloned()
    }

    /// Record a visited entry. Triggers eviction if over the limit.
    pub fn record(&self, entry: VisitedEntry, cursor_ns: i64) {
        let mut visited = self.visited.lock().unwrap();
        visited.entries.insert(entry.timestamp_ns, entry);

        if visited.entries.len() > self.visited_limit {
            Self::evict_far_from(&mut visited, cursor_ns, self.visited_limit);
        }
    }

    /// Number of entries in the visited index.
    pub fn visited_len(&self) -> usize {
        self.visited.lock().unwrap().entries.len()
    }

    /// Evict entries furthest from cursor until we're at 75% of the limit.
    fn evict_far_from(visited: &mut VisitedIndex, cursor_ns: i64, limit: usize) {
        let target = limit * 3 / 4;
        if visited.entries.len() <= target {
            return;
        }

        // Collect timestamps with their distance from cursor, sort by distance descending.
        let mut by_distance: Vec<(i64, i64)> = visited
            .entries
            .keys()
            .map(|&ts| (ts, (ts - cursor_ns).abs()))
            .collect();
        by_distance.sort_by(|a, b| b.1.cmp(&a.1));

        let to_remove = visited.entries.len() - target;
        for (ts, _) in by_distance.into_iter().take(to_remove) {
            visited.entries.remove(&ts);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn make_metadata(start_ns: i64, end_ns: i64) -> BagMetadata {
        BagMetadata {
            topics: vec![],
            message_count: 1000,
            duration: Duration::from_nanos((end_ns - start_ns) as u64),
            start_time_ns: start_ns,
            end_time_ns: end_ns,
            storage_identifier: "sqlite3".to_string(),
        }
    }

    #[test]
    fn test_milestones_from_single_bag() {
        let meta = make_metadata(1_000_000_000, 2_000_000_000);
        let idx = IndexManager::new(&[meta], 100, 1000);

        assert_eq!(idx.milestones().len(), 100);
        assert_eq!(
            idx.milestones().first().unwrap().timestamp_ns,
            1_000_000_000
        );
        // Last milestone is at start + range * 99/100
        let last = idx.milestones().last().unwrap();
        assert!(last.timestamp_ns < 2_000_000_000);
        // All belong to bag 0
        assert!(idx.milestones().iter().all(|m| m.bag_id == 0));
    }

    #[test]
    fn test_milestones_from_multi_bag() {
        let meta_a = make_metadata(1_000, 2_000);
        let meta_b = make_metadata(1_500, 3_000);
        let idx = IndexManager::new(&[meta_a, meta_b], 10, 1000);

        // 10 milestones per bag = 20 total
        assert_eq!(idx.milestones().len(), 20);

        // Overlapping region: timestamp 1500 should be in both bags
        let bags = idx.bags_containing(1_500);
        assert!(bags.contains(&0));
        assert!(bags.contains(&1));
    }

    #[test]
    fn test_visited_record_and_lookup() {
        let meta = make_metadata(0, 10_000);
        let idx = IndexManager::new(&[meta], 10, 1000);

        idx.record(
            VisitedEntry {
                timestamp_ns: 5000,
                topic: "/test".to_string(),
                bag_id: 0,
                data_size: 100,
            },
            5000,
        );

        let entry = idx.lookup(5000).unwrap();
        assert_eq!(entry.topic, "/test");
        assert!(idx.lookup(9999).is_none());
    }

    #[test]
    fn test_visited_eviction() {
        let meta = make_metadata(0, 100_000);
        let idx = IndexManager::new(&[meta], 10, 100); // limit of 100

        // Insert 101 entries, cursor at 50_000
        for i in 0..=100 {
            idx.record(
                VisitedEntry {
                    timestamp_ns: i * 1000,
                    topic: "/t".to_string(),
                    bag_id: 0,
                    data_size: 10,
                },
                50_000,
            );
        }

        // Should have been evicted down to 75
        assert_eq!(idx.visited_len(), 75);
    }

    #[test]
    fn test_visited_eviction_preserves_cursor_window() {
        let meta = make_metadata(0, 200_000);
        let idx = IndexManager::new(&[meta], 10, 100);

        let cursor_ns = 100_000;

        // Insert 101 entries spread across the range
        for i in 0..=100 {
            idx.record(
                VisitedEntry {
                    timestamp_ns: i * 2000,
                    topic: "/t".to_string(),
                    bag_id: 0,
                    data_size: 10,
                },
                cursor_ns,
            );
        }

        // Entries near cursor (100_000) should survive
        assert!(idx.lookup(100_000).is_some());
        // Entries far from cursor (0, 200_000) should be evicted
        assert!(idx.lookup(0).is_none());
    }

    #[test]
    fn test_bags_containing_non_overlapping() {
        let meta_a = make_metadata(0, 1000);
        let meta_b = make_metadata(2000, 3000);
        let idx = IndexManager::new(&[meta_a, meta_b], 10, 100);

        assert_eq!(idx.bags_containing(500), vec![0]);
        assert_eq!(idx.bags_containing(2500), vec![1]);
        assert!(idx.bags_containing(1500).is_empty());
    }

    #[test]
    fn test_bags_containing_overlapping() {
        let meta_a = make_metadata(0, 2000);
        let meta_b = make_metadata(1000, 3000);
        let idx = IndexManager::new(&[meta_a, meta_b], 10, 100);

        let bags = idx.bags_containing(1500);
        assert_eq!(bags.len(), 2);
        assert!(bags.contains(&0));
        assert!(bags.contains(&1));
    }
}
