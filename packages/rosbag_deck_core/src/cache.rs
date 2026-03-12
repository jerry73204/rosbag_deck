use std::{
    collections::BTreeMap,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Mutex,
    },
};

use crate::types::RawMessage;

/// Configuration for the message cache.
#[derive(Debug, Clone)]
pub struct CacheConfig {
    /// Maximum total bytes of cached message data (default 256 MB).
    pub max_bytes: usize,
    /// Number of messages to prefetch ahead of the cursor.
    pub prefetch_ahead: usize,
}

impl Default for CacheConfig {
    fn default() -> Self {
        Self {
            max_bytes: 256 * 1024 * 1024,
            prefetch_ahead: 1000,
        }
    }
}

/// Thread-safe message cache bounded by memory budget.
///
/// Messages are keyed by `(timestamp_ns, sub_index)` to support multiple
/// messages at the same nanosecond timestamp. When the cache exceeds its
/// memory budget, messages furthest from the cursor are evicted.
pub struct MessageCache {
    inner: Mutex<CacheInner>,
    current_bytes: AtomicUsize,
    max_bytes: usize,
    prefetch_ahead: usize,
}

struct CacheInner {
    entries: BTreeMap<(i64, u32), RawMessage>,
}

impl MessageCache {
    pub fn new(config: CacheConfig) -> Self {
        Self {
            inner: Mutex::new(CacheInner {
                entries: BTreeMap::new(),
            }),
            current_bytes: AtomicUsize::new(0),
            max_bytes: config.max_bytes,
            prefetch_ahead: config.prefetch_ahead,
        }
    }

    /// Retrieve the first message at a timestamp. Returns a clone.
    pub fn get(&self, timestamp_ns: i64) -> Option<RawMessage> {
        self.get_at(timestamp_ns, 0)
    }

    /// Retrieve the message at `(timestamp_ns, sub_index)`.
    pub fn get_at(&self, timestamp_ns: i64, sub: u32) -> Option<RawMessage> {
        let inner = self.inner.lock().unwrap();
        inner.entries.get(&(timestamp_ns, sub)).cloned()
    }

    /// Insert a message into the cache. Evicts if over budget.
    pub fn insert(&self, msg: RawMessage, cursor_ns: i64) {
        let msg_size = msg.data.len();
        let ts = msg.timestamp_ns;

        {
            let mut inner = self.inner.lock().unwrap();

            // Find next available sub-index at this timestamp.
            let sub = inner
                .entries
                .range((ts, 0)..=(ts, u32::MAX))
                .next_back()
                .map(|(&(_, s), _)| s + 1)
                .unwrap_or(0);

            inner.entries.insert((ts, sub), msg);
            self.current_bytes.fetch_add(msg_size, Ordering::Relaxed);

            if self.current_bytes.load(Ordering::Relaxed) > self.max_bytes {
                self.evict_locked(&mut inner, cursor_ns);
            }
        }
    }

    /// Clear all entries.
    pub fn clear(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.entries.clear();
        self.current_bytes.store(0, Ordering::Relaxed);
    }

    /// Current total bytes of cached data.
    pub fn current_bytes(&self) -> usize {
        self.current_bytes.load(Ordering::Relaxed)
    }

    /// Number of cached messages.
    pub fn len(&self) -> usize {
        self.inner.lock().unwrap().entries.len()
    }

    /// Whether the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Prefetch-ahead count.
    pub fn prefetch_ahead(&self) -> usize {
        self.prefetch_ahead
    }

    /// Returns the first (smallest) key in the cache, if any.
    pub fn first_key(&self) -> Option<(i64, u32)> {
        let inner = self.inner.lock().unwrap();
        inner.entries.keys().next().copied()
    }

    /// Returns the next cache key `(timestamp, sub)` after the given key.
    pub fn next_key_after(&self, ts: i64, sub: u32) -> Option<(i64, u32)> {
        let inner = self.inner.lock().unwrap();
        inner
            .entries
            .range((
                std::ops::Bound::Excluded((ts, sub)),
                std::ops::Bound::Unbounded,
            ))
            .next()
            .map(|(&k, _)| k)
    }

    /// Returns the next timestamp strictly after `cursor_ns` (any sub-index).
    pub fn next_timestamp_after(&self, cursor_ns: i64) -> Option<i64> {
        let inner = self.inner.lock().unwrap();
        inner
            .entries
            .range((
                std::ops::Bound::Excluded((cursor_ns, u32::MAX)),
                std::ops::Bound::Unbounded,
            ))
            .next()
            .map(|(&(ts, _), _)| ts)
    }

    /// Returns the previous cache key before `(ts, sub)`.
    pub fn prev_key_before(&self, ts: i64, sub: u32) -> Option<(i64, u32)> {
        let inner = self.inner.lock().unwrap();
        inner
            .entries
            .range(..(ts, sub))
            .next_back()
            .map(|(&k, _)| k)
    }

    /// Returns the previous timestamp before `cursor_ns` in the cache, if any.
    pub fn prev_timestamp_before(&self, cursor_ns: i64) -> Option<i64> {
        let inner = self.inner.lock().unwrap();
        inner
            .entries
            .range(..(cursor_ns, 0))
            .next_back()
            .map(|(&(ts, _), _)| ts)
    }

    /// Evict entries furthest from cursor until under budget.
    fn evict_locked(&self, inner: &mut CacheInner, cursor_ns: i64) {
        let target = self.max_bytes * 3 / 4;

        while self.current_bytes.load(Ordering::Relaxed) > target && !inner.entries.is_empty() {
            // Find the entry furthest from cursor.
            let &(first_ts, first_sub) = inner.entries.keys().next().unwrap();
            let &(last_ts, last_sub) = inner.entries.keys().next_back().unwrap();

            let dist_first = (first_ts - cursor_ns).unsigned_abs();
            let dist_last = (last_ts - cursor_ns).unsigned_abs();

            let remove_key = if dist_first >= dist_last {
                (first_ts, first_sub)
            } else {
                (last_ts, last_sub)
            };

            if let Some(removed) = inner.entries.remove(&remove_key) {
                self.current_bytes
                    .fetch_sub(removed.data.len(), Ordering::Relaxed);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_msg(ts: i64, size: usize) -> RawMessage {
        RawMessage {
            timestamp_ns: ts,
            topic: "/test".to_string(),
            data: vec![0u8; size],
        }
    }

    #[test]
    fn test_insert_and_get() {
        let cache = MessageCache::new(CacheConfig::default());
        let msg = make_msg(1000, 64);
        cache.insert(msg.clone(), 1000);

        let got = cache.get(1000).unwrap();
        assert_eq!(got.timestamp_ns, 1000);
        assert_eq!(got.data.len(), 64);
        assert!(cache.get(9999).is_none());
    }

    #[test]
    fn test_duplicate_timestamps() {
        let cache = MessageCache::new(CacheConfig::default());

        // Insert 3 messages at the same timestamp with different topics.
        for i in 0..3 {
            cache.insert(
                RawMessage {
                    timestamp_ns: 1000,
                    topic: format!("/topic_{i}"),
                    data: vec![i as u8; 32],
                },
                1000,
            );
        }

        assert_eq!(cache.len(), 3);
        assert_eq!(cache.get_at(1000, 0).unwrap().topic, "/topic_0");
        assert_eq!(cache.get_at(1000, 1).unwrap().topic, "/topic_1");
        assert_eq!(cache.get_at(1000, 2).unwrap().topic, "/topic_2");
    }

    #[test]
    fn test_next_key_after() {
        let cache = MessageCache::new(CacheConfig::default());
        cache.insert(make_msg(1000, 10), 0);
        cache.insert(make_msg(1000, 10), 0); // same ts, sub=1
        cache.insert(make_msg(2000, 10), 0);

        assert_eq!(cache.next_key_after(1000, 0), Some((1000, 1)));
        assert_eq!(cache.next_key_after(1000, 1), Some((2000, 0)));
        assert_eq!(cache.next_key_after(2000, 0), None);
    }

    #[test]
    fn test_memory_budget_eviction() {
        let config = CacheConfig {
            max_bytes: 500,
            prefetch_ahead: 10,
        };
        let cache = MessageCache::new(config);

        // Insert 6 messages of 100 bytes each = 600 bytes, exceeding 500 budget.
        for i in 0..6 {
            cache.insert(make_msg(i * 1000, 100), 3000); // cursor at 3000
        }

        assert!(cache.current_bytes() <= 500);
    }

    #[test]
    fn test_window_eviction_keeps_cursor_neighborhood() {
        let config = CacheConfig {
            max_bytes: 300,
            prefetch_ahead: 10,
        };
        let cache = MessageCache::new(config);

        let cursor = 5000;
        // Messages at 1000, 3000, 5000, 7000, 9000 — 100 bytes each = 500 total
        for ts in [1000, 3000, 5000, 7000, 9000] {
            cache.insert(make_msg(ts, 100), cursor);
        }

        // Cursor is at 5000, so 1000 and 9000 should be evicted first
        assert!(cache.get(5000).is_some());
        // Far entries should be gone
        assert!(cache.get(1000).is_none() || cache.get(9000).is_none());
    }

    #[test]
    fn test_distant_seek_clears_cache() {
        let cache = MessageCache::new(CacheConfig::default());
        for i in 0..10 {
            cache.insert(make_msg(i * 100, 50), 0);
        }
        assert!(!cache.is_empty());

        cache.clear();
        assert!(cache.is_empty());
        assert_eq!(cache.current_bytes(), 0);
    }

    #[test]
    fn test_concurrent_access() {
        use std::{sync::Arc, thread};

        let cache = Arc::new(MessageCache::new(CacheConfig {
            max_bytes: 10_000,
            prefetch_ahead: 10,
        }));

        let mut handles = vec![];

        // Writer threads
        for t in 0..4 {
            let cache = Arc::clone(&cache);
            handles.push(thread::spawn(move || {
                for i in 0..100 {
                    let ts = t * 1000 + i;
                    cache.insert(make_msg(ts, 10), 200);
                }
            }));
        }

        // Reader threads
        for _ in 0..4 {
            let cache = Arc::clone(&cache);
            handles.push(thread::spawn(move || {
                for ts in 0..400 {
                    let _ = cache.get(ts);
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        // Just verify no panic or corruption
        assert!(cache.len() > 0);
    }
}
