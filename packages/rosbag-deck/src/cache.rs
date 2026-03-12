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
/// Messages are keyed by timestamp (nanoseconds). When the cache exceeds
/// its memory budget, messages furthest from the cursor are evicted.
pub struct MessageCache {
    inner: Mutex<CacheInner>,
    current_bytes: AtomicUsize,
    max_bytes: usize,
    prefetch_ahead: usize,
}

struct CacheInner {
    entries: BTreeMap<i64, RawMessage>,
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

    /// Retrieve a message by timestamp. Returns a clone.
    pub fn get(&self, timestamp_ns: i64) -> Option<RawMessage> {
        let inner = self.inner.lock().unwrap();
        inner.entries.get(&timestamp_ns).cloned()
    }

    /// Insert a message into the cache. Evicts if over budget.
    pub fn insert(&self, msg: RawMessage, cursor_ns: i64) {
        let msg_size = msg.data.len();
        let ts = msg.timestamp_ns;

        {
            let mut inner = self.inner.lock().unwrap();
            if let Some(old) = inner.entries.insert(ts, msg) {
                // Replacing existing entry — adjust accounting.
                let old_size = old.data.len();
                if msg_size >= old_size {
                    self.current_bytes
                        .fetch_add(msg_size - old_size, Ordering::Relaxed);
                } else {
                    self.current_bytes
                        .fetch_sub(old_size - msg_size, Ordering::Relaxed);
                }
            } else {
                self.current_bytes.fetch_add(msg_size, Ordering::Relaxed);
            }

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

    /// Returns the next timestamp after `cursor_ns` in the cache, if any.
    pub fn next_timestamp_after(&self, cursor_ns: i64) -> Option<i64> {
        let inner = self.inner.lock().unwrap();
        inner
            .entries
            .range((
                std::ops::Bound::Excluded(cursor_ns),
                std::ops::Bound::Unbounded,
            ))
            .next()
            .map(|(&ts, _)| ts)
    }

    /// Returns the previous timestamp before `cursor_ns` in the cache, if any.
    pub fn prev_timestamp_before(&self, cursor_ns: i64) -> Option<i64> {
        let inner = self.inner.lock().unwrap();
        inner
            .entries
            .range(..cursor_ns)
            .next_back()
            .map(|(&ts, _)| ts)
    }

    /// Evict entries furthest from cursor until under budget.
    fn evict_locked(&self, inner: &mut CacheInner, cursor_ns: i64) {
        let target = self.max_bytes * 3 / 4;

        while self.current_bytes.load(Ordering::Relaxed) > target && !inner.entries.is_empty() {
            // Find the entry furthest from cursor.
            let (&first_ts, _) = inner.entries.iter().next().unwrap();
            let (&last_ts, _) = inner.entries.iter().next_back().unwrap();

            let dist_first = (first_ts - cursor_ns).unsigned_abs();
            let dist_last = (last_ts - cursor_ns).unsigned_abs();

            let remove_ts = if dist_first >= dist_last {
                first_ts
            } else {
                last_ts
            };

            if let Some(removed) = inner.entries.remove(&remove_ts) {
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
