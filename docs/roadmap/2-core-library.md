# Phase 2: Core Library

Build the playback engine on top of the `BagReader` trait from Phase 1.

## 2.1 Index Manager

Consumes `BagReader` to build a searchable index of all messages.

- [ ] Build message index from `BagMetadata` and message iteration (timestamp, topic, offset)
- [ ] Support multi-bag indexing (merge-sort by timestamp)
- [ ] Provide efficient time-based lookups (binary search)
- [ ] Memory-efficient index (~100 bytes per message entry)

## 2.2 Message Cache

Pure in-memory component, no FFI dependency.

- [ ] Implement LRU cache for decoded messages
- [ ] Thread-safe concurrent read/write
- [ ] Window-based eviction strategy for streaming playback
- [ ] Configurable cache size

## 2.3 Bag Worker

Background thread that uses `BagReader` to stream messages into the cache.

- [ ] Background I/O thread for streaming messages from disk
- [ ] Request-based architecture (range reads, seek requests)
- [ ] Async message delivery via channels
- [ ] Graceful shutdown and error propagation

## 2.4 Virtual Timeline

Pure playback logic, no FFI dependency.

- [ ] Handle rewind operations by creating timeline segments
- [ ] Track segment IDs for frame disambiguation
- [ ] Support loop playback
- [ ] Playback speed control

## 2.5 Message Type Registry

Populated from `BagMetadata::topics`, then used for filtering.

- [ ] Dynamic type discovery from bag metadata
- [ ] Topic and type filtering
- [ ] Thread-safe type registration

## Criteria

- Can load a bag file, build an index, and iterate messages in order
- Streaming playback with play/pause/stop/step controls
- Rewind creates new timeline segments correctly
- Cache hit rate > 90% for sequential playback
