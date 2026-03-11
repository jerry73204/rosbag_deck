# Phase 2: Core Library

Port the core playback engine from C++ to Rust.

## Index Manager

- [ ] Build message index from bag metadata (timestamp, topic, offset)
- [ ] Support multi-bag indexing
- [ ] Provide efficient time-based lookups (binary search)
- [ ] Memory-efficient index (~100 bytes per message entry)

## Message Cache

- [ ] Implement LRU cache for decoded messages
- [ ] Thread-safe concurrent read/write
- [ ] Window-based eviction strategy for streaming playback
- [ ] Configurable cache size

## Bag Worker

- [ ] Background I/O thread for streaming messages from disk
- [ ] Request-based architecture (range reads, seek requests)
- [ ] Async message delivery via channels
- [ ] Graceful shutdown and error propagation

## Virtual Timeline

- [ ] Handle rewind operations by creating timeline segments
- [ ] Track segment IDs for frame disambiguation
- [ ] Support loop playback
- [ ] Playback speed control

## Message Type Registry

- [ ] Dynamic type discovery from bag metadata
- [ ] Topic and type filtering
- [ ] Thread-safe type registration

## Criteria

- Can load a bag file, build an index, and iterate messages in order
- Streaming playback with play/pause/stop/step controls
- Rewind creates new timeline segments correctly
- Cache hit rate > 90% for sequential playback
