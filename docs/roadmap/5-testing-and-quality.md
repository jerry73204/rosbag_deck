# Phase 5: Testing & Quality

Comprehensive test coverage and CI/CD.

## 5.1 Unit Tests (per-crate `#[cfg(test)]`)

- [ ] Index manager: construction, multi-bag, time queries, boundary conditions
- [ ] Message cache: insert/retrieve, LRU eviction, thread safety, window eviction
- [ ] Bag worker: lifecycle, request processing, error handling, async delivery
- [ ] Virtual timeline: segment creation, rewind, loop, speed control
- [ ] Message type registry: registration, filtering, thread safety

## 5.2 Integration Tests (`rosbag-deck-tests` crate, nextest)

- [ ] End-to-end: load bag, build index, play back, verify message order
- [ ] CLI: `play` and `info` subcommands produce correct output
- [ ] Python: import, create Deck, iterate messages, verify callbacks
- [ ] Large bag handling (GB+ sizes, streaming without OOM)
- [ ] Storage format compatibility (sqlite3, mcap)

## 5.3 Performance Benchmarks

- [ ] Index build time for bags with 1M+ messages
- [ ] Cache hit/miss rates for various access patterns
- [ ] Message throughput (messages/sec)
- [ ] Python callback overhead

## 5.4 CI/CD

- [ ] GitHub Actions workflow: build, test, clippy, rustfmt
- [ ] Coverage reporting (cargo-llvm-cov or tarpaulin)
- [ ] Python test suite via pytest
- [ ] Platform matrix: Linux (Ubuntu 22.04/24.04)

## Criteria

- All tests pass on CI
- Line coverage > 80%
- No clippy warnings
- Benchmarks tracked over time
