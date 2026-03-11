// TODO: Port core library from C++ to Rust
//
// Key components to implement:
// - IndexManager: Build and query message indices from bag files
// - MessageCache: LRU cache for decoded messages
// - BagWorker: Background I/O thread for streaming messages
// - VirtualTimeline: Handle rewinds with segment tracking
// - MessageTypeRegistry: Dynamic message type discovery
