#ifndef ROSBAG2_WRAPPER_H
#define ROSBAG2_WRAPPER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle to a rosbag2 reader. */
typedef struct Rosbag2Reader Rosbag2Reader;

/* Topic information returned from metadata queries. */
typedef struct {
    char *name;
    char *type_name;
    char *serialization_format;
    uint64_t message_count;
} Rosbag2TopicInfo;

/* Bag-level metadata. Caller must free with rosbag2_metadata_free(). */
typedef struct {
    Rosbag2TopicInfo *topics;
    size_t topic_count;
    uint64_t message_count;
    int64_t duration_ns;
    int64_t start_time_ns;
    int64_t end_time_ns;
    char *storage_identifier;
} Rosbag2Metadata;

/* A single serialized message. Caller must free with rosbag2_message_free(). */
typedef struct {
    int64_t timestamp_ns;
    char *topic;
    uint8_t *data;
    size_t data_len;
} Rosbag2Message;

/*
 * Open a bag file for reading.
 *
 * @param uri        Path to the bag directory.
 * @param storage_id Storage backend ("sqlite3", "mcap", or "" for auto-detect).
 * @return           Reader handle, or NULL on failure. Call rosbag2_last_error() for details.
 */
Rosbag2Reader *rosbag2_reader_open(const char *uri, const char *storage_id);

/* Close and free a reader. Safe to call with NULL. */
void rosbag2_reader_close(Rosbag2Reader *reader);

/* Returns true if more messages are available. */
bool rosbag2_reader_has_next(const Rosbag2Reader *reader);

/*
 * Read the next message. Allocates msg_out fields; caller must free with
 * rosbag2_message_free(). Returns 0 on success, -1 on error or end-of-bag.
 */
int rosbag2_reader_read_next(Rosbag2Reader *reader, Rosbag2Message *msg_out);

/* Free memory allocated by rosbag2_reader_read_next(). */
void rosbag2_message_free(Rosbag2Message *msg);

/* Seek to a timestamp (nanoseconds since epoch). Returns 0 on success, -1 on error. */
int rosbag2_reader_seek(Rosbag2Reader *reader, int64_t timestamp_ns);

/*
 * Get bag metadata. Allocates meta_out fields; caller must free with
 * rosbag2_metadata_free(). Returns 0 on success, -1 on error.
 */
int rosbag2_reader_get_metadata(const Rosbag2Reader *reader, Rosbag2Metadata *meta_out);

/* Free memory allocated by rosbag2_reader_get_metadata(). */
void rosbag2_metadata_free(Rosbag2Metadata *meta);

/*
 * Set a topic filter (whitelist). Only messages on these topics will be
 * returned by read_next(). Pass topic_count == 0 to clear the filter.
 * Returns 0 on success, -1 on error.
 */
int rosbag2_reader_set_filter(Rosbag2Reader *reader,
                              const char **topics, size_t topic_count);

/* Clear any topic filter. Returns 0 on success, -1 on error. */
int rosbag2_reader_reset_filter(Rosbag2Reader *reader);

/* Opaque handle to a rosbag2 writer. */
typedef struct Rosbag2Writer Rosbag2Writer;

/*
 * Open a bag file for writing.
 *
 * @param uri        Path to the output bag directory.
 * @param storage_id Storage backend ("sqlite3" or "mcap").
 * @return           Writer handle, or NULL on failure.
 */
Rosbag2Writer *rosbag2_writer_open(const char *uri, const char *storage_id);

/* Close and free a writer. Flushes any buffered data. Safe to call with NULL. */
void rosbag2_writer_close(Rosbag2Writer *writer);

/*
 * Create a topic in the output bag. Must be called before writing messages
 * on that topic. Returns 0 on success, -1 on error.
 */
int rosbag2_writer_create_topic(Rosbag2Writer *writer,
                                const char *name,
                                const char *type_name,
                                const char *serialization_format);

/*
 * Write a serialized message to the output bag.
 * The topic must have been created first via rosbag2_writer_create_topic().
 * Returns 0 on success, -1 on error.
 */
int rosbag2_writer_write(Rosbag2Writer *writer,
                         const char *topic,
                         int64_t timestamp_ns,
                         const uint8_t *data,
                         size_t data_len);

/* Returns the last error message, or NULL if no error. Thread-local. */
const char *rosbag2_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* ROSBAG2_WRAPPER_H */
