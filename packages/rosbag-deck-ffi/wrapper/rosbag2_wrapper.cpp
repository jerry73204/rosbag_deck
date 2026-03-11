#include "rosbag2_wrapper.h"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

/* Per-reader error message. */
struct Rosbag2Reader {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::BagMetadata metadata;
    bool metadata_loaded;
};

/* Thread-local error string. */
static thread_local std::string g_last_error;

static void set_error(const std::string &msg) { g_last_error = msg; }

static char *strdup_safe(const std::string &s) { return strdup(s.c_str()); }

extern "C" {

Rosbag2Reader *rosbag2_reader_open(const char *uri, const char *storage_id) {
    auto *r = new (std::nothrow) Rosbag2Reader();
    if (!r) {
        set_error("allocation failed");
        return nullptr;
    }
    r->metadata_loaded = false;

    try {
        rosbag2_storage::StorageOptions opts;
        opts.uri = uri;
        if (storage_id && storage_id[0] != '\0') {
            opts.storage_id = storage_id;
        }
        r->reader.open(opts);
        r->metadata = r->reader.get_metadata();
        r->metadata_loaded = true;
    } catch (const std::exception &e) {
        set_error(std::string("failed to open bag: ") + e.what());
        delete r;
        return nullptr;
    }
    return r;
}

void rosbag2_reader_close(Rosbag2Reader *reader) {
    if (reader) {
        try {
            reader->reader.close();
        } catch (...) {
        }
        delete reader;
    }
}

bool rosbag2_reader_has_next(const Rosbag2Reader *reader) {
    if (!reader) return false;
    try {
        return const_cast<rosbag2_cpp::Reader &>(reader->reader).has_next();
    } catch (...) {
        return false;
    }
}

int rosbag2_reader_read_next(Rosbag2Reader *reader, Rosbag2Message *msg_out) {
    if (!reader || !msg_out) {
        set_error("null argument");
        return -1;
    }

    try {
        if (!reader->reader.has_next()) {
            return -1;
        }
        auto msg = reader->reader.read_next();

        msg_out->timestamp_ns = msg->time_stamp;
        msg_out->topic = strdup_safe(msg->topic_name);

        auto &buf = msg->serialized_data;
        msg_out->data_len = buf->buffer_length;
        msg_out->data = static_cast<uint8_t *>(malloc(buf->buffer_length));
        if (!msg_out->data) {
            free(msg_out->topic);
            set_error("malloc failed for message data");
            return -1;
        }
        memcpy(msg_out->data, buf->buffer, buf->buffer_length);

        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("read_next failed: ") + e.what());
        return -1;
    }
}

void rosbag2_message_free(Rosbag2Message *msg) {
    if (msg) {
        free(msg->topic);
        free(msg->data);
        msg->topic = nullptr;
        msg->data = nullptr;
        msg->data_len = 0;
    }
}

int rosbag2_reader_seek(Rosbag2Reader *reader, int64_t timestamp_ns) {
    if (!reader) {
        set_error("null reader");
        return -1;
    }
    try {
        reader->reader.seek(timestamp_ns);
        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("seek failed: ") + e.what());
        return -1;
    }
}

int rosbag2_reader_get_metadata(const Rosbag2Reader *reader,
                                Rosbag2Metadata *meta_out) {
    if (!reader || !meta_out) {
        set_error("null argument");
        return -1;
    }
    if (!reader->metadata_loaded) {
        set_error("metadata not loaded");
        return -1;
    }

    const auto &meta = reader->metadata;

    meta_out->message_count = meta.message_count;
    meta_out->duration_ns = meta.duration.count();
    meta_out->start_time_ns =
        meta.starting_time.time_since_epoch().count();
    meta_out->end_time_ns =
        meta_out->start_time_ns + meta_out->duration_ns;
    meta_out->storage_identifier = strdup_safe(meta.storage_identifier);

    size_t n = meta.topics_with_message_count.size();
    meta_out->topic_count = n;
    meta_out->topics =
        static_cast<Rosbag2TopicInfo *>(calloc(n, sizeof(Rosbag2TopicInfo)));
    if (!meta_out->topics && n > 0) {
        free(meta_out->storage_identifier);
        set_error("calloc failed for topics");
        return -1;
    }

    for (size_t i = 0; i < n; ++i) {
        const auto &ti = meta.topics_with_message_count[i];
        meta_out->topics[i].name = strdup_safe(ti.topic_metadata.name);
        meta_out->topics[i].type_name = strdup_safe(ti.topic_metadata.type);
        meta_out->topics[i].serialization_format =
            strdup_safe(ti.topic_metadata.serialization_format);
        meta_out->topics[i].message_count = ti.message_count;
    }

    return 0;
}

void rosbag2_metadata_free(Rosbag2Metadata *meta) {
    if (!meta) return;
    for (size_t i = 0; i < meta->topic_count; ++i) {
        free(meta->topics[i].name);
        free(meta->topics[i].type_name);
        free(meta->topics[i].serialization_format);
    }
    free(meta->topics);
    free(meta->storage_identifier);
    meta->topics = nullptr;
    meta->storage_identifier = nullptr;
    meta->topic_count = 0;
}

const char *rosbag2_last_error(void) {
    return g_last_error.empty() ? nullptr : g_last_error.c_str();
}

} /* extern "C" */
