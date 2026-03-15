#include "rosbag2_wrapper.h"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <rosidl_typesupport_introspection_c/field_types.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>

#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

/* Per-reader error message. */
struct Rosbag2Reader {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::BagMetadata metadata;
    bool metadata_loaded;
};

/* Thread-local error string. */
static thread_local std::string g_last_error;

static void set_error(const std::string &msg) { g_last_error = msg; }

static const char *safe_dlerror() {
    const char *err = dlerror();
    return err ? err : "unknown dl error";
}

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

int rosbag2_reader_set_filter(Rosbag2Reader *reader,
                              const char **topics, size_t topic_count) {
    if (!reader) {
        set_error("null reader");
        return -1;
    }
    try {
        rosbag2_storage::StorageFilter filter;
        for (size_t i = 0; i < topic_count; ++i) {
            if (topics[i]) {
                filter.topics.emplace_back(topics[i]);
            }
        }
        reader->reader.set_filter(filter);
        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("set_filter failed: ") + e.what());
        return -1;
    }
}

int rosbag2_reader_reset_filter(Rosbag2Reader *reader) {
    if (!reader) {
        set_error("null reader");
        return -1;
    }
    try {
        reader->reader.reset_filter();
        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("reset_filter failed: ") + e.what());
        return -1;
    }
}

/* ----- Writer API ----- */

struct Rosbag2Writer {
    rosbag2_cpp::Writer writer;
};

Rosbag2Writer *rosbag2_writer_open(const char *uri, const char *storage_id) {
    auto *w = new (std::nothrow) Rosbag2Writer();
    if (!w) {
        set_error("allocation failed");
        return nullptr;
    }

    try {
        rosbag2_storage::StorageOptions opts;
        opts.uri = uri;
        if (storage_id && storage_id[0] != '\0') {
            opts.storage_id = storage_id;
        }
        w->writer.open(opts);
    } catch (const std::exception &e) {
        set_error(std::string("failed to open writer: ") + e.what());
        delete w;
        return nullptr;
    }
    return w;
}

void rosbag2_writer_close(Rosbag2Writer *writer) {
    if (writer) {
        try {
            writer->writer.close();
        } catch (...) {
        }
        delete writer;
    }
}

int rosbag2_writer_create_topic(Rosbag2Writer *writer,
                                const char *name,
                                const char *type_name,
                                const char *serialization_format) {
    if (!writer || !name || !type_name || !serialization_format) {
        set_error("null argument");
        return -1;
    }
    try {
        rosbag2_storage::TopicMetadata topic;
        topic.name = name;
        topic.type = type_name;
        topic.serialization_format = serialization_format;
        writer->writer.create_topic(topic);
        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("create_topic failed: ") + e.what());
        return -1;
    }
}

int rosbag2_writer_write(Rosbag2Writer *writer,
                         const char *topic,
                         int64_t timestamp_ns,
                         const uint8_t *data,
                         size_t data_len) {
    if (!writer || !topic || (!data && data_len > 0)) {
        set_error("null argument");
        return -1;
    }
    try {
        auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        msg->topic_name = topic;
        msg->time_stamp = timestamp_ns;
        msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
        msg->serialized_data->buffer = const_cast<uint8_t *>(data);
        msg->serialized_data->buffer_length = data_len;
        msg->serialized_data->buffer_capacity = data_len;
        msg->serialized_data->allocator = rcutils_get_default_allocator();
        writer->writer.write(msg);

        // Prevent the shared_ptr from freeing our borrowed buffer.
        msg->serialized_data->buffer = nullptr;
        msg->serialized_data->buffer_length = 0;
        msg->serialized_data->buffer_capacity = 0;

        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("write failed: ") + e.what());
        return -1;
    }
}

/* ----- Type introspection ----- */

/* Cache for dlopen handles keyed by library name. */
static std::mutex g_dl_mutex;
static std::unordered_map<std::string, void *> g_dl_cache;

static void *cached_dlopen(const std::string &lib_name) {
    std::lock_guard<std::mutex> lock(g_dl_mutex);
    auto it = g_dl_cache.find(lib_name);
    if (it != g_dl_cache.end()) {
        return it->second;
    }
    void *handle = dlopen(lib_name.c_str(), RTLD_NOW | RTLD_GLOBAL);
    g_dl_cache[lib_name] = handle; // cache even if null
    return handle;
}

int rosbag2_type_has_header_first(const char *type_name) {
    if (!type_name) {
        set_error("null type_name");
        return -1;
    }

    try {
        // Parse "pkg/msg/Type" -> package="pkg", type="Type"
        std::string full(type_name);
        auto slash1 = full.find('/');
        if (slash1 == std::string::npos) {
            set_error("invalid type_name format, expected pkg/msg/Type");
            return -1;
        }
        auto slash2 = full.find('/', slash1 + 1);
        if (slash2 == std::string::npos) {
            set_error("invalid type_name format, expected pkg/msg/Type");
            return -1;
        }

        std::string package = full.substr(0, slash1);
        std::string msg_type = full.substr(slash2 + 1);

        // Build library name for introspection typesupport
        std::string lib_name = "lib" + package +
            "__rosidl_typesupport_introspection_c.so";

        void *handle = cached_dlopen(lib_name);
        if (!handle) {
            set_error("dlopen failed for " + lib_name + ": " + safe_dlerror());
            return -1;
        }

        // Build symbol name:
        // rosidl_typesupport_introspection_c__get_message_type_support_handle__pkg__msg__Type
        std::string symbol =
            "rosidl_typesupport_introspection_c__get_message_type_support_handle__" +
            package + "__msg__" + msg_type;

        using GetHandleFn = const rosidl_message_type_support_t *(*)();
        auto get_handle = reinterpret_cast<GetHandleFn>(dlsym(handle, symbol.c_str()));
        if (!get_handle) {
            set_error("dlsym failed for " + symbol + ": " + safe_dlerror());
            return -1;
        }

        const rosidl_message_type_support_t *ts = get_handle();
        if (!ts || !ts->data) {
            set_error("type support handle returned null");
            return -1;
        }

        auto *members = static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
            ts->data);
        if (!members || members->member_count_ == 0) {
            return 0;
        }

        if (!members->members_) {
            return 0;
        }

        const auto &first = members->members_[0];

        // Check that first field is a nested message type
        if (first.type_id_ != rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {
            return 0;
        }

        // Get the nested type's type support to check namespace/name.
        // first.members_ points to the rosidl_message_type_support_t for the nested type.
        if (!first.members_) {
            return 0;
        }
        const rosidl_message_type_support_t *nested_ts = first.members_;
        if (!nested_ts->data) {
            return 0;
        }

        auto *nested_members =
            static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
                nested_ts->data);
        if (!nested_members) {
            return 0;
        }

        // Check if nested type is std_msgs::msg::Header
        if (nested_members->message_namespace_ &&
            nested_members->message_name_) {
            std::string ns(nested_members->message_namespace_);
            std::string name(nested_members->message_name_);
            if (ns == "std_msgs__msg" && name == "Header") {
                return 1;
            }
        }

        return 0;
    } catch (const std::exception &e) {
        set_error(std::string("type_has_header_first failed: ") + e.what());
        return -1;
    } catch (...) {
        set_error("type_has_header_first failed: unknown exception");
        return -1;
    }
}

const char *rosbag2_last_error(void) {
    return g_last_error.empty() ? nullptr : g_last_error.c_str();
}

} /* extern "C" */
