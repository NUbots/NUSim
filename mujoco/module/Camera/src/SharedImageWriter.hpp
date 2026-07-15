#ifndef K1SIM_MODULE_CAMERA_SHAREDIMAGEWRITER_HPP
#define K1SIM_MODULE_CAMERA_SHAREDIMAGEWRITER_HPP

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <cstddef>
#include <cstdint>
#include <string>

namespace k1sim::module::camera {

namespace bip = boost::interprocess;

// Byte-for-byte identical to the writer-side contract documented in NUbots_K1's
// module/input/K1Camera/src/K1Camera.cpp ("SharedImageHeader ... Must match the
// writer-side layout in NUbridge exactly"). That struct lives in a different
// repo/build -- there is no shared header to #include, so this copy *is* the
// contract. If you change one, change the other; field order/types must match
// exactly (mutex + condition first, so `this + 1` is the first pixel byte).
struct SharedImageHeader {
    // magic/version must lead the struct — RoboCup 2026 NUbridge added them and
    // K1Camera's layout has them before the mutex; without them the reader's
    // mutex offset lands 8 bytes into ours (glibc owner assertion on first lock).
    static constexpr uint32_t MAGIC   = 0x4E42494D;  // "NBIM"
    static constexpr uint32_t VERSION = 1;
    uint32_t magic{MAGIC};
    uint32_t version{VERSION};
    bip::interprocess_mutex mutex;
    bip::interprocess_condition has_new_frame;
    uint64_t sequence{0};
    uint32_t data_size{0};
    uint32_t width{0};
    uint32_t height{0};
    char encoding[32]{};
    float focal_length{0.0f};
    float fov{0.0f};
    float centre_x{0.0f};
    float centre_y{0.0f};
    float k1{0.0f};
    float k2{0.0f};
};

// Owns the POSIX shared-memory segment NUbots' input::K1Camera reads. On
// construction, (re)creates the segment sized for width x height rgb8 -- any
// stale segment from a previous run is removed first, so we never
// placement-new a mutex/condition on top of memory a stale reader might still
// reference, and the size always matches the current config. The segment (and
// the in-place-constructed header) exists as soon as this constructor returns,
// independent of whether the MuJoCo model has loaded yet -- K1Camera retries
// opening it every 500 ms until it appears, so creation order with NUbots
// doesn't matter.
//
// Not thread-safe against concurrent publish() calls; Camera only ever calls
// it from its one dedicated render thread.
class SharedImageWriter {
public:
    // Throws std::runtime_error (width/height <= 0) or
    // boost::interprocess::interprocess_exception (shm create/map failure).
    SharedImageWriter(const std::string& segment_name, int width, int height);
    ~SharedImageWriter();

    SharedImageWriter(const SharedImageWriter&)            = delete;
    SharedImageWriter& operator=(const SharedImageWriter&) = delete;

    // `rgb` must be exactly width*height*3 bytes (rgb8), top-down row order (row 0
    // = top of image) -- matches ros_encoding_to_fourcc("rgb8") on the reader
    // side. Locks the header mutex, copies the pixels + intrinsics fields,
    // increments `sequence`, unlocks, then notifies has_new_frame. No-op
    // (defensive only -- rgb_size is always frame_bytes by construction on the
    // one call site) if rgb_size doesn't match the segment's data_size.
    void publish(const uint8_t* rgb,
                 std::size_t rgb_size,
                 float focal_length,
                 float fov,
                 float centre_x,
                 float centre_y);

private:
    std::string segment_name_;
    bip::shared_memory_object shm_;
    bip::mapped_region region_;
    SharedImageHeader* header_ = nullptr;  // = region_.get_address(), once mapped
    uint8_t* pixels_           = nullptr;  // = header_ + 1
};

}  // namespace k1sim::module::camera

#endif  // K1SIM_MODULE_CAMERA_SHAREDIMAGEWRITER_HPP
