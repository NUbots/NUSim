#ifndef K1SIM_MODULE_CAMERA_SHAREDIMAGEWRITER_HPP
#define K1SIM_MODULE_CAMERA_SHAREDIMAGEWRITER_HPP

#include <atomic>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <cstddef>
#include <cstdint>
#include <string>

namespace k1sim::module::camera {

    namespace bip = boost::interprocess;

    // Must match the reader's copy in NUbots_K1's module/input/K1Camera/src/K1Camera.cpp byte for
    // byte. There is no shared header, so change both together. The header comes first, so
    // `this + 1` is the first pixel byte.
    //
    // Lock-free seqlock (VERSION 2): the writer bumps seqlock odd before writing a frame and even
    // after (frame number = seqlock / 2); a reader snapshots it before and after copying and retries
    // if it changed or was odd. Nothing is locked, so killing either side (e.g. restarting behaviour)
    // leaves the segment usable.
    struct SharedImageHeader {
        // magic/version come first so a mismatched build is rejected, not misread. They're at the v1
        // offsets, so a v2 reader also rejects a stale v1 segment.
        static constexpr uint32_t MAGIC   = 0x4E42494D;  // "NBIM"
        static constexpr uint32_t VERSION = 2;
        uint32_t magic{MAGIC};
        uint32_t version{VERSION};
        // Seqlock: even = a complete frame is published; odd = a write is in progress.
        std::atomic<uint64_t> seqlock{0};
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

    // Owns the POSIX shared-memory segment input::K1Camera reads. The constructor recreates it for
    // width x height rgb8, removing any stale segment so seqlock starts at 0 and the size matches.
    // K1Camera retries opening it every 500 ms, so start order doesn't matter.
    //
    // publish() is single-writer: Camera calls it only from its render thread.
    class SharedImageWriter {
    public:
        // Throws std::runtime_error (width/height <= 0) or
        // boost::interprocess::interprocess_exception (shm create/map failure).
        SharedImageWriter(const std::string& segment_name, int width, int height);
        ~SharedImageWriter();

        SharedImageWriter(const SharedImageWriter&)            = delete;
        SharedImageWriter& operator=(const SharedImageWriter&) = delete;

        // `rgb` is width*height*3 bytes of top-down rgb8. Publishes under the seqlock (see
        // SharedImageHeader). No-op if rgb_size doesn't match the segment's data_size.
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
