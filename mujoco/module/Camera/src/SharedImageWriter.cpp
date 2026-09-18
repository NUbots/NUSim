#include "module/Camera/src/SharedImageWriter.hpp"

#include <atomic>
#include <cstring>
#include <new>
#include <stdexcept>

namespace k1sim::module::camera {

    namespace {
        constexpr const char* kEncoding = "rgb8";
    }

    SharedImageWriter::SharedImageWriter(const std::string& segment_name, int width, int height)
        : segment_name_(segment_name) {

        if (width <= 0 || height <= 0) {
            throw std::runtime_error("SharedImageWriter: width/height must be positive");
        }

        const auto data_size  = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3;
        const auto total_size = sizeof(SharedImageHeader) + data_size;

        // Discard any stale segment (e.g. left behind by a crashed previous run) so we
        // never in-place-construct a mutex/condition over memory a stuck waiter might
        // still reference, and so size always matches width/height. remove() is a
        // no-op (returns false, doesn't throw) if nothing was there.
        bip::shared_memory_object::remove(segment_name_.c_str());

        shm_ = bip::shared_memory_object(bip::create_only, segment_name_.c_str(), bip::read_write);
        shm_.truncate(static_cast<bip::offset_t>(total_size));
        region_ = bip::mapped_region(shm_, bip::read_write);

        // Placement-new the header on the freshly-mapped memory so its members (incl.
        // the seqlock atomic) are initialised exactly once (seqlock = 0 = "no frame yet").
        header_            = new (region_.get_address()) SharedImageHeader();
        header_->width     = static_cast<uint32_t>(width);
        header_->height    = static_cast<uint32_t>(height);
        header_->data_size = static_cast<uint32_t>(data_size);
        // encoding[] is already zero-filled by SharedImageHeader's default member
        // initializer; just copy the (shorter, NUL-terminated-by-the-zero-fill) tag in.
        std::memcpy(header_->encoding, kEncoding, std::strlen(kEncoding));

        pixels_ = reinterpret_cast<uint8_t*>(header_ + 1);
    }

    SharedImageWriter::~SharedImageWriter() {
        if (header_ != nullptr) {
            header_->~SharedImageHeader();
        }
        bip::shared_memory_object::remove(segment_name_.c_str());
    }

    void SharedImageWriter::publish(const uint8_t* rgb,
                                    std::size_t rgb_size,
                                    float focal_length,
                                    float fov,
                                    float centre_x,
                                    float centre_y) {
        if (header_ == nullptr || rgb_size != header_->data_size) {
            return;
        }
        // Seqlock publish (single writer). Bump odd = "write in progress", write the
        // frame + intrinsics, bump even = "frame N published" (N = seqlock/2). A reader
        // that snapshots seqlock before and after its copy sees an odd or changed value
        // if it raced this write, and retries. No lock is ever held, so a reader that
        // dies mid-read cannot wedge this writer.
        const uint64_t s = header_->seqlock.load(std::memory_order_relaxed);
        header_->seqlock.store(s + 1, std::memory_order_release);  // odd: write in progress
        std::atomic_thread_fence(std::memory_order_release);

        std::memcpy(pixels_, rgb, rgb_size);
        header_->focal_length = focal_length;
        header_->fov          = fov;
        header_->centre_x     = centre_x;
        header_->centre_y     = centre_y;
        header_->k1           = 0.0f;
        header_->k2           = 0.0f;

        std::atomic_thread_fence(std::memory_order_release);
        header_->seqlock.store(s + 2, std::memory_order_release);  // even: frame published
    }

}  // namespace k1sim::module::camera
