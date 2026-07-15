#include "module/Camera/src/SharedPoseWriter.hpp"

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <new>

namespace k1sim::module::camera {

SharedPoseWriter::SharedPoseWriter(const std::string& segment_name) : segment_name_(segment_name) {
    // Same rationale as SharedImageWriter: never in-place-construct over a
    // stale segment a crashed run's reader might still reference.
    bip::shared_memory_object::remove(segment_name_.c_str());

    shm_ = bip::shared_memory_object(bip::create_only, segment_name_.c_str(), bip::read_write);
    shm_.truncate(static_cast<bip::offset_t>(sizeof(SharedPoseHeader)));
    region_ = bip::mapped_region(shm_, bip::read_write);

    header_ = new (region_.get_address()) SharedPoseHeader();
}

SharedPoseWriter::~SharedPoseWriter() {
    if (header_ != nullptr) {
        header_->~SharedPoseHeader();
    }
    bip::shared_memory_object::remove(segment_name_.c_str());
}

void SharedPoseWriter::publish(const double position[3], const double orientation_xyzw[4]) {
    if (header_ == nullptr) {
        return;
    }
    {
        bip::scoped_lock<bip::interprocess_mutex> lock(header_->mutex);
        for (int i = 0; i < 3; ++i) {
            header_->position[i] = position[i];
        }
        for (int i = 0; i < 4; ++i) {
            header_->orientation[i] = orientation_xyzw[i];
        }
        ++header_->sequence;
    }
    header_->has_new_data.notify_all();
}

}  // namespace k1sim::module::camera
