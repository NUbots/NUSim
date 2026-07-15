#ifndef K1SIM_MODULE_CAMERA_SHAREDPOSEWRITER_HPP
#define K1SIM_MODULE_CAMERA_SHAREDPOSEWRITER_HPP

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <cstdint>
#include <string>

namespace k1sim::module::camera {

namespace bip = boost::interprocess;

// Byte-for-byte match of NUbots' input::K1Sensors SharedPoseHeader ("NBPO",
// see module/input/K1Sensors/src/K1Sensors.cpp) — the head-pose segment
// NUbridge publishes on the real robot. K1Sensors folds this pose into
// Sensors.Htw, which is the only place torso tilt enters the NUbots side
// (odometry is yaw-only), so without this segment fall detection — and
// therefore the whole GetUp chain — never fires.
struct SharedPoseHeader {
    static constexpr uint32_t MAGIC   = 0x4E42504F;  // "NBPO"
    static constexpr uint32_t VERSION = 1;
    uint32_t magic{MAGIC};
    uint32_t version{VERSION};
    bip::interprocess_mutex mutex;
    bip::interprocess_condition has_new_data;
    uint64_t sequence{0};
    double position[3]{0.0, 0.0, 0.0};
    double orientation[4]{0.0, 0.0, 0.0, 1.0};  // xyzw
};

// Owns the POSIX shared-memory segment K1Sensors reads. Same lifecycle rules
// as SharedImageWriter: stale segments removed on construction, segment
// removed again on destruction, single-writer only.
class SharedPoseWriter {
public:
    explicit SharedPoseWriter(const std::string& segment_name);
    ~SharedPoseWriter();

    SharedPoseWriter(const SharedPoseWriter&)            = delete;
    SharedPoseWriter& operator=(const SharedPoseWriter&) = delete;

    // position: metres; orientation: quaternion xyzw. Frame contract matches
    // NUbridge: head (pitch-link) pose in the yaw-only base footprint frame,
    // i.e. Hrh = (translate(base_x, base_y, 0) * rotz(base_yaw))^-1 * Hwh.
    void publish(const double position[3], const double orientation_xyzw[4]);

private:
    std::string segment_name_;
    bip::shared_memory_object shm_;
    bip::mapped_region region_;
    SharedPoseHeader* header_ = nullptr;
};

}  // namespace k1sim::module::camera

#endif  // K1SIM_MODULE_CAMERA_SHAREDPOSEWRITER_HPP
