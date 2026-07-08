#ifndef K1SIM_SHARED_SIM_MODELMAP_HPP
#define K1SIM_SHARED_SIM_MODELMAP_HPP

#include <array>
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>

#include "shared/k1/JointIndex.hpp"

namespace k1sim {

// Resolves the frozen JointIndexK1 ordering against a loaded mjModel:
// joint qpos/dof addresses and actuator ids per joint, plus the root free joint
// and IMU sensor addresses. Built once after model load; immutable afterwards.
struct ModelMap {
    std::array<int, JOINT_COUNT> qpos_adr{};  // d->qpos index of each joint
    std::array<int, JOINT_COUNT> dof_adr{};   // d->qvel index of each joint
    std::array<int, JOINT_COUNT> act_id{};    // actuator id of each joint

    int root_qpos_adr = -1;  // 7 entries: xyz + wxyz quat (free joint)
    int root_dof_adr  = -1;  // 6 entries: linear + angular velocity
    int root_body_id  = -1;

    int imu_site_id = -1;
    // sensordata start addresses (-1 if the sensor is absent)
    int sens_quat = -1, sens_gyro = -1, sens_acc = -1, sens_linvel = -1;

    static ModelMap build(const mjModel* m) {
        ModelMap map;
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            const int jnt = mj_name2id(m, mjOBJ_JOINT, JOINT_NAMES[i]);
            const int act = mj_name2id(m, mjOBJ_ACTUATOR, JOINT_NAMES[i]);
            if (jnt < 0 || act < 0) {
                throw std::runtime_error(std::string("model is missing joint/actuator '") + JOINT_NAMES[i] + "'");
            }
            map.qpos_adr[i] = m->jnt_qposadr[jnt];
            map.dof_adr[i]  = m->jnt_dofadr[jnt];
            map.act_id[i]   = act;
        }

        for (int j = 0; j < m->njnt; ++j) {
            if (m->jnt_type[j] == mjJNT_FREE) {
                const int body = m->jnt_bodyid[j];
                // the robot's root free joint, not the ball's: it owns the imu site's body chain
                if (map.root_qpos_adr < 0 || m->body_subtreemass[body] > m->body_subtreemass[map.root_body_id]) {
                    map.root_qpos_adr = m->jnt_qposadr[j];
                    map.root_dof_adr  = m->jnt_dofadr[j];
                    map.root_body_id  = body;
                }
            }
        }
        if (map.root_qpos_adr < 0) {
            throw std::runtime_error("model has no free root joint");
        }

        map.imu_site_id = mj_name2id(m, mjOBJ_SITE, "imu");

        auto sensor_adr = [m](const char* name) {
            const int id = mj_name2id(m, mjOBJ_SENSOR, name);
            return id < 0 ? -1 : m->sensor_adr[id];
        };
        map.sens_quat   = sensor_adr("orientation");
        map.sens_gyro   = sensor_adr("angular-velocity");
        map.sens_acc    = sensor_adr("acceleration");
        map.sens_linvel = sensor_adr("linear-velocity");

        return map;
    }
};

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_MODELMAP_HPP
