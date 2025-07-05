/**
 * @file humanoid_sim.h
 *
 * @brief This file contains the declarations of classes related to the control of humanoid robots in simulation.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_HUMANOID_SIM_H_
#define _LIMX_SDK_HUMANOID_SIM_H_

#include <string>
#include <functional>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk
{
  /**
   * @class HumanoidSim
   * @brief Class for controlling humanoid robots in simulation.
   */
  class LIMX_SDK_API HumanoidSim : public ApiBase
  {
  public:
    /**
     * @brief Get an instance of the HumanoidSim class.
     * @return A pointer to a HumanoidSim instance (Singleton pattern).
     */
    static HumanoidSim *getInstance();

    /**
     * @brief Initialize the simulation with the given robot IP address.
     * @param robot_ip_address The IP address of the robot. Default is "127.0.0.1".
     * @return True if initialization is successful, false otherwise.
     */
    bool init(const std::string &robot_ip_address = "127.0.0.1") override;

    /**
     * @brief Subscribe to the robot control command.
     * The motor order for the commnd data is as follows:
     * Humanoid:
     *   0: left_hip_pitch_joint,   1: left_hip_roll_joint,   2: left_hip_yaw_joint  3: left_knee_joint  4: left_ankle_pitch_joint   5: left_ankle_roll_joint
     *   6: right_hip_pitch_joint,  7: right_hip_roll_joint,  8: right_hip_yaw_joint 9: right_knee_joint 10: right_ankle_pitch_joint 11: right_ankle_roll_joint
     *  12: waist_yaw_joint,       13: waist_roll_joint,     14: waist_pitch_joint,
     *  15: head_pitch_joint,      16: head_yaw_joint
     *  17: left_shoulder_pitch_joint,  18: left_shoulder_roll_joint,  19: left_shoulder_yaw_joint  20: left_elbow_joint
     *  21: left_hand_yaw_joint,  22: left_hand_roll_joint,  23: left_hand_pitch_joint
     *  24: right_shoulder_pitch_joint,  25: right_shoulder_roll_joint,  26: right_shoulder_yaw_joint,  27: right_elbow_joint
     *  28: right_hand_yaw_joint,  29: right_hand_roll_joint,  30: right_hand_pitch_joint
     *
     * @param cb The callback function to be called when a control command is received.
     */
    void subscribeRobotCmdForSim(std::function<void(const RobotCmdConstPtr &)> cb) override;

    /**
     * @brief Publish the robot state to the motion control algorithm.
     * The motor order for the state data is as follows:
     * Humanoid:
     *   0: left_hip_pitch_joint,   1: left_hip_roll_joint,   2: left_hip_yaw_joint  3: left_knee_joint  4: left_ankle_pitch_joint   5: left_ankle_roll_joint
     *   6: right_hip_pitch_joint,  7: right_hip_roll_joint,  8: right_hip_yaw_joint 9: right_knee_joint 10: right_ankle_pitch_joint 11: right_ankle_roll_joint
     *  12: waist_yaw_joint,       13: waist_roll_joint,     14: waist_pitch_joint,
     *  15: head_pitch_joint,      16: head_yaw_joint
     *  17: left_shoulder_pitch_joint,  18: left_shoulder_roll_joint,  19: left_shoulder_yaw_joint  20: left_elbow_joint
     *  21: left_hand_yaw_joint,  22: left_hand_roll_joint,  23: left_hand_pitch_joint
     *  24: right_shoulder_pitch_joint,  25: right_shoulder_roll_joint,  26: right_shoulder_yaw_joint,  27: right_elbow_joint
     *  28: right_hand_yaw_joint,  29: right_hand_roll_joint,  30: right_hand_pitch_joint
     *
     * @param state The robot state to be published.
     * @return True if publishing is successful, false otherwise.
     */
    bool publishRobotStateForSim(const RobotState &state) override;

    /**
     * @brief Publishes IMU (Inertial Measurement Unit) data to the motion control algorithm for simulation.
     * The order of IMU data is as follows:
     *        Accelerometer: imu.acc
     *        Gyroscope: imu.gyro
     *        Quaternion: imu.quat
     *
     * @param imu The IMU data to be published.
     * @return True if publishing is successful, false otherwise.
     */
    bool publishImuDataForSim(const ImuData &imu) override;

    /**
     * @brief Destructor for the HumanoidSim class.
     */
    virtual ~HumanoidSim();

  private:
    /**
     * @brief Constructor for the HumanoidSim class.
     */
    HumanoidSim();
  };
}

#endif