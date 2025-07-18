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
     *
     * @param cb The callback function to be called when a control command is received.
     */
    void subscribeRobotCmdForSim(std::function<void(const RobotCmdConstPtr &)> cb) override;

    /**
     * @brief Publish the robot state to the motion control algorithm.
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