/**
 * @file wheellegged_sim.h
 *
 * @brief This file contains the declarations of classes related to the control of wheeled-legged robots in simulation.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_WHEEL_LEGGED_SIM_H_
#define _LIMX_SDK_WHEEL_LEGGED_SIM_H_

#include <string>
#include <functional>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk
{
  /**
   * @class WheelleggedSim
   * @brief Class for controlling wheeled-legged robots in simulation.
   */
  class LIMX_SDK_API WheelleggedSim : public ApiBase
  {
  public:
    /**
     * @brief Get an instance of the WheelleggedSim class.
     * @return A pointer to a WheelleggedSim instance (Singleton pattern).
     */
    static WheelleggedSim *getInstance();

    /**
     * @brief Initialize the simulation with the given robot IP address.
     * @param robot_ip_address The IP address of the robot. Default is "127.0.0.1".
     * @return True if initialization is successful, false otherwise.
     */
    bool init(const std::string &robot_ip_address = "127.0.0.1") override;

    /**
     * @brief Get the number of motors in the robot.
     * @return The number of motors.
     */
    uint32_t getMotorNumber() override;

    /**
     * @brief Subscribe to the robot control command.
     * The motor order for the commnd data is as follows:
     *        0: LF_HAA,  1: LF_HFE,  2: LF_KFE,  3: LF_WHL
     *        4: LH_HAA,  5: LH_HFE,  6: LH_KFE,  7: LH_WHL
     *        8: RF_HAA,  9: RF_HFE,  10: RF_KFE, 11: RF_WH
     *        12: RH_HAA, 13: RH_HFE, 14: RH_KFE, 15: RH_WHL
     * @param cb The callback function to be called when a control command is received.
     */
    void subscribeRobotCmdForSim(std::function<void(const RobotCmdConstPtr &)> cb) override;

    /**
     * @brief Publish the robot state to the motion control algorithm.
     * The motor order for the state data is as follows:
     *        0: LF_HAA,  1: LF_HFE,  2: LF_KFE,  3: LF_WHL
     *        4: LH_HAA,  5: LH_HFE,  6: LH_KFE,  7: LH_WHL
     *        8: RF_HAA,  9: RF_HFE,  10: RF_KFE, 11: RF_WH
     *        12: RH_HAA, 13: RH_HFE, 14: RH_KFE, 15: RH_WHL
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
     * @brief Destructor for the WheelleggedSim class.
     */
    virtual ~WheelleggedSim();

  private:
    /**
     * @brief Constructor for the WheelleggedSim class.
     */
    WheelleggedSim();
  };
}

#endif