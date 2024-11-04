/**
 * @file pointfoot_sim.h
 *
 * @brief This file contains the declarations of classes related to the control of pointfoot robots in simulation.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_POINTFOOT_SIM_H_
#define _LIMX_SDK_POINTFOOT_SIM_H_

#include <string>
#include <functional>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk {
  /**
   * @class PointFootSim
   * @brief Class for controlling pointfoot robots in simulation.
   */
  class LIMX_SDK_API PointFootSim : public ApiBase {
    public:
      /**
       * @brief Get an instance of the PointFootSim class.
       * @return A pointer to a PointFootSim instance (Singleton pattern).
       */
      static PointFootSim* getInstance();
      
      /**
       * @brief Initialize the simulation with the given robot IP address.
       * @param robot_ip_address The IP address of the robot. Default is "127.0.0.1".
       * @return True if initialization is successful, false otherwise.
       */
      bool init(const std::string& robot_ip_address = "127.0.0.1") override;

      /**
       * @brief Get the number of motors in the robot.
       * @return The number of motors.
       */
      uint32_t getMotorNumber() override;

      /**
       * @brief Subscribe to the robot control command.
       * The motor order for the commnd data is as follows:
       * PointFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint
       *   3: abad_R_Joint,  4: hip_R_Joint,  5: knee_R_Joint
       * BipedFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: ankle_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: ankle_R_Joint
       * WheelFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: wheel_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: wheel_R_Joint
       * 
       * @param cb The callback function to be called when a control command is received.
       */
      void subscribeRobotCmdForSim(std::function<void(const RobotCmdConstPtr&)> cb) override;

      /**
       * @brief Publish the robot state to the motion control algorithm.
       * The motor order for the state data is as follows:
       * PointFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint
       *   3: abad_R_Joint,  4: hip_R_Joint,  5: knee_R_Joint
       * BipedFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: ankle_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: ankle_R_Joint
       * WheelFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: wheel_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: wheel_R_Joint
       * 
       * @param state The robot state to be published.
       * @return True if publishing is successful, false otherwise.
       */
      bool publishRobotStateForSim(const RobotState& state) override;

      /**
       * @brief Subscribes to the robot's state for visualization purposes.
       * 
       * @param cb The callback function to handle the robot state data.
       */
      void subscribeRobotStateForVis(std::function<void(const RobotStateConstPtr&)> cb);

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
      bool publishImuDataForSim(const ImuData& imu) override;

      /**
       * @brief Destructor for the PointFootSim class.
       */
      virtual ~PointFootSim();

    private:
      /**
       * @brief Constructor for the PointFootSim class.
       */
      PointFootSim();
  };
}

#endif