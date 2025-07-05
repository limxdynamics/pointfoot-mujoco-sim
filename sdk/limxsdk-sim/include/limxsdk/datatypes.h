/**
 * @file datatypes.h
 *
 * @brief This file contains the declarations of classes and structures related to robotics.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_DATATYPES_H_
#define _LIMX_SDK_DATATYPES_H_

#include <stdint.h>
#include <vector>
#include <memory>
#include <string>

namespace limxsdk
{
  /**
   * @struct ImuData
   *
   * @brief Structure representing IMU (Inertial Measurement Unit) data of a robot based on sensor feedback.
   *
   * This structure encapsulates IMU data including accelerometer, gyroscope, and quaternion.
   */
  struct ImuData
  {
    ImuData()
    {
      for (int i = 0; i < 3; i++)
      {
        acc[i] = 0.0;
      }
      for (int i = 0; i < 3; i++)
      {
        gyro[i] = 0.0;
      }
      for (int i = 0; i < 4; i++)
      {
        quat[i] = 0.0;
      }
    }
    uint64_t stamp; // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    float acc[3];   // Array to store IMU accelerometer data for tracking linear acceleration along three axes (X, Y, Z).
    float gyro[3];  // Array to store IMU gyroscope data for tracking angular velocity or rotational speed along three axes (X, Y, Z).
    float quat[4];  // Array to store IMU quaternion data, representing orientation in 3D space (w, x, y, z).
  };
  typedef std::shared_ptr<ImuData> ImuDataPtr;
  typedef std::shared_ptr<ImuData const> ImuDataConstPtr;

  /**
   * @struct RobotState
   *
   * @brief Structure representing the state of a robot based on sensor feedback.
   *
   * This structure encapsulates various data points that could be used to monitor and control a robot, including output torque, current angles, and velocities.
   */
  struct RobotState
  {
    RobotState() {}
    RobotState(int motor_num)
        : tau(motor_num, 0.0), q(motor_num, 0.0), dq(motor_num, 0.0), motor_names(motor_num, "") {}

    void resize(int motor_num)
    {
      tau.resize(motor_num, 0.0);
      q.resize(motor_num, 0.0);
      dq.resize(motor_num, 0.0);
      motor_names.resize(motor_num, "");
    }

    uint64_t stamp;                       // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    std::vector<float> tau;               // Vector to store the current estimated output torque (in Newton meters)
    std::vector<float> q;                 // Vector to store the current angles (in radians)
    std::vector<float> dq;                // Vector to store the current velocities (in radians per second)
    std::vector<std::string> motor_names; // Vector storing the names of the motors
  };
  typedef std::shared_ptr<RobotState> RobotStatePtr;
  typedef std::shared_ptr<RobotState const> RobotStateConstPtr;

  /**
   * @struct RobotCmd
   *
   * @brief Structure representing the command for controlling a robot.
   *
   * This structure holds various commands that can be used to control a robot, including the desired working mode, desired angles, desired velocities, desired output torque, desired position stiffness, and desired velocity stiffness.
   */
  struct RobotCmd
  {
    RobotCmd() {}
    RobotCmd(int motor_num)
        : mode(motor_num, 0), q(motor_num, 0.0), dq(motor_num, 0.0), tau(motor_num, 0.0), Kp(motor_num, 0.0), Kd(motor_num, 0.0), motor_names(motor_num, "") {}

    void resize(int motor_num)
    {
      mode.resize(motor_num, 0.0);
      q.resize(motor_num, 0.0);
      dq.resize(motor_num, 0.0);
      tau.resize(motor_num, 0.0);
      Kp.resize(motor_num, 0.0);
      Kd.resize(motor_num, 0.0);
      motor_names.resize(motor_num, "");
    }

    uint64_t stamp;                       // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    std::vector<uint8_t> mode;            // The desired working mode of the robot.
    std::vector<float> q;                 // Vector storing the desired angles (in radians).
    std::vector<float> dq;                // Vector storing the desired velocities (in radians per second).
    std::vector<float> tau;               // Vector storing the desired output torque (in Newton meters).
    std::vector<float> Kp;                // Vector storing the desired position stiffness (in Newton meters per radian).
    std::vector<float> Kd;                // Vector storing the desired velocity stiffness (in Newton meters per radian per second).
    std::vector<std::string> motor_names; // Vector storing the names of the motors
  };
  typedef std::shared_ptr<RobotCmd> RobotCmdPtr;
  typedef std::shared_ptr<RobotCmd const> RobotCmdConstPtr;

  /**
   * @struct SensorJoy
   *
   * @brief Structure representing sensor inputs from robot joystick.
   *
   * This structure contains timestamp information along with axis and button values obtained from a joystick sensor.
   */
  struct SensorJoy
  {
    uint64_t stamp;               // Timestamp in nanoseconds, associated with the sensor input.
    std::vector<float> axes;      // Values representing the positions of the joystick axes.
    std::vector<int32_t> buttons; // Values representing the state of joystick buttons.
  };
  typedef std::shared_ptr<SensorJoy> SensorJoyPtr;
  typedef std::shared_ptr<SensorJoy const> SensorJoyConstPtr;

  /**
   * @struct DiagnosticValue
   *
   * @brief Structure representing diagnostic values.
   *
   * This structure contains information about the diagnostic level, name, code, and message.
   */
  struct DiagnosticValue
  {
#if defined(_WIN32) && defined(OK)
#undef OK
#endif
#if defined(_WIN32) && defined(WARN)
#undef WARN
#endif
#if defined(_WIN32) && defined(ERROR)
#undef ERROR
#endif
    enum
    {
      OK = 0
    };
    enum
    {
      WARN = 1
    };
    enum
    {
      ERROR = 2
    };

    uint64_t stamp;      // Timestamp in nanoseconds.
    int32_t level;       // Level associated with the diagnostic value.
    std::string name;    // Name identifying the diagnostic value.
    int32_t code;        // Code corresponding to the diagnostic value.
    std::string message; // Detailed message related to the diagnostic value.
  };
  typedef std::shared_ptr<DiagnosticValue> DiagnosticValuePtr;
  typedef std::shared_ptr<DiagnosticValue const> DiagnosticValueConstPtr;
}

#endif