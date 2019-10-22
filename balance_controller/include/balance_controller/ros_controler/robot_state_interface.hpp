/*
 *  robot_state_interface.hpp
 *  Descriotion: This is a robot state data handle for the data transit between
 *               hardware interface and ros_controllers
 *
 *  Created on: Mar 7, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#ifndef ROBOT_STATE_INTERFACE_H
#define ROBOT_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <string>

namespace hardware_interface
{

/** \brief A handle used to read the state of a IMU sensor.
 *
 * Depending on the sensor, not all readings exposed by the handle class might be available.
 * TODO: Document more!
 */
class RobotStateHandle : public JointHandle
{
public:
  struct Data
  {
    Data()
      : name(),
        frame_id(),
        orientation(0),
        position(0),
        angular_velocity(0),
        linear_acceleration(0),
        linear_velocity(0),
        joint_position_read(0),
        joint_position_write(0),
        joint_velocity_read(0),
        joint_velocity_write(0),
        joint_effort_read(0),
        joint_effort_write(0),
        foot_contact(0),
        contact_pressure(0),
        motor_status_word(0){}

    std::string name;                       ///< The name of the sensor
    std::string frame_id;                   ///< The reference frame to which this sensor is associated
    double* orientation;                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
    double* position;         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
    double* angular_velocity;               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
    double* linear_acceleration;            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
    double* linear_velocity; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
    double* joint_position_read;
    double* joint_position_write;
    double* joint_velocity_read;
    double* joint_velocity_write;
    double* joint_effort_read;
    double* joint_effort_write;
    int* foot_contact;
    double* contact_pressure;
    int* motor_status_word;///< A pointer to store if a leg is contact [LF,RF,RH,LH], 0-swing,1-stance,2-early,3-late
//    double* lf_leg_phase;       ///< A pointer store the phase(0~1) of leg [sw,st]
//    double* rf_leg_phase;       ///< A pointer store the phase(0~1) of leg [sw,st]
//    double* rh_leg_phase;       ///< A pointer store the phase(0~1) of leg [sw,st]
//    double* lh_leg_phase;       ///< A pointer store the phase(0~1) of leg [sw,st]

  };

  RobotStateHandle(const Data& data = Data())
    : name_(data.name),
      frame_id_(data.frame_id),
      orientation_(data.orientation),
      position_(data.position),
      angular_velocity_(data.angular_velocity),
      linear_acceleration_(data.linear_acceleration),
      linear_velocity_(data.linear_velocity),
      joint_position_read_(data.joint_position_read),
      joint_position_write_(data.joint_position_write),
      joint_velocity_read_(data.joint_velocity_read),
      joint_velocity_write_(data.joint_velocity_write),
      joint_effort_read_(data.joint_effort_read),
      joint_effort_write_(data.joint_effort_write),
      foot_contact_(data.foot_contact),
      contact_pressure_(data.contact_pressure),
      motor_status_word_(data.motor_status_word)
  {}

  RobotStateHandle(
        const std::string& name,                      ///< The name of the sensor
        const std::string& frame_id,                  ///< The reference frame to which this sensor is associated
        double* orientation,                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
        double* position,         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
        double* angular_velocity,               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
        double* linear_acceleration,            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
        double* linear_velocity,  ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
        const double* joint_position_read,
        double* joint_position_write,
        const double* joint_velocity_read,
        double* joint_velocity_write,
        const double* joint_effort_read,
        double* joint_effort_write,
        int* foot_contact,            ///< A pointer to store if a leg is contact [LF,RF,RH,LH]
        double* contact_pressure,
        int* motor_status_word
      )
    : name_(name),
      frame_id_(frame_id),
      orientation_(orientation),
      position_(position),
      angular_velocity_(angular_velocity),
      linear_acceleration_(linear_acceleration),
      linear_velocity_(linear_velocity),
      joint_position_read_(joint_position_read),
      joint_position_write_(joint_position_write),
      joint_velocity_read_(joint_velocity_read),
      joint_velocity_write_(joint_velocity_write),
      joint_effort_read_(joint_effort_read),
      joint_effort_write_(joint_effort_write),
      foot_contact_(foot_contact),
      contact_pressure_(contact_pressure),
      motor_status_word_(motor_status_word)
  {}

  std::string getName()                           const {return name_;}
  std::string getFrameId()                        const {return frame_id_;}
  double* getOrientation()                  const {return orientation_;}
  double* getPosition()                     const {return position_;}
  double* getAngularVelocity()              const {return angular_velocity_;}
  double* getLinearAcceleration()           const {return linear_acceleration_;}
  double* getLinearVelocity()               const {return linear_velocity_;}
  const double* getJointPositionRead()            const {return joint_position_read_;}
  double* getJointPositionWrite()                       {return joint_position_write_;}
  const double* getJointVelocityRead()            const {return joint_velocity_read_;}
  double* getJointVelocityWrite()                       {return joint_velocity_write_;}
  const double* getJointEffortRead()              const {return joint_effort_read_;}
  double* getJointEffortWrite()                         {return joint_effort_write_;}
  int* getFootContact()                    const {return foot_contact_;}
  void setFootContact(int state, int leg_index)  {foot_contact_[leg_index] = state;}
  void setPosition(double x, double y, double z) {position_[0] = x;position_[1] =y;position_[2]=z;}
  //  void setJointPositionWrite(double* cmd)
//  {
//    for(unsigned int i=0;i<12;i++)
//    joint_position_write_[i] = cmd[i];
//  }
//  hardware_interface::JointCommandInterface joint_effort_interfaces_;
  double* orientation_;
  double* position_;
  double* angular_velocity_;
  double* linear_acceleration_;
  double* linear_velocity_;
  int* foot_contact_;
  double* contact_pressure_;
  int* motor_status_word_;///< A pointer to store if a leg is contact [LF,RF,RH,LH]
private:
  std::string name_;
  std::string frame_id_;


  const double* joint_position_read_;
  double* joint_position_write_;
  const double* joint_velocity_read_;
  double* joint_velocity_write_;
  const double* joint_effort_read_;
  double* joint_effort_write_;
//  int* foot_contact_;            ///< A pointer to store if a leg is contact [LF,RF,RH,LH]
//  const double* lf_leg_phase_;       ///< A pointer store the phase(0~1) of leg [sw,st]
//  const double* rf_leg_phase_;       ///< A pointer store the phase(0~1) of leg [sw,st]
//  const double* rh_leg_phase_;       ///< A pointer store the phase(0~1) of leg [sw,st]
//  const double* lh_leg_phase_;       ///< A pointer store the phase(0~1) of leg [sw,st]


};

/** \brief Hardware interface to support reading the state of an Robot State. */
class RobotStateInterface : public HardwareResourceManager<RobotStateHandle> {
public:
  /**
   * @brief joint_effort_interfaces_, to add a additional joint command in the
   * RobotStateInterface for command updating to joints
   */
  hardware_interface::JointCommandInterface joint_effort_interfaces_;
};

}

#endif // ROBOT_STATE_INTERFACE_H
