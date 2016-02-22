#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "gecko_msgs/BaseVelocity.h"
#include "gecko_msgs/FlipperVelocity.h"

class TeleopSetVelocity
{
public:
  TeleopSetVelocity();

private:
  ros::NodeHandle nh_;
  ros::Publisher  base_velocity_pub_;
  ros::Publisher  flipper_velocity_pub_;
  ros::Subscriber joy_sub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_set_velocity_node");
  TeleopSetVelocity teleop_set_velocity;

  ros::NodeHandle nh;
  int CONTROL_FREQUENCY;
  if(!nh.getParam("/control_frequency", CONTROL_FREQUENCY))
  {
    ROS_ERROR("/control_frequency is not defined!");
  }
  ros::Rate loop_rate_hz(CONTROL_FREQUENCY);

  while(ros::ok())
  {
    loop_rate_hz.sleep();
    ros::spinOnce();
  }

  return 0;
}


/*!
 * \brief Initialize publisher and subscriber.
 */
TeleopSetVelocity::TeleopSetVelocity()
{
  base_velocity_pub_ = nh_.advertise<gecko_msgs::BaseVelocity>("base_velocity", 10);
  flipper_velocity_pub_= nh_.advertise<gecko_msgs::FlipperVelocity>("flipper_velocity", 10);
  joy_sub_ = nh_.subscribe("joy", 10, &TeleopSetVelocity::joyCallback, this);
}


/*!
 * \brief Compute base & flipper velocity from joy message, and publish them.
 * \param msg
 *
 * WARN: Cannot control both crawler and flipper at the same time.
 */
void TeleopSetVelocity::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  static const int VELOCITY_SCALE  = 100;

  gecko_msgs::BaseVelocity    base_velocity;
  gecko_msgs::FlipperVelocity flipper_velocity;

  bool flipper_active = static_cast<bool>(msg->buttons[4] || msg->buttons[5] || msg->buttons[6] || msg->buttons[7]);

  if (flipper_active)
  {
    // フリッパーの正転・逆転は十字ボタンの上下(msg->axes[5])で決まる
    flipper_velocity.front_left  = msg->axes[1] * msg->buttons[4] * VELOCITY_SCALE;
    flipper_velocity.rear_left   = msg->axes[1] * msg->buttons[6] * VELOCITY_SCALE;
    flipper_velocity.front_right = msg->axes[1] * msg->buttons[5] * VELOCITY_SCALE;
    flipper_velocity.rear_right  = msg->axes[1] * msg->buttons[7] * VELOCITY_SCALE;
    flipper_velocity_pub_.publish(flipper_velocity);
  }
  else
  {
    base_velocity.linear  = msg->axes[1] * VELOCITY_SCALE;
    base_velocity.angular = msg->axes[2] * VELOCITY_SCALE;
    base_velocity_pub_.publish(base_velocity);
  }
}
