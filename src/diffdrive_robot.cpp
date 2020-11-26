#include "ros/ros.h"

#include "diffdrive_arduino/real_robot.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "real_robot");
  ros::NodeHandle n("~");

  RealRobot::Config robotCfg;

  if (!n.getParam("left_wheel_name", robotCfg.left_wheel_name))
  {
    robotCfg.left_wheel_name = "left_wheel";
  }

  if (!n.getParam("right_wheel_name", robotCfg.right_wheel_name))
  {
    robotCfg.right_wheel_name = "right_wheel";
  }


  RealRobot robot(robotCfg);
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prevTime = ros::Time::now();

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();

    loop_rate.sleep();
  }
}
