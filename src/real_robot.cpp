#include "diffdrive_arduino/real_robot.h"

RealRobot::RealRobot()
    : arduino("/dev/ttyUSB0", 57600, 1000)
{

  l_wheel.pos = 0;
  l_wheel.vel = 0;
  l_wheel.eff = 0;
  l_wheel.cmd = 0;
  l_wheel.enc = 0;
  l_wheel.velSetPt = 0;

  r_wheel.pos = 0;
  r_wheel.vel = 0;
  r_wheel.eff = 0;
  r_wheel.cmd = 0;
  r_wheel.enc = 0;
  r_wheel.velSetPt = 0;

  arduino.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  arduino.setPidValues(30, 20, 0, 100);

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_a("base_to_left_wheel", &l_wheel.pos, &l_wheel.vel, &l_wheel.eff);
  jnt_state_interface.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b("base_to_right_wheel", &r_wheel.pos, &r_wheel.vel, &r_wheel.eff);
  jnt_state_interface.registerHandle(state_handle_b);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("base_to_left_wheel"), &l_wheel.cmd);
  jnt_vel_interface.registerHandle(vel_handle_a);

  hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("base_to_right_wheel"), &r_wheel.cmd);
  jnt_vel_interface.registerHandle(vel_handle_b);

  registerInterface(&jnt_vel_interface);
}

void RealRobot::read()
{

  ros::Time newTime = ros::Time::now();
  period = newTime - time;
  time = newTime;

  if (!arduino.connected())
  {
    return;
  }

  arduino.readEncoderValues(l_wheel.enc, r_wheel.enc);

  double posPrev = l_wheel.pos;
  l_wheel.UpdatePosFromEnc();
  l_wheel.vel = (l_wheel.pos - posPrev) / period.toSec();

  posPrev = r_wheel.pos;
  r_wheel.UpdatePosFromEnc();
  r_wheel.vel = (r_wheel.pos - posPrev) / period.toSec();
}

void RealRobot::write()
{
  if (!arduino.connected())
  {
    return;
  }

  arduino.setMotorValues(l_wheel.cmd / l_wheel.rads_per_count / loop_rate, r_wheel.cmd / r_wheel.rads_per_count / loop_rate);
}
