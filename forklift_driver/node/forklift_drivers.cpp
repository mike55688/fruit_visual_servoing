#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <string>
#include <cmath> //using isinf(), isnan()

#include <stm32.h>
#include <forklift_driver/msg/meteorcar.hpp>

#define Sign(A) ((A) >= 0 ? 1 : -1)

class ForkLiftDriver : public rclcpp::Node
{
  // publisher & subscriber declare
  std::string topic_odom, topic_imu, topic_forklift_pose, topic_cmd_vel, topic_cmd_fork ,lidar_tf_name;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<forklift_driver::msg::Meteorcar>::SharedPtr pub_forklift;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  rclcpp::Subscription<forklift_driver::msg::Meteorcar>::SharedPtr sub_cmd_fork;
  std::vector<double> lidar_tf_dist;

  // forklift variable declare
  float wheel_base, wheel_angle, wheel_speed, fork_velocity, theta_bias;
  bool use_imu_flag, odom_tf_flag, init_fork_flag;
  rclcpp::Time last_time, current_time, last_cmdvelcb_time, last_cmdforkcb_time;
  rclcpp::Rate *r;
  int rate, timeout;
  STM32 *stm32;

  // function declare
  void CmdVelCB(const geometry_msgs::msg::Twist &msg);
  void CmdForkCB(const forklift_driver::msg::Meteorcar &msg);
  void PublishOdom();
  void PublishLidarTF();
  void PublishImu();
  void PublishForklift();
  void broadcastTransform(const geometry_msgs::msg::Transform &transform, const std::string &child_frame_id,
                          const std::string &camera_tf);

public:
  ForkLiftDriver(STM32 &stm32);
  ~ForkLiftDriver();
  void spin();
};
ForkLiftDriver::ForkLiftDriver(STM32 &stm32) : Node("forklift_driver"), stm32(&stm32){};
ForkLiftDriver::~ForkLiftDriver()
{
  delete r, stm32, pub_odom, pub_imu, pub_forklift, sub_cmd_vel, sub_cmd_fork;  // delete all pointer
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Forklift driver closed");
};
void ForkLiftDriver::spin()
{
  // Get parameters
  rate = this->declare_parameter<int>("rate", 25);
  timeout = this->declare_parameter<int>("timeout", 5);
  wheel_base = this->declare_parameter<float>("wheel_base", 0.3);
  theta_bias = this->declare_parameter<float>("theta_bias");
  use_imu_flag = this->declare_parameter<bool>("use_imu_flag", false);
  odom_tf_flag = this->declare_parameter<bool>("odom_tf_flag", false);
  init_fork_flag = this->declare_parameter<bool>("init_fork_flag", false);
  lidar_tf_dist = this->declare_parameter<std::vector<double>>("lidar_tf_dist", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  lidar_tf_name = this->declare_parameter<std::string>("lidar_tf_name", "velodyne");

  topic_cmd_vel = this->declare_parameter<std::string>("topic_cmd_vel", "/cmd_vel");
  topic_cmd_fork = this->declare_parameter<std::string>("topic_cmd_fork", "/cmd_fork");

  topic_odom = this->declare_parameter<std::string>("topic_odom", "/odom");
  topic_imu = this->declare_parameter<std::string>("topic_imu", "/imu");
  topic_forklift_pose = this->declare_parameter<std::string>("topic_forklift_pose", "/forklift_pose");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\x1B[0;32m"
                                            "Forklift driver parameters:\n"
                                            "rate: %d\n"
                                            "timeout: %d\n"
                                            "wheel_base: %f\n"
                                            "theta_bias: %f\n"
                                            "use_imu_flag: %d\n"
                                            "odom_tf_flag: %d\n"
                                            "init_fork_flag: %d\n"
                                            "topic_cmd_vel: %s\n"
                                            "topic_cmd_fork: %s\n"
                                            "topic_odom: %s\n"
                                            "topic_imu: %s\n"
                                            "topic_forklift_pose: %s\n"
                                            "\x1B[0m",
              rate, timeout, wheel_base, theta_bias, use_imu_flag, odom_tf_flag, init_fork_flag, topic_cmd_vel.c_str(), topic_cmd_fork.c_str(), topic_odom.c_str(), topic_imu.c_str(), topic_forklift_pose.c_str());

  // Publisher & Subscriber
  pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom.c_str(), 10);
  pub_imu = this->create_publisher<sensor_msgs::msg::Imu>(topic_imu.c_str(), 10);
  pub_forklift = this->create_publisher<forklift_driver::msg::Meteorcar>(topic_forklift_pose.c_str(), 10);
  sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(topic_cmd_vel.c_str(), 10, std::bind(&ForkLiftDriver::CmdVelCB, this, std::placeholders::_1));
  sub_cmd_fork = this->create_subscription<forklift_driver::msg::Meteorcar>(topic_cmd_fork.c_str(), 10, std::bind(&ForkLiftDriver::CmdForkCB, this, std::placeholders::_1));

  // Initialize variables
  wheel_angle = wheel_speed = fork_velocity = 0.0f;
  r = new rclcpp::Rate(rate);
  last_time = current_time = last_cmdvelcb_time = last_cmdforkcb_time = this->get_clock()->now();

  // initial fork
  while (rclcpp::ok() && init_fork_flag && stm32->Data14 < 1.0f)
  {
    stm32->read_data();
    stm32->send_data(1, 0, 0, 0, 2500, 0, 0, 0, 0, 0, 0, 0); // 电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
    r->sleep();
  }
  if (init_fork_flag)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\x1B[0;32m"
                                              "Forklift fork initialization complete. Forklift is now ready to be activated.\n"
                                              "\x1B[0m");
  else
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\x1B[0;32m"
                                              "Forklift is already active.\n"
                                              "\x1B[0m");

  // main loop
  while (rclcpp::ok())
  {
    current_time = this->get_clock()->now();
    rclcpp::spin_some(this->get_node_base_interface());

    if ((current_time - last_cmdvelcb_time).seconds() > timeout)
    {
      wheel_speed = 0.0f;
      wheel_angle = 0.0f;
    }

    if ((current_time - last_cmdforkcb_time).seconds() > timeout)
      fork_velocity = 0.0f;

    stm32->read_data();
    PublishOdom();
    PublishImu();
    PublishForklift();
    stm32->send_data(1, wheel_speed, 0, wheel_angle + theta_bias, fork_velocity, 0, 0, 0, 0, 0, 0, 0); // 电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
    last_time = current_time;
    r->sleep();
  }

  rclcpp::shutdown();
  return;
}
void ForkLiftDriver::CmdVelCB(const geometry_msgs::msg::Twist &msg) // 參考cmd_vel_to_ackermann_drive.py
{
  static float r; // r = 旋轉半徑
  static float wheel_angle_last(0.0), wheel_speed_last(0.0);
  last_cmdvelcb_time = this->get_clock()->now();
  wheel_speed = msg.linear.x;                                        // 速度v = 圓周運動速度
  if (fabs(msg.linear.x) <= 0.0001 && fabs(msg.angular.z) <= 0.0001) // 判斷是否停止，如果是的話wheel_speed為0，但wheel_angle維持不變
  {
    wheel_speed = 0.0;
    wheel_angle = wheel_angle_last;
    return;
  }
  else if (std::isnan(r = msg.linear.x / msg.angular.z)) // 判斷旋轉半徑是否為無限大(直走)
    r = INFINITY;
  else if (abs(r) < wheel_base) // 判斷旋轉半徑是否小於wheel_base(自轉)
  {
    wheel_speed = msg.angular.z * wheel_base;
    r = 0.0;
  }
  else
    r = wheel_speed / msg.angular.z;  // 旋轉半徑r = 速度v / 角速度w
  wheel_angle = atan(wheel_base / r); // theta = arctan(前輪到兩後輪中心軸距wheel_base / 旋轉半徑r)
  wheel_angle *= 180 / M_PI;          // 轉換為角度

  wheel_angle_last = wheel_angle; // 記錄上一次的角度
  wheel_speed_last = wheel_speed; // 記錄上一次的速度
};
void ForkLiftDriver::CmdForkCB(const forklift_driver::msg::Meteorcar &msg)
{
  last_cmdforkcb_time = current_time;
  fork_velocity = -msg.fork_velocity; // fork_velocity上升為負，下降為正，因為stm32的起重電機PWM是負值上升，正值下降
  // 上限3600，下限1000
  if (abs(fork_velocity) > 3600)
    fork_velocity = 3600 * Sign(fork_velocity);
  else if (abs(fork_velocity) < 1000 && abs(fork_velocity) > 1)
    fork_velocity = 1000 * Sign(fork_velocity);
  else if (abs(fork_velocity) < 1)
    fork_velocity = 0;
};

void ForkLiftDriver::PublishOdom()
{
  static nav_msgs::msg::Odometry odom;
  static tf2::Quaternion th_quat;
  static geometry_msgs::msg::TransformStamped odom_trans;
  static auto odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static float x, y, th, linear_x, angular_z, delta_th, delta_x, delta_y, dt;

  linear_x = stm32->Data2 * cos(stm32->Data3 * M_PI / 180);
  (use_imu_flag) ? angular_z = stm32->angular_velocity_z :                    // 使用imu計算角速度
      angular_z = stm32->Data2 * sin(stm32->Data3 * M_PI / 180) / wheel_base; // 使用里程計計算角速度

  dt = (current_time - last_time).seconds();
  delta_th = angular_z * dt;
  delta_x = linear_x * cos(th + delta_th / 2) * dt;
  delta_y = linear_x * sin(th + delta_th / 2) * dt;
  // 避免在車子停止不動時誤差累積
  if (fabs(delta_x) < 1e-6)
  {
    delta_x = 0.0f;
  }
  if (fabs(delta_y) < 1e-6)
  {
    delta_y = 0.0f;
  }
  if (fabs(delta_th) < 1e-5) // 話說為啥其他都是1e-6，這個是1e-5 ???
  {
    delta_th = 0.0f;
  }
  x += delta_x;
  y += delta_y;
  th += delta_th;
  // Debug
  // cout << "dt: " << dt << endl
  //      << endl;
  // cout << "delta_x: " << delta_x << " | delta_y: " << delta_y << " | delta_th: " << delta_th << endl
  //      << endl;
  // cout << "x: " << x << " | y: " << y << " | th: " << th << endl
  //      << endl;

  th_quat.setRPY(0, 0, th); // 歐拉角轉換為四元數

  odom.header.stamp = current_time;
  odom.header.frame_id = topic_odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0f;
  odom.pose.pose.orientation.x = th_quat.x();
  odom.pose.pose.orientation.y = th_quat.y();
  odom.pose.pose.orientation.z = th_quat.z();
  odom.pose.pose.orientation.w = th_quat.w();
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = linear_x;
  odom.twist.twist.linear.y = 0.0f;
  odom.twist.twist.angular.z = angular_z;
  odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};
  odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                           0, 1e-3, 0, 0, 0, 0,
                           0, 0, 1e6, 0, 0, 0,
                           0, 0, 0, 1e6, 0, 0,
                           0, 0, 0, 0, 1e6, 0,
                           0, 0, 0, 0, 0, 1e3};
  pub_odom->publish(odom);

  if (odom_tf_flag)
  {
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = topic_odom;
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = th_quat.x();
    odom_trans.transform.rotation.y = th_quat.y();
    odom_trans.transform.rotation.z = th_quat.z();
    odom_trans.transform.rotation.w = th_quat.w();
    odom_broadcaster->sendTransform(odom_trans);
    PublishLidarTF();
  }
};

void ForkLiftDriver::PublishLidarTF()
{
  static geometry_msgs::msg::TransformStamped lidar_trans;
  static auto lidar_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static float x, y, th, linear_x, angular_z, delta_th, delta_x, delta_y, dt;
  lidar_trans.header.stamp = current_time;
  lidar_trans.header.frame_id = "base_link";
  lidar_trans.child_frame_id = lidar_tf_name;

  lidar_trans.transform.translation.x = lidar_tf_dist[0];
  lidar_trans.transform.translation.y = lidar_tf_dist[1];
  lidar_trans.transform.translation.z = lidar_tf_dist[2];

  static tf2::Quaternion lidar_th_quat;
  lidar_th_quat.setRPY(lidar_tf_dist[3], lidar_tf_dist[4], lidar_tf_dist[5]);
  lidar_trans.transform.rotation.x = lidar_th_quat.x();
  lidar_trans.transform.rotation.y = lidar_th_quat.y();
  lidar_trans.transform.rotation.z = lidar_th_quat.z();
  lidar_trans.transform.rotation.w = lidar_th_quat.w();
  lidar_broadcaster->sendTransform(lidar_trans);
}

void ForkLiftDriver::PublishImu()
{
  static sensor_msgs::msg::Imu imu_data;
  static float dt, delta_th, th;
  static tf2::Quaternion th_quat;
  dt = (current_time - last_time).seconds();
  delta_th = stm32->angular_velocity_z * dt;
  th += delta_th;

  th_quat.setRPY(0, 0, th);
  imu_data.header.stamp = current_time;
  imu_data.header.frame_id = "imu";
  imu_data.orientation.x = th_quat.x();
  imu_data.orientation.y = th_quat.y();
  imu_data.orientation.z = th_quat.z();
  imu_data.orientation.w = th_quat.w();
  imu_data.linear_acceleration.x = stm32->accelerated_wheel_speed;
  imu_data.linear_acceleration.y = stm32->accelerated_speed_y;
  imu_data.linear_acceleration.z = stm32->accelerated_speed_z;

  imu_data.angular_velocity.x = stm32->angular_velocity_x;
  imu_data.angular_velocity.y = stm32->angular_velocity_y;
  imu_data.angular_velocity.z = stm32->angular_velocity_z;
  imu_data.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
  imu_data.linear_acceleration_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

  pub_imu->publish(imu_data);
}

void ForkLiftDriver::PublishForklift()
{
  static float dt;
  static forklift_driver::msg::Meteorcar forklift_msg;

  forklift_msg.wheel_velocity = stm32->Data2;
  forklift_msg.wheel_angle = stm32->Data3;

  dt = (current_time - last_time).seconds();
  forklift_msg.fork_velocity = stm32->Data13 / 30 / 60 * 2;
  forklift_msg.fork_position -= forklift_msg.fork_velocity * dt;

  (stm32->Data14 == true /*限位開關被壓住*/) ? forklift_msg.fork_position = 0.0 : forklift_msg.fork_position = forklift_msg.fork_position;

  pub_forklift->publish(forklift_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  STM32 stm32;
  ForkLiftDriver(stm32).spin(); // 建立叉車驅動物件，傳入stm32物件，開始spin

  return 0;
}
/*
STM32 物件的 public variable
Data1   // 电机启动停止控制位（1/0 启动/停止）
Data2   // 前轮线速度(m/s)
Data3   // 前轮转角(角度°)
Data4   // 绕X轴角速度 gyro_Roll 原始数值
Data5   // 绕Y轴角速度 gyro_Pitch 原始数值
Data6   // 绕Z轴角速度 gyro_Yaw 原始数值
Data7   // X轴加速度 accel_x 原始数值
Data8   // Y轴加速度 accel_y 原始数值
Data9   // Z轴加速度 accel_z 原始数值
Data10  // Yaw Z轴角度
Data11  // 电池电压              24-25   <24.3  low
Data12  // 红色紧急开关位0/1 运行/停止
Data13  // 起重电机编码器原始数据（未转换） 如果有需要可以添加发送指令去清0，上面的发送命令还有剩余   gearrate 30  dt 5 ms
Data14  // 起重电机下行限位开关（用于校准） 1代表开关被压住
Data15  // 起重电机上行限位开关（用于校准） 1代表开关被压住

angular_velocity_x = Data4 * 0.001064;  //转换成 rad/s
angular_velocity_y = Data5 * 0.001064;  //转换成 rad/s
angular_velocity_z = Data6 * 0.001064;  //转换成 rad/s
accelerated_wheel_speed = Data7 / 2048; //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
accelerated_speed_y = Data8 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
accelerated_speed_z = Data9 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
*/
