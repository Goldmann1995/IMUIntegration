//
// Created by adam on 18-9-21.
//

#include "imu_integration.h"
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <mutex>
IMUIntegration imu_integ;
std::mutex mtx_imu;
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    std::lock_guard<std::mutex> lg(mtx_imu);
    Vec3d gyro (msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);  
    Vec3d acc (msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    IMU imu(msg->header.stamp.toSec(),gyro,acc);
    imu_integ.AddIMU(imu);
    Eigen::Matrix3d R = imu_integ.GetR().matrix();
    std::cout << "Postion: " <<  imu_integ.GetP() << "\n" << "------------------\n" <<
    " Vel:" << imu_integ.GetV()<< "\n"  <<"------------------\n" <<
    "R =" << R << "\n" <<"------------------\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_integration");

    ros::NodeHandle nh("~");
    ros::Subscriber sub_imu;
    std::string imu_topic = "/xsens_mti_node/imu/data";

    const Vec3d gravity(0, 0, -9.8);  // 重力方向
    const Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    const Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);
    imu_integ.Init(gravity, init_bg, init_ba);
    Vec3d pp(100,200,500);
    imu_integ.SetP(pp);
    sub_imu = nh.subscribe(imu_topic, 100000, imu_cbk);
    // ros::ServiceServer command_service = nh.advertiseService("/cal", commandCallback);

    ros::spin();
    return 0;
}