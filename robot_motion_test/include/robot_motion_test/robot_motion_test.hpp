#ifndef ROBOT_MOTION_TEST_HPP__
#define ROBOT_MOTION_TEST_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <fstream>
#include <thread>

// ros
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "kdl/frames.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

// custom
#include "robot_motion_base/robot_motion_base.hpp"

using namespace robot_motion_base;

class RobotMotionTest : public RobotMotionBase
{
public:
    RobotMotionTest();

private:
    /* function */
    virtual void custom_init();
    //tasks
    void test_service(bool on);
    virtual void tasks_init();

    /* variable */
    // service client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr test_client_;

    // load parameters test
    Eigen::Isometry3d calibration_matrix_;

    uint8_t test_task_num1_;
    uint8_t test_task_num2_;
    uint8_t test_task_num3_;
};

#endif // ROBOT_MOTION_TEST_HPP__