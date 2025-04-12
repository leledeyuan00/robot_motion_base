#ifndef ROBOT_MOTION_BASE_HPP__
#define ROBOT_MOTION_BASE_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <thread>
#include <cstring>

// ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "kdl/frames.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// custom
#include "robot_motion_base/robot_motion_tasks.hpp"

// eigen
#include <eigen3/Eigen/Dense>

namespace robot_motion_base
{

/**
 * @brief Base class for robot motion tasks.
 * @param node_name Name of the node.
 * @param rate Rate of the task execution.
*/
class RobotMotionBase : public rclcpp::Node
{

public:
    RobotMotionBase(const std::string &node_name, uint16_t rate = 125)
        : Node(node_name), rate_(rate)
    {
        // Initialize the node
        init();
        RCLCPP_INFO(this->get_logger(), "RobotMotionBase initialized");
    }
    virtual ~RobotMotionBase() = default;
    void start();
    void active(){
        robot_l_.target_pose = robot_l_.target_monitor;
        robot_r_.target_pose = robot_r_.target_monitor;
        robot_l_.start_pose  = robot_l_.target_monitor;
        robot_r_.start_pose  = robot_r_.target_monitor;
        active_ = true;
    };
    void deactivate(){active_ = false;};

protected:
    /* functions */
    void init();
    virtual void custom_init(){};

    virtual geometry_msgs::msg::PoseStamped intepolation(geometry_msgs::msg::PoseStamped init_pose, geometry_msgs::msg::PoseStamped target_pose,
                        double duration, double current_duration)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = init_pose.header.frame_id;
        pose.pose.position.x = init_pose.pose.position.x + (target_pose.pose.position.x - init_pose.pose.position.x) * current_duration / duration;
        pose.pose.position.y = init_pose.pose.position.y + (target_pose.pose.position.y - init_pose.pose.position.y) * current_duration / duration;
        pose.pose.position.z = init_pose.pose.position.z + (target_pose.pose.position.z - init_pose.pose.position.z) * current_duration / duration;

        Eigen::Quaterniond init_q, target_q;
        tf2::fromMsg(init_pose.pose.orientation, init_q);
        tf2::fromMsg(target_pose.pose.orientation, target_q);

        Eigen::Quaterniond q = init_q.slerp(current_duration / duration, target_q);
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

    virtual geometry_msgs::msg::WrenchStamped intepolation_wrench(geometry_msgs::msg::WrenchStamped init_wrench, geometry_msgs::msg::WrenchStamped target_wrench,
                            double duration, double current_duration)
    {
        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header = init_wrench.header;
        wrench.wrench.force.x = init_wrench.wrench.force.x + (target_wrench.wrench.force.x - init_wrench.wrench.force.x) * current_duration / duration;
        wrench.wrench.force.y = init_wrench.wrench.force.y + (target_wrench.wrench.force.y - init_wrench.wrench.force.y) * current_duration / duration;
        wrench.wrench.force.z = init_wrench.wrench.force.z + (target_wrench.wrench.force.z - init_wrench.wrench.force.z) * current_duration / duration;
        wrench.wrench.torque.x = init_wrench.wrench.torque.x + (target_wrench.wrench.torque.x - init_wrench.wrench.torque.x) * current_duration / duration;
        wrench.wrench.torque.y = init_wrench.wrench.torque.y + (target_wrench.wrench.torque.y - init_wrench.wrench.torque.y) * current_duration / duration;
        wrench.wrench.torque.z = init_wrench.wrench.torque.z + (target_wrench.wrench.torque.z - init_wrench.wrench.torque.z) * current_duration / duration;

        return wrench;
    }


    // robot state
    RobotMotionState get_robot_state_l() {return robot_l_;};
    RobotMotionState get_robot_state_r() {return robot_r_;};
    SystemState get_system_state() {return system_state_;};


    // set robot state
    void set_target_pose_l(geometry_msgs::msg::PoseStamped target_pose) {robot_l_.target_pose = target_pose;};
    void set_target_pose_r(geometry_msgs::msg::PoseStamped target_pose) {robot_r_.target_pose = target_pose;};

    /* FSM tasks */ 
    /* @brief from start_pose(initialized when this task running) to target_pose
    *  @param left_target 
    *  @param right_target 
    *  @param duration 
    *  @return finish or not
    */ 
    bool move(geometry_msgs::msg::PoseStamped left_target, geometry_msgs::msg::PoseStamped right_target, double duration);

    /* @brief from start_pose(initialized when this task running) to target_pose
    *  @param left_target 
    *  @param right_target 
    *  @param duration 
    *  @return finish or not
    */
    bool move_wrench(geometry_msgs::msg::PoseStamped left_target, geometry_msgs::msg::PoseStamped right_target, 
                            geometry_msgs::msg::WrenchStamped left_wrench, geometry_msgs::msg::WrenchStamped right_wrench, double duration);

    /*
    * @brief sleep for a while
    * @param duration
    * @return finish or not
    */
    bool sleep(double duration);

    /* @brief joint move
    *  @param left_target
    *  @param right_target
    *  @return finish or not
    */
    bool joint_move(std::vector<double> left_joints, std::vector<double> right_joints, double time);

    
    /*
    * @brief Push back the task, the task is derived from GarmentTaskBase
    * @param task 
    * @return the task number
    */
    uint8_t task_pushback(std::shared_ptr<robot_motion_base::RobotMotionTask> task);
    virtual void tasks_init() = 0;
    void task_execute();

    /// @brief set the task finished
    void set_task_finished(){
        next_task_offset_ = 1;
        tasks_vector_[system_state_.task_num]->set_state(TaskState::FINISH);
    };
    void goto_specific_task(uint8_t task_num){
        next_task_offset_ = task_num - system_state_.task_num;
        tasks_vector_[system_state_.task_num]->set_state(TaskState::FINISH);
    };
    void goto_init_task(){
        goto_specific_task(0);
    };
    void goto_last_task(){
        goto_specific_task(tasks_vector_.size()-1);
    };

    // states
    std::shared_ptr<ServiceFlags> service_flags_;
    bool initialized_l_ = false;
    bool initialized_r_ = false;
    inline bool check_init() { return initialized_l_ && initialized_r_; }

    SystemState system_state_; // current task number, current time, start time
    uint8_t next_task_offset_ = 1;

    // std
    std::thread control_loop_thread_;

    // time
    rclcpp::Clock my_clock_;

    // system state
    // create a vector for TaskWrapper
    std::vector<std::shared_ptr<robot_motion_base::RobotMotionTask>> tasks_vector_;
    
    
    bool active_ = true;

private:

    /* function */
    void control_loop();
    // ros
    void ros_init();
    

    /* variable */ 
    uint16_t rate_;
    // pub
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_l_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_r_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_l_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_r_;

    // sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_l_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_r_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_monitor_sub_l_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_monitor_sub_r_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_l_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_r_;

    // service client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr joint_move_client_l_; // an example for service client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr joint_move_client_r_; // please change the serevice type on your case

    // robot state
    RobotMotionState robot_l_;
    RobotMotionState robot_r_;


}; // class RobotMotionBase


} // namespace robot_motion_base


#endif // ROBOT_MOTION_BASE_HPP__