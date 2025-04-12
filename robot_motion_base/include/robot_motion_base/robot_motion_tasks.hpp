#ifndef ROBOT_MOTION_TASKS_HPP__
#define ROBOT_MOTION_TASKS_HPP__

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

#define TaskPtr std::make_shared<robot_motion_base::RobotMotionTask>
#define TaskPtrConst std::make_shared<const robot_motion_base::RobotMotionTask>

namespace robot_motion_base
{

// enum class for task status
enum class TaskState
{
    INIT = 0,
    EXECUTE,
    FINISH,
};

// struct for service flags
struct ServiceFlags{
    bool service_called = false;
    bool model_joint = false; // when moving by the joint, stop the cartesian motion
};

// struct for robot motion states
struct RobotMotionState
{
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::PoseStamped target_monitor; // for getting the latest command
    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::WrenchStamped current_wrench;
    geometry_msgs::msg::WrenchStamped target_wrench;
};

// struct for System States Machine
struct SystemState
{
    uint8_t task_num = 0;
    rclcpp::Time start_time;
    rclcpp::Time current_time;
};

/**
 * @brief Abstract base class for robot motion tasks.
 *
 * This class defines the interface for robot motion tasks. Derived classes should implement
 * the execute, cancel, isFinished, isActive, getName, getStatus, and getErrorMessage methods.
 */
class RobotMotionTask
{
    public:

    /// @brief Initialize the task with task_name and execute function
    /// @param task_name 
    /// @param execute 
    RobotMotionTask(std::string task_name, std::function<void()> execute)
        : task_name_(task_name), execute_(execute){
            init_ = [](){}; // empty init function
        };
    
    /// @brief Initialize the task with task_name, init function, and execute function
    /// @param task_name 
    /// @param init 
    /// @param execute 
    RobotMotionTask(std::string task_name, std::function<void()> init, std::function<void()> execute)
        : task_name_(task_name), init_(init), execute_(execute){};

    void init(){
        init_();
    }

    void execute(){
        execute_();
    }

    std::string get_name(){
        return task_name_;
    }

    TaskState get_state(){
        return state_;
    }

    void set_state(TaskState state){
        state_ = state;
    }

private:
    std::string task_name_;
    std::function<void()> init_;
    std::function<void()> execute_;

    TaskState state_ = TaskState::INIT;
};



} // namespace robot_motion_base



#endif // ROBOT_MOTION_TASKS_HPP__