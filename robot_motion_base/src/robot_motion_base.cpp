#include "robot_motion_base/robot_motion_base.hpp"

namespace robot_motion_base
{

void RobotMotionBase::init()
{
    // Initialize the node
    ros_init();
    // Initialize other functions
}

void RobotMotionBase::ros_init()
{
    // clock
    my_clock_ = rclcpp::Clock(RCL_ROS_TIME);

    // publishers
    target_pose_pub_l_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/left_cartesian_compliance_controller/target_frame", rclcpp::SystemDefaultsQoS());
    target_pose_pub_r_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/right_cartesian_compliance_controller/target_frame", rclcpp::SystemDefaultsQoS());
    wrench_pub_l_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/left_cartesian_compliance_controller/target_wrench", rclcpp::SystemDefaultsQoS());
    wrench_pub_r_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/right_cartesian_compliance_controller/target_wrench", rclcpp::SystemDefaultsQoS());

    // subscribers
    current_pose_sub_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left_cartesian_compliance_controller/current_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            robot_l_.current_pose = *msg;
            if (!initialized_l_)
            {
                robot_l_.target_pose = *msg;
                robot_l_.start_pose = *msg;
                system_state_.start_time = my_clock_.now();
                initialized_l_ = true;
            }
        });
    current_pose_sub_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right_cartesian_compliance_controller/current_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            robot_r_.current_pose = *msg;
            if (!initialized_r_)
            {
                robot_r_.target_pose = *msg;
                robot_r_.start_pose = *msg;
                system_state_.start_time = my_clock_.now();
                initialized_r_ = true;
            }
        });
    target_monitor_sub_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left_cartesian_compliance_controller/target_frame_monitor", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            robot_l_.target_monitor = *msg;
        });

    target_monitor_sub_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right_cartesian_compliance_controller/target_frame_monitor", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            robot_r_.target_monitor = *msg;
        });

    wrench_sub_l_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/left_cartesian_compliance_controller/current_wrench", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            robot_l_.current_wrench = *msg;
        });
    wrench_sub_r_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/right_cartesian_compliance_controller/current_wrench", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            robot_r_.current_wrench = *msg;
        });
    
    // services
    joint_move_client_l_ = this->create_client<std_srvs::srv::SetBool>("/left_cartesian_compliance_controller/target_joint");
    joint_move_client_r_ = this->create_client<std_srvs::srv::SetBool>("/right_cartesian_compliance_controller/target_joint");

    // system flags reset
    service_flags_ .reset(new ServiceFlags());
}

/* Predefined Functions */

bool RobotMotionBase::move(geometry_msgs::msg::PoseStamped left_target, geometry_msgs::msg::PoseStamped right_target, double duration)
{
    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= duration)
    {
        return true;
    }
    robot_l_.target_pose = intepolation(robot_l_.start_pose, left_target, duration, current_duration);
    robot_r_.target_pose = intepolation(robot_r_.start_pose, right_target, duration, current_duration);
    return false;
}

bool RobotMotionBase::move_wrench(geometry_msgs::msg::PoseStamped left_target, geometry_msgs::msg::PoseStamped right_target, 
                            geometry_msgs::msg::WrenchStamped left_wrench, geometry_msgs::msg::WrenchStamped right_wrench, double duration)
{
    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= duration)
    {
        return true;
    }
    robot_l_.target_pose = intepolation(robot_l_.start_pose, left_target, duration, current_duration);
    robot_r_.target_pose = intepolation(robot_r_.start_pose, right_target, duration, current_duration);
    robot_l_.target_wrench = intepolation_wrench(robot_l_.current_wrench, left_wrench, duration, current_duration);
    robot_r_.target_wrench = intepolation_wrench(robot_r_.current_wrench, right_wrench, duration, current_duration);
    return false;
}

bool RobotMotionBase::sleep(double duration)
{
    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= duration)
    {
        return true;
    }
    return false;
}

bool RobotMotionBase::joint_move(std::vector<double> left_joints, std::vector<double> right_joints, double time)
{
    if (!service_flags_->service_called)
    {
        service_flags_->service_called = true;
        service_flags_->model_joint = true;
        auto left_request = std::make_shared<std_srvs::srv::SetBool::Request>();
        auto right_request = std::make_shared<std_srvs::srv::SetBool::Request>();
        
        auto result_l = joint_move_client_l_->async_send_request(left_request, [this](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture motor_future) {
            auto result = motor_future.get();
            // print result
            RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "Joint left send complete");
            RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "success: %d", result->success);
            bool success = result->success;
            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint left Service Failed");
                rclcpp::shutdown();
            }
            
        });

        auto result_r = joint_move_client_r_->async_send_request(right_request, [this](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture motor_future) {
            auto result = motor_future.get();
            // print result
            RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "Joint right send complete");
            RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "success: %d", result->success);
            bool success = result->success;
            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint right Service Failed");
                rclcpp::shutdown();
            }
            
        });
    }

    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= time+0.5)
    {
        robot_l_.target_pose = robot_l_.target_monitor;
        robot_r_.target_pose = robot_r_.target_monitor;

        service_flags_->service_called = false;
        service_flags_->model_joint = false;

        return true;
    }
    return false;
}

// Execute the task

void RobotMotionBase::task_execute()
{
    auto current_task = tasks_vector_[system_state_.task_num];
    system_state_.current_time = my_clock_.now();
    // The task state for executing the task
    switch (current_task->get_state())
    {
    case TaskState::INIT:
    {
        RCLCPP_INFO(this->get_logger(), "Init task [%d]: %s", system_state_.task_num, current_task->get_name().c_str());
        service_flags_->service_called = false;

        robot_l_.start_pose = robot_l_.target_monitor;
        robot_r_.start_pose = robot_r_.target_monitor;

        system_state_.start_time = my_clock_.now();

        current_task->init();        
        
        current_task->set_state(TaskState::EXECUTE);
        break;
    }

    case TaskState::EXECUTE:
    {
        current_task->execute();
        break;
    }

    case TaskState::FINISH:
    {        

        current_task->set_state(TaskState::INIT);
        RCLCPP_INFO(this->get_logger(), "The task [%d]: %s finished", system_state_.task_num, current_task->get_name().c_str());
        // Check if the task is the last task
        if (system_state_.task_num == tasks_vector_.size() - 1)
        {
            RCLCPP_INFO(this->get_logger(), "All tasks finished");
            active_ = false;
            return;
        }
        system_state_.task_num = system_state_.task_num + next_task_offset_;
        break;
    }
    
    default:
        break;
    }
}

uint8_t RobotMotionBase::task_pushback(std::shared_ptr<robot_motion_base::RobotMotionTask> task)
{        
    tasks_vector_.push_back(task);
    return tasks_vector_.size() - 1;
}

// Start the control loop
void RobotMotionBase::start()
{
    control_loop_thread_ = std::thread(&RobotMotionBase::control_loop, this);
}

void RobotMotionBase::control_loop()
{
    rclcpp::Rate rate(rate_);
    RCLCPP_INFO(this->get_logger(), "Waiting for initialization");   
    while (rclcpp::ok()) 
    {
        if (check_init())
        {
            task_execute();

            if (!service_flags_->model_joint && active_)
            {
                robot_l_.target_pose.header.stamp = this->now();
                robot_r_.target_pose.header.stamp = this->now();
                target_pose_pub_l_->publish(robot_l_.target_pose);
                target_pose_pub_r_->publish(robot_r_.target_pose);

                wrench_pub_l_->publish(robot_l_.target_wrench);
                wrench_pub_r_->publish(robot_r_.target_wrench);
            }
        }
        rate.sleep();
    }
}

}  // namespace robot_motion_base