//
// Created by jh on 25. 2. 3.
//

#include "ROSCommunication.hpp"

extern pCAMEL_SHM sharedCamel;

ROSCommunication::ROSCommunication(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("canine_middleware", opts)
{
    subscription_canine_command = this->create_subscription<canine_msgs_v2::msg::CANINECommand>(
        "canine_command", 10,std::bind(&ROSCommunication::topic_callback_canine_command, this, std::placeholders::_1));
    publisher_canine_states = this->create_publisher<canine_msgs_v2::msg::CANINEState>("canine_states", 10);
    timer_canine_states = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&ROSCommunication::publishStates, this));
}

void ROSCommunication::publishStates()
{
    if (sharedCamel->bIsConnect)
    {
        auto msg = canine_msgs_v2::msg::CANINEState();
        package_canine_state_msg(msg);
        publisher_canine_states->publish(msg);
    }
}

void ROSCommunication::package_canine_state_msg(canine_msgs_v2::msg::CANINEState& msg)
{
    msg.local_time = sharedCamel->CAMEL_DATA_NEW.middlewareData.localTime;
    msg.fsm_state = sharedCamel->CAMEL_DATA_NEW.middlewareData.FSMState;

    msg.global_base_position.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.basePosition.x();
    msg.global_base_position.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.basePosition.y();
    msg.global_base_position.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.basePosition.z();
    msg.global_base_velocity.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.baseVelocity.x();
    msg.global_base_velocity.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.baseVelocity.y();
    msg.global_base_velocity.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.baseVelocity.z();
    msg.global_base_euler_angle.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.rpy.x();
    msg.global_base_euler_angle.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.rpy.y();
    msg.global_base_euler_angle.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.rpy.z();
    msg.global_base_quaternion.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.quat[1];
    msg.global_base_quaternion.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.quat[2];
    msg.global_base_quaternion.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.quat[3];
    msg.global_base_quaternion.w = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.quat[0];
    msg.global_base_angular_velocity.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.baseAngularVelocity.x();
    msg.global_base_angular_velocity.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.baseAngularVelocity.y();
    msg.global_base_angular_velocity.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.global.baseAngularVelocity.z();

    msg.body_base_velocity.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.body.baseVelocity.x();
    msg.body_base_velocity.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.body.baseVelocity.y();
    msg.body_base_velocity.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.body.baseVelocity.z();
    msg.body_base_angular_velocity.x = sharedCamel->CAMEL_DATA_NEW.middlewareData.body.baseAngularVelocity.x();
    msg.body_base_angular_velocity.y = sharedCamel->CAMEL_DATA_NEW.middlewareData.body.baseAngularVelocity.y();
    msg.body_base_angular_velocity.z = sharedCamel->CAMEL_DATA_NEW.middlewareData.body.baseAngularVelocity.z();
}

void ROSCommunication::topic_callback_canine_command(const canine_msgs_v2::msg::CANINECommand::SharedPtr msg) const
{
    msg->reference_base_velocity.resize(2);

    sharedCamel->ros2Data.prevCommand = sharedCamel->ros2Data.command;
    sharedCamel->ros2Data.command = msg->command;
    sharedCamel->ros2Data.baseRefVelX = msg->reference_base_velocity[0];
    sharedCamel->ros2Data.baseRefVelY = msg->reference_base_velocity[1];
    sharedCamel->ros2Data.baseRefAngVelZ = msg->reference_base_yaw_velocity;

    if (sharedCamel->ros2Data.command != sharedCamel->ros2Data.prevCommand)
    {
        sharedCamel->ros2Data.bNewCommand = true;
    }
}


