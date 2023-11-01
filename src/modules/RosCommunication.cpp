/*********************************
* RosCommunication.cpp
* Rviz Panel for controlling SR1 within Robot
* Author : Theppasith N. <theppasith.n@hiveground.com>
* Date : 13-July-2020
*********************************/

#include "obo_rviz_util/modules/RosCommunication.h"

// Constructor
RosCommunication::RosCommunication(ros::NodeHandle *nh)
{
    state_sub = nh->subscribe<std_msgs::String>(
        "/sr1_state_machine/current_state",
        1,
        &RosCommunication::state_callback,
        this
    );

    clicked_sub = nh->subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal",
        1,
        &RosCommunication::goal_clicked_callback,
        this
    );

    safety_mux_sub = nh->subscribe<std_msgs::String>(
        "/mux_cmd_vel/selected",
        1,
        &RosCommunication::cmd_mux_monitor_callback,
        this
    );
    user_mission_sub = nh->subscribe<sr1_msgs::MissionItemList>(
        "/sr1_mission_executor/user_mission_item",
        1,
        &RosCommunication::user_mission_callback,
        this
    );
    worker_mission_sub = nh->subscribe<sr1_msgs::WorkerMissionReportList>(
        "/sr1_mission_executor/worker_mission_item",
        1,
        &RosCommunication::worker_mission_callback,
        this
    );
    battery_sub = nh->subscribe<sensor_msgs::BatteryState>(
        "/battery/data",
        1,
        &RosCommunication::battery_sub_callback,
        this
    );

    battery_raw_sub = nh->subscribe<sensor_msgs::BatteryState>(
        "/battery/raw",
        10,
        &RosCommunication::battery_raw_sub_callback,
        this
    );

    drive_mode_sub = nh->subscribe<std_msgs::String>(
        "/robot_base/drive_mode",
        5,
        &RosCommunication::drive_mode_sub_callback,
        this
    );

    motor_drive_state_sub = nh->subscribe<sr1b_base::MotorState>(
        "/device/wheel_drive/data",
        10,
        &RosCommunication::motor_drive_sub_callback,
        this
    );
    motor_steer_state_sub = nh->subscribe<sr1b_base::MotorState>(
        "/device/wheel_steering/data",
        10,
        &RosCommunication::motor_steer_sub_callback,
        this
    );

    flashlight_state_sub = nh->subscribe<sr1_light::FlashlightStatus>(
        "/light_node/flashlight_status",
        10,
        &RosCommunication::flashlight_sub_callback,
        this
    );

    dock_state_sub = nh->subscribe<sr1_charging_dock::ChargingDockStatus>(
        "/charging_dock/data",
        10,
        &RosCommunication::dock_state_sub_callback,
        this
    );

    set_state_client_ = nh->serviceClient<sr1_state_controller::SetState>(
        "/sr1_state_machine/set_state"
    );
    return_home_srv_client_ = nh->serviceClient<std_srvs::Trigger>(
        "/sr1_mission_executor/return_home"
    );
    inject_mission_client_ = nh->serviceClient<sr1_msgs::InjectMission>(
        "/sr1_mission_manager/inject_mission"
    );
    inject_user_mission_client_ = nh->serviceClient<std_srvs::Trigger>(
        "/sr1_mission_executor/inject_user_mission"
    );
    mux_cmd_vel_client_ = nh->serviceClient<topic_tools::MuxSelect>(
        "/mux_cmd_vel/select"
    );

    ROS_INFO("[SR1Panel] ROS Communication Thread Start");
    start();
}
// Destructor
RosCommunication::~RosCommunication()
{
    ROS_INFO("[SR1Panel] ROS Communication Thread Stop");
    // if(ros::isStarted())
    // {
    //     ros::shutdown();
    //     ros::waitForShutdown();
    // }
    // wait();
}
void RosCommunication::dock_state_sub_callback(const sr1_charging_dock::ChargingDockStatus::ConstPtr& msg)
{
    sr1_charging_dock::ChargingDockStatus dock_status = *msg;
    Q_EMIT dock_state_signal(dock_status);
}
void RosCommunication::flashlight_sub_callback(const sr1_light::FlashlightStatus::ConstPtr &msg)
{
    sr1_light::FlashlightStatus flashlight_status = *msg;
    Q_EMIT flashlight_state_signal(flashlight_status);
}

void RosCommunication::battery_sub_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    sensor_msgs::BatteryState bat_state = *msg;
    Q_EMIT battery_state_signal(bat_state);
}

void RosCommunication::battery_raw_sub_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    sensor_msgs::BatteryState bat_state = *msg;
    Q_EMIT battery_raw_state_signal(bat_state);
}

void RosCommunication::drive_mode_sub_callback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String drive_mode = *msg;
    Q_EMIT drive_mode_signal(drive_mode);
}

void RosCommunication::motor_drive_sub_callback(const sr1b_base::MotorState::ConstPtr &msg)
{
    sr1b_base::MotorState drive_state = *msg;
    Q_EMIT motor_drive_state_signal(drive_state);
}
void RosCommunication::motor_steer_sub_callback(const sr1b_base::MotorState::ConstPtr &msg)
{
    sr1b_base::MotorState steer_state = *msg;
    Q_EMIT motor_steer_state_signal(steer_state);
}


void RosCommunication::state_callback(const std_msgs::String::ConstPtr &msg)
{
    QString state = msg->data.c_str();
    Q_EMIT state_signal(state);
}

void RosCommunication::goal_clicked_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    QString frame = msg->header.frame_id.c_str();
    Q_EMIT clicked_point_signal(x,y,z,frame);
}

void RosCommunication::cmd_mux_monitor_callback(const std_msgs::String::ConstPtr &msg)
{
    QString topic = msg->data.c_str();
    Q_EMIT mux_topic_signal(topic);
}

void RosCommunication::user_mission_callback(const sr1_msgs::MissionItemList::ConstPtr &msg)
{
    std::vector<QString> result;
    for(int i = 0 ; i < msg->missionList.size() ; i++){
        std::ostringstream stringStream;
        stringStream << (i+1) << ". "
        << msg->missionList[i].x_pos
        << ","
        << msg->missionList[i].y_pos
        << ","
        << msg->missionList[i].z_pos;

        std::string item_str = stringStream.str();

        result.push_back(QString(item_str.c_str()));
    }
    Q_EMIT user_mission_signal(result);
}

void RosCommunication::worker_mission_callback(const sr1_msgs::WorkerMissionReportList::ConstPtr &msg)
{
    std::vector<QString> result;
    for(int i = 0 ; i < msg->worker_mission_list.size() ; i++){
        std::ostringstream stringStream;
        stringStream << (i+1) << ". "
        << msg->worker_mission_list[i].mission_type.substr(0,5)
        << "@" << msg->worker_mission_list[i].map_id
        << "([" << msg->worker_mission_list[i].start_node
        << "]->[" << msg->worker_mission_list[i].end_node << "])" ;
        std::string item_str = stringStream.str();

        result.push_back(QString(item_str.c_str()));
    }
    Q_EMIT worker_mission_signal(result);
}

void RosCommunication::handleQuitSignal()
{
    ROS_INFO("[ROS Comm] Receiving Quitting Thread Signal");
}