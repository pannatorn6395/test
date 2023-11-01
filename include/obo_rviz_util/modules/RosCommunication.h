/*******************************
 * RosCommunication.h
 * Class to interfacing between ROS And QT Signaling
 *
 *******************************/
// QT
#include <QObject>
#include <QThread>
#include <QString>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>

// ROS Custom Datatypes
#include "sr1_msgs/MissionItemList.h"
#include "sr1_msgs/WorkerMissionReportList.h"
#include "sr1b_base/MotorState.h"
#include <sr1_charging_dock/ChargingDockStatus.h>

// ROS Custom Services
#include "sr1_state_controller/SetState.h"
#include "std_srvs/Trigger.h"
#include "sr1_msgs/InjectMission.h"
#include "sr1_msgs/MissionItemList.h"
#include "topic_tools/MuxSelect.h"
#include "sr1_light/FlashlightStatus.h"

// ROS Communication Node to Receive the Callback from subscription
class RosCommunication: public QThread{

    Q_OBJECT
    public:
        ros::Subscriber    state_sub;
        ros::Subscriber    clicked_sub;
        ros::Subscriber    safety_mux_sub;
        ros::Subscriber    user_mission_sub;
        ros::Subscriber    worker_mission_sub;
        ros::Subscriber    battery_sub;
        ros::Subscriber    battery_raw_sub;
        ros::Subscriber    drive_mode_sub;
        ros::Subscriber    motor_drive_state_sub;
        ros::Subscriber    motor_steer_state_sub;
        ros::Subscriber    flashlight_state_sub;
        ros::Subscriber    dock_state_sub;

        ros::Publisher     initial_pose_pub_;
        ros::ServiceClient set_state_client_;
        ros::ServiceClient inject_mission_client_;
        ros::ServiceClient inject_user_mission_client_;
        ros::ServiceClient mux_cmd_vel_client_;
        ros::ServiceClient return_home_srv_client_;

        // Constructor
        RosCommunication(ros::NodeHandle *nh);
        ~RosCommunication();

        // ROS Callbacks
        void state_callback(const std_msgs::String::ConstPtr &msg);
        void goal_clicked_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void cmd_mux_monitor_callback(const std_msgs::String::ConstPtr &msg);
        void user_mission_callback(const sr1_msgs::MissionItemList::ConstPtr &msg);
        void worker_mission_callback(const sr1_msgs::WorkerMissionReportList::ConstPtr &msg);
        void battery_sub_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
        void battery_raw_sub_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
        void drive_mode_sub_callback(const std_msgs::String::ConstPtr &msg);
        void motor_drive_sub_callback(const sr1b_base::MotorState::ConstPtr &msg);
        void motor_steer_sub_callback(const sr1b_base::MotorState::ConstPtr &msg);
        void flashlight_sub_callback(const sr1_light::FlashlightStatus::ConstPtr &msg);
        void dock_state_sub_callback(const sr1_charging_dock::ChargingDockStatus::ConstPtr &msg);

    //Use Q_SIGNALS, Q_SLOTS macro to replace the signals keyword in class declarations
    //when you want to use Qt Signals and Slots with a 3rd party signal/slot mechanism.
    Q_SIGNALS:
        void state_signal(QString);
        void mux_topic_signal(QString);
        void user_mission_signal(std::vector<QString>);
        void worker_mission_signal(std::vector<QString>);
        void clicked_point_signal(double, double, double, QString);
        void battery_state_signal(sensor_msgs::BatteryState);
        void battery_raw_state_signal(sensor_msgs::BatteryState);
        void drive_mode_signal(std_msgs::String);
        void motor_drive_state_signal(sr1b_base::MotorState);
        void motor_steer_state_signal(sr1b_base::MotorState);
        void flashlight_state_signal(sr1_light::FlashlightStatus);
        void dock_state_signal(sr1_charging_dock::ChargingDockStatus);

    protected Q_SLOTS:
        void handleQuitSignal();

};