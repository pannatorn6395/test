/*********************************
* ControlPanel.h
* Rviz Panel for controlling SR1 within Robot
* Author : Theppasith N. <theppasith.n@hiveground.com>
* 12-Dec-2018
*********************************/
// QT Stuff
#include <QObject>
#include <QString>
#include <QThread>

// UI Stuff
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QLayout>
#include <QPushButton>
#include <QVector>
#include <QGroupBox>
#include <QRadioButton>
#include <QTableWidget>
#include <QListWidget>
#include <QScrollArea>

// ROS Stuff
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
// RVIZ Stuff
#include <rviz/panel.h>

#include <obo_rviz_util/modules/RosCommunication.h>
#include <obo_rviz_util/modules/MissionDisplayWidget.h>
#include <obo_rviz_util/modules/StateDisplayWidget.h>
#include <obo_rviz_util/modules/MotorDisplayWidget.h>
#include <obo_rviz_util/modules/DockDisplayWidget.h>

// SR1 Stuff
#include "sr1_state_controller/SetState.h"
#include "sr1_msgs/InjectMission.h"
#include "sr1_msgs/MissionItemList.h"
#include "sr1_msgs/WorkerMissionReportList.h"
#include "std_srvs/Trigger.h"
#include "sr1_light/FlashlightStatus.h"
#include "sr1_charging_dock/ChargingDockStatus.h"
#include "topic_tools/MuxSelect.h"


namespace obo_rviz_util{
    class ControlPanel: public rviz::Panel{
        Q_OBJECT
        QThread *commThread;

        public:
            // Constructor & Destructor
            ControlPanel(QWidget *parent = 0);
            ~ControlPanel();
            // Rviz Mandatory virtual abstraction
            virtual void load(const rviz::Config &config);
            virtual void save(rviz::Config config) const;
            tf::TransformListener listener;
            // ROS
            ros::NodeHandle nh_;

        protected Q_SLOTS:
            // Signal handling to update UI
            void handleStateSignal(QString);
            void handleClickedPointSignal(
                double, double, double, QString);
            void handleMuxtopicSignal(QString);
            void handleBatteryStateSignal(sensor_msgs::BatteryState);
            void handleRawBatteryStateSignal(sensor_msgs::BatteryState);
            void handleDriveModeSignal(std_msgs::String);
            void handleMotorDriveSignal(sr1b_base::MotorState);
            void handleMotorSteerSignal(sr1b_base::MotorState);
            void handleFlashlightStatusSignal(sr1_light::FlashlightStatus);

            // Click
            void handleGoHomeButton();
            void handleInjectButton();
            void handleStartClick();
            void handleStopClick();
            void handleReleaseManualClick();
            void handleRadioClicked();
            void handleRadio2Clicked();

        private:
            void init_gui();
            QHBoxLayout* generate_line();
            // Set Pose
            tf::TransformListener listener_;
            bool do_set_robot_pose(geometry_msgs::PoseStamped pose);
            void set_initial_worker(geometry_msgs::Pose pose, std::string frame_id);
            bool set_state(std::string trigger);
            // Inject Mission
            bool inject_mission(double x , double y , double z);
            bool muxSelect(std::string topic);
            // Current State
            QString current_state_;
            QLineEdit *current_state_textbox_;

            // Position Textbot (global scope for callback accessing)
            QLineEdit *x_pos_textbox_;
            QLineEdit *y_pos_textbox_;
            QLineEdit *z_pos_textbox_;
            QRadioButton *radioButton;
            QRadioButton *radioButton_2;

            QListWidget *user_mission_list_widget;
            QListWidget *worker_mission_list_widget;
            StateDisplayWidget *state_display_widget;
            MotorDisplayWidget *motor_display_widget;
            MissionDisplayWidget *mission_display_widget;
            DockDisplayWidget *dock_display_widget;

            // Set State
            ros::Publisher  initial_pose_pub_;
            ros::ServiceClient set_state_client_;
            ros::ServiceClient inject_mission_client_;
            ros::ServiceClient inject_user_mission_client_;
            ros::ServiceClient mux_cmd_vel_client_;
            ros::ServiceClient return_home_srv_client_;


        Q_SIGNALS:
            void quit_signal();

    };
}//namespace obo_rviz_util

