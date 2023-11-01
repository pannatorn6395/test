/*********************************
* ControlPanel.cpp
* Rviz Panel for controlling SR1 within Robot
* Author : Theppasith N. <theppasith.n@hiveground.com>
* 12-Dec-2018
*********************************/
#include <stdio.h>
#include <iostream>
#include <iterator>

#include "obo_rviz_util/ControlPanel.h"

namespace obo_rviz_util
{
    // Constructor
    ControlPanel::ControlPanel(QWidget * parent) : rviz::Panel(parent)
    {
        qRegisterMetaType<QVector<int> >();
        // Create ROS Communication Class
        RosCommunication *ros_comm = new RosCommunication(&nh_);

        // Publisher
        set_state_client_ = nh_.serviceClient<sr1_state_controller::SetState>("/sr1_state_machine/set_state");
        return_home_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/sr1_mission_executor/return_home");
        inject_mission_client_ = nh_.serviceClient<sr1_msgs::InjectMission>("/sr1_mission_manager/inject_mission");
        inject_user_mission_client_ = nh_.serviceClient<std_srvs::Trigger>("/sr1_mission_executor/inject_user_mission");
        mux_cmd_vel_client_ = nh_.serviceClient<topic_tools::MuxSelect>("/mux_cmd_vel/select");

        // Tab1
        mission_display_widget = new MissionDisplayWidget(this, &nh_);
        // Tab2
        state_display_widget = new StateDisplayWidget(this, &nh_);
        // Tab3
        motor_display_widget = new MotorDisplayWidget(this, &nh_);
        // Tab4
        dock_display_widget = new DockDisplayWidget(this, &nh_);
        // Signal handling
        connect(
            ros_comm,
            SIGNAL(state_signal(QString)),
            this,
            SLOT(handleStateSignal(QString))
        );
        connect(
            ros_comm,
            SIGNAL(clicked_point_signal(
                double, double, double, QString)
            ),
            this,
            SLOT(
                handleClickedPointSignal(
                double, double, double, QString)
            )
        );

        connect(
            ros_comm,
            SIGNAL(mux_topic_signal(QString)),
            this,
            SLOT(handleMuxtopicSignal(QString))
        );

        connect(
            ros_comm,
            SIGNAL(user_mission_signal(std::vector<QString>)),
            mission_display_widget,
            SLOT(update_user_mission_item(std::vector<QString>))
        );

        connect(
            ros_comm,
            SIGNAL(worker_mission_signal(std::vector<QString>)),
            mission_display_widget,
            SLOT(update_worker_mission_item(std::vector<QString>))
        );

        connect(
            ros_comm,
            SIGNAL(battery_state_signal(sensor_msgs::BatteryState)),
            this,
            SLOT(handleBatteryStateSignal(sensor_msgs::BatteryState))
        );

        connect(
            ros_comm,
            SIGNAL(battery_raw_state_signal(sensor_msgs::BatteryState)),
            this,
            SLOT(handleRawBatteryStateSignal(sensor_msgs::BatteryState))
        );

        connect(
            ros_comm,
            SIGNAL(drive_mode_signal(std_msgs::String)),
            this,
            SLOT(handleDriveModeSignal(std_msgs::String))
        );

        connect(
            ros_comm,
            SIGNAL(motor_drive_state_signal(sr1b_base::MotorState)),
            this,
            SLOT(handleMotorDriveSignal(sr1b_base::MotorState))
        );

        connect(
            ros_comm,
            SIGNAL(flashlight_state_signal(sr1_light::FlashlightStatus)),
            this,
            SLOT(handleFlashlightStatusSignal(sr1_light::FlashlightStatus))
        );

        connect(
            ros_comm,
            SIGNAL(motor_steer_state_signal(sr1b_base::MotorState)),
            this,
            SLOT(handleMotorSteerSignal(sr1b_base::MotorState))
        );

        connect(
            ros_comm,
            SIGNAL(dock_state_signal(sr1_charging_dock::ChargingDockStatus)),
            dock_display_widget,
            SLOT(update_dock_station(sr1_charging_dock::ChargingDockStatus))
        );

        connect(
            this,
            SIGNAL(quit_signal()),
            ros_comm,
            SLOT(handleQuitSignal())
        );

        // Move To Thread
        commThread = new QThread;
        ros_comm->moveToThread(commThread);
        // Init GUI
        init_gui();
    }
    // Destructor
    ControlPanel::~ControlPanel()
    {
        ROS_INFO("[SR1Panel] Quitting Monitoring System");
        Q_EMIT quit_signal();
        commThread->exit();
        ROS_INFO("[SR1Panel] ROS Communication Thread Exited !");
    }

    // Virtual implement
    void ControlPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }
    void ControlPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void ControlPanel::handleFlashlightStatusSignal(sr1_light::FlashlightStatus msg)
    {
        motor_display_widget->update_flashlight_status(msg);
    }

    void ControlPanel::handleBatteryStateSignal(sensor_msgs::BatteryState msg)
    {
        state_display_widget->update_avg_battery(msg);
    }

    void ControlPanel::handleRawBatteryStateSignal(sensor_msgs::BatteryState msg)
    {
        state_display_widget->update_raw_battery(msg);
    }

    void ControlPanel::handleStateSignal(QString received_string)
    {
        current_state_ = received_string;
        current_state_textbox_->setText(current_state_);
    }

    void ControlPanel::handleDriveModeSignal(std_msgs::String msg)
    {
        motor_display_widget->update_drive_mode(msg);
    }

    void ControlPanel::handleMotorDriveSignal(sr1b_base::MotorState msg)
    {
        motor_display_widget->update_drive_status(msg);
    }

    void ControlPanel::handleMotorSteerSignal(sr1b_base::MotorState msg)
    {
        motor_display_widget->update_steer_status(msg);
    }

    void ControlPanel::handleMuxtopicSignal(QString topic)
    {
        std::string current_topic = topic.toStdString();
        if(current_topic == "/cmd_vel/filtered/state_cmd_vel")
        {
            radioButton->setChecked(false);
            radioButton_2->setChecked(true);
        }
        if(current_topic == "/cmd_vel/filtered/bumper_cmd_vel")
        {
            radioButton->setChecked(true);
            radioButton_2->setChecked(false);
        }
    }

    void ControlPanel::handleRadioClicked()
    {
        // SAFETY
        muxSelect("/cmd_vel/filtered/bumper_cmd_vel");
    }

    void ControlPanel::handleRadio2Clicked()
    {
        // DIRECT
        muxSelect("/cmd_vel/filtered/state_cmd_vel");
    }

    bool ControlPanel::muxSelect(std::string topic)
    {
        // Service Call to sr1_mission_manager/inject_mission
        topic_tools::MuxSelect rpc;
        if(topic == "/cmd_vel/filtered/state_cmd_vel" ||
            topic == "/cmd_vel/filtered/bumper_cmd_vel")
        {
            rpc.request.topic = topic;
            mux_cmd_vel_client_.call(rpc);
            return true;
        }
        return false;
    }

    void ControlPanel::handleClickedPointSignal(
        double x, double y , double z , QString frame)
    {
        // Convert to Semantic pos
        // Create input for tf system
        std::string current_frame = frame.toStdString();
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0);
        tf::Stamped<tf::Pose> input = tf::Stamped<tf::Pose>(
                tf::Pose(quat, tf::Point(x, y, z)),
                ros::Time::now(),
                current_frame
        );
        tf::Stamped<tf::Pose> output;
        try
        {
            listener.transformPose(
                "semantic", ros::Time(0),
                input,
                current_frame,
                output
            );
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Cannot find corresponding frame - Ignore pose resetting");
            return;
        }

        // Do the convertion
        geometry_msgs::PoseStamped goal_pose;
        tf::poseStampedTFToMsg(output, goal_pose);
        QString x_val = QString::fromStdString(
            std::to_string(
                goal_pose.pose.position.x
            )
        );
        QString y_val = QString::fromStdString(
            std::to_string(
                goal_pose.pose.position.y
            )
        );
        QString z_val = QString::fromStdString(
            std::to_string(
                goal_pose.pose.position.z
            )
        );
        // Set Result
        x_pos_textbox_->setText(x_val);
        y_pos_textbox_->setText(y_val);
        z_pos_textbox_->setText(z_val);
    }

    void ControlPanel::handleGoHomeButton()
    {
        ROS_INFO("GO HOME BUTTON PRESSED !");

        std_srvs::Trigger trigger_payload;
        bool result = return_home_srv_client_.call(trigger_payload);
        if(result)
        {
            ROS_INFO("Go Home User Mission is Injected !");
        }
        else
        {
            ROS_ERROR("Go Home User Mission can't call service");
        }
    }

    void ControlPanel::handleInjectButton()
    {
        // Gathered the values from textboxes
        double x = x_pos_textbox_->text().toDouble();
        double y = y_pos_textbox_->text().toDouble();
        double z = z_pos_textbox_->text().toDouble();
        ROS_INFO("x = %lf , y = %lf ,z = %lf", x, y ,z);

        inject_mission(x,y,z);
    }

    bool ControlPanel::inject_mission(double x , double y , double z)
    {
        // Service Call to sr1_mission_manager/inject_mission
        sr1_msgs::InjectMission inject_request;
        inject_request.request.item.x_pos = x;
        inject_request.request.item.y_pos = y;
        inject_request.request.item.z_pos = z;

        bool result = inject_mission_client_.call(inject_request);

        std_srvs::Trigger inject_user_mission_request;

        result = result && inject_user_mission_client_.call(inject_user_mission_request);

        if(result){
            ROS_INFO("Mission Inject SUCCESS");
            return true;
        }else{
            ROS_WARN("MISSION_inject Failed");
            return false;
        }
    }

    // Set Robot pose !
    bool ControlPanel::do_set_robot_pose(geometry_msgs::PoseStamped pose)
    {
        // Guaranteed World Frame ID when arrived
        set_state("stop_mission");
        set_initial_worker(pose.pose, pose.header.frame_id);
        return true;
    }

    void ControlPanel::handleReleaseManualClick()
    {
        set_state("release_manual");
    }

    void ControlPanel::handleStartClick()
    {
        set_state("start_mission");
        ROS_INFO("START_MISSION SUCCESS");
    }
    void ControlPanel::handleStopClick()
    {
        set_state("stop_mission");
        ROS_INFO("STOP_MISSION SUCCESS");
    }
    void ControlPanel::set_initial_worker(geometry_msgs::Pose pose, std::string frame_id)
    {
        // Create PoseStamped
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header.frame_id = frame_id;

        // Transform pose to current map frame
        tf::Stamped<tf::Pose> output;
        tf::Stamped<tf::Pose> world_tf;
        tf::poseStampedMsgToTF(pose_stamped, world_tf);

        try
        {
            listener_.transformPose("map", ros::Time(0) , world_tf , world_tf.frame_id_ , output);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        geometry_msgs::PoseStamped current_map_to_robot_pose;
        tf::poseStampedTFToMsg(output, current_map_to_robot_pose);

        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.pose.pose = current_map_to_robot_pose.pose;
        // initial_pose.pose.pose = pose;

        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = ros::Time::now();

        // Covariance
        initial_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
        initial_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
        initial_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
        ROS_INFO("Setting Initial Worker !");
        initial_pose_pub_.publish(initial_pose);
    }


    bool ControlPanel::set_state(std::string trigger)
    {
        sr1_state_controller::SetState srv;
        srv.request.state = trigger;
        bool result = set_state_client_.call(srv);
        if(result)
        {
            ROS_INFO("Set State with trigger : %s", trigger.c_str());
            return true;
        }
        else
        {
            ROS_ERROR("Set State with trigger : %s", trigger.c_str());
            return false;
        }
    }

    QHBoxLayout* ControlPanel::generate_line()
    {
        // Create Layout which contain hr line
        QHBoxLayout *lineLayout = new QHBoxLayout;
        QFrame* myFrame1 = new QFrame();
        myFrame1->setFrameShape(QFrame::HLine);
        lineLayout->addWidget(myFrame1);
        return lineLayout;
    }

    void ControlPanel::init_gui()
    {
        //Title
        QHBoxLayout *header = new QHBoxLayout;
        QLabel *headerLabel = new QLabel("SR1 Control Panel");
        QFont head_font = headerLabel->font();
        head_font.setPointSize(20);
        headerLabel->setFont(head_font);
        headerLabel->setAlignment(Qt::AlignCenter);
        header->addWidget(headerLabel);


        // State Display box
        QHBoxLayout *state_box = new QHBoxLayout;
        QLabel *state_title = new QLabel("STATE : ");
        current_state_textbox_ = new QLineEdit();
        current_state_textbox_->setReadOnly(true);
        current_state_textbox_->setAlignment(Qt::AlignCenter);

        // Display State
        state_box->addWidget(state_title);
        state_box->addWidget(current_state_textbox_);

        // Display Buttons
        QHBoxLayout *rel_home_layout = new QHBoxLayout;
        QPushButton *release_button = new QPushButton("Release Manual");
        QPushButton *home_set_button = new QPushButton("Go Home !");
        rel_home_layout->addWidget(release_button);
        rel_home_layout->addWidget(home_set_button);

        // Horizontal Line
        QHBoxLayout *linePane3 = new QHBoxLayout;
        QFrame* myFrame3 = new QFrame();
        myFrame3->setFrameShape(QFrame::HLine);
        linePane3->addWidget(myFrame3);
        // Buttons
        QPushButton *start_button = new QPushButton("START");
        QPushButton *stop_button = new QPushButton("STOP");
        QPushButton *inject_button = new QPushButton("INJECT");

        // Mission Injection box
        QVBoxLayout *mission_inject_box = new QVBoxLayout;
        QHBoxLayout *first_line_box = new QHBoxLayout;
        QHBoxLayout *second_line_box = new QHBoxLayout;
        QHBoxLayout *z_box = new QHBoxLayout;
        // QHBoxLayout *btn_box = new QHBoxLayout;

        // X POS
        QLabel *x_pos_label = new QLabel("x:");
        x_pos_textbox_ = new QLineEdit();
        x_pos_textbox_->setText("0.00");
        x_pos_textbox_->setAlignment(Qt::AlignCenter);
        // Y POS
        QLabel *y_pos_label = new QLabel("y:");
        y_pos_textbox_ = new QLineEdit();
        y_pos_textbox_->setText("0.00");
        y_pos_textbox_->setAlignment(Qt::AlignCenter);
        // Z POS
        QLabel *z_pos_label = new QLabel("z:");
        z_pos_textbox_ = new QLineEdit();
        z_pos_textbox_->setText("0.00");
        z_pos_textbox_->setAlignment(Qt::AlignCenter);


        // Add Label , Textbox
        first_line_box->addWidget(x_pos_label);
        first_line_box->addWidget(x_pos_textbox_);
        first_line_box->addWidget(y_pos_label);
        first_line_box->addWidget(y_pos_textbox_);
        first_line_box->addWidget(z_pos_label);
        first_line_box->addWidget(z_pos_textbox_);
        second_line_box->addWidget(start_button);
        second_line_box->addWidget(stop_button);
        second_line_box->addWidget(inject_button);

        // Mux Selector
        QHBoxLayout *safety_box = new QHBoxLayout;
        QLabel *safety_label = new QLabel("DRIVE : ");
        radioButton = new QRadioButton("SAFETY");
        radioButton_2 = new QRadioButton("STATE");
        safety_box->addWidget(safety_label);
        safety_box->addWidget(radioButton);
        safety_box->addWidget(radioButton_2);

        // Tab Area
        QHBoxLayout *tab_box = new QHBoxLayout;
        QTabWidget *tab = new QTabWidget();

        QWidget *tab1 = new QWidget();
        QWidget *tab2 = new QWidget();
        QWidget *tab3 = new QWidget();
        QWidget *tab4 = new QWidget();

        QVBoxLayout *tab1_layout = new QVBoxLayout;
        QVBoxLayout *tab2_layout = new QVBoxLayout;
        QVBoxLayout *tab3_layout = new QVBoxLayout;
        QVBoxLayout *tab4_layout = new QVBoxLayout;

        // First Tab
        tab1_layout->addWidget(mission_display_widget);
        tab1_layout->addLayout(first_line_box);
        tab1_layout->addLayout(second_line_box);
        tab1_layout->addLayout(z_box);
        tab1->setLayout(tab1_layout);

        // Second Tab
        tab2_layout->addWidget(state_display_widget);
        tab2->setLayout(tab2_layout);

        // Third Tab
        tab3_layout->addWidget(motor_display_widget);
        tab3->setLayout(tab3_layout);

        // Fourth Tab
        tab4_layout->addWidget(dock_display_widget);
        tab4->setLayout(tab4_layout);

        // Add All Tabs
        tab->addTab(tab1,QString("Mission"));
        tab->addTab(tab2,QString("Battery"));
        tab->addTab(tab3,QString("Motor"));
        tab->addTab(tab4,QString("Dock"));

        tab_box->addWidget(tab);

        // Main Layout
        QVBoxLayout *layout = new QVBoxLayout;
        // layout->addLayout(header);
        // layout->addLayout(linePane1);
        layout->addLayout(state_box);
        layout->addLayout(rel_home_layout);
        layout->addLayout(generate_line());
        layout->addLayout(tab_box);
        layout->addLayout(generate_line());
        layout->addLayout(safety_box);
        layout->setAlignment(Qt::AlignTop);
        setLayout(layout);

        // Connect Event in this ui
        connect(
            home_set_button,
            SIGNAL(clicked()),
            this,
            SLOT(handleGoHomeButton())
        );
        connect(
            start_button,
            SIGNAL(clicked()),
            this,
            SLOT(handleStartClick())
        );
        connect(
            stop_button,
            SIGNAL(clicked()),
            this,
            SLOT(handleStopClick())
        );
        connect(
            inject_button,
            SIGNAL(clicked()),
            this,
            SLOT(handleInjectButton())
        );
        connect(
            release_button,
            SIGNAL(clicked()),
            this,
            SLOT(handleReleaseManualClick())
        );
        connect(
            radioButton,
            SIGNAL(clicked()),
            this,
            SLOT(handleRadioClicked())
        );
        connect(
            radioButton_2,
            SIGNAL(clicked()),
            this,
            SLOT(handleRadio2Clicked())
        );
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obo_rviz_util::ControlPanel, rviz::Panel)