/*******************************
 * DockDisplayWidget.cpp
 * Mission Display Widgets
 * Date : 01-April-2021
 * Maintainer : Theppasith N. <theppasith.n@obodroid.com>
*******************************/

#include <obo_rviz_util/modules/DockDisplayWidget.h>

DockDisplayWidget::DockDisplayWidget(QWidget *parent, ros::NodeHandle *nh){
    ROS_INFO("[DockCtrl] Contructor Called !");

    bluetooth_connected = new QLabel("-");
    idle = new QLabel("-");
    docking = new QLabel("-");
    standby = new QLabel("-");
    charging = new QLabel("-");
    error = new QLabel("-");
    timeout = new QLabel("-");

    dock_enable_btn = new QPushButton("Dock Enable");
    dock_disable_btn = new QPushButton("Dock Disable");
    charging_enable_btn = new QPushButton("Charge Enable");
    charging_disable_btn = new QPushButton("Charge Disable");
    reset_dock_btn = new QPushButton("Reset");

    debug_request_undock_btn = new QPushButton("Req-Undock");
    debug_start_dock_btn = new QPushButton("St-Dock");
    debug_preempt_btn = new QPushButton("Preempt");
    debug_start_undock_btn = new QPushButton("St-Undock");

    dock_enable_client_ = nh->serviceClient<std_srvs::Trigger>("/charging_dock/docking_enable");
    dock_disable_client_ = nh->serviceClient<std_srvs::Trigger>("/charging_dock/docking_disable");
    charging_enable_client_ = nh->serviceClient<std_srvs::Trigger>("/charging_dock/charging_enable");
    charging_disable_client_ = nh->serviceClient<std_srvs::Trigger>("/charging_dock/charging_disable");
    reset_dock_client_ = nh->serviceClient<std_srvs::Trigger>("/charging_dock/reset");

    request_undock_client_ = nh->serviceClient<std_srvs::Trigger>("/docking_worker/request_undocking");
    debug_start_client_ = nh->serviceClient<std_srvs::Trigger>("/docking_worker/debug/start");
    debug_preempt_client_ = nh->serviceClient<std_srvs::Trigger>("/docking_worker/debug/stop");
    debug_undock_client_ = nh->serviceClient<std_srvs::Trigger>("/docking_worker/debug/undock");
    // Create UI
    create_ui_();
    create_connect_();
}

DockDisplayWidget::~DockDisplayWidget()
{
    ROS_INFO("[DockCtrl] Destructor Called");
}

void DockDisplayWidget::update_dock_station(sr1_charging_dock::ChargingDockStatus msg)
{
    QString st_bluetooth_connected = QString::fromStdString((msg.bluetooth_connected == true)? "True":"False");
    QString st_idle = QString::fromStdString((msg.idle == true)? "True":"False");
    QString st_docking = QString::fromStdString((msg.docking == true)? "True":"False");
    QString st_standby = QString::fromStdString((msg.standby == true)? "True":"False");
    QString st_charging = QString::fromStdString((msg.charging == true)? "True":"False");
    QString st_error = QString::fromStdString((msg.error == true)? "True":"False");
    QString st_timeout = QString::fromStdString((msg.timeout == true)? "True":"False");

    bluetooth_connected->setText(st_bluetooth_connected);
    idle->setText(st_idle);
    docking->setText(st_docking);
    standby->setText(st_standby);
    charging->setText(st_charging);
    error->setText(st_error);
    timeout->setText(st_timeout);
}

void DockDisplayWidget::create_ui_()
{
/*
"bool bluetooth_connected\n"
"bool idle\n"
"bool docking\n"
"bool standby\n"
"bool charging\n"
"bool error\n"
"bool timeout\n"
*/
    QVBoxLayout *main_layout = new QVBoxLayout();

    QHBoxLayout *bluetooth_connected_layout = new QHBoxLayout();
    QLabel *bluetooth_connected_label = new QLabel("bluetooth :");
    bluetooth_connected_layout->addWidget(bluetooth_connected_label);
    bluetooth_connected_layout->addWidget(bluetooth_connected);

    QHBoxLayout *idle_layout = new QHBoxLayout();
    QLabel *idle_label = new QLabel("idle :");
    idle_layout->addWidget(idle_label);
    idle_layout->addWidget(idle);

    QHBoxLayout *docking_layout = new QHBoxLayout();
    QLabel *docking_label = new QLabel("docking :");
    docking_layout->addWidget(docking_label);
    docking_layout->addWidget(docking);

    QHBoxLayout *standby_layout = new QHBoxLayout();
    QLabel *standby_label = new QLabel("standby :");
    standby_layout->addWidget(standby_label);
    standby_layout->addWidget(standby);

    QHBoxLayout *charging_layout = new QHBoxLayout();
    QLabel *charging_label = new QLabel("charging :");
    charging_layout->addWidget(charging_label);
    charging_layout->addWidget(charging);

    QHBoxLayout *error_layout = new QHBoxLayout();
    QLabel *error_label = new QLabel("error :");
    error_layout->addWidget(error_label);
    error_layout->addWidget(error);

    QHBoxLayout *timeout_layout = new QHBoxLayout();
    QLabel *timeout_label = new QLabel("timeout :");
    timeout_layout->addWidget(timeout_label);
    timeout_layout->addWidget(timeout);

    QGridLayout *buttons = new QGridLayout();
    buttons->addWidget(dock_enable_btn, 0 , 0);
    buttons->addWidget(dock_disable_btn, 0 , 1);
    buttons->addWidget(charging_enable_btn , 1 , 0 );
    buttons->addWidget(charging_disable_btn , 1 , 1);
    buttons->addWidget(reset_dock_btn, 2 , 0);
    buttons->addWidget(debug_request_undock_btn, 2,1);

    QHBoxLayout *dock_debug_layout = new QHBoxLayout();
    dock_debug_layout->addWidget(debug_start_dock_btn);
    dock_debug_layout->addWidget(debug_preempt_btn);
    dock_debug_layout->addWidget(debug_start_undock_btn);

    main_layout->addLayout(bluetooth_connected_layout);
    main_layout->addLayout(idle_layout);
    main_layout->addLayout(docking_layout);
    main_layout->addLayout(standby_layout);
    main_layout->addLayout(charging_layout);
    main_layout->addLayout(error_layout);
    main_layout->addLayout(timeout_layout);
    main_layout->addLayout(buttons);
    main_layout->addLayout(dock_debug_layout);


    this->setLayout(main_layout);

}

void DockDisplayWidget::create_connect_()
{
    connect(
        this->dock_enable_btn,
        SIGNAL(clicked()),
        this,
        SLOT(dock_enable_trigger())
    );
    connect(
        this->dock_disable_btn,
        SIGNAL(clicked()),
        this,
        SLOT(dock_disable_trigger())
    );
    connect(
        this->charging_enable_btn,
        SIGNAL(clicked()),
        this,
        SLOT(charging_enable_trigger())
    );
    connect(
        this->charging_disable_btn,
        SIGNAL(clicked()),
        this,
        SLOT(charging_disable_trigger())
    );
    connect(
        this->reset_dock_btn,
        SIGNAL(clicked()),
        this,
        SLOT(reset_dock_trigger())
    );
    connect(
        this->debug_request_undock_btn,
        SIGNAL(clicked()),
        this,
        SLOT(debug_request_undock_trigger())
    );
    connect(
        this->debug_start_dock_btn,
        SIGNAL(clicked()),
        this,
        SLOT(debug_dock_trigger())
    );
    connect(
        this->debug_preempt_btn,
        SIGNAL(clicked()),
        this,
        SLOT(debug_preempt_dock_trigger())
    );
    connect(
        this->debug_start_undock_btn,
        SIGNAL(clicked()),
        this,
        SLOT(debug_undock_trigger())
    );
}

void DockDisplayWidget::dock_enable_trigger()
{
    std_srvs::Trigger srv;
    bool result = dock_enable_client_.call(srv);
    if(result){
        ROS_INFO("CALL dock_enable_trigger Success");
    }
    else{
        ROS_ERROR("CALL dock_enable_trigger Failed");
    }
}
void DockDisplayWidget::dock_disable_trigger()
{
    std_srvs::Trigger srv;
    bool result = dock_disable_client_.call(srv);
    if(result){
        ROS_INFO("CALL dock_disable_trigger Success");
    }
    else{
        ROS_ERROR("CALL dock_disable_trigger Failed");
    }
}
void DockDisplayWidget::charging_enable_trigger()
{
    std_srvs::Trigger srv;
    bool result = charging_enable_client_.call(srv);
    if(result){
        ROS_INFO("CALL charging_enable_trigger Success");
    }
    else{
        ROS_ERROR("CALL charging_enable_trigger Failed");
    }
}
void DockDisplayWidget::charging_disable_trigger()
{
    std_srvs::Trigger srv;
    bool result = charging_disable_client_.call(srv);
    if(result){
        ROS_INFO("CALL charging_disable_trigger Success");
    }
    else{
        ROS_ERROR("CALL charging_disable_trigger Failed");
    }
}
void DockDisplayWidget::reset_dock_trigger()
{
    std_srvs::Trigger srv;
    bool result = reset_dock_client_.call(srv);
    if(result){
        ROS_INFO("CALL reset_dock_trigger Success");
    }
    else{
        ROS_ERROR("CALL reset_dock_trigger Failed");
    }
}

void DockDisplayWidget::debug_request_undock_trigger()
{
    std_srvs::Trigger srv;
    bool result = request_undock_client_.call(srv);
    if(result){
        ROS_INFO("CALL request_undock  Success");
    }
    else{
        ROS_ERROR("CALL request_undock Failed");
    }
}
void DockDisplayWidget::debug_dock_trigger()
{
    std_srvs::Trigger srv;
    bool result = debug_start_client_.call(srv);
    if(result)
    {
        ROS_INFO("CALL debug_start_dock_trigger Success");
    }
    else
    {
        ROS_ERROR("DEBUG debug_start_dock_trigger CALL Failed");
    }
}
void DockDisplayWidget::debug_preempt_dock_trigger()
{
    std_srvs::Trigger srv;
    bool result = debug_preempt_client_.call(srv);
    if(result)
    {
        ROS_INFO("CALL debug_stop_dock_trigger Success");
    }
    else
    {
        ROS_ERROR("DEBUG debug_stop_dock_trigger CALL Failed");
    }
}
void DockDisplayWidget::debug_undock_trigger()
{
    std_srvs::Trigger srv;
    bool result = debug_undock_client_.call(srv);
    if(result){
        ROS_INFO("CALL request_undock  Success");
    }
    else{
        ROS_ERROR("CALL request_undock Failed");
    }
}

