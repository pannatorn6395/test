/*******************************
 * MotorDisplayWidget.cpp
 * State Display Widgets
 * Date : 16-Aug-2020
 * Maintainer : Theppasith N. <theppasith.n@obodroid.com>
*******************************/
#include <obo_rviz_util/modules/MotorDisplayWidget.h>

// 0 1 2  F L R

MotorDisplayWidget::MotorDisplayWidget(QWidget *parent, ros::NodeHandle *nh): QWidget(parent)
{
    ROS_INFO("[MotorState] Contructor Called !");
    // Initialize Drive Mode
    drive_mode.mode_label = new QLabel("-");
    // Initialize Motor State
    for(int i=0 ; i<3 ; i++)
    {
        motor_state[i].steer_status = false;
        motor_state[i].drive_status = false;
        motor_state[i].steer_id = i;
        motor_state[i].drive_id = i;
        motor_state[i].drive_label = new QLabel("-");
        motor_state[i].steer_label = new QLabel("-");

        light_state[i].power_label = new QLabel("0");
    }
    drive_mode.name = new QLabel("DRIVE MODE: ");

    motor_state[0].name = new QLabel("FRONT");
    motor_state[1].name = new QLabel("LEFT");
    motor_state[2].name = new QLabel("RIGHT");

    light_state[0].name = new QLabel("FRONT");
    light_state[1].name = new QLabel("LEFT");
    light_state[2].name = new QLabel("RIGHT");

    this->park_mode_button_ = new QPushButton("Park");
    this->neutral_mode_button_ = new QPushButton("Neutral");
    this->drive_mode_button_ = new QPushButton("Drive");
    this->release_mode_button_ = new QPushButton("Release");
    
    this->front_light_on_button_ = new QPushButton("on");
    this->left_light_on_button_ = new QPushButton("on");
    this->right_light_on_button_ = new QPushButton("on");
    this->front_light_off_button_ = new QPushButton("off");
    this->left_light_off_button_ = new QPushButton("off");
    this->right_light_off_button_ = new QPushButton("off");

    this->set_flashlight_client = nh->serviceClient<sr1_light::SetFlashlightSingle>("/light_node/set_flashlight_single");
    this->set_drive_mode_client = nh->serviceClient<sr1b_base::SetDriveMode>("/robot_base/set_mode");
    create_ui_();
    create_connect_();
}

MotorDisplayWidget::~MotorDisplayWidget()
{
    ROS_INFO("[MotorState] Destructor Called");
}

void MotorDisplayWidget::update_drive_mode(std_msgs::String msg)
{
    QString drive_mode_str = QString::fromStdString(
        msg.data);
    drive_mode.mode_label->setText(drive_mode_str);
}

void MotorDisplayWidget::update_flashlight_status(sr1_light::FlashlightStatus msg)
{
    QString front_label = QString::fromStdString(
        std::to_string(msg.light_front));
    light_state[0].power_label->setText(front_label);

    QString left_label = QString::fromStdString(
        std::to_string(msg.light_left));
    light_state[1].power_label->setText(left_label);

    QString right_label = QString::fromStdString(
        std::to_string(msg.light_right));
    light_state[2].power_label->setText(right_label);

}

void MotorDisplayWidget::update_drive_status(sr1b_base::MotorState msg)
{
    if(msg.id-1 >= 0 && msg.id-1 <= 3)
    {
        if(msg.state == 1)
        {
            motor_state[msg.id-1].drive_status = true;
            motor_state[msg.id-1].drive_label->setText("D:True");
        }
        else
        {
            motor_state[msg.id-1].drive_status = false;
            motor_state[msg.id-1].drive_label->setText("D:False");
        }

    }
}

void MotorDisplayWidget::update_steer_status(sr1b_base::MotorState msg)
{
    if(msg.id-1 >= 0 && msg.id-1 < 3)
    {
        if(msg.state == 1)
        {
            motor_state[msg.id-1].steer_status = true;
            motor_state[msg.id-1].steer_label->setText("S:True");
        }
        else
        {
            motor_state[msg.id-1].steer_status = false;
            motor_state[msg.id-1].steer_label->setText("S:False");
        }
    }
}

void MotorDisplayWidget::create_ui_()
{
    // Main Layout
    QVBoxLayout *main_layout = new QVBoxLayout;
    // main_layout->setSpacing(0);
    // main_layout->setContentsMargins(0,0,0,0);

    // Drive Mode View
    QWidget *drive_mode_widget = create_drive_mode_widget();

    // Three Motor View
    QWidget *motor_widget = create_motor_widget();
    // Add Widget to layout
    main_layout->addWidget(drive_mode_widget);
    main_layout->addWidget(motor_widget);
    // Set the layout to this main widget entity
    this->setLayout(main_layout);
}

void MotorDisplayWidget::create_connect_()
{
    connect(
        this->park_mode_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_drive_mode_buttons())
    );
    connect(
        this->neutral_mode_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_drive_mode_buttons())
    );
    connect(
        this->drive_mode_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_drive_mode_buttons())
    );
    connect(
        this->release_mode_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_drive_mode_buttons())
    );
    connect(
        this->front_light_on_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_flashlight_buttons())
    );
    connect(
        this->left_light_on_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_flashlight_buttons())
    );
    connect(
        this->right_light_on_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_flashlight_buttons())
    );

    connect(
        this->front_light_off_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_flashlight_buttons())
    );
    connect(
        this->left_light_off_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_flashlight_buttons())
    );
    connect(
        this->right_light_off_button_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_flashlight_buttons())
    );
}

bool MotorDisplayWidget::set_drive_mode(int value)
{
    sr1b_base::SetDriveMode srv;
    srv.request.value = value;

    bool result = set_drive_mode_client.call(srv);
    if(result)
    {
        ROS_INFO("Set Drive Mode: %d Success", value);
        return true;
    }
    else
    {
        ROS_ERROR("Set Drive Mode: %d Failed", value);
        return false;
    }
}

bool MotorDisplayWidget::set_flashlight(int idx, int power)
{
    sr1_light::SetFlashlightSingle srv;
    srv.request.idx = idx;
    srv.request.power = power;

    bool result = set_flashlight_client.call(srv);
    if(result)
    {
        ROS_INFO("Set Flashlight %d = %d Success", idx, power);
        return true;
    }
    else
    {
        ROS_ERROR("Set Flashlight %d Failed", idx);
        return false;
    }
    return false;
}

void MotorDisplayWidget::handle_set_drive_mode_buttons()
{
    sr1b_base::SetDriveMode srv;
    QPushButton* target_button = qobject_cast<QPushButton*>(sender());
    if(target_button == park_mode_button_)set_drive_mode(srv.request.PARK);
    if(target_button == neutral_mode_button_)set_drive_mode(srv.request.NEUTRAL);
    if(target_button == drive_mode_button_)set_drive_mode(srv.request.DRIVE);
    if(target_button == release_mode_button_)set_drive_mode(srv.request.RELEASE);
}

void MotorDisplayWidget::handle_set_flashlight_buttons()
{
    QPushButton* target_button = qobject_cast<QPushButton*>(sender());
    if(target_button == front_light_on_button_)set_flashlight(0, 80);
    if(target_button == left_light_on_button_)set_flashlight(1, 80);
    if(target_button == right_light_on_button_)set_flashlight(2, 80);
    if(target_button == front_light_off_button_)set_flashlight(0, 0);
    if(target_button == left_light_off_button_)set_flashlight(1, 0);
    if(target_button == right_light_off_button_)set_flashlight(2, 0);
}

QWidget* MotorDisplayWidget::create_drive_mode_widget()
{
    QWidget *drive_mode_widget = new QWidget();

    // Create Main Layout Here
    QVBoxLayout *main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(0, 0, 0, 0);

    // Drive Mode Status
    QHBoxLayout *drive_mode_status = new QHBoxLayout();
    drive_mode_status->setContentsMargins(0, 0, 0, 0);
    drive_mode_status->addWidget(drive_mode.name);
    drive_mode_status->addWidget(drive_mode.mode_label);

    // Drive Mode Button
    QGridLayout *drive_mode_button = new QGridLayout();
    drive_mode_button->setContentsMargins(0, 0, 0, 0);
    drive_mode_button->addWidget(park_mode_button_, 0, 0);
    drive_mode_button->addWidget(neutral_mode_button_, 0, 1);
    drive_mode_button->addWidget(drive_mode_button_, 1, 0);
    drive_mode_button->addWidget(release_mode_button_, 1, 1);

    main_layout->addLayout(drive_mode_status);
    main_layout->addLayout(drive_mode_button);
    
    drive_mode_widget->setLayout(main_layout);
    drive_mode_widget->setFixedHeight(100);
    return drive_mode_widget;
}

QWidget* MotorDisplayWidget::create_motor_widget()
{
    QWidget *motor_widget = new QWidget();

    QVBoxLayout *front_motor = new QVBoxLayout();

    QWidget *front_light_widget = new QWidget();
    QVBoxLayout *front_light_layout = new QVBoxLayout();
    front_light_layout->setContentsMargins(0, 0, 0, 0);
    front_light_layout->addWidget(light_state[0].power_label);
    front_light_layout->addWidget(front_light_on_button_);
    front_light_layout->addWidget(front_light_off_button_);
    front_light_widget->setLayout(front_light_layout);
    front_motor->addWidget(front_light_widget);

    front_motor->addWidget(motor_state[0].name);
    front_motor->addWidget(motor_state[0].steer_label);
    front_motor->addWidget(motor_state[0].drive_label);

    QVBoxLayout *left_motor = new QVBoxLayout();
    QWidget *left_light_widget = new QWidget();
    QVBoxLayout *left_light_layout = new QVBoxLayout();
    left_light_layout->setContentsMargins(0, 0, 0, 0);
    left_light_layout->addWidget(light_state[1].power_label);
    left_light_layout->addWidget(left_light_on_button_);
    left_light_layout->addWidget(left_light_off_button_);
    left_light_widget->setLayout(left_light_layout);
    left_motor->addWidget(left_light_widget);

    left_motor->addWidget(motor_state[1].name);
    left_motor->addWidget(motor_state[1].steer_label);
    left_motor->addWidget(motor_state[1].drive_label);

    QVBoxLayout *right_motor = new QVBoxLayout();
    QWidget *right_light_widget = new QWidget();
    QVBoxLayout *right_light_layout = new QVBoxLayout();
    right_light_layout->setContentsMargins(0, 0, 0, 0);
    right_light_layout->addWidget(light_state[2].power_label);
    right_light_layout->addWidget(right_light_on_button_);
    right_light_layout->addWidget(right_light_off_button_);
    right_light_widget->setLayout(right_light_layout);
    right_motor->addWidget(right_light_widget);

    right_motor->addWidget(motor_state[2].name);
    right_motor->addWidget(motor_state[2].steer_label);
    right_motor->addWidget(motor_state[2].drive_label);

    QHBoxLayout *button_layout = new QHBoxLayout();
    QPushButton *button1 = new QPushButton("PARK");
    QPushButton *button2 = new QPushButton("D:SAFE");
    QPushButton *button3 = new QPushButton("D:UNSAFE");
    button_layout->addWidget(button1);
    button_layout->addWidget(button2);
    button_layout->addWidget(button3);

    // Create Main Layout Here
    QVBoxLayout *main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(0, 0, 0, 0);

    // Gather Motors
    QHBoxLayout *motor_layout = new QHBoxLayout();
    motor_layout->addLayout(left_motor);
    motor_layout->addLayout(front_motor);
    motor_layout->addLayout(right_motor);

    main_layout->addLayout(motor_layout);
    // main_layout->addLayout(button_layout);

    motor_widget->setLayout(main_layout);
    motor_widget->setFixedHeight(200);

    return motor_widget;
}
