#include <obo_rviz_util/modules/StateDisplayWidget.h>

StateDisplayWidget::StateDisplayWidget(QWidget *parent, ros::NodeHandle *nh): QWidget(parent)
{
    // ROS_INFO("[Battery State] Constructor called !");
    // Init the Object
    this->battery_avg_percentage_ = new QLabel(QString("No data received"));
    this->battery_avg_current_ = new QLabel(QString("No data received"));
    this->battery_avg_voltage_ = new QLabel(QString("No data received"));

    for(int i = 0 ; i < 4 ; i++)
    {
        this->battery_raw_percentage_[i] = new QLabel(QString("-"));
        this->battery_raw_current_[i] = new QLabel(QString("-"));
        this->battery_raw_voltage_[i] = new QLabel(QString("-"));
    }

    this->battery_avg_percentage_bar_ = new QProgressBar(this);
    this->battery_avg_percentage_bar_->setValue(0);

    // Battery Force Charge
    this->set_battery_charge_srv_ = nh->serviceClient<obo_battery::SetCharging>("/battery/set_charging");
    this->force_charge_btn_ = new QPushButton("Battery Force Charge");

    create_ui_();
    create_connect_();
}

StateDisplayWidget::~StateDisplayWidget()
{
    // ROS_INFO("[Battery State] Destructor called !");
}

void StateDisplayWidget::update_avg_battery(sensor_msgs::BatteryState msg)
{
    QString percent_text = QString::fromStdString(
        std::to_string(msg.percentage));
    QString current_text = QString::fromStdString(
        std::to_string(msg.current));
    QString voltage_text = QString::fromStdString(
        std::to_string(msg.voltage));

    int percent = ((int)msg.percentage);
    this->battery_avg_percentage_bar_->setValue(percent);
    this->battery_avg_percentage_->setText(percent_text);
    this->battery_avg_current_->setText(current_text);
    this->battery_avg_voltage_->setText(voltage_text);
}

void StateDisplayWidget::update_raw_battery(sensor_msgs::BatteryState msg)
{
    int battery_idx = (std::stoi(msg.location)) - 1;
    if (battery_idx < 0 || battery_idx > 9)
    {
        // No Update if index is invalid
        return;
    }
    QString percent_text = QString::fromStdString(
        std::to_string(msg.percentage));
    QString current_text = QString::fromStdString(
        std::to_string(msg.current));
    QString voltage_text = QString::fromStdString(
        std::to_string(msg.voltage));

    set_color_label(this->battery_raw_percentage_[battery_idx], msg.percentage, 60.0, 15.0);
    // this->battery_raw_percentage_[battery_idx]->setText(percent_text);
    this->battery_raw_current_[battery_idx]->setText(current_text);
    this->battery_raw_voltage_[battery_idx]->setText(voltage_text);
}

void StateDisplayWidget::create_ui_()
{
    // Main Layout
    QVBoxLayout *main_layout = new QVBoxLayout;
    // Average Battery Widget
    QWidget *avg_battery_widget = create_battery_widget();
    QWidget *raw_battery_widget = create_battery_raw_widget();
    // Add sub-widgets to this
    main_layout->addWidget(avg_battery_widget);
    main_layout->addWidget(force_charge_btn_);
    main_layout->addWidget(raw_battery_widget);

    main_layout->setSpacing(0);
    main_layout->setContentsMargins(0,0,0,0);
    // Set the layout to this widget
    this->setLayout(main_layout);

}

void StateDisplayWidget::create_connect_()
{
    connect(
        force_charge_btn_,
        SIGNAL(clicked()),
        this,
        SLOT(handle_set_charging_clicked_())
    );
}

void StateDisplayWidget::handle_set_charging_clicked_()
{
    obo_battery::SetCharging srv;
    srv.request.station_id = 0;
    srv.request.enable = true;
    bool result = set_battery_charge_srv_.call(srv);
    if(result)
    {
        ROS_INFO("Set Force Charge : True");
    }
    else
    {
        ROS_ERROR("Set Force Charge Failed");
    }
}

void StateDisplayWidget::set_color_label(QLabel *target_label, double input, double okay_level, double warn_level)
{
    target_label->setText(QString::fromStdString((std::to_string(input))));
    if(input > okay_level)
    {
        target_label->setStyleSheet(
            "QLabel {background-color : green; color : white;}"
        );
    }
    else if(input > warn_level)
    {
        target_label->setStyleSheet(
            "QLabel {background-color : orange; color : black;}"
        );
    }
    else
    {
        target_label->setStyleSheet(
            "QLabel {background-color : red; color : black;}"
        );
    }
}

QWidget* StateDisplayWidget::create_battery_raw_widget()
{
    QWidget *raw_battery_widget = new QWidget();
    QHBoxLayout *raw_battery_layout = new QHBoxLayout();

    // QVBoxLayout *single_batt_container = new QVBoxLayout();
    // QLabel *title_label = new QLabel("Batt No. : ");
    // QLabel *percent_label = new QLabel("Percent : ");
    // QLabel *current_label = new QLabel("Current : ");
    // QLabel *voltage_label = new QLabel("Voltage : ");
    // single_batt_container->addWidget(title_label);
    // single_batt_container->addWidget(percent_label);
    // single_batt_container->addWidget(current_label);
    // single_batt_container->addWidget(voltage_label);
    // raw_battery_layout->addLayout(single_batt_container);

    for(int i = 0 ; i < 4 ; i++)
    {
        QVBoxLayout *single_batt_container = new QVBoxLayout();
        // Create Battery Widget
        QLabel *title_label = new QLabel(
            QString::fromStdString(std::to_string(i))
        );

        single_batt_container->addWidget(title_label);
        single_batt_container->addWidget(this->battery_raw_percentage_[i]);
        single_batt_container->addWidget(this->battery_raw_current_[i]);
        single_batt_container->addWidget(this->battery_raw_voltage_[i]);

        raw_battery_layout->addLayout(single_batt_container);
    }

    raw_battery_widget->setLayout(raw_battery_layout);
    return raw_battery_widget;
}

QWidget* StateDisplayWidget::create_battery_widget()
{
    QWidget *battery_widget = new QWidget();
    QVBoxLayout *battery_layout = new QVBoxLayout();

    // Create Battery Widget
    QLabel *title_label = new QLabel("Battery Status");
    QLabel *percent_label = new QLabel("Percentage(avg) : ");
    QLabel *current_label = new QLabel("Current(avg) : ");
    QLabel *voltage_label = new QLabel("Voltage(avg) : ");

    QHBoxLayout *title_layout = new QHBoxLayout;
    title_layout->addWidget(title_label);

    QHBoxLayout *bar_layout = new QHBoxLayout;
    bar_layout->addWidget(this->battery_avg_percentage_bar_);

    QHBoxLayout *percent_layout = new QHBoxLayout;
    percent_layout->addWidget(percent_label);
    percent_layout->addWidget(battery_avg_percentage_);

    QHBoxLayout *current_layout = new QHBoxLayout;
    current_layout->addWidget(current_label);
    current_layout->addWidget(battery_avg_current_);

    QHBoxLayout *voltage_layout = new QHBoxLayout;
    voltage_layout->addWidget(voltage_label);
    voltage_layout->addWidget(battery_avg_voltage_);

    // Add Every Sub Component
    // battery_layout->addLayout(title_layout);
    battery_layout->addLayout(bar_layout);
    battery_layout->addLayout(percent_layout);
    battery_layout->addLayout(current_layout);
    battery_layout->addLayout(voltage_layout);

    // Add all to the widget
    battery_widget->setLayout(battery_layout);

    return battery_widget;

}