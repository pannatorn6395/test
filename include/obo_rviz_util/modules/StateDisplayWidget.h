/*******************************
 * StateDisplayWidget.h
 * State Display Widgets
 * Date : 24-Jul-2020
 * Maintainer : theppasith.n@obodroid.com
*******************************/

#include <ros/ros.h>
#include <QWidget>
#include <QString>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QProgressBar>
#include <QPushButton>

#include <sensor_msgs/BatteryState.h>
#include "obo_battery/SetCharging.h"


class StateDisplayWidget: public QWidget{
    Q_OBJECT
    public:
        StateDisplayWidget(QWidget *parent, ros::NodeHandle *nh);
        ~StateDisplayWidget();
        void update_avg_battery(sensor_msgs::BatteryState msg);
        void update_raw_battery(sensor_msgs::BatteryState msg);

    private:
        QWidget* create_battery_widget();
        QWidget* create_battery_raw_widget();

        void set_color_label(QLabel *target_label, double input, double okay_level, double warn_level);
        void create_ui_();
        void create_connect_();
        QLabel *battery_avg_percentage_;
        QLabel *battery_avg_current_;
        QLabel *battery_avg_voltage_;
        QProgressBar *battery_avg_percentage_bar_;

        QLabel* battery_raw_percentage_[4];
        QLabel* battery_raw_current_[4];
        QLabel* battery_raw_voltage_[4];

        //Buttons
        QPushButton* force_charge_btn_;

        // ROS Service Client
        ros::ServiceClient set_battery_charge_srv_;

    protected Q_SLOTS:
        void handle_set_charging_clicked_();

};