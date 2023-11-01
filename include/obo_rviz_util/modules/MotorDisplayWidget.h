/*******************************
 * MotorDisplayWidget.h
 * State Display Widgets
 * Date : 16-Aug-2020
 * Maintainer : Theppasith N. <theppasith.n@obodroid.com>
*******************************/

#include <ros/ros.h>
#include <QWidget>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include "std_msgs/String.h"
#include "sr1b_base/MotorState.h"
#include "sr1b_base/SetDriveMode.h"
#include "sr1_light/FlashlightStatus.h"
#include "sr1_light/SetFlashlightSingle.h"

struct DriveModeStatus
{
    QLabel *name;
    QLabel *mode_label;
};

struct MotorStatus
{
    QLabel *name;
    bool steer_status;
    bool drive_status;
    int steer_id;
    int drive_id;
    QLabel *drive_label;
    QLabel *steer_label;
};

struct LightStatusDisplay
{
    QLabel *name;
    QLabel *power_label;
};

class MotorDisplayWidget: public QWidget{
    Q_OBJECT
    public:
        MotorDisplayWidget(QWidget *parent, ros::NodeHandle *nh);
        ~MotorDisplayWidget();

    public:
        void update_drive_mode(std_msgs::String);
        void update_drive_status(sr1b_base::MotorState);
        void update_steer_status(sr1b_base::MotorState);
        void update_flashlight_status(sr1_light::FlashlightStatus);
        ros::ServiceClient set_flashlight_client;
        ros::ServiceClient set_drive_mode_client;
        bool set_drive_mode(int value);
        bool set_flashlight(int idx, int power);

    private:
        QPushButton* park_mode_button_;
        QPushButton* neutral_mode_button_;
        QPushButton* drive_mode_button_;
        QPushButton* release_mode_button_;

        QPushButton* front_light_on_button_;
        QPushButton* left_light_on_button_;
        QPushButton* right_light_on_button_;

        QPushButton* front_light_off_button_;
        QPushButton* left_light_off_button_;
        QPushButton* right_light_off_button_;
        void create_ui_();
        void create_connect_();
        QWidget* create_drive_mode_widget();
        QWidget* create_motor_widget();
        struct DriveModeStatus drive_mode;
        struct MotorStatus motor_state[3];
        struct LightStatusDisplay light_state[3];

    protected Q_SLOTS:
        void handle_set_drive_mode_buttons();
        void handle_set_flashlight_buttons();
};