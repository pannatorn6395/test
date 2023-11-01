/*******************************
 * DockDisplayWidget.h
 * Mission Display Widgets
 * Date : 01-April-2021
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
#include <QScrollArea>
#include <sr1_charging_dock/ChargingDockStatus.h>
#include <std_srvs/Trigger.h>

class DockDisplayWidget: public QWidget{
    Q_OBJECT
    public:
        DockDisplayWidget(QWidget *parent, ros::NodeHandle *nh);
        ~DockDisplayWidget();

    protected Q_SLOTS:
        void update_dock_station(sr1_charging_dock::ChargingDockStatus msg);
        void dock_enable_trigger();
        void dock_disable_trigger();
        void charging_enable_trigger();
        void charging_disable_trigger();
        void reset_dock_trigger();

        void debug_request_undock_trigger();
        void debug_dock_trigger();
        void debug_preempt_dock_trigger();
        void debug_undock_trigger();


    private:
        void create_ui_(); // Generate UI ending with set layout
        void create_connect_(); // UI to function
        QPushButton* dock_enable_btn;
        QPushButton* dock_disable_btn;
        QPushButton* charging_enable_btn;
        QPushButton* charging_disable_btn;
        QPushButton* reset_dock_btn;

        QPushButton* debug_request_undock_btn;
        QPushButton* debug_start_dock_btn;
        QPushButton* debug_preempt_btn;
        QPushButton* debug_start_undock_btn;

        QLabel* bluetooth_connected;
        QLabel* idle;
        QLabel* docking;
        QLabel* standby;
        QLabel* charging;
        QLabel* error;
        QLabel* timeout;

    private:
        ros::ServiceClient dock_enable_client_;
        ros::ServiceClient dock_disable_client_;
        ros::ServiceClient charging_enable_client_;
        ros::ServiceClient charging_disable_client_;
        ros::ServiceClient reset_dock_client_;

        ros::ServiceClient request_undock_client_;
        ros::ServiceClient debug_start_client_;
        ros::ServiceClient debug_preempt_client_;
        ros::ServiceClient debug_undock_client_;

};