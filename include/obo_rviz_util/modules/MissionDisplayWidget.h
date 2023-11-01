/*******************************
 * MissionDisplayWidget.h
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
#include <QListWidget>
#include <QScrollArea>

class MissionDisplayWidget: public QWidget{
    Q_OBJECT
    public:
        MissionDisplayWidget(QWidget *parent, ros::NodeHandle *nh);
        ~MissionDisplayWidget();

    protected Q_SLOTS:
        void update_user_mission_item(std::vector<QString> msg);
        void update_worker_mission_item(std::vector<QString> msg);

    private:
        void create_ui_();
        void create_connect_();
        QListWidget* user_mission_list_widget;
        QListWidget* worker_mission_list_widget;


};