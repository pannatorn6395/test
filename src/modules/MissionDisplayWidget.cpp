/*******************************
 * MissionDisplayWidget.cpp
 * Mission Display Widgets
 * Date : 01-April-2021
 * Maintainer : Theppasith N. <theppasith.n@obodroid.com>
*******************************/

#include <obo_rviz_util/modules/MissionDisplayWidget.h>


MissionDisplayWidget::MissionDisplayWidget(QWidget *parent, ros::NodeHandle *nh): QWidget(parent)
{
    ROS_INFO("[MissionDisp] Contructor Called !");

    // Create the Mission Lists
    user_mission_list_widget = new QListWidget();
    worker_mission_list_widget = new QListWidget();

    // Create UI
    create_ui_();
    create_connect_();
}

MissionDisplayWidget::~MissionDisplayWidget()
{
    ROS_INFO("[MissionDisp] Destructor Called");
}

void MissionDisplayWidget::update_user_mission_item(std::vector<QString> msg)
{
    user_mission_list_widget->clear();
    for(int i=0; i<msg.size() ;i++){
        user_mission_list_widget->addItem(msg[i]);
    }
}

void MissionDisplayWidget::update_worker_mission_item(std::vector<QString> msg)
{
    worker_mission_list_widget->clear();
    for(int i=0; i<msg.size() ;i++){
        worker_mission_list_widget->addItem(msg[i]);
    }
}

void MissionDisplayWidget::create_ui_()
{
    // Create customized font view
    QFont font;
    font.setPointSize(8);
    // Init the List Widget
    user_mission_list_widget->setFont(font);
    worker_mission_list_widget->setFont(font);
    // Size Customization
    user_mission_list_widget->setMinimumSize(100,100);
    worker_mission_list_widget->setMinimumSize(100,100);

    QVBoxLayout *left_layout = new QVBoxLayout;
    QLabel *usr_label = new QLabel("User Mission");
    usr_label->setFont(font);
    left_layout->addWidget(usr_label);
    left_layout->addWidget(user_mission_list_widget);

    QVBoxLayout *right_layout = new QVBoxLayout;
    QLabel *work_label = new QLabel("Worker Mission");
    work_label->setFont(font);
    right_layout->addWidget(work_label);
    right_layout->addWidget(worker_mission_list_widget);


    QHBoxLayout *main_layout = new QHBoxLayout;
    main_layout->addLayout(left_layout);
    main_layout->addLayout(right_layout);

    // Set Layout
    this->setLayout(main_layout);

}

void MissionDisplayWidget::create_connect_()
{

}