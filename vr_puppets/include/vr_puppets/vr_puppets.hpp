#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <rqt_gui_cpp/plugin.h>
#include <vr_puppets_demo/ui_vr_puppets_demo.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <QWidget>
#include <QLabel>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/UDPSocket.hpp>
#include <thread>
#include <QScrollArea>
#include <QRadioButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QtWidgets/QCheckBox>
#include <mutex>

#endif

class VRPuppets
        : public rqt_gui_cpp::Plugin {
Q_OBJECT
public:
    VRPuppets();
    ~VRPuppets() override{
        system("rosnode kill serial_node");
        if(udp_thread->joinable())
            udp_thread->join();
    }

    void initPlugin(qt_gui_cpp::PluginContext &context) override;

    void shutdownPlugin() override;

    void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const override;

    void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings) override;


public Q_SLOTS:
    void plotData();
    void rescale();
    void sendCommand();
    void radioButton_CB();
    void allToPositionSetpoint();
    void allToVelocitySetpoint();
    void allToDisplacement();
    void sliderMoved();
    void setDisplacementModeForAll();
    void sliderMovedAll();
    void stop();
    void sendMotorCommandLinearActuators();
    void serialNode();
    void zero();
    void newMotor();
    void controlModeChangedSingleMotor(const int &m, string &ip);
    void moveSlider();
    void setPositionModeForAll();
private:
    void receiveStatusUDP();
    void updateMotorCommands();
Q_SIGNALS:
    void new_data();
    void new_motor();
private:
    Ui::VRPuppets_demo ui;
    QWidget *widget_;
    QWidget* motor_command_scrollarea;
    vector<QWidget*> widgets;
    map<int,QCheckBox*> check;
    map<int,QSlider*> sliders;
    map<int, bool>init_set;
    map<int,QLineEdit*> single_motor_setpoints;
    map<int, int> init_setpoints;
    QVector<double> time;
    int counter = 0;
    map<int,QVector<double>> motor_position, motor_velocity, motor_displacement, motor_force, motor_pwm;
    map<int,string> ip_address;
    map<int,QRadioButton*> active,pos,vel,dis;
    map<int,int> Kp,Ki,Kd;
    map<int,int> control_mode;
    map<int, double> status_pos;
    map<int, int> control_mode_temp;
    bool saved_temp_control_mode = false;
    int samples_per_plot = 200;
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
    ros::NodeHandlePtr nh;
    ros::Subscriber motorStatus;
    ros::Publisher motor_command;
    ros::ServiceClient zero_srv;
    ros::ServiceServer e_stop_server;
    ros::ServiceServer state_transmission_server;
    ros::ServiceServer obstacle_reached_server;
    ros::Time start_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    UDPSocketPtr udp, udp_command;
    boost::shared_ptr<std::thread> udp_thread;
    bool initialized = false;
    map<int,int> set_points;
    mutex mux;

    bool EmergencyCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool StateTransmissionCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool ObstacleReachedCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};
