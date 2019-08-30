//
// Created by laura on 27.08.19.
//
#include <vr_puppets_demo/vr_puppets_demo.hpp>
#include <QtWidgets/QCheckBox>

VRPuppets_demo::VRPuppets_demo()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("VRPuppets");
}

/** Initialize RQT Gui Plugin**/
void VRPuppets_demo::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();

    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_status_rqt_plugin");
    }

    motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/stepper_motor_shield/MotorCommand", 1);
    zero_srv = nh->serviceClient<std_srvs::Empty>("/stepper_motor_shield/zero");
    e_stop_server = nh->advertiseService("/m3/emergency_stop", &VRPuppets_demo::EmergencyCallback, this);
    state_transmission_server = nh->advertiseService("/vr_puppets/state_transmission", &VRPuppets_demo::StateTransmissionCallback, this);

    QObject::connect(this, SIGNAL(new_data()), this, SLOT(plotData()));
    QObject::connect(this, SIGNAL(new_motor()), this, SLOT(newMotor()));
    QObject::connect(ui.all_to_position, SIGNAL(clicked()), this, SLOT(allToPosition()));
    QObject::connect(ui.all_to_velocity, SIGNAL(clicked()), this, SLOT(allToVelocity()));
    QObject::connect(ui.all_to_displacement, SIGNAL(clicked()), this, SLOT(allToDisplacement()));
    QObject::connect(ui.stop, SIGNAL(clicked()), this, SLOT(stop()));
    QObject::connect(ui.motor0, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor1, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor2, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor3, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor4, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor5, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.serial_node, SIGNAL(clicked()), this, SLOT(serialNode()));
    QObject::connect(ui.zero, SIGNAL(clicked()), this, SLOT(zero()));
    ui.stop->setStyleSheet("background-color: red");
    QObject::connect(ui.setpoint_all, SIGNAL(valueChanged(int)), this, SLOT(sliderMovedAll()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();

    QScrollArea *scrollArea = widget_->findChild<QScrollArea *>("motor_command");
    scrollArea->setBackgroundRole(QPalette::Window);
    scrollArea->setFrameShadow(QFrame::Plain);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);

    //vertical box that contains all the checkboxes for the filters
    motor_command_scrollarea = new QWidget(widget_);
    motor_command_scrollarea->setObjectName("motor_command_scrollarea");
    motor_command_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    motor_command_scrollarea->setLayout(new QVBoxLayout(motor_command_scrollarea));
    scrollArea->setWidget(motor_command_scrollarea);

    uint32_t ip;
    inet_pton(AF_INET, "10.42.0.1", &ip); //todo: insert HOST_IP -> roboy wifi: 192.168.255.255
    udp.reset(new UDPSocket(8000));
    udp_command.reset(new UDPSocket(8001));
    udp_thread.reset(new std::thread(&VRPuppets_demo::receiveStatusUDP, this));
    udp_thread->detach();

    for (uint motor = 0; motor < 20; motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(
                QPen(color_pallette[motor % 16])); // %16 because we only have 16 colors :(
    }
    ui.position_plot->plotLayout()->insertRow(0);
    ui.position_plot->plotLayout()->addElement(0, 0, new QCPPlotTitle(ui.position_plot, "M3 Positions"));

    ui.position_plot->xAxis->setLabel("time[s]");
    ui.position_plot->yAxis->setLabel("ticks");
    ui.position_plot->replot();

    updateMotorCommands();

    initialized = true;
}

void VRPuppets_demo::shutdownPlugin() {
    motorStatus.shutdown();
}

void VRPuppets_demo::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                  qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void VRPuppets_demo::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                     const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

/** Start ROSSerial Node for the Linear Actuators **/
void VRPuppets_demo::serialNode() {
    system("rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0&");
}

/** Read out and send Commands to Lin. Actuators **/
void VRPuppets_demo::sendMotorCommandLinearActuators() {
    roboy_middleware_msgs::MotorCommand msg;
    msg.id = 69;
    msg.motors = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    msg.set_points = {(float) ui.motor0->value(),
                      (float) ui.motor1->value(),
                      (float) ui.motor2->value(),
                      (float) ui.motor3->value(),
                      (float) ui.motor4->value(),
                      (float) ui.motor5->value(), 0, 0, 0, 0};
    motor_command.publish(msg);
}

/** Receive current motor pos, vel, dis, pwm **/
void VRPuppets_demo::receiveStatusUDP() {
    ROS_INFO("start receiving udp");
    ros::Time t0 = ros::Time::now(), t1;
    bool first_motor_appearance = true;
    while (ros::ok()) {
        int bytes_received = udp->receiveUDPFromClient();
        if (bytes_received == 20) {
            t1 = ros::Time::now();
            ros::Duration d = (t1 - t0);
            t0 = t1;
            float hz = d.toSec();
            int approx_hz = hz;
            ROS_INFO_THROTTLE(60, "receiving motor status at %f Hz", hz);
            ros::Duration delta = (ros::Time::now() - start_time);
            lock_guard<mutex> lock(mux);
            time.push_back(delta.toSec());
            int32_t pos = (int32_t) ((uint8_t) udp->buf[7] << 24 | (uint8_t) udp->buf[6] << 16 |
                                     (uint8_t) udp->buf[5] << 8 | (uint8_t) udp->buf[4]);
            int32_t vel = (int32_t) ((uint8_t) udp->buf[11] << 24 | (uint8_t) udp->buf[10] << 16 |
                                     (uint8_t) udp->buf[9] << 8 | (uint8_t) udp->buf[8]);
            int32_t dis = (int32_t) ((uint8_t) udp->buf[15] << 24 | (uint8_t) udp->buf[14] << 16 |
                                     (uint8_t) udp->buf[13] << 8 | (uint8_t) udp->buf[12]);
            int32_t pwm = (int32_t) ((uint8_t) udp->buf[19] << 24 | (uint8_t) udp->buf[18] << 16 |
                                     (uint8_t) udp->buf[17] << 8 | (uint8_t) udp->buf[16]);

            int motor = udp->buf[0];
            init_set[motor]=false;
            char str_pos[100];
            auto it = ip_address.find(motor);
            if (it == ip_address.end()) { ;
                char IP[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &udp->client_addr.sin_addr, IP, INET_ADDRSTRLEN);
                ROS_INFO("new motor %d %s", motor, IP);
                ip_address[motor] = IP;
                Q_EMIT new_motor();
                ROS_INFO("A total of %i motors connected.", ip_address.size());
                sprintf(str_pos, "%d", pos);
                ROS_INFO_THROTTLE(1,"Added motor %d, with init pos=%d",motor,pos);
                if (!init_set[motor]){
                    init_setpoints[motor]=pos;
                    init_set[motor]=true;
                }
                break;
            }

            motor_position[motor].push_back(pos);
            motor_velocity[motor].push_back(vel);
            motor_displacement[motor].push_back(dis);
            motor_pwm[motor].push_back(pwm);
            for (auto m:ip_address) {
                if (m.first == motor)
                    continue;
                motor_position[m.first].push_back(motor_position[m.first].back());
                motor_velocity[m.first].push_back(motor_velocity[m.first].back());
                motor_displacement[m.first].push_back(motor_displacement[m.first].back());
                motor_pwm[m.first].push_back(motor_pwm[m.first].back());
            }
            if (motor_position[motor].size() > samples_per_plot) {
                for (auto m:ip_address) {
                    motor_position[m.first].pop_front();
                    motor_velocity[m.first].pop_front();
                    motor_displacement[m.first].pop_front();
                    motor_pwm[m.first].pop_front();
                }
            }
            if (time.size() > samples_per_plot)
                time.pop_front();

            if ((counter++) % (approx_hz + 1) == 0) {
                Q_EMIT new_data();
            }
            if (counter % ((approx_hz + 5) * 10) == 0 && initialized) {
                rescale();
            }
        }
    }
    ROS_INFO("stop receiving udp");
}

/** Updade motor command widget in Gui when new motors are available **/
void VRPuppets_demo::updateMotorCommands() {
    for (auto w:widgets) {
        motor_command_scrollarea->layout()->removeWidget(w);
        delete w;
    }
    widgets.clear();

    for (auto m:ip_address) {
        QWidget *widget = new QWidget(motor_command_scrollarea);
        widget->setObjectName(m.second.c_str());
        widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        widget->setLayout(new QHBoxLayout(widget));

        QCheckBox *c = new QCheckBox(widget);
        c->setFixedSize(20, 30);
        c->setCheckable(true);
        c->setChecked(true);
        check[m.first] = c;
        widget->layout()->addWidget(c);

        QLabel *label2 = new QLabel(widget);
        label2->setFixedSize(30, 30);
        char str[22];
        sprintf(str, "ID %d", m.first);
        label2->setText(str);
        widget->layout()->addWidget(label2);

        QLabel *label = new QLabel(widget);
        label->setFixedSize(100, 30);
        label->setText(m.second.c_str());
        widget->layout()->addWidget(label);

        QRadioButton *p = new QRadioButton(widget);
        p->setText("pos");
        p->setFixedSize(50, 30);
        p->setCheckable(true);
        p->setObjectName("pos");
        p->setChecked(true);
        pos[m.first] = p;
        QObject::connect(p, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        widget->layout()->addWidget(p);

        QRadioButton *v = new QRadioButton(widget);
        v->setText("vel");
        v->setFixedSize(50, 30);
        v->setCheckable(true);
        v->setObjectName("vel");
        widget->layout()->addWidget(v);
        vel[m.first] = v;
        QObject::connect(v, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QRadioButton *d = new QRadioButton(widget);
        d->setText("dis");
        d->setFixedSize(50, 30);
        d->setCheckable(true);
        d->setObjectName("dis");
        control_mode[m.first] = DISPLACEMENT;
        widget->layout()->addWidget(d);
        dis[m.first] = d;
        QObject::connect(d, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QSlider *slider = new QSlider(Qt::Orientation::Horizontal, widget);
        slider->setFixedSize(100, 30);
        slider->setValue(50);
        widget->layout()->addWidget(slider);
        sliders[m.first] = slider;
        QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved()));

        bool ok;
        QLineEdit *motor_setpoint = new QLineEdit(widget);
        sprintf(str, "%d", init_setpoints[m.first]);
        motor_setpoint->setText(str);
        motor_setpoint->setObjectName("motor_setpoint");
        widget->layout()->addWidget(motor_setpoint);
        single_motor_setpoints[m.first]=motor_setpoint;
        QObject::connect(motor_setpoint, SIGNAL( returnPressed()), this, SLOT(moveSlider()));


        // Disable untested control modes
        vel[m.first]->setDisabled(false);
        dis[m.first]->setDisabled(false);

        motor_command_scrollarea->layout()->addWidget(widget);
        widgets.push_back(widget);
    }
}

/** Plot motor position feedback for all Units**/
void VRPuppets_demo::plotData() {
    lock_guard<mutex> lock(mux);
    if (!ros::ok())
        return;
    for (auto m:ip_address) {
        ui.position_plot->graph(m.first)->setData(time, motor_position[m.first]);
    }
    ui.position_plot->xAxis->rescale();
    ui.position_plot->replot();
}

/** Rescale Plot **/
void VRPuppets_demo::rescale() {
    double minima[NUMBER_OF_MOTORS_PER_FPGA][4], maxima[NUMBER_OF_MOTORS_PER_FPGA][4];
    uint minimal_motor[4] = {0, 0, 0, 0}, maximal_motor[4] = {0, 0, 0, 0};
    map<int, QVector<double>> *motorData[4];
    motorData[0] = &motor_position;
    motorData[1] = &motor_velocity;
    motorData[2] = &motor_displacement;
    motorData[3] = &motor_pwm;
    for (uint type = 0; type < 4; type++) {
        for (auto m:ip_address) {
            minima[m.first][type] = 0;
            maxima[m.first][type] = 0;
            for (auto val:motorData[type]->at(m.first)) {
                if (val < minima[m.first][type])
                    minima[m.first][type] = val;
                if (val > maxima[m.first][type])
                    maxima[m.first][type] = val;
            }
        }

        for (auto m:ip_address) {
            if (minima[m.first][type] <= minima[minimal_motor[type]][type] && check[m.first]->isChecked())
                minimal_motor[type] = m.first;
            if (maxima[m.first][type] <= maxima[maximal_motor[type]][type] && check[m.first]->isChecked())
                maximal_motor[type] = m.first;
        }
    }

    for (auto m:ip_address) {
        if (minimal_motor[0] == m.first || maximal_motor[0] == m.first)
            ui.position_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[1] == m.first || maximal_motor[1] == m.first)
            ui.position_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[2] == m.first || maximal_motor[2] == m.first)
            ui.position_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[3] == m.first || maximal_motor[3] == m.first)
            ui.position_plot->graph(m.first)->rescaleAxes();
    }

    for (auto m:ip_address) {
        if (minimal_motor[0] != m.first || maximal_motor[0] != m.first)
            ui.position_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[1] != m.first || maximal_motor[1] != m.first)
            ui.position_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[2] != m.first || maximal_motor[2] != m.first)
            ui.position_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[3] != m.first || maximal_motor[3] != m.first)
            ui.position_plot->graph(m.first)->rescaleAxes(true);
    }
}

/** Send Motor setpoints to M3 Units **/
void VRPuppets_demo::sendCommand() {

    udp_command->client_addr.sin_port = htons(8001);
    udp_command->numbytes = 10;
    int motorcount = 0;
    for (auto m:ip_address) {
        mempcpy(&udp_command->buf, &set_points[m.first], 4);
        mempcpy(&udp_command->buf[4], &m.first, 4);
        udp_command->numbytes = 10;
        udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
        udp_command->sendUDPToClient();
//        ROS_INFO("Setpoint Motor %d after sendCommand is %d", motorcount, set_points[m.first]);
        motorcount += 1;
    }

}

/** Only position mode K-value changes **/
void VRPuppets_demo::controlModeChangedSingleMotor(const int &motor_nr, string &ip) {
    udp_command->client_addr.sin_port = htons(8001);
    udp_command->numbytes = 20;
    int Kp, Ki, Kd;
    bool ok;
    if (pos[motor_nr]->isChecked()) {
        Kp = ui.Kp_pos->text().toInt(&ok);
        ROS_INFO("ControlModeChanged: ui.KP used for motor %d is %d", motor_nr, Kp);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Kp");
            return;
        }
        Ki = ui.Ki_pos->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Ki");
            return;
        }
        Kd = ui.Kd_pos->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Kd");
            return;
        }
    } else if (vel[motor_nr]->isChecked()) {
        Kp = ui.Kp_vel->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Kp");
            return;
        }
        Ki = ui.Ki_vel->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Ki");
            return;
        }
        Kd = ui.Kd_vel->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Kd");
            return;
        }
    } else if (dis[motor_nr]->isChecked()) {
        Kp = ui.Kp_dis->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Kp");
            return;
        }
        Ki = ui.Ki_dis->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Ki");
            return;
        }
        Kd = ui.Kd_dis->text().toInt(&ok);
        if (!ok) {
            ROS_ERROR("invalid conversion to integer of Kd");
            return;
        }
    }
    mempcpy(&udp_command->buf[0], &Kd, 4);
    mempcpy(&udp_command->buf[4], &Ki, 4);
    mempcpy(&udp_command->buf[8], &Kp, 4);
    mempcpy(&udp_command->buf[12], &control_mode[motor_nr], 4);
    mempcpy(&udp_command->buf[16], &motor_nr, 4);
    udp_command->client_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    udp_command->sendUDPToClient();
    ROS_INFO("SEND controlModeChanged to motor %d", motor_nr);
}

/** Change controlmode on radioButton->checked() event **/
void VRPuppets_demo::controlModeChanged() {
    udp_command->client_addr.sin_port = htons(8001);
    udp_command->numbytes = 20;
    int Kp, Ki, Kd;
    int motor = 0;
    for (auto m:ip_address) {
        bool ok;
        if (pos[m.first]->isChecked()) {
            Kp = ui.Kp_pos->text().toInt(&ok);
            ROS_INFO("ControlModeChanged: ui.KP used for motor %d is %d", motor, Kp);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_pos->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_pos->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        } else if (vel[m.first]->isChecked()) {
            Kp = ui.Kp_vel->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_vel->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_vel->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        } else if (dis[m.first]->isChecked()) {
            Kp = ui.Kp_dis->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_dis->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_dis->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        } else {
            ROS_ERROR("haeeee????");
            return;
        }
        motor += 1;
        mempcpy(&udp_command->buf[0], &Kd, 4);
        mempcpy(&udp_command->buf[4], &Ki, 4);
        mempcpy(&udp_command->buf[8], &Kp, 4);
        mempcpy(&udp_command->buf[12], &control_mode[m.first], 4);
        mempcpy(&udp_command->buf[16], &m.first, 4);
        udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
        udp_command->sendUDPToClient();
    }
}

/** Set all M3s to Position Mode with same setpoint given in ui.setpoint_pos **/
void VRPuppets_demo::allToPosition() {
    bool ok;
    char str[100];
    for (auto m:ip_address) {
        control_mode[m.first] = POSITION;
        pos[m.first]->setChecked(true);
        vel[m.first]->setChecked(false);
        dis[m.first]->setChecked(false);
        set_points[m.first] = ui.setpoint_pos->text().toInt(&ok);
        sliders[m.first]->setValue(set_points[m.first] + 50);

        //Update single motor setpoints in Gui
        sprintf(str, "%d", set_points[m.first]);
        single_motor_setpoints[m.first]->setText(str);
        controlModeChangedSingleMotor(m.first, m.second);
    }
    ui.setpoint->setText(ui.setpoint_pos->text());
    sendCommand();
}

/** Set all M3s to Velocity Mode with same setpoint given in ui.setpoint_vel **/
void VRPuppets_demo::allToVelocity() {
    bool ok;
    for (auto m:ip_address) {
        control_mode[m.first] = VELOCITY;
        pos[m.first]->setChecked(false);
        vel[m.first]->setChecked(true);
        dis[m.first]->setChecked(false);
        set_points[m.first] = ui.setpoint_vel->text().toInt(&ok);
        controlModeChangedSingleMotor(m.first, m.second);

    }
    ui.setpoint->setText(ui.setpoint_vel->text());
    sendCommand();
}

/** Set all M3s to displacement Mode with same setpoint given in ui.setpoint_dis **/
void VRPuppets_demo::allToDisplacement() {
    bool ok;
    for (auto m:ip_address) {
        control_mode[m.first] = DISPLACEMENT;
        pos[m.first]->setChecked(false);
        vel[m.first]->setChecked(false);
        dis[m.first]->setChecked(true);
        set_points[m.first] = ui.setpoint_dis->text().toInt(&ok);
        controlModeChangedSingleMotor(m.first, m.second);
    }
    ui.setpoint->setText(ui.setpoint_dis->text());
    sendCommand();
}

/** Adjust sliders on motor_setpoint returnPressed() event **/
void VRPuppets_demo::moveSlider(){
    bool ok;
    QLineEdit *tmp;
    for (auto m:ip_address) {
        if (check[m.first]->isChecked()) {
            int motor_scale = ui.scale->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("motor scale invalid");
                return;
            }
            tmp = single_motor_setpoints[m.first];
            sliders[m.first]->setValue(((tmp->text().toInt(&ok)) / motor_scale) + 50);
        }
    }
}

/** Update motor_setpoint and set_points when sliders are moved **/
void VRPuppets_demo::sliderMoved() {
    bool ok;
    char str[100];
    for (auto m:ip_address) {
        if (check[m.first]->isChecked()) {
            int motor_scale = ui.scale->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("motor scale invalid");
                return;
            }
            set_points[m.first] = (sliders[m.first]->value() - 50) * motor_scale;
            sprintf(str, "%d", set_points[m.first]);
            single_motor_setpoints[m.first]->setText(str);
        }
    }
    sendCommand();
}

/** Move all M3s together according to main slider setpoint**/
void VRPuppets_demo::sliderMovedAll() {
    bool ok;
    int motor_scale = ui.scale->text().toInt(&ok);
    for (auto m:ip_address) {
        if (check[m.first]->isChecked()) {
            if (!ok) {
                ROS_ERROR("motor scale invalid");
                return;
            }
            set_points[m.first] = (ui.setpoint_all->value() - 50) * motor_scale;
        } else {
            ROS_WARN("ignoring motor %d because it is not activated", m.first);
        }
    }
    char str[100];
    sprintf(str, "%d", (ui.setpoint_all->value() - 50) * motor_scale);
    ui.setpoint->setText(str);
    sendCommand();
}

/** Stop and restart M3s.
 * On "continue" by using the 'follow' checkboxeiter old setpoints can be resumed
 * and motors start moving right away, or motors stay at their stop point.
 */
void VRPuppets_demo::stop() {
    if (!ui.stop->isChecked()) {
        ui.stop->setStyleSheet("background-color: red");
        ui.all_to_position->setEnabled(true);
        ui.all_to_velocity->setEnabled(true);
        ui.all_to_displacement->setEnabled(true);
        char str[100];
        for (auto m:ip_address) {
            if (!saved_temp_control_mode) {
                switch (control_mode[m.first]) {
                    case POSITION:
                        sprintf(str, "%d", Kp[m.first]);
                        ui.Kp_pos->setText(str);
                        sprintf(str, "%d", Ki[m.first]);
                        ui.Ki_pos->setText(str);
                        sprintf(str, "%d", Kd[m.first]);
                        ui.Kd_pos->setText(str);
                        controlModeChangedSingleMotor(m.first, m.second);
                        break;
                    case VELOCITY:
                        sprintf(str, "%d", Kp[m.first]);
                        ui.Kp_vel->setText(str);
                        sprintf(str, "%d", Ki[m.first]);
                        ui.Ki_vel->setText(str);
                        sprintf(str, "%d", Kd[m.first]);
                        ui.Kd_vel->setText(str);
                        controlModeChangedSingleMotor(m.first, m.second);
                        break;
                    case DISPLACEMENT:
                        sprintf(str, "%d", Kp[m.first]);
                        ui.Kp_dis->setText(str);
                        sprintf(str, "%d", Ki[m.first]);
                        ui.Ki_dis->setText(str);
                        sprintf(str, "%d", Kd[m.first]);
                        ui.Kd_dis->setText(str);
                        controlModeChangedSingleMotor(m.first, m.second);
                        break;
                }
            } else {
                switch (control_mode_temp[m.first]) {
                    case POSITION:
                        sprintf(str, "%d", Kp[m.first]);
                        ui.Kp_pos->setText(str);
                        sprintf(str, "%d", Ki[m.first]);
                        ui.Ki_pos->setText(str);
                        sprintf(str, "%d", Kd[m.first]);
                        ui.Kd_pos->setText(str);
                        controlModeChangedSingleMotor(m.first, m.second);
                        break;
                    case VELOCITY:
                        sprintf(str, "%d", Kp[m.first]);
                        ui.Kp_vel->setText(str);
                        sprintf(str, "%d", Ki[m.first]);
                        ui.Ki_vel->setText(str);
                        sprintf(str, "%d", Kd[m.first]);
                        ui.Kd_vel->setText(str);
                        controlModeChangedSingleMotor(m.first, m.second);
                        break;
                    case DISPLACEMENT:
                        sprintf(str, "%d", Kp[m.first]);
                        ui.Kp_dis->setText(str);
                        sprintf(str, "%d", Ki[m.first]);
                        ui.Ki_dis->setText(str);
                        sprintf(str, "%d", Kd[m.first]);
                        ui.Kd_dis->setText(str);
                        controlModeChangedSingleMotor(m.first, m.second);
                        break;
                }
            }
        }
        saved_temp_control_mode = false;
        ui.pos_frame->setEnabled(true);
        ui.vel_frame->setEnabled(true);
        ui.dis_frame->setEnabled(true);
        ui.motor_command->setEnabled(true);
        ui.setpoint_all->setEnabled(true);
        ui.stop->setText("STOP M3");

        if(ui.follow->isChecked()){
            // Follow previous motor set-point
            sendCommand();
        }else{
            // No further movement when resuming operation
            ROS_DEBUG("Not following setpoint on 'continue'");
        }
    } else {
        ui.stop->setStyleSheet("background-color: green");
        int motor_count = 0;
        bool ok;
        // Save all current Kp, Ki, Kd settings for all motors
        int Kp_pos = ui.Kp_pos->text().toInt(&ok);
        int Ki_pos = ui.Ki_pos->text().toInt(&ok);
        int Kd_pos = ui.Kd_pos->text().toInt(&ok);
        int Kp_vel = ui.Kp_vel->text().toInt(&ok);
        int Ki_vel = ui.Ki_vel->text().toInt(&ok);
        int Kd_vel = ui.Kd_vel->text().toInt(&ok);
        int Kp_dis = ui.Kp_dis->text().toInt(&ok);
        int Ki_dis = ui.Ki_dis->text().toInt(&ok);
        int Kd_dis = ui.Kd_dis->text().toInt(&ok);
        for (auto m:ip_address) {
            ROS_INFO("Shutting down motor %d", motor_count);
            switch (control_mode[m.first]) {
                case POSITION:
                    Kp[m.first] = Kp_pos;
                    Ki[m.first] = Ki_pos;
                    Kd[m.first] = Kd_pos;
                    ui.Kp_pos->setText("0");
                    ui.Ki_pos->setText("0");
                    ui.Kd_pos->setText("0");
                    control_mode_temp[m.first] = 0;
                    ROS_INFO("ON STOP: Setpoint motor %d is %d", motor_count, set_points[m.first]);
                    controlModeChangedSingleMotor(m.first, m.second);
                    break;
                case VELOCITY:
                    Kp[m.first] = Kp_vel;
                    Ki[m.first] = Ki_vel;
                    Kd[m.first] = Kd_vel;
                    ui.Kp_vel->setText("0");
                    ui.Ki_vel->setText("0");
                    ui.Kd_vel->setText("0");
                    control_mode_temp[m.first] = 1;
                    controlModeChangedSingleMotor(m.first, m.second);
                    break;
                case DISPLACEMENT:
                    Kp[m.first] = Kp_dis;
                    Ki[m.first] = Ki_dis;
                    Kd[m.first] = Kd_dis;
                    ui.Kp_dis->setText("0");
                    ui.Ki_dis->setText("0");
                    ui.Kd_dis->setText("0");
                    control_mode_temp[m.first] = 2;
                    controlModeChangedSingleMotor(m.first, m.second);
                    break;
            }
        }
        saved_temp_control_mode = true;
        ui.pos_frame->setEnabled(false);
        ui.vel_frame->setEnabled(false);
        ui.dis_frame->setEnabled(false);
        ui.motor_command->setEnabled(false);
        ui.setpoint_all->setEnabled(false);
        ui.all_to_position->setEnabled(false);
        ui.all_to_velocity->setEnabled(false);
        ui.all_to_displacement->setEnabled(false);
        ui.stop->setText("CONTINUE");
    }
}

/** Detect wifi boards and add new motors to gui **/
void VRPuppets_demo::newMotor() {
    time.clear();
    for (auto m:ip_address) {
        motor_position[m.first].clear();
        motor_velocity[m.first].clear();
        motor_displacement[m.first].clear();
        motor_pwm[m.first].clear();
    }
    updateMotorCommands();
    udp_thread.reset(new std::thread(&VRPuppets_demo::receiveStatusUDP, this));
    udp_thread->detach();
}

/* ROS related stuff (Pupblisher, Subscriber, Service servers...) */

/** React to external Emegencystop
 * (old setup -> probably not applicable anymore) **/
bool VRPuppets_demo::EmergencyCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if (req.data == 1) {
        ROS_INFO("M3-Emergency stop service called.");
        ui.stop->setChecked(true);
        stop();
        res.success = true;
        res.message = "Emergency stop service called";
    } else {
        ROS_INFO("Resuming normal operation.");
        ui.stop->setChecked(false);
        stop();
        res.success = true;
        res.message = "Resuming normal operation.";
    }
    return true;
}

/** Service callback for OUI-Team State transmission
 * Initiates short pulling of the tendons of M3s 0, 1, 2
 * to slightly pull back the VR-operator.
 * @param req
 * @param res
 * @return
 */
bool VRPuppets_demo::StateTransmissionCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if (req.data == 1) {
        ROS_INFO("State transition service called.");
        // Do stuff
        bool ok;
        int pull_displacement = 200;

        for (auto m:ip_address){
            char str[100];
            sprintf(str, "%d", (single_motor_setpoints[m.first]->text().toInt(&ok) + pull_displacement));
            ROS_INFO("current setpoint: %d", single_motor_setpoints[m.first]->text().toInt(&ok));
            single_motor_setpoints[m.first]->setText(str);
            ROS_INFO("New setpoint: %s", str);
        }
        moveSlider();
        res.success = true;
        res.message = "State transition service called";
    } else {
        // There is nothing to do
        res.success = false;
        res.message = "State transition service call unsuccsessful.";
    }
    return true;
}

/** Send all Lin. Actuators to Zero position
 * !CAUTION!: Do only use when end stops are mounted!
 **/
void VRPuppets_demo::zero() {
    std_srvs::Empty msg;
    zero_srv.call(msg);
}

PLUGINLIB_EXPORT_CLASS(VRPuppets_demo, rqt_gui_cpp::Plugin)
