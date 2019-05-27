#include <vr_puppets/vr_puppets.hpp>
#include <QtWidgets/QCheckBox>

VRPuppets::VRPuppets()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("VRPuppets");
}

void VRPuppets::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor%16])); // %16 because we only have 16 colors :(
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        ui.displacement_plot->addGraph();
        ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        ui.pwm_plot->addGraph();
        ui.pwm_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
    }
    ui.position_plot->xAxis->setLabel("time[s]");
    ui.position_plot->yAxis->setLabel("ticks");
    ui.position_plot->replot();

    ui.velocity_plot->xAxis->setLabel("time[s]");
    ui.velocity_plot->yAxis->setLabel("ticks/s");
    ui.velocity_plot->replot();

    ui.displacement_plot->xAxis->setLabel("time[s]");
    ui.displacement_plot->yAxis->setLabel("ticks");
    ui.displacement_plot->replot();

    ui.pwm_plot->xAxis->setLabel("time[s]");
    ui.pwm_plot->yAxis->setLabel("[1]");
    ui.pwm_plot->replot();

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_status_rqt_plugin");
    }

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.all_to_position, SIGNAL(clicked()), this, SLOT(allToPosition()));
    QObject::connect(ui.all_to_velocity, SIGNAL(clicked()), this, SLOT(allToVelocity()));
    QObject::connect(ui.all_to_displacement, SIGNAL(clicked()), this, SLOT(allToDisplacement()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();

    uint32_t ip;
    inet_pton(AF_INET, "192.168.255.255", &ip);
    udp.reset(new UDPSocket(8000));
    udp_command.reset(new UDPSocket(8001));
    udp_thread.reset(new std::thread(&VRPuppets::receiveStatusUDP, this));
    udp_thread->detach();

    QScrollArea* scrollArea = widget_->findChild<QScrollArea *>("motor_command");
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

    ros::Duration d(3);
    ROS_INFO("waiting 3 seconds for active motors");
    d.sleep();
    updateMotorCommands();
    initialized = true;
}

void VRPuppets::shutdownPlugin() {
    motorStatus.shutdown();
}

void VRPuppets::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void VRPuppets::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void VRPuppets::receiveStatusUDP() {
    ROS_INFO("start receiving udp");
    while (ros::ok()) {
        int bytes_received = udp->receiveUDPFromClient();
        if (bytes_received == 20) {
            ros::Duration delta = (ros::Time::now() - start_time);
            time.push_back(delta.toSec());
            int motor = udp->buf[0];
            auto it = ip_address.find(motor);
            if (it == ip_address.end()) {
                ROS_INFO("new motor");
                char IP[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &udp->client_addr.sin_addr, IP, INET_ADDRSTRLEN);
                ip_address[motor] = IP;
            }
            int32_t pos = (int32_t) ((uint8_t) udp->buf[7] << 24 | (uint8_t) udp->buf[6] << 16 |
                                        (uint8_t) udp->buf[5] << 8 | (uint8_t) udp->buf[4]);
            int32_t vel = (int32_t) ((uint8_t) udp->buf[11] << 24 | (uint8_t) udp->buf[10] << 16 |
                                     (uint8_t) udp->buf[9] << 8 | (uint8_t) udp->buf[8]);
            int32_t dis = (int32_t) ((uint8_t) udp->buf[15] << 24 | (uint8_t) udp->buf[14] << 16 |
                                     (uint8_t) udp->buf[13] << 8 | (uint8_t) udp->buf[12]);
            int32_t pwm = (int32_t) ((uint8_t) udp->buf[19] << 24 | (uint8_t) udp->buf[18] << 16 |
                                     (uint8_t) udp->buf[17] << 8 | (uint8_t) udp->buf[16]);
//            ROS_INFO_THROTTLE(1,"%d",vel);
            motor_position[motor].push_back(pos);
            motor_velocity[motor].push_back(vel);
            motor_displacement[motor].push_back(dis);
            motor_pwm[motor].push_back(pwm);
//            ROS_INFO_THROTTLE(1,"receiving status from motor %d, pos=%d, vel=%d, dis=%d, pwm=%d",motor,pos,vel,dis,pwm);
            if (motor_position[motor].size() > samples_per_plot) {
                motor_position[motor].pop_front();
                motor_velocity[motor].pop_front();
                motor_displacement[motor].pop_front();
                motor_pwm[motor].pop_front();
            }
            if (time.size() > samples_per_plot)
                time.pop_front();

            if ((counter++) % 20 == 0) {
                Q_EMIT newData();
            }
            if (counter % 100 == 0 && initialized) {
                rescale();
            }
        }
    }
    ROS_INFO("stop receiving udp");
}

void VRPuppets::updateMotorCommands(){
    for(auto w:widgets) {
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
        c->setText(QString(m.first));
        c->setFixedSize(60,30);
        c->setCheckable(true);
        c->setObjectName(QString(m.first));
        check[m.first] = c;
        widget->layout()->addWidget(c);

        QLabel *label = new QLabel(widget);
        label->setFixedSize(120,30);
        label->setText(m.second.c_str());
        widget->layout()->addWidget(label);

        QRadioButton *p = new QRadioButton(widget);
        p->setText("pos");
        p->setFixedSize(60,30);
        p->setCheckable(true);
        p->setObjectName("pos");
        pos[m.first] = p;
        QObject::connect(p, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        widget->layout()->addWidget(p);

        QRadioButton *v = new QRadioButton(widget);
        v->setText("vel");
        v->setFixedSize(60,30);
        v->setCheckable(true);
        v->setObjectName("vel");
        widget->layout()->addWidget(v);
        vel[m.first] = v;
        QObject::connect(v, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QRadioButton *d = new QRadioButton(widget);
        d->setText("dis");
        d->setFixedSize(60,30);
        d->setCheckable(true);
        d->setObjectName("dis");
        d->setChecked(true);
        control_mode[m.first] = DISPLACEMENT;
        widget->layout()->addWidget(d);
        dis[m.first] = d;
        QObject::connect(d, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
        slider->setFixedSize(100,30);
        slider->setValue(50);
        widget->layout()->addWidget(slider);
        sliders[m.first] = slider;

        QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sendCommand()));

        motor_command_scrollarea->layout()->addWidget(widget);
        widgets.push_back(widget);
    }
}

void VRPuppets::plotData() {
    for (auto m:ip_address) {
        ui.position_plot->graph(m.first)->setData(time, motor_position[m.first]);
        ui.velocity_plot->graph(m.first)->setData(time, motor_velocity[m.first]);
        ui.displacement_plot->graph(m.first)->setData(time, motor_displacement[m.first]);
        ui.pwm_plot->graph(m.first)->setData(time, motor_pwm[m.first]);
    }

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.displacement_plot->xAxis->rescale();
    ui.pwm_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.displacement_plot->replot();
    ui.pwm_plot->replot();
}

void VRPuppets::rescale(){
    double minima[NUMBER_OF_MOTORS_PER_FPGA][4], maxima[NUMBER_OF_MOTORS_PER_FPGA][4];
    uint minimal_motor[4] = {0,0,0,0}, maximal_motor[4] = {0,0,0,0};
    map<int,QVector<double>> *motorData[4];
    motorData[0] = &motor_position;
    motorData[1] = &motor_velocity;
    motorData[2] = &motor_displacement;
    motorData[3] = &motor_pwm;
    for(uint type=0;type<4;type++) {
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
        if (minimal_motor[0] == m.first||maximal_motor[0] == m.first)
            ui.position_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[1] == m.first||maximal_motor[1] == m.first)
            ui.velocity_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[2] == m.first||maximal_motor[2] == m.first)
            ui.displacement_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[3] == m.first||maximal_motor[3] == m.first)
            ui.pwm_plot->graph(m.first)->rescaleAxes();
    }

    for (auto m:ip_address) {
        if (minimal_motor[0] != m.first||maximal_motor[0] != m.first)
            ui.position_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[1] != m.first||maximal_motor[1] != m.first)
            ui.velocity_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[2] != m.first||maximal_motor[2] != m.first)
            ui.displacement_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[3] != m.first||maximal_motor[3] != m.first)
            ui.pwm_plot->graph(m.first)->rescaleAxes(true);
    }
}

void VRPuppets::sendCommand(){
    udp_command->client_addr.sin_port = htons(8001);
    udp_command->numbytes = 10;
    for(auto m:ip_address){
        bool ok;
        int motor_scale = ui.scale->text().toInt(&ok);
        if(!ok)
            return;
        set_points[m.first] = (sliders[m.first]->value()-50)*motor_scale;
        mempcpy(udp_command->buf,&set_points[m.first],4);
        mempcpy(&udp_command->buf[4],&m.first,4);
        udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
        udp_command->sendUDPToClient();
    }
}

void VRPuppets::controlModeChanged(){
    udp_command->client_addr.sin_port = htons(8001);
    udp_command->numbytes = 20;
    int Kp, Ki, Kd;
    for(auto m:ip_address) {
        bool ok;
        if (pos[m.first]->isChecked()){
            Kp = ui.Kp_pos->text().toInt(&ok);
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
        }else if(vel[m.first]->isChecked()) {
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
        }else if(dis[m.first]->isChecked()) {
            Kp = ui.Kp_dis->text().toInt(&ok);
            if(!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_dis->text().toInt(&ok);
            if(!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_dis->text().toInt(&ok);
            if(!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        }else{
            ROS_ERROR("haeeee????");
            return;
        }
        mempcpy(&udp_command->buf[0],&Kd,4);
        mempcpy(&udp_command->buf[4],&Ki,4);
        mempcpy(&udp_command->buf[8],&Kp,4);
        mempcpy(&udp_command->buf[12],&control_mode[m.first],4);
        mempcpy(&udp_command->buf[16],&m.first,4);
        udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
        udp_command->sendUDPToClient();
    }
}

void VRPuppets::allToPosition(){
    for(auto m:ip_address){
        control_mode[m.first] = POSITION;
        pos[m.first]->setChecked(true);
        vel[m.first]->setChecked(false);
        dis[m.first]->setChecked(false);
    }
    controlModeChanged();
}

void VRPuppets::allToVelocity(){
    for(auto m:ip_address){
        control_mode[m.first] = VELOCITY;
        pos[m.first]->setChecked(false);
        vel[m.first]->setChecked(true);
        dis[m.first]->setChecked(false);
    }
    controlModeChanged();
}

void VRPuppets::allToDisplacement(){
    for(auto m:ip_address){
        control_mode[m.first] = DISPLACEMENT;
        pos[m.first]->setChecked(false);
        vel[m.first]->setChecked(false);
        dis[m.first]->setChecked(true);
    }
    controlModeChanged();
}

PLUGINLIB_EXPORT_CLASS(VRPuppets, rqt_gui_cpp::Plugin)