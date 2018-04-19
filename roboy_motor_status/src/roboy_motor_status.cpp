#include <roboy_motor_status/roboy_motor_status.hpp>

RoboyMotorStatus::RoboyMotorStatus()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorStatus");
}

void RoboyMotorStatus::initPlugin(qt_gui_cpp::PluginContext &context) {
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
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.displacement_plot->addGraph();
        ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.current_plot->addGraph();
        ui.current_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
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

    ui.current_plot->xAxis->setLabel("time[s]");
    ui.current_plot->yAxis->setLabel("mA");
    ui.current_plot->replot();

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_status_rqt_plugin");
    }

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyMotorStatus::MotorStatus, this);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();
}

void RoboyMotorStatus::shutdownPlugin() {
    motorStatus.shutdown();
}

void RoboyMotorStatus::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorStatus::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyMotorStatus::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving motor status");
    ros::Duration delta = (ros::Time::now()-start_time);
    time.push_back(delta.toSec());
    for (uint motor = 0; motor < msg->position.size(); motor++) {
        std::vector<int>::iterator it = std::find (active_motors[msg->id].begin(), active_motors[msg->id].end(), motor);
        if (it != active_motors[msg->id].end()) {
            motorData[msg->id][motor][0].push_back(msg->position[motor]);
            motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
            motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
            motorData[msg->id][motor][3].push_back(msg->current[motor]);
            if (motorData[msg->id][motor][0].size() > samples_per_plot) {
                motorData[msg->id][motor][0].pop_front();
                motorData[msg->id][motor][1].pop_front();
                motorData[msg->id][motor][2].pop_front();
                motorData[msg->id][motor][3].pop_front();
            }
        }
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if ((counter++) % 20 == 0){
        Q_EMIT newData();
    }

    if(counter%1000 == 0){
        if(msg->id == ui.fpga->value()) {
            if (msg->power_sense)
                ui.power_sense->setStyleSheet("background-color:green;");
            else
                ui.power_sense->setStyleSheet("background-color:red;");
        }
        rescale();
    }
}

void RoboyMotorStatus::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.position_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][0]);
        ui.velocity_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][1]);
        ui.displacement_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][2]);
        ui.current_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][3]);
    }

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.displacement_plot->xAxis->rescale();
    ui.current_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.displacement_plot->replot();
    ui.current_plot->replot();
}

void RoboyMotorStatus::rescale(){
    double minima[NUMBER_OF_MOTORS_PER_FPGA][4], maxima[NUMBER_OF_MOTORS_PER_FPGA][4];
    uint minimal_motor[4] = {0,0,0,0}, maximal_motor[4] = {0,0,0,0};
    for(uint type=0;type<4;type++) {
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            minima[motor][type] = 0;
            maxima[motor][type] = 0;
            for (auto val:motorData[ui.fpga->value()][motor][type]) {
                if (val < minima[motor][type])
                    minima[motor][type] = val;
                if (val > maxima[motor][type])
                    maxima[motor][type] = val;
            }
        }

        for (uint motor = 1; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            if (minima[motor][type] < minima[minimal_motor[type]][type])
                minimal_motor[type] = motor;
            if (maxima[motor][type] < maxima[maximal_motor[type]][type])
                maximal_motor[type] = motor;
        }
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        if (minimal_motor[0] == motor||maximal_motor[0] == motor)
            ui.position_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[1] == motor||maximal_motor[1] == motor)
            ui.velocity_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[2] == motor||maximal_motor[2] == motor)
            ui.displacement_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[3] == motor||maximal_motor[3] == motor)
            ui.current_plot->graph(motor)->rescaleAxes();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        if (minimal_motor[0] != motor||maximal_motor[0] != motor)
            ui.position_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[1] != motor||maximal_motor[1] != motor)
            ui.velocity_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[2] != motor||maximal_motor[2] != motor)
            ui.displacement_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[3] != motor||maximal_motor[3] != motor)
            ui.current_plot->graph(motor)->rescaleAxes(true);
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_status, RoboyMotorStatus, RoboyMotorStatus, rqt_gui_cpp::Plugin)
