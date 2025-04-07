#include "riptide_rviz/ElectricalPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    ElectricalPanel::ElectricalPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_ElectricalPanel();
        ui->setupUi(this);
        setStatus("", false);
        loaded = false;
    }


    ElectricalPanel::~ElectricalPanel()
    {
        delete ui;
    }


    void ElectricalPanel::load(const rviz_common::Config &config) 
    {
        config.mapGetString("robot_namespace", &robotNs);
        if(robotNs == "")
        {
            robotNs = QString::fromStdString("/talos"); 
            RVIZ_COMMON_LOG_WARNING("ElectricalPanel: Using /talos as the default value for robot_namespace"); 
        }

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // make the publisher for electrical command
        std::string topicName = robotNs.toStdString() + "/command/electrical";
        pub = node->create_publisher<riptide_msgs2::msg::ElectricalCommand>(topicName, 10);

        //make the action client for the imu mag cal
        std::string 
            fullMagCalActionName = robotNs.toStdString() + MAG_CAL_ACTION_NAME,
            fullTareGyroActionName = robotNs.toStdString() + TARE_GYRO_ACTION_NAME,
            fullDepressurizeActionName = robotNs.toStdString() + DEPRESSURIZE_ACTION_NAME;

        imuCalClient = rclcpp_action::create_client<MagCal>(node, fullMagCalActionName);
        tareGyroClient = rclcpp_action::create_client<TareGyro>(node, fullTareGyroActionName);
        depressurizeClient = rclcpp_action::create_client<Depressurize>(node, fullDepressurizeActionName);

        // Make client for imu register config
        std::string fullServiceName = robotNs.toStdString() + CONFIG_SERVICE_NAME;
        imuConfigClient = node->create_client<ImuConfig>(fullServiceName);

        loaded = true;
    }


    void ElectricalPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }


    void ElectricalPanel::onInitialize()
    {
        connect(ui->commandSend, &QPushButton::clicked, this, &ElectricalPanel::sendElectricalCommand);
        connect(ui->magCalSend, &QPushButton::clicked, this, &ElectricalPanel::sendMagCal);

        connect(ui->imuRead_2, &QPushButton::clicked, this, &ElectricalPanel::readIMU);
        connect(ui->imuWrite_2, &QPushButton::clicked, this, &ElectricalPanel::writeIMU);
        connect(ui->imuWriteSettings_2, &QPushButton::clicked, this, &ElectricalPanel::saveImuSettings);
        connect(ui->commandTareFog, &QPushButton::clicked, this, &ElectricalPanel::sendTareGyro);
        connect(ui->PVTButton, &QPushButton::clicked, this, &ElectricalPanel::sendDepressurizationCommand);

        //initial UI state
        ui->calibProgress->setValue(0);
        setStatus("", false);
        ui->magCalSend->setText("Calibrate");

        ui->registerNum_2->setText("");
        ui->registerData_2->setText("");
    }


    void ElectricalPanel::sendDepressurizationCommand(){
        if(!loaded)
        {
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
            return;
        }

        if(depressurizationInProgress){
            depressurizeClient->async_cancel_all_goals();
            setStatus("Cancelling Depressurization - Please allow air back into AUV before restarting!", true);
            return;
        }
        depressurizationInProgress = true;


        if(imuCalInProgress || imuCalInProgress){
            setStatus("Already running a different calibration! Not doin it chief...", true);
            return;
        }

        // make sure the depressurization server is online
        if(!depressurizeClient->wait_for_action_server(1s)){
            setStatus("Pressure Server Unavailable!", true);
            return;
        }

        //set UI to zero percent complete
        ui->calibProgress->setValue(0);

        //get values from samples
        double sampleTime = ui->PVTSampleTime->value();
        netDepressurization = ui->PVTDepressurization->value();

        if(netDepressurization > .3){
            setStatus("You gonna break Talos! Cancelling! P.S. If for whatever cursed reason you do in fact need to lower the pressure this far, get send the action from the command line! But, be carefull, the electronics should be exposed to a pressure no less than .7bar", true);
            return;
        }

        Depressurize::Goal depressurize_goal;
        depressurize_goal.sampling_time = sampleTime;
        depressurize_goal.net_depressuization = netDepressurization;

        DepressurizeSendGoalOptions options;
        options.goal_response_callback  = std::bind(&ElectricalPanel::depressurizeGoalResponseCb, this, _1);
        options.feedback_callback       = std::bind(&ElectricalPanel::depressurizeFeedbackCb, this, _1, _2);
        options.result_callback         = std::bind(&ElectricalPanel::depressurizeResultCb, this, _1);

        depressurizeClient->async_send_goal(depressurize_goal, options);
        setStatus("Sent goal!", true);

        ui->PVTButton->setText("Cancel");
    }

    void ElectricalPanel::depressurizeGoalResponseCb(const DepressurizeGoalHandle::SharedPtr & goal_handle){

    }

    void ElectricalPanel::depressurizeFeedbackCb(DepressurizeGoalHandle::SharedPtr, const std::shared_ptr<const Depressurize::Feedback> feedback){
        double percent_depressurized = 100* feedback->current_pressure / netDepressurization;
        
        ui->calibProgress->setValue((int) percent_depressurized);

        if( percent_depressurized > 99.999){
            //if fully depressurized, show message to have user pull pump

            setStatus("Please remove the pump and replace the plug. Rather quickly if you will!", true);
        }
    }

    void ElectricalPanel::depressurizeResultCb(const DepressurizeGoalHandle::WrappedResult & result){
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                ui->calibProgress->setValue(100);
                setStatus("Pressue Lowered", true);

                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Depressurization aborted", true);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Depressurization canceled!", false);
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                setStatus("Uknown depressurization result recieved???", true);
                break;
        }

        depressurizationInProgress = false;
        ui->PVTButton->setText("Start PVT");
    }

    void ElectricalPanel::sendElectricalCommand()
    {
        if(loaded)
        {
            riptide_msgs2::msg::ElectricalCommand msg;
            msg.command = ui->commandSelect->currentIndex();
            pub->publish(msg);
            setStatus("", false);
        } else 
        {
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
        }
    }


    void ElectricalPanel::sendMagCal()
    {
        if(! loaded){
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
            return;
        }

        if(imuCalInProgress){
            imuCalClient->async_cancel_all_goals();
            setStatus("Cancelling calibration request", false);
            return;
        }

        if(depressurizationInProgress || gyroTareInProgress){
            setStatus("Already running a different calibration! Not doin it chief...", true);
            return;
        }

        // reset the panel state for cal
        ui->calibProgress->setValue(0);
        maxVar = 1e-10;

        // make sure the cal server is online
        if(!imuCalClient->wait_for_action_server(1s)){
            setStatus("Calibration server unavailiable!", true);
            return;
        }

        // create and send goal
        MagCal::Goal cal_goal;
        MagSendGoalOptions options;
        options.goal_response_callback  = std::bind(&ElectricalPanel::magCalGoalResponseCb, this, _1);
        options.feedback_callback       = std::bind(&ElectricalPanel::magCalFeedbackCb, this, _1, _2);
        options.result_callback         = std::bind(&ElectricalPanel::magCalResultCb, this, _1);
        imuCalClient->async_send_goal(cal_goal, options);
        imuCalInProgress = true;

        ui->magCalSend->setText("Cancel");
    }


    void ElectricalPanel::sendTareGyro()
    {
        if(! loaded){
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
            return;
        }

        if(gyroTareInProgress){
            tareGyroClient->async_cancel_all_goals();
            setStatus("Cancelling tare request", false);
            return;
        }
                
        if(depressurizationInProgress || imuCalInProgress){
            setStatus("Already running a different calibration! Not doin it chief...", true);
            return;
        }
        
        setStatus("Sending gyro tare goal", false);

        // reset the panel state for cal
        ui->calibProgress->setValue(0);

        // make sure the cal server is online
        if(!tareGyroClient->wait_for_action_server(1s)){
            setStatus("Tare server unavailiable!", true);
            return;
        }

        // create and send goal
        TareGyro::Goal tare_goal;
        tare_goal.num_samples = ui->tareSamples->value();
        tare_goal.timeout_seconds = ui->tareTimeout->value();
        TareGyroSendGoalOptions options;
        options.goal_response_callback  = std::bind(&ElectricalPanel::tareGyroGoalResponseCb, this, _1);
        options.result_callback         = std::bind(&ElectricalPanel::tareGyroResultCb, this, _1);
        tareGyroClient->async_send_goal(tare_goal, options);
        ui->commandTareFog->setText("Cancel");
    }


    void ElectricalPanel::setStatus(const QString& status, bool error)
    {
        ui->errLabel->setText(status);
        ui->errLabel->setStyleSheet(error ? "color: red" : "");

        if(error)
        {
            RVIZ_COMMON_LOG_ERROR(status.toStdString());
        } else
        {
            RVIZ_COMMON_LOG_INFO(status.toStdString());
        }
    }


    void ElectricalPanel::magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle){
        if(goal_handle)
        {
            switch(goal_handle->get_status())
            {
                case GOAL_STATE_ACCEPTED:
                    setStatus("Performing mag cal", false);
                    break;
                case GOAL_STATE_CANCELING:
                    setStatus("Canceling calibration...", false);
                    break;
                default:
                    setStatus("Unknown goal state", true);
                    break;
            }
        } else
        {
            setStatus("Calibration request rejected!", true);
        }   
    }


    void ElectricalPanel::magCalFeedbackCb( MagGoalHandle::SharedPtr,
        const std::shared_ptr<const MagCal::Feedback> feedback){


        // compute the new total variance
        double sum_sqaured = 0.0;
        for(double variance : feedback->curr_avg_dev){
            sum_sqaured += variance * variance;
        }
        double total_var = sqrtf64(sum_sqaured);

        // figure out if this is a new max
        if(total_var > maxVar){
            maxVar = total_var;
        }

        // take max divided by current
        float disp_var = 100 * (1.0 - total_var / maxVar);

        // show it to the user
        ui->calibProgress->setValue((int)disp_var);
    }


    void ElectricalPanel::magCalResultCb(const MagGoalHandle::WrappedResult & result){
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                ui->calibProgress->setValue(100);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Calibration aborted, Examine driver logs for info", true);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Calibration canceled!", false);
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                setStatus("Uknown calibration result recieved???", true);
                break;
        }

        imuCalInProgress = false;
        ui->magCalSend->setText("Calibrate");
    }


    void ElectricalPanel::tareGyroGoalResponseCb(const TareGyroGoalHandle::SharedPtr & goal_handle){
        if(goal_handle)
        {
            switch(goal_handle->get_status())
            {
                case GOAL_STATE_ACCEPTED:
                    setStatus("Performing gyro tare", false);
                    gyroTareInProgress = true;
                    break;
                case GOAL_STATE_CANCELING:
                    setStatus("Canceling tare", false);
                    break;
                default:
                    setStatus("Unknown goal state", true);
                    break;
            }
        } else
        {
            setStatus("Tare request rejected!", true);
        }   
    }


    void ElectricalPanel::tareGyroResultCb(const TareGyroGoalHandle::WrappedResult & result){
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                setStatus("Tare succeeded!", false);
                ui->calibProgress->setValue(100);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Tare aborted: " + QString::fromStdString(result.result->result), true);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Tare canceled!", false);
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                setStatus("Unknown tare result recieved???", true);
                break;
        }

        gyroTareInProgress = false;
        ui->commandTareFog->setText("Calibrate");
    }

    void ElectricalPanel::sendIMUConfigRequest(const std::string& requestStr, bool extResponseTime) {
        // yoink local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        auto start = node->get_clock()->now();
        while (!imuConfigClient->wait_for_service(100ms)) {
            if (node->get_clock()->now() - start > 1s || rclcpp::ok()) {
                setStatus("Config server unavailable", true);
                return;
            }
        }

        auto request = std::make_shared<ImuConfig::Request>();
        request->request = requestStr;

        auto imuConfigFutureInfo = imuConfigClient->async_send_request(request);
        imuConfigFuture = imuConfigFutureInfo.share();
        imuConfigFutureId = imuConfigFutureInfo.request_id;

        timerTick = 0;
        QTimer::singleShot(250, [this, extResponseTime]() { waitForConfig(extResponseTime); });
    }

    void ElectricalPanel::waitForConfig(bool extResponseTime) {
        if (!imuConfigFuture.valid()) {
            setStatus("Future invalidated", true);
            return;   
        }

        int maxNumTicks = extResponseTime ? 20 : 10;

        auto futureStatus = imuConfigFuture.wait_for(10ms);
        if (futureStatus == std::future_status::timeout && timerTick < 10) {
            QTimer::singleShot(250, [this, extResponseTime]() {waitForConfig(extResponseTime);});
            timerTick++;
        }
        else if (futureStatus != std::future_status::timeout) {
            // Future validated, request returned
            auto response = imuConfigFuture.get();

            std::string strResponse = response->response;
            strResponse = strResponse.substr(strResponse.find(',') + 1);
            strResponse = strResponse.substr(strResponse.find(',') + 1);
            strResponse = strResponse.substr(0, strResponse.find('*'));

            ui->registerData_2->setText(strResponse.c_str());

            if (strResponse.substr(0, strResponse.find('*')) == "$VNWNV")
                setStatus("Flashed IMU settings", false);
        }
        else if (timerTick >= maxNumTicks) {
            setStatus("Config service never responded", true);
            imuConfigClient->remove_pending_request(imuConfigFutureId);
        }
    }

    void ElectricalPanel::readIMU() {
        std::string requestStr = "$VNRRG," + ui->registerNum_2->text().toStdString();
        sendIMUConfigRequest(requestStr);
    }

    void ElectricalPanel::writeIMU() {
        std::string requestStr = "$VNWRG," + ui->registerNum_2->text().toStdString() + "," + ui->registerData_2->text().toStdString();
        sendIMUConfigRequest(requestStr);
    }

    void ElectricalPanel::saveImuSettings() {
        std::string requestStr = "$VNWNV";
        sendIMUConfigRequest(requestStr, true);
    }

}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ElectricalPanel, rviz_common::Panel);
