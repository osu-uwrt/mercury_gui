#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rviz_common/panel.hpp>
#include <QTimer>

#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/imu_config.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>
#include <riptide_msgs2/action/depressurize.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>

#include "ui_ElectricalPanel.h"

namespace riptide_rviz
{
    const static std::string 
        MAG_CAL_ACTION_NAME = "/vectornav/mag_cal",
        TARE_GYRO_ACTION_NAME = "/gyro/tare",
        DEPRESSURIZE_ACTION_NAME = "/depressurize",
        CONFIG_SERVICE_NAME = "/vectornav/config";

    class ElectricalPanel : public rviz_common::Panel
    {
        using MagCal = riptide_msgs2::action::MagCal;
        using MagSendGoalOptions = rclcpp_action::Client<MagCal>::SendGoalOptions;
        using MagGoalHandle = rclcpp_action::Client<MagCal>::GoalHandle;

        using TareGyro = riptide_msgs2::action::TareGyro;
        using TareGyroSendGoalOptions = rclcpp_action::Client<TareGyro>::SendGoalOptions;
        using TareGyroGoalHandle = rclcpp_action::Client<TareGyro>::GoalHandle;

        using Depressurize = riptide_msgs2::action::Depressurize;
        using DepressurizeSendGoalOptions = rclcpp_action::Client<Depressurize>::SendGoalOptions;
        using DepressurizeGoalHandle = rclcpp_action::Client<Depressurize>::GoalHandle;
        using ImuConfig = riptide_msgs2::srv::QueryImuSerial;
        

        Q_OBJECT
        public:
        ElectricalPanel(QWidget *parent = 0);
        ~ElectricalPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private Q_SLOTS:
        void sendElectricalCommand();
        void sendDepressurizationCommand();
        void sendMagCal();
        void sendTareGyro();

        void depressurizeGoalResponseCb(const DepressurizeGoalHandle::SharedPtr & goal_handle);
        void depressurizeFeedbackCb(DepressurizeGoalHandle::SharedPtr, const std::shared_ptr<const Depressurize::Feedback> feedback);
        void depressurizeResultCb(const DepressurizeGoalHandle::WrappedResult & result);

        void writeIMU();
        void readIMU();
        void saveImuSettings();

        private:
        void setStatus(const QString& status, bool error);
        void magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle);
        void magCalFeedbackCb(
            MagGoalHandle::SharedPtr,
            const std::shared_ptr<const MagCal::Feedback> feedback);
        void magCalResultCb(const MagGoalHandle::WrappedResult & result);
        void tareGyroGoalResponseCb(const TareGyroGoalHandle::SharedPtr & goal_handle);
        void tareGyroResultCb(const TareGyroGoalHandle::WrappedResult & result);
        Qt::CheckState processCheckState(bool state);



        void resultCb(const MagGoalHandle::WrappedResult & result);

        void sendIMUConfigRequest(const std::string& request, bool extResponseTime = false);
        void waitForConfig(bool extResponseTime = false);

        // electrical command vars
        bool loaded = false;
        Ui_ElectricalPanel *ui;
        QString robotNs;

        // mag cal vars
        bool 
            imuCalInProgress = false,
            gyroTareInProgress = false,
            depressurizationInProgress = false;
            
        double maxVar = 0.0;
        double netDepressurization = 100000;

        // Continuous mag cal vars
        bool imuHsiEnable = false;
        bool imuHsiOutput = false;
        int imuConvergenceRate = 1; 

        rclcpp::Publisher<riptide_msgs2::msg::ElectricalCommand>::SharedPtr pub;
        rclcpp_action::Client<MagCal>::SharedPtr imuCalClient;
        rclcpp_action::Client<TareGyro>::SharedPtr tareGyroClient;
        rclcpp_action::Client<Depressurize>::SharedPtr depressurizeClient;
        
        rclcpp::Client<ImuConfig>::SharedPtr imuConfigClient;
        std::shared_future<std::shared_ptr<riptide_msgs2::srv::QueryImuSerial_Response>> imuConfigFuture;
        int timerTick {};
        int imuConfigFutureId {};
    };
}