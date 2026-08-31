#pragma once

#include <QTimer>
#include <mercury_msgs/action/mag_cal.hpp>
#include <mercury_msgs/msg/electrical_command.hpp>
#include <mercury_msgs/msg/u_int8_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "ui_ElectricalPanel.h"

namespace mercury_rviz {

const static std::string MAG_CAL_ACTION_NAME = "/vectornav/mag_cal";

class ElectricalPanel : public rviz_common::Panel {
    using MagCal = mercury_msgs::action::MagCal;
    using MagSendGoalOptions = rclcpp_action::Client<MagCal>::SendGoalOptions;
    using MagGoalHandle = rclcpp_action::Client<MagCal>::GoalHandle;

    Q_OBJECT
public:
    ElectricalPanel(QWidget * parent = 0);
    ~ElectricalPanel();

    void load(const rviz_common::Config & config) override;
    void save(rviz_common::Config config) const override;
    void onInitialize() override;

private Q_SLOTS:
    void sendElectricalCommand();
    void sendMagCal();

    void sendIvcMsg();

private:
    void setStatus(const QString & status, bool error);
    void appendIvcConsole(const QString & prefix, const uint8_t & msg);
    void magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle);
    void magCalFeedbackCb(
        MagGoalHandle::SharedPtr, const std::shared_ptr<const MagCal::Feedback> feedback);
    void magCalResultCb(const MagGoalHandle::WrappedResult & result);
    void ivcTxCb(const std_msgs::msg::UInt8::SharedPtr msg);
    void ivcRxCb(const std_msgs::msg::UInt8::SharedPtr msg);
    void ivcTxSuccessCb(const mercury_msgs::msg::UInt8Stamped::SharedPtr msg);

    // electrical command vars
    bool loaded = false;
    Ui_ElectricalPanel * ui;
    QString robotNs;

    // mag cal vars
    bool imuCalInProgress = false;
    double magCalMaxVar = 0.0;

    // Continuous mag cal vars
    bool imuHsiEnable = false;
    bool imuHsiOutput = false;
    int imuConvergenceRate = 1;

    rclcpp::Publisher<mercury_msgs::msg::ElectricalCommand>::SharedPtr elecPub;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ivcTxPub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ivcTxSub, ivcRxSub;

    rclcpp::Subscription<mercury_msgs::msg::UInt8Stamped>::SharedPtr ivcSuccessSub;

    rclcpp_action::Client<MagCal>::SharedPtr imuCalClient;
};

} // namespace mercury_rviz
