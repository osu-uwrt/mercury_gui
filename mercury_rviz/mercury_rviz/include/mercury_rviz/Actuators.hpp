#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>
#include <QMessageBox>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <riptide_msgs2/msg/actuator_status.hpp>
#include "ui_Actuators.h"

using namespace std::chrono_literals;

namespace riptide_rviz
{
    using Empty = std_msgs::msg::Empty;
    using Bool = std_msgs::msg::Bool;
    using ActuatorStatus = riptide_msgs2::msg::ActuatorStatus;

    class Actuators : public rviz_common::Panel
    {
        Q_OBJECT public : Actuators(QWidget *parent = 0);
        ~Actuators();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    private slots:
        void handleArm();
        void handleDisarm();
        void handleOpenClaw();
        void handleCloseClaw();
        void handleFireTorpedo();
        void handleDropMarker();
        void handleReload();
        void handleClawGoHome();
        void handleClawSetHome();
        void handleTorpGoHome();
        void handleTorpSetHome();

    private:
        void statusCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg);
        void updateStatus();
        void cmdFeedbackCb(const std_msgs::msg::String::SharedPtr msg);
        void cmdStatusCb(const Bool::SharedPtr msg);
        void pubEmptyTopic(rclcpp::Publisher<Empty>::SharedPtr pub);
        void pubBoolTopic(rclcpp::Publisher<Bool>::SharedPtr pub, bool value);

        // UI Panel instance
        Ui_Actuators *uiPanel;

        //process vars
        std::string robotNs;

        //status subscriber
        rclcpp::Subscription<ActuatorStatus>::SharedPtr statusSub;

        // Actuator command calls
        rclcpp::Publisher<Empty>::SharedPtr
            dropperTopic,
            torpedoTopic,
            reloadTopic,
            torpMarkerGoHomeTopic,
            torpMarkerSetHomeTopic,
            clawSetClosedPosTopic;

        rclcpp::Publisher<Bool>::SharedPtr
            armTopic,
            clawTopic;

        // Actuator feedback subscriptions
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdFeedback;
        rclcpp::Subscription<Bool>::SharedPtr cmdStatus;

        std::string latestCmdFeedback;
        bool latestCmdStatus = false;
    };

} // namespace riptide_rviz
