#pragma once
#include <yaml-cpp/yaml.h>  // Include the yaml-cpp library

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "ui_FeedforwardPanel.h"

namespace mercury_rviz
{

class FeedforwardPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  FeedforwardPanel(QWidget * parent = 0);
  ~FeedforwardPanel();

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void onInitialize() override;

private slots:
  void XFunction(double value);
  void YFunction(double value);
  void ZFunction(double value);
  void RFunction(double value);
  void PFunction(double value);
  void YAWFunction(double value);
  void togglePublish();

private:
  void timerCb();

  bool loaded = false, publishing = false;

  int counter;
  Ui_FeedforwardPanel * ui;
  QString robotNs;

  rclcpp::TimerBase::SharedPtr pubTimer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ffForcePub;
  geometry_msgs::msg::Twist msgToPublish;
};

}  // namespace mercury_rviz
