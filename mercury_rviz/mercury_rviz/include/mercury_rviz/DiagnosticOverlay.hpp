#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mercury_msgs/msg/battery_status.hpp>
#include <mercury_msgs/msg/electrical_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "mercury_rviz/OverlayDisplay.hpp"

namespace mercury_rviz
{

class DiagnosticOverlay : public OverlayDisplay
{
  Q_OBJECT
public:
  DiagnosticOverlay();

  ~DiagnosticOverlay();

protected:
  virtual void onInitialize() override;
  virtual void onEnable() override;
  virtual void onDisable() override;
  virtual void update(float wall_dt, float ros_dt) override;
  virtual void reset() override;

  void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg);
  void killCallback(const std_msgs::msg::Bool & msg);
  void zedCallback(const sensor_msgs::msg::Temperature & msg);
  void leakCallback(const std_msgs::msg::Bool & msg);
  void batteryCallback(const mercury_msgs::msg::BatteryStatus & msg);

  void checkTimeout();

protected Q_SLOTS:
  void updateFont();
  void updateNS();

private:
  // timer for determining timeouts
  rclcpp::TimerBase::SharedPtr checkTimer;

  // times for stamping
  rclcpp::Time lastDiag, lastKill, lastZed, lastLeak, lastCpuTemp, stbdlastBattery, portlastBattery;
  bool diagsTimedOut, killTimedOut, zedTimedOut, leakTimedOut, cpuTempTimedOut, stbdBatteryTimedOut,
    portBatteryTimedOut;

  bool startedLeaking = false;
  bool redFlash = true;

  // subscription for diagnostics
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killSub;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr zedSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leakSub;
  rclcpp::Subscription<mercury_msgs::msg::BatteryStatus>::SharedPtr batterySub;

  // ids for rendering items so that we can edit them
  int diagLedConfigId = -1;
  int killLedConfigId = -1;
  int zedLedConfigId = -1;
  int leakLedConfigId = -1;
  int portBatterySocTextId = -1;
  int stbdBatterySocTextId = -1;

  // font configuration info
  QStringList fontFamilies;
  std::string fontName;

  // Temp vars
  double tempMin{0.0};
  double tempMax{100.0};

  // Addtional RVIZ settings
  rviz_common::properties::EnumProperty * fontProperty;
  rviz_common::properties::StringProperty * robotNsProperty;
  rviz_common::properties::FloatProperty * timeoutProperty;

  // configurations for display items
  PaintedCircleConfig diagLedConfig = {
    20, 50, 0, 0, 7, 9, QColor(255, 0, 255, 255), QColor(0, 0, 0, 255)};
  PaintedCircleConfig killLedConfig = {
    60, 50, 0, 0, 7, 9, QColor(255, 0, 255, 255), QColor(0, 0, 0, 255)};
  PaintedCircleConfig zedLedConfig = {
    100, 50, 0, 0, 7, 9, QColor(255, 0, 255, 255), QColor(0, 0, 0, 255)};
  PaintedCircleConfig leakLedConfig = {
    20, 100, 0, 0, 7, 9, QColor(255, 0, 255, 255), QColor(0, 0, 0, 255)};
  PaintedCircleConfig pressureLedConfig = {
    180, 50, 0, 0, 7, 9, QColor(255, 0, 255, 255), QColor(0, 0, 0, 255)};

  PaintedTextConfig stbdBatterySocConfig = {70,       0,     0, 0,  "S:0%",
                                            fontName, false, 2, 12, QColor(255, 0, 0, 255)};

  PaintedTextConfig portBatterySocConfig = {12,       0,     0, 0,  "P:0%",
                                            fontName, false, 2, 12, QColor(255, 0, 0, 255)};

  // Gauge display elements
  PaintedArcConfig tempGaugeArc{
    60,
    100,   // x, y pos
    10,    // radius
    90,    // start angle (degrees)
    -270,  // end angle (degrees)
    2,     // line width
    QColor(40, 40, 40, 255)};
  int tempGaugeArcId;

  PaintedArcConfig tempGaugeIndicator{
    60,
    100,  // x, y pos
    10,   // radius
    90,   // start angle
    90,   // end angle
    3,    // line thickness
    QColor(0, 255, 0, 255)};
  int tempGaugeIndicatorId;

  PaintedTextConfig tempTextConfig{
    45,
    115,  // x, y pos
    0,
    0,        // offset
    "0.0°C",  // default text
    fontName,
    false,                      // not bold
    2,                          // outline width
    9,                          // font size
    QColor(255, 255, 255, 255)  // white text
  };
  int tempTextId;

  // CPU Temperature gauge elements
  PaintedArcConfig cpuTempGaugeArc{
    100,
    100,   // x, y pos (adjust as needed)
    10,    // radius
    90,    // start angle (degrees)
    -270,  // end angle (degrees)
    2,     // line width
    QColor(40, 40, 40, 255)};
  int cpuTempGaugeArcId;

  PaintedArcConfig cpuTempGaugeIndicator{
    100,
    100,  // x, y pos
    10,   // radius
    90,   // start angle
    90,   // end angle
    3,    // line thickness
    QColor(0, 255, 0, 255)};
  int cpuTempGaugeIndicatorId;

  PaintedTextConfig cpuTempTextConfig{
    85,
    115,  // x, y pos
    0,
    0,        // offset
    "0.0°C",  // default text
    fontName,
    false,                      // not bold
    2,                          // outline width
    9,                          // font size
    QColor(255, 255, 255, 255)  // white text
  };
  int cpuTempTextId;
};

}  // namespace mercury_rviz
