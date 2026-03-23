#include "mercury_rviz/DiagnosticOverlay.hpp"

#include <QFontDatabase>
#include <QMessageBox>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace mercury_rviz
{

DiagnosticOverlay::DiagnosticOverlay()
{
  // init font parameter
  QFontDatabase database;
  fontFamilies = database.families();
  fontProperty = new rviz_common::properties::EnumProperty(
    "font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
  for (ssize_t i = 0; i < fontFamilies.size(); i++) {
    fontProperty->addOption(fontFamilies[i], (int)i);
  }

  robotNsProperty = new rviz_common::properties::StringProperty(
    "robot_namespace", "/talos", "Robot namespace to attach to", this, SLOT(updateNS()));

  timeoutProperty = new rviz_common::properties::FloatProperty(
    "diagnostic_timeout", 10.0, "Maximum time between diagnostic packets before default", this);
}

DiagnosticOverlay::~DiagnosticOverlay()
{
  delete robotNsProperty;
  delete timeoutProperty;
}

void DiagnosticOverlay::onInitialize()
{
  OverlayDisplay::onInitialize();

  // get our local rosnode
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // backdate timeouts
  lastDiag = node->get_clock()->now() - 1h;
  lastKill = node->get_clock()->now() - 1h;
  lastZed = node->get_clock()->now() - 1h;
  lastLeak = node->get_clock()->now() - 1h;
  lastCpuTemp = node->get_clock()->now() - 1h;
  stbdlastBattery = node->get_clock()->now() - 1h;
  portlastBattery = node->get_clock()->now() - 1h;

  // subs
  diagSub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", rclcpp::SystemDefaultsQoS(),
    std::bind(&DiagnosticOverlay::diagnosticCallback, this, _1));

  std::string zedTopic = robotNsProperty->getStdString() + "/ffc/zed_node/temperature/imu";
  zedSub = node->create_subscription<sensor_msgs::msg::Temperature>(
    zedTopic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::zedCallback, this, _1));

  std::string leakTopic = robotNsProperty->getStdString() + "/state/leak";
  leakSub = node->create_subscription<std_msgs::msg::Bool>(
    leakTopic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::leakCallback, this, _1));

  std::string batteryKillTopic = robotNsProperty->getStdString() + "/command/electrical";

  std::string batteryInfoTopic = robotNsProperty->getStdString() + "/state/battery";
  batterySub = node->create_subscription<mercury_msgs::msg::BatteryStatus>(
    batteryInfoTopic, rclcpp::SystemDefaultsQoS(),
    std::bind(&DiagnosticOverlay::batteryCallback, this, _1));

  // watchdog timers for handling timeouts
  checkTimer = node->create_wall_timer(0.25s, std::bind(&DiagnosticOverlay::checkTimeout, this));

  // add all of the variable design items

  diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
  diagLedConfigId = addCircle(diagLedConfig);

  killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
  killLedConfigId = addCircle(killLedConfig);

  zedLedConfig.inner_color_ = QColor(255, 0, 255, 255);
  zedLedConfigId = addCircle(zedLedConfig);

  leakLedConfig.inner_color_ = QColor(255, 0, 255, 255);
  leakLedConfigId = addCircle(leakLedConfig);

  portBatterySocConfig.text_color_ = QColor(255, 255, 0, 255);
  portBatterySocConfig.text_ = "P:??%";
  portBatterySocTextId = addText(portBatterySocConfig);

  stbdBatterySocConfig.text_color_ = QColor(255, 255, 0, 255);
  stbdBatterySocConfig.text_ = "S:??%";
  stbdBatterySocTextId = addText(stbdBatterySocConfig);

  // add the static design items
  PaintedTextConfig diagLedLabel = {6,        20,    0, 0,  "Diag",
                                    fontName, false, 2, 12, QColor(255, 255, 255, 255)};
  PaintedTextConfig killLedLabel = {50,       20,    0, 0,  "Kill",
                                    fontName, false, 2, 12, QColor(255, 255, 255, 255)};
  PaintedTextConfig ffcLedLabel = {87,       20,    0, 0,  "FFC",
                                   fontName, false, 2, 12, QColor(255, 255, 255, 255)};
  PaintedTextConfig leakLedLabel = {6,        70,    0, 0,  "Leak",
                                    fontName, false, 2, 12, QColor(255, 255, 255, 255)};

  PaintedTextConfig cpuTempLabel = {90,       70,    0, 0,  "CPU",
                                    fontName, false, 2, 12, QColor(255, 255, 255, 255)};

  // init temperature gauge
  cpuTempGaugeArcId = addArc(cpuTempGaugeArc);
  cpuTempGaugeIndicatorId = addArc(cpuTempGaugeIndicator);
  cpuTempTextId = addText(cpuTempTextConfig);

  // Labels
  addText(diagLedLabel);
  addText(killLedLabel);
  addText(ffcLedLabel);
  addText(leakLedLabel);
  addText(cpuTempLabel);
}

void DiagnosticOverlay::updateNS()
{
  RVIZ_COMMON_LOG_INFO_STREAM(
    "DiagnosticOverlay: Robot NS update " << robotNsProperty->getStdString());
  killSub.reset();

  // get our local rosnode
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  killSub = node->create_subscription<std_msgs::msg::Bool>(
    robotNsProperty->getStdString() + "/state/kill", rclcpp::SystemDefaultsQoS(),
    std::bind(&DiagnosticOverlay::killCallback, this, _1));
}

void DiagnosticOverlay::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  // get our local rosnode
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // write down the timestamp that it was recieved
  lastDiag = node->get_clock()->now();
  diagsTimedOut = false;

  // look for specific packets
  for (auto diagnostic : msg.status) {
    // handle general packet for LED
    if (diagnostic.name == "/Robot Diagnostics") {
      // Determine the LED color to use
      if (diagnostic.level == diagnostic.ERROR) {
        diagLedConfig.inner_color_ = QColor(255, 0, 0, 255);
      } else if (diagnostic.level == diagnostic.WARN) {
        diagLedConfig.inner_color_ = QColor(255, 255, 0, 255);
      } else if (diagnostic.level == diagnostic.STALE) {
        diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
      } else {
        diagLedConfig.inner_color_ = QColor(0, 255, 0, 255);
      }

      updateCircle(diagLedConfigId, diagLedConfig);
    }

    else if (diagnostic.name == "/Robot Diagnostics/Computers/Core Temperature") {
      lastCpuTemp = node->get_clock()->now();
      cpuTempTimedOut = false;

      // Find the maximum temperature among cores
      double max_temp = 0.0;
      bool found_temp = false;

      for (auto pair : diagnostic.values) {
        try {
          // Extract temperature value (format: "XX.XX C")
          std::string temp_str = pair.value;
          size_t pos = temp_str.find(" C");
          if (pos != std::string::npos) {
            double temp = std::stod(temp_str.substr(0, pos));
            max_temp = std::max(max_temp, temp);
            found_temp = true;
          }
        } catch (const std::exception & e) {
          // Handle parsing errors
          RVIZ_COMMON_LOG_ERROR_STREAM("Error parsing CPU temperature: " << e.what());
        }
      }

      if (found_temp) {
        // Update gauge indicator
        double angleRange = 360.0;
        double normalizedTemp = std::min(std::max(max_temp, 0.0), 100.0) / 100.0;  // 0-100 C range
        double newAngle = 90.0 - (normalizedTemp * angleRange);  // Start at top and go clockwise
        cpuTempGaugeIndicator.end_angle_ = newAngle;

        // Update color based on temperature thresholds
        if (max_temp > 85.0) {
          cpuTempGaugeIndicator.line_color_ = QColor(255, 0, 0, 255);  // Red
        } else if (max_temp > 70.0) {
          cpuTempGaugeIndicator.line_color_ = QColor(255, 255, 0, 255);  // Yellow
        } else {
          cpuTempGaugeIndicator.line_color_ = QColor(0, 255, 0, 255);  // Green
        }

        // Update text
        cpuTempTextConfig.text_ = QString::number(max_temp, 'f', 1).toStdString() + "°C";

        updateArc(cpuTempGaugeIndicatorId, cpuTempGaugeIndicator);
        updateText(cpuTempTextId, cpuTempTextConfig);
      }
    }
  }
}

// Callback for the FFC indicator
void DiagnosticOverlay::zedCallback(const sensor_msgs::msg::Temperature & msg)
{
  // get our local rosnode
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  zedTimedOut = false;

  zedLedConfig.inner_color_ = QColor(0, 255, 0, 255);
  updateCircle(zedLedConfigId, zedLedConfig);

  lastZed = node->get_clock()->now();
}

// TODO: This should be redone
void DiagnosticOverlay::leakCallback(const std_msgs::msg::Bool & msg)
{
  rclcpp::Node::SharedPtr node;
  leakTimedOut = false;

  if (msg.data) {
    if (redFlash) {
      leakLedConfig.inner_color_ = QColor(255, 0, 0, 255);  // Flash red
      redFlash = false;
    } else {
      leakLedConfig.inner_color_ = QColor(252, 126, 0, 255);  // Flash orange
      redFlash = true;
    }

    if (!startedLeaking) {
      // Pop up an annoying box
      QMessageBox msgBox;
      msgBox.setWindowTitle("LEAK");
      msgBox.setIcon(QMessageBox::Warning);
      msgBox.setText(QString::fromStdString("Water was detected in one of the cages."));

      QPushButton * killButton = msgBox.addButton("Kill Power", QMessageBox::AcceptRole);
      msgBox.setStandardButtons(QMessageBox::Ok);
      msgBox.setDefaultButton(killButton);
      msgBox.exec();

      // Get our local rosnode
      // This has to happen after msgBox.exec or it will hang ros pubs while the msgbox is up
      node = context_->getRosNodeAbstraction().lock()->get_raw_node();

      if (msgBox.clickedButton() == (QAbstractButton *)killButton) {
        RVIZ_COMMON_LOG_WARNING(
          "DiagnosticOverlay: NOT sending battery kill message. This feature was disabled");
      }

      startedLeaking = true;
    }
  } else {
    leakLedConfig.inner_color_ = QColor(0, 255, 0, 255);
    startedLeaking = false;
  }

  // Set node now if it wasn't earlier
  if (!node) node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  updateCircle(leakLedConfigId, leakLedConfig);
  lastLeak = node->get_clock()->now();
}

void DiagnosticOverlay::killCallback(const std_msgs::msg::Bool & msg)
{
  // get our local rosnode
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // write down the time that we recieve the last kill msg
  lastKill = node->get_clock()->now();
  killTimedOut = false;

  if (!msg.data) {
    killLedConfig.inner_color_ = QColor(0, 255, 0, 255);
  } else {
    killLedConfig.inner_color_ = QColor(255, 0, 0, 255);
  }
  updateCircle(killLedConfigId, killLedConfig);
}

void DiagnosticOverlay::updateFont()
{
  int font_index = fontProperty->getOptionInt();
  if (font_index < fontFamilies.size()) {
    fontName = fontFamilies[font_index].toStdString();
  } else {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "DiagnosticOverlay: Unexpected error at selecting font index " << font_index);
    return;
  }

  require_update_texture_ = true;
}

void DiagnosticOverlay::batteryCallback(const mercury_msgs::msg::BatteryStatus & msg)
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  int soc = msg.soc;

  // Determine which battery the state of charge applies to
  if (msg.detect == mercury_msgs::msg::BatteryStatus::DETECT_PORT) {
    int portBatterySoc = soc;

    if (soc < 20) {
      portBatterySocConfig.text_color_ = QColor(255, 0, 0, 255);  // Red
    } else if (soc < 50) {
      portBatterySocConfig.text_color_ = QColor(255, 255, 0, 255);  // Yellow
    } else {
      portBatterySocConfig.text_color_ = QColor(0, 255, 0, 255);  // Green
    }

    portBatterySocConfig.text_ = "P:" + std::to_string(soc) + "%";
    updateText(portBatterySocTextId, portBatterySocConfig);
    portlastBattery = node->get_clock()->now();
    portBatteryTimedOut = false;

  } else if (msg.detect == mercury_msgs::msg::BatteryStatus::DETECT_STBD) {
    int stbdBatterySoc = soc;

    if (soc < 20) {
      stbdBatterySocConfig.text_color_ = QColor(255, 0, 0, 255);  // Red
    } else if (soc < 50) {
      stbdBatterySocConfig.text_color_ = QColor(255, 255, 0, 255);  // Yellow
    } else {
      stbdBatterySocConfig.text_color_ = QColor(0, 255, 0, 255);  // Green
    }

    stbdBatterySocConfig.text_ = "S:" + std::to_string(soc) + "%";
    updateText(stbdBatterySocTextId, stbdBatterySocConfig);
    stbdlastBattery = node->get_clock()->now();
    stbdBatteryTimedOut = false;

  } else {
    RVIZ_COMMON_LOG_WARNING_STREAM(
      "DiagnosticOverlay: Unknown battery detect value: " << (int)msg.detect);
  }
}

void DiagnosticOverlay::onEnable() { OverlayDisplay::onEnable(); }

void DiagnosticOverlay::onDisable() { OverlayDisplay::onDisable(); }

void DiagnosticOverlay::update(float wall_dt, float ros_dt)
{
  OverlayDisplay::update(wall_dt, ros_dt);
}

void DiagnosticOverlay::checkTimeout()
{
  // read current timeout property and convert to duration
  auto timeoutDur = std::chrono::duration<double>(timeoutProperty->getFloat());

  // get our local rosnode
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // check for diagnostic timeout
  auto duration = node->get_clock()->now() - lastDiag;
  if (duration > timeoutDur) {
    if (!diagsTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Diagnostics timed out!");
      diagsTimedOut = true;
    }

    // diagnostics timed out, reset them
    diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
    updateCircle(diagLedConfigId, diagLedConfig);
  }

  // check for kill timeout
  duration = node->get_clock()->now() - lastKill;
  if (duration > timeoutDur) {
    if (!killTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Kill switch timed out!");
      killTimedOut = true;
    }

    // diagnostics timed out, reset them
    killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
    updateCircle(killLedConfigId, killLedConfig);
  }

  duration = node->get_clock()->now() - lastZed;
  if (duration > std::chrono::duration<double>(1.0f)) {
    if (!zedTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: FFC connection timed out!");
      zedTimedOut = true;
    }

    zedLedConfig.inner_color_ = QColor(255, 0, 0, 255);
    updateCircle(zedLedConfigId, zedLedConfig);
  }

  duration = node->get_clock()->now() - lastLeak;
  if (duration > std::chrono::duration<double>(2.0s)) {
    if (!leakTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Leak sensors timed out!");
      leakTimedOut = true;
    }

    leakLedConfig.inner_color_ = QColor(255, 0, 255, 255);
    updateCircle(leakLedConfigId, leakLedConfig);
  }

  duration = node->get_clock()->now() - lastCpuTemp;
  if (duration > std::chrono::duration<double>(2.0)) {
    if (!cpuTempTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: CPU temperature timed out!");
      cpuTempTimedOut = true;
    }

    cpuTempGaugeIndicator.line_color_ = QColor(255, 0, 255, 255);
    cpuTempTextConfig.text_ = "-.--°C";

    updateArc(cpuTempGaugeIndicatorId, cpuTempGaugeIndicator);
    updateText(cpuTempTextId, cpuTempTextConfig);
  }

  duration = node->get_clock()->now() - stbdlastBattery;
  if (duration > std::chrono::duration<double>(10.0)) {
    if (!stbdBatteryTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Starboard battery SOC timed out!");
      stbdBatteryTimedOut = true;
    }
    stbdBatterySocConfig.text_color_ = QColor(255, 0, 255, 255);
    stbdBatterySocConfig.text_ = "S:--%";
    updateText(stbdBatterySocTextId, stbdBatterySocConfig);
  }

  duration = node->get_clock()->now() - portlastBattery;
  if (duration > std::chrono::duration<double>(10.0)) {
    if (!portBatteryTimedOut) {
      RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Port battery SOC timed out!");
      portBatteryTimedOut = true;
    }
    portBatterySocConfig.text_color_ = QColor(255, 0, 255, 255);
    portBatterySocConfig.text_ = "P:--%";
    updateText(portBatterySocTextId, portBatterySocConfig);
  }
}

void DiagnosticOverlay::reset() { OverlayDisplay::reset(); }

}  // namespace mercury_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mercury_rviz::DiagnosticOverlay, rviz_common::Display)
