#include "mercury_rviz/ElectricalPanel.hpp"

#include <math.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace mercury_rviz
{

ElectricalPanel::ElectricalPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setFocusPolicy(Qt::ClickFocus);

  ui = new Ui_ElectricalPanel();
  ui->setupUi(this);
  setStatus("", false);
  loaded = false;
}

ElectricalPanel::~ElectricalPanel() { delete ui; }

void ElectricalPanel::load(const rviz_common::Config & config)
{
  config.mapGetString("robot_namespace", &robotNs);
  if (robotNs == "") {
    robotNs = QString::fromStdString("/mercury");
    RVIZ_COMMON_LOG_WARNING(
      "ElectricalPanel: Using /mercury as the default value for robot_namespace");
  }

  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // make the publisher for electrical command
  std::string topicName = robotNs.toStdString() + "/command/electrical";
  elecPub = node->create_publisher<mercury_msgs::msg::ElectricalCommand>(topicName, 10);

  // ivc pubs and subs
  ivcTxPub = node->create_publisher<std_msgs::msg::UInt8>(robotNs.toStdString() + "/ivc/tx", 10);
  ivcTxSub = node->create_subscription<std_msgs::msg::UInt8>(
    robotNs.toStdString() + "/ivc/tx", 10, std::bind(&ElectricalPanel::ivcTxCb, this, _1));
  ivcRxSub = node->create_subscription<std_msgs::msg::UInt8>(
    robotNs.toStdString() + "/ivc/rx", 10, std::bind(&ElectricalPanel::ivcRxCb, this, _1));
  ivcSuccessSub = node->create_subscription<mercury_msgs::msg::UInt8Stamped>(
    robotNs.toStdString() + "/ivc/tx_success", 10,
    std::bind(&ElectricalPanel::ivcTxSuccessCb, this, _1));

  //make the action client for the imu mag cal
  std::string fullMagCalActionName = robotNs.toStdString() + MAG_CAL_ACTION_NAME;
  imuCalClient = rclcpp_action::create_client<MagCal>(node, fullMagCalActionName);

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
  connect(ui->ivcSend, &QPushButton::clicked, this, &ElectricalPanel::sendIvcMsg);
  connect(ui->magCalSend, &QPushButton::clicked, this, &ElectricalPanel::sendMagCal);

  //initial UI state
  ui->calibProgress->setValue(0);
  setStatus("", false);
  ui->magCalSend->setText("Calibrate");

  ui->ivcConsole->setTabStopWidth(7);
}

void ElectricalPanel::sendElectricalCommand()
{
  if (loaded) {
    mercury_msgs::msg::ElectricalCommand msg;
    msg.command = ui->commandSelect->currentIndex();
    elecPub->publish(msg);
    setStatus("", false);
  } else {
    setStatus("Panel not loaded! Please save your config and restart RViz", true);
  }
}

void ElectricalPanel::sendMagCal()
{
  if (!loaded) {
    setStatus("Panel not loaded! Please save your config and restart RViz", true);
    return;
  }

  if (imuCalInProgress) {
    imuCalClient->async_cancel_all_goals();
    setStatus("Cancelling calibration request", false);
    return;
  }

  // reset the panel state for cal
  ui->calibProgress->setValue(0);
  magCalMaxVar = 1e-10;

  // make sure the cal server is online
  if (!imuCalClient->wait_for_action_server(1s)) {
    setStatus("Calibration server unavailiable!", true);
    return;
  }

  // create and send goal
  MagCal::Goal cal_goal;
  MagSendGoalOptions options;
  options.goal_response_callback = std::bind(&ElectricalPanel::magCalGoalResponseCb, this, _1);
  options.feedback_callback = std::bind(&ElectricalPanel::magCalFeedbackCb, this, _1, _2);
  options.result_callback = std::bind(&ElectricalPanel::magCalResultCb, this, _1);
  imuCalClient->async_send_goal(cal_goal, options);
  imuCalInProgress = true;

  ui->magCalSend->setText("Cancel");
}

void ElectricalPanel::sendIvcMsg()
{
  int idx = ui->consoleTab->currentIndex();
  if (idx != 1)  // 1 is the index of the ivc console tab
  {
    ui->consoleTab->setCurrentIndex(1);
  }

  int8_t header = ui->ivcHeader->currentIndex(), command = 0;

  if (header < 2)  //indicates status message, use status combo box
  {
    command = ui->ivcStatus->currentIndex();
  } else {
    // ... otherwise use generic commmand number
    command = ui->ivcCommand->value();
  }

  //now assemble command
  std_msgs::msg::UInt8 msg;
  msg.data = (header & 0x7) << 5;
  msg.data |= command & 0x1F;

  // ...and send
  ivcTxPub->publish(msg);
}

void ElectricalPanel::setStatus(const QString & status, bool error)
{
  ui->errLabel->setText(status);
  ui->errLabel->setStyleSheet(error ? "color: red" : "");

  if (error) {
    RVIZ_COMMON_LOG_ERROR(status.toStdString());
  } else {
    RVIZ_COMMON_LOG_INFO(status.toStdString());
  }
}

void ElectricalPanel::appendIvcConsole(const QString & prefix, const uint8_t & msg)
{
  uint8_t header = (msg & 0xE0) >> 5, data = msg & 0x1F;

  ui->ivcConsole->append(
    prefix + ": " + QString::fromStdString(std::to_string(header)) + " - " +
    QString::fromStdString(std::to_string(data)));
}

void ElectricalPanel::magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle)
{
  if (goal_handle) {
    switch (goal_handle->get_status()) {
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
  } else {
    setStatus("Calibration request rejected!", true);
  }
}

void ElectricalPanel::magCalFeedbackCb(
  MagGoalHandle::SharedPtr, const std::shared_ptr<const MagCal::Feedback> feedback)
{
  // compute the new total variance
  double sum_sqaured = 0.0;
  for (double variance : feedback->curr_avg_dev) {
    sum_sqaured += variance * variance;
  }
  double total_var = sqrtf64(sum_sqaured);

  // figure out if this is a new max
  if (total_var > magCalMaxVar) {
    magCalMaxVar = total_var;
  }

  // take max divided by current
  float disp_var = 100 * (1.0 - total_var / magCalMaxVar);

  // show it to the user
  ui->calibProgress->setValue((int)disp_var);
}

void ElectricalPanel::magCalResultCb(const MagGoalHandle::WrappedResult & result)
{
  switch (result.code) {
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

void ElectricalPanel::ivcTxCb(const std_msgs::msg::UInt8::SharedPtr msg)
{
  appendIvcConsole("[ SEND --> ] ", msg->data);
}

void ElectricalPanel::ivcRxCb(const std_msgs::msg::UInt8::SharedPtr msg)
{
  appendIvcConsole("[ RECV <-- ] ", msg->data);
}

void ElectricalPanel::ivcTxSuccessCb(const mercury_msgs::msg::UInt8Stamped::SharedPtr msg)
{
  appendIvcConsole("\t[ CONFIRM ]", msg->data);
}

}  // namespace mercury_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mercury_rviz::ElectricalPanel, rviz_common::Panel);
