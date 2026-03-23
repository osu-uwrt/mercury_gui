#include <QStandardItemModel>
#include <QTimer>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mercury_msgs/action/execute_tree.hpp>
#include <mercury_msgs/msg/led_command.hpp>
#include <mercury_msgs/msg/tree_stack.hpp>
#include <mercury_msgs/srv/list_trees.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>

#include "ui_MissionPanel.h"

namespace mercury_rviz
{

using ExecuteTree = mercury_msgs::action::ExecuteTree;
using GHExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

class MissionPanel : public rviz_common::Panel
{
Q_OBJECT public : MissionPanel(QWidget * parent = 0);
  ~MissionPanel();

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void onInitialize() override;
protected Q_SLOTS:
  // QT slots (function callbacks)
  void refresh();
  void handleSelectTree(int);

  void startTask();
  void cancelTask();

protected:
  bool event(QEvent * event);

  // action server callbacks
  void taskStartCb(const GHExecuteTree::SharedPtr & goalHandle);
  void cancelAccept(const action_msgs::srv::CancelGoal::Response::SharedPtr);
  void taskCompleteCb(const GHExecuteTree::WrappedResult & result);
  void taskFeedbackCb(
    GHExecuteTree::SharedPtr goalHandle, ExecuteTree::Feedback::ConstSharedPtr feedback);

  //subscriber callback on the tree stack
  void stackCb(const mercury_msgs::msg::TreeStack & stack);

  //subscriber callback for led commands
  void ledCb(const mercury_msgs::msg::LedCommand & cmd);

  // timer callabck for refresh future
  void waitForRefresh(void);

private:
  void updateLedReadout(void);

  Ui_MissionPanel * uiPanel;

  // tree view item model
  QStandardItemModel * model;

  // stack
  std::vector<std::string> treeList;

  // led command
  mercury_msgs::msg::LedCommand lastLedCommand;
  rclcpp::TimerBase::SharedPtr ledTimer;

  rclcpp::Subscription<mercury_msgs::msg::TreeStack>::SharedPtr stackSub;
  rclcpp::Subscription<mercury_msgs::msg::LedCommand>::SharedPtr ledSub;

  // refresh request info
  rclcpp::Client<mercury_msgs::srv::ListTrees>::SharedPtr refreshClient;
  std::shared_future<std::shared_ptr<mercury_msgs::srv::ListTrees_Response>> refreshFuture;
  int64_t refreshFutureid = -1;

  int timerTick = -1;

  rclcpp_action::Client<ExecuteTree>::SharedPtr actionServer;

  std::string robot_ns;
};

}  // namespace mercury_rviz
