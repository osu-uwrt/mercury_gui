#pragma once
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLineEdit>
#include <QMap>
#include <QPushButton>
#include <QStatusBar>
#include <QVariant>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace mercury_rviz
{

class ParamPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  ParamPanel(QWidget * parent = 0);
  ~ParamPanel();
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void onInitialize() override;

private slots:
  void apply();
  void setNode(const QString & text);
  void refresh();
  void filterParameters(const QString & text);
  void parameterValueChanged();
  void showStatusMessage(const QString & message, int timeout = 3000);

private:
  void getNodes();
  std::vector<rclcpp::Parameter> getParams();
  void createParams(std::vector<rclcpp::Parameter> params);
  void eraseLayout(QLayout * layout);
  void storeOriginalValue(QWidget * widget, const QString & name);
  void highlightChanges(QWidget * widget, const QString & name);
  QVariant getCurrentValue(QWidget * widget);
  bool loaded = 0, connected = 0;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::SyncParametersClient> param_client;
  QComboBox * nodes;
  QLineEdit * searchBox;
  QWidget * searchContainer;
  QVBoxLayout * paramLayout;
  QPushButton * applyButton;
  QPushButton * refreshButton;
  QStatusBar * statusBar;
  QString robotNs;
  QMap<QString, QVariant> originalValues;
  QMap<QString, QWidget *> arrayContainers;
  bool eventFilter(QObject * obj, QEvent * event) override;
};

}  // namespace mercury_rviz
