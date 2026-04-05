#include "mercury_rviz/DiagnosticsPanel.hpp"

#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QTreeView>
#include <QVBoxLayout>
#include <algorithm>
#include <chrono>
#include <map>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

#include "ui_DiagnosticsPanel.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace mercury_rviz
{

DiagnosticsPanel::DiagnosticsPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setFocusPolicy(Qt::ClickFocus);

  // Initialize UI using the generated header
  uiPanel = new Ui::DiagnosticsPanel();
  uiPanel->setupUi(this);

  // Create model
  model = new QStandardItemModel(this);

  // Set up column headers - now just 2 columns
  model->setColumnCount(2);
  model->setHorizontalHeaderLabels({"Name", "Message"});

  // Flag to track if we've done initial expansion
  timerTick = 0;

  // Create status icons
  createStatusIcons();
}

void DiagnosticsPanel::createStatusIcons()
{
  // Create a set of status icons - simple colored circle icons
  const int iconSize = 16;

  // OK - Green circle
  QPixmap okPixmap(iconSize, iconSize);
  okPixmap.fill(Qt::transparent);
  QPainter okPainter(&okPixmap);
  okPainter.setRenderHint(QPainter::Antialiasing);
  okPainter.setBrush(QColor(0, 255, 0));  // Green
  okPainter.setPen(Qt::darkGreen);
  okPainter.drawEllipse(1, 1, iconSize - 2, iconSize - 2);
  statusIcons[0] = QIcon(okPixmap);

  // WARN - Yellow circle
  QPixmap warnPixmap(iconSize, iconSize);
  warnPixmap.fill(Qt::transparent);
  QPainter warnPainter(&warnPixmap);
  warnPainter.setRenderHint(QPainter::Antialiasing);
  warnPainter.setBrush(QColor(255, 255, 0));  // Yellow
  warnPainter.setPen(Qt::darkYellow);
  warnPainter.drawEllipse(1, 1, iconSize - 2, iconSize - 2);
  statusIcons[1] = QIcon(warnPixmap);

  // ERROR - Red circle
  QPixmap errorPixmap(iconSize, iconSize);
  errorPixmap.fill(Qt::transparent);
  QPainter errorPainter(&errorPixmap);
  errorPainter.setRenderHint(QPainter::Antialiasing);
  errorPainter.setBrush(QColor(255, 0, 0));  // Red
  errorPainter.setPen(Qt::darkRed);
  errorPainter.drawEllipse(1, 1, iconSize - 2, iconSize - 2);
  statusIcons[2] = QIcon(errorPixmap);

  // STALE - Gray circle
  QPixmap stalePixmap(iconSize, iconSize);
  stalePixmap.fill(Qt::transparent);
  QPainter stalePainter(&stalePixmap);
  stalePainter.setRenderHint(QPainter::Antialiasing);
  stalePainter.setBrush(QColor(128, 128, 128));  // Gray
  stalePainter.setPen(Qt::darkGray);
  stalePainter.drawEllipse(1, 1, iconSize - 2, iconSize - 2);
  statusIcons[3] = QIcon(stalePixmap);

  // UNKNOWN - White circle with question mark
  QPixmap unknownPixmap(iconSize, iconSize);
  unknownPixmap.fill(Qt::transparent);
  QPainter unknownPainter(&unknownPixmap);
  unknownPainter.setRenderHint(QPainter::Antialiasing);
  unknownPainter.setBrush(QColor(255, 255, 255));  // White
  unknownPainter.setPen(Qt::gray);
  unknownPainter.drawEllipse(1, 1, iconSize - 2, iconSize - 2);
  unknownPainter.setPen(Qt::black);
  unknownPainter.setFont(QFont("Arial", 10, QFont::Bold));
  unknownPainter.drawText(QRect(0, 0, iconSize, iconSize), Qt::AlignCenter, "?");
  statusIcons[4] = QIcon(unknownPixmap);
}

void DiagnosticsPanel::onInitialize()
{
  // Set the model for the tree view
  uiPanel->diagStackView->setModel(model);

  // Set up the tree view
  uiPanel->diagStackView->setAlternatingRowColors(true);

  // Configure the header if using a tree view
  if (auto treeView = qobject_cast<QTreeView *>(uiPanel->diagStackView)) {
    // Make columns resizable by the user (Interactive mode)
    treeView->header()->setSectionResizeMode(QHeaderView::Interactive);

    // Stretch the last section (message) to fill available space
    treeView->header()->setStretchLastSection(true);

    // Set reasonable default widths for columns
    treeView->setColumnWidth(0, 250);  // Name column width
    // Message column will stretch

    // Enable item decoration (for icons)
    treeView->setIconSize(QSize(16, 16));
  }

  RVIZ_COMMON_LOG_INFO("DiagnosticsPanel: Initialized");
}

void DiagnosticsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  RVIZ_COMMON_LOG_INFO("DiagnosticsPanel: Loaded parent panel config");

  // create our value containers
  QString * str = new QString();

  // load the namespace param
  if (config.mapGetString("robot_namespace", str)) {
    robot_ns = str->toStdString();
  } else {
    // default value
    robot_ns = "/mercury";
    RVIZ_COMMON_LOG_WARNING("DiagnosticsPanel: Loading default value for 'namespace'");
  }

  // get our local rosnode
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Subscribe to diagnostic aggregated topic
  diagSub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", rclcpp::SystemDefaultsQoS(),
    std::bind(&DiagnosticsPanel::diagCb, this, _1));

  delete str;

  // refresh the UI
  refresh();
}

void DiagnosticsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  // write our config values
  config.mapSetValue("robot_namespace", QString::fromStdString(robot_ns));
}

bool DiagnosticsPanel::event(QEvent * event) { return rviz_common::Panel::event(event); }

DiagnosticsPanel::~DiagnosticsPanel()
{
  // master window control removal
  delete uiPanel;
  diagSub.reset();
}

void DiagnosticsPanel::refresh()
{
  // Clean the model and wait for new diagnostics
  model->clear();
  model->setColumnCount(2);
  model->setHorizontalHeaderLabels({"Name", "Message"});

  // Reset our timer tick counter
  timerTick = 0;

  RVIZ_COMMON_LOG_INFO("DiagnosticsPanel: Refreshed panel");
}

void DiagnosticsPanel::waitForRefresh()
{
  // This is a placeholder for any waiting logic
  // Not needed for diagnostics which come in as messages
}

void DiagnosticsPanel::diagCb(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  // Check if UI and model are available
  auto treeView = qobject_cast<QTreeView *>(uiPanel->diagStackView);
  if (!treeView || !model) {
    RVIZ_COMMON_LOG_WARNING("DiagnosticsPanel: UI components not ready yet");
    return;
  }

  try {
    // Only rebuild the tree if we don't have one yet or if we're explicitly refreshing
    if (timerTick == 0) {
      RVIZ_COMMON_LOG_INFO("DiagnosticsPanel: Building initial tree structure");

      // Clear the model before building
      model->clear();
      model->setColumnCount(2);
      model->setHorizontalHeaderLabels({"Name", "Message"});

      // Create a map to store items by hierarchy
      std::map<std::string, QStandardItem *> parentItems;

      // Get the root item
      QStandardItem * rootItem = model->invisibleRootItem();

      // First pass: build the tree structure
      for (const auto & status : msg.status) {
        std::string fullName = status.name;

        // Skip empty names
        if (fullName.empty()) {
          continue;
        }

        // Split the path into components
        std::vector<std::string> pathComponents;
        std::string::size_type startPos = 0;
        std::string::size_type pos = 0;

        // Handle leading slash if present
        if (fullName[0] == '/') {
          startPos = 1;
        }

        // Split by '/'
        while ((pos = fullName.find('/', startPos)) != std::string::npos) {
          pathComponents.push_back(fullName.substr(startPos, pos - startPos));
          startPos = pos + 1;
        }

        // Add final component (leaf name)
        if (startPos < fullName.length()) {
          pathComponents.push_back(fullName.substr(startPos));
        }

        // Skip if no components
        if (pathComponents.empty()) {
          continue;
        }

        // Build path
        QStandardItem * parentItem = rootItem;
        std::string currentPath = "";

        // Create hierarchy
        for (size_t i = 0; i < pathComponents.size(); ++i) {
          std::string component = pathComponents[i];

          // Update the current path
          if (currentPath.empty()) {
            currentPath = component;
          } else {
            currentPath += "/" + component;
          }

          // Set the storage path (includes leading slash if original had it)
          std::string storagePath;
          if (fullName[0] == '/' && currentPath[0] != '/') {
            storagePath = "/" + currentPath;
          } else {
            storagePath = currentPath;
          }

          // Check if we already have this path in our map
          if (parentItems.find(storagePath) == parentItems.end()) {
            // Create a new item for this path component
            QStandardItem * newItem = new QStandardItem(QString::fromStdString(component));
            QStandardItem * messageItem = new QStandardItem("");

            // Store the full path for future reference
            newItem->setData(QString::fromStdString(storagePath), Qt::UserRole);

            // Add both columns
            QList<QStandardItem *> rowItems;
            rowItems.append(newItem);
            rowItems.append(messageItem);
            parentItem->appendRow(rowItems);

            // Store in map and update parent
            parentItems[storagePath] = newItem;
            parentItem = newItem;
          } else {
            // Path exists, just update parent
            parentItem = parentItems[storagePath];
          }
        }
      }

      // Expand all after creating the structure
      treeView->expandAll();

      // Mark that we've completed the tree structure
      timerTick++;
      RVIZ_COMMON_LOG_INFO("DiagnosticsPanel: Initial tree structure complete");
    }

    // Now update the status icon and message for each item
    for (const auto & status : msg.status) {
      std::string statusName = status.name;

      // Skip empty names
      if (statusName.empty()) {
        continue;
      }

      // Find the item matching this status
      QStandardItem * matchingItem = nullptr;

      // Try to find by stored path in UserRole
      QList<QStandardItem *> matches;
      findMatchingItems(model->invisibleRootItem(), QString::fromStdString(statusName), matches);

      if (!matches.isEmpty()) {
        matchingItem = matches.first();
      } else {
        // If no matching item was found and we're past the initial build,
        // this could be a new item that wasn't present during the initial tree build
        if (timerTick > 0) {
          // For simplicity, just rebuild the entire tree
          // In a production environment, you might want to add new nodes to the existing tree instead
          RVIZ_COMMON_LOG_INFO("DiagnosticsPanel: New diagnostic item found, refreshing tree");
          timerTick = 0;  // Reset to trigger a rebuild on next callback
          return;         // Skip this update, we'll handle it on the next callback
        }

        // Log that we skipped an item
        RVIZ_COMMON_LOG_WARNING("DiagnosticsPanel: Couldn't find item for status: " + statusName);
        continue;
      }

      // Get indices
      QModelIndex idx = matchingItem->index();
      QModelIndex messageIdx = idx.sibling(idx.row(), 1);

      if (!idx.isValid() || !messageIdx.isValid()) {
        RVIZ_COMMON_LOG_WARNING("DiagnosticsPanel: Invalid model index for: " + statusName);
        continue;
      }

      // Update name column with status icon and color
      QIcon icon;
      QColor textColor;
      QBrush backgroundBrush;

      switch (status.level) {
        case 0:  // OK
          icon = statusIcons[0];
          textColor = QColor(0, 0, 0);  // Black text
          backgroundBrush = QBrush();   // Default background
          break;
        case 1:  // WARN
          icon = statusIcons[1];
          textColor = QColor(0, 0, 0);                      // Black text
          backgroundBrush = QBrush(QColor(255, 255, 200));  // Light yellow
          break;
        case 2:  // ERROR
          icon = statusIcons[2];
          textColor = QColor(0, 0, 0);                      // Black text
          backgroundBrush = QBrush(QColor(255, 200, 200));  // Light red
          break;
        case 3:  // STALE
          icon = statusIcons[3];
          textColor = QColor(0, 0, 0);                      // Black text
          backgroundBrush = QBrush(QColor(230, 230, 230));  // Light gray
          break;
        default:
          icon = statusIcons[4];
          textColor = QColor(0, 0, 0);  // Black text
          backgroundBrush = QBrush();   // Default background
      }

      model->setData(idx, icon, Qt::DecorationRole);
      model->setData(idx, textColor, Qt::ForegroundRole);
      model->setData(idx, backgroundBrush, Qt::BackgroundRole);

      // Add status text to tooltip
      QString statusText;
      switch (status.level) {
        case 0:
          statusText = "OK";
          break;
        case 1:
          statusText = "WARN";
          break;
        case 2:
          statusText = "ERROR";
          break;
        case 3:
          statusText = "STALE";
          break;
        default:
          statusText = "UNKNOWN";
      }

      QString tooltipText = QString("Status: %1\nMessage: %2")
                              .arg(statusText)
                              .arg(QString::fromStdString(status.message));

      // Add key-value pairs to the tooltip if they exist
      if (!status.values.empty()) {
        tooltipText += "\n\nDetails:";
        for (const auto & kv : status.values) {
          tooltipText += QString("\n%1: %2")
                           .arg(QString::fromStdString(kv.key))
                           .arg(QString::fromStdString(kv.value));
        }
      }

      model->setData(idx, tooltipText, Qt::ToolTipRole);

      // Update message column
      model->setData(messageIdx, QString::fromStdString(status.message));
    }
  } catch (const std::exception & e) {
    RVIZ_COMMON_LOG_ERROR("DiagnosticsPanel: Exception occurred: " + std::string(e.what()));
    // Reset the panel to recover from error state
    refresh();
  }
}

void DiagnosticsPanel::findMatchingItems(
  QStandardItem * item, const QString & path, QList<QStandardItem *> & results)
{
  // Safety check for null item
  if (!item) {
    return;
  }

  // Check if this item matches
  if (item->data(Qt::UserRole).toString() == path) {
    results.append(item);
    return;
  }

  // If no match, check all children
  for (int i = 0; i < item->rowCount(); ++i) {
    QStandardItem * childItem = item->child(i);
    if (childItem) {
      findMatchingItems(childItem, path, results);
    }
  }
}

}  // namespace mercury_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mercury_rviz::DiagnosticsPanel, rviz_common::Panel);
