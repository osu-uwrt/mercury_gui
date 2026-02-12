#ifndef RIPTIDE_RVIZ_DIAGNOSTICS_PANEL_HPP
#define RIPTIDE_RVIZ_DIAGNOSTICS_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <QStandardItemModel>
#include <QTimer>
#include <QIcon>
#include <QPainter>
#include <QMap>

// Forward declaration of UI
namespace Ui {
class DiagnosticsPanel;
}

namespace riptide_rviz
{
    class DiagnosticsPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit DiagnosticsPanel(QWidget *parent = nullptr);
        ~DiagnosticsPanel() override;

        void onInitialize() override;
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        bool event(QEvent *event) override;

    private slots:
        void refresh();
        void waitForRefresh();

    private:
        Ui::DiagnosticsPanel *uiPanel;
        QStandardItemModel *model;
        std::string robot_ns;

        // Status icons
        QMap<int, QIcon> statusIcons;
        void createStatusIcons();
        
        // Helper method to find items by path
        void findMatchingItems(QStandardItem* item, const QString& path, QList<QStandardItem*>& results);

        // ROS Subscribers
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;

        // Timer for refresh operations
        int timerTick;

        // Callback for diagnostics messages
        void diagCb(const diagnostic_msgs::msg::DiagnosticArray & msg);
    };

} // namespace riptide_rviz

#endif // RIPTIDE_RVIZ_DIAGNOSTICS_PANEL_HPP