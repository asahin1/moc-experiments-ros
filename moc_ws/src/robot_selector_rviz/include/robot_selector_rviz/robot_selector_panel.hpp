#ifndef ROBOT_SELECTOR_PANEL_HPP
#define ROBOT_SELECTOR_PANEL_HPP

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_selector_rviz {

// Define a structure to hold the entire YAML
// config in memory
struct RobotConfig {
  std::map<std::string, std::vector<std::string>> first_end;
  std::map<std::string, std::vector<std::string>> last_end;
};

class RobotSelectorPanel : public rviz_common::Panel {
  // Q_OBJECT is required for Qt's signal/slot mechanism
  Q_OBJECT

public:
  RobotSelectorPanel(QWidget *parent = 0);

  // Overridden methods from rviz_common::panel::Panel
  virtual void onInitialize() override;
  virtual void load(const rviz_common::Config &config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  /**
   * @brief Slot called when the 'Load YAML' button is clicked.
   */
  void handleLoadFileButtonClicked();

  /**
   * @brief Slot called when the selection in the Robot dropdown changes (to
   * publish the choice).
   * @param index The index of the newly selected item.
   */
  void publishSelection(int index);

  /**
   * @brief Slot called when the Setup Type dropdown (Real/Sim) changes.
   * @param setup_type The string name of the newly selected setup (e.g.,
   * "real", "simulation").
   */
  void handleSetupTypeChanged(const QString &setup_type);

private:
  /**
   * @brief Sets up the panel's layout and UI elements.
   */
  void setupUi();

  /**
   * @brief Core function to parse the entire YAML file and store all data into
   * all_robot_setups_.
   * @param file_path The path to the YAML file.
   */
  void parseAndStoreYaml(const std::string &file_path);

  /**
   * @brief Populates the robot dropdown based on the selected setup type
   * (real/simulation).
   * @param setup_type The key ("real" or "simulation") to use from the stored
   * data.
   */
  void populateRobotDropdown(const std::string &setup_type);

  // --- UI Elements ---
  QComboBox *setupTypeDropdown_; // For selecting "real" or "simulation"
  QComboBox *robotDropdown_;     // For selecting the actual robot name
  QPushButton *loadFileButton_;
  QLabel *statusLabel_;

  // --- ROS 2 Communication ---
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // --- Stored Data ---
  // Key: setup name (e.g., "real"), Value: RobotConfig structure
  std::map<std::string, RobotConfig> all_robot_setups_;

  // --- Panel State Data (Saved/Loaded with .rviz) ---
  std::string topic_name_ = "/selected_robot"; // Fixed topic name
  std::string yaml_file_path_;                 // Path to the config file
  std::string
      current_setup_type_; // The currently selected setup (e.g., "real")
};
} // namespace robot_selector_rviz

#endif