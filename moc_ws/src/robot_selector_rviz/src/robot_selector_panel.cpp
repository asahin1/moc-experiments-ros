#include "robot_selector_rviz/robot_selector_panel.hpp"

// Includes for YAML parsing and file dialogs
#include <QDateTime>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <yaml-cpp/yaml.h>

// Includes for rviz configuration
#include <rviz_common/config.hpp>

// Use a unique node name based on the current time to avoid conflicts
static const std::string NODE_NAME_PREFIX = "robot_selector_rviz_panel_";

namespace robot_selector_rviz {

RobotSelectorPanel::RobotSelectorPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();

  // Connect new dropdown signal
  connect(setupTypeDropdown_, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(handleSetupTypeChanged(const QString &)));

  // Connect existing signals
  connect(loadFileButton_, SIGNAL(clicked()), this,
          SLOT(handleLoadFileButtonClicked()));
  connect(robotDropdown_, SIGNAL(currentIndexChanged(int)), this,
          SLOT(publishSelection(int)));
}

void RobotSelectorPanel::setupUi() {
  // Setup Main Layout (Vertical)
  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  // 1. Topic Label and Name (optional, but good practice)
  QLabel *topicLabel =
      new QLabel("Topic: /selected_robot (std_msgs::msg::String)");
  topicLabel->setFont(QFont("Arial", 8));
  mainLayout->addWidget(topicLabel);

  // 2. File Loading Section (Horizontal Layout)
  QHBoxLayout *fileLayout = new QHBoxLayout();
  loadFileButton_ = new QPushButton("Load YAML File...");
  fileLayout->addWidget(loadFileButton_);

  statusLabel_ = new QLabel("No file loaded.");
  statusLabel_->setStyleSheet("color: gray;");
  fileLayout->addWidget(statusLabel_);
  fileLayout->setStretch(1, 1); // Status label takes up more space

  mainLayout->addLayout(fileLayout);
  mainLayout->addWidget(new QFrame); // Simple separator line

  // 2. Setup Type Dropdown
  mainLayout->addWidget(new QLabel("Select Setup Type:"));
  setupTypeDropdown_ = new QComboBox();
  setupTypeDropdown_->addItem("--- Select Setup ---");
  setupTypeDropdown_->setEnabled(false);
  mainLayout->addWidget(setupTypeDropdown_);

  // 3. Robot Selection Dropdown (formerly dropdown_)
  mainLayout->addWidget(new QLabel("Select Robot:"));
  robotDropdown_ = new QComboBox();
  robotDropdown_->addItem("--- Load file first ---");
  robotDropdown_->setEnabled(false);
  mainLayout->addWidget(robotDropdown_);

  // 4. Spacer for clean look
  mainLayout->addStretch(1);

  // Set the layout for the panel
  setLayout(mainLayout);
}

void RobotSelectorPanel::onInitialize() {
  // The node is created using a unique name
  node_ = std::make_shared<rclcpp::Node>(
      NODE_NAME_PREFIX + std::to_string(QDateTime::currentMSecsSinceEpoch()));

  // Create the publisher
  publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name_, 10);

  RCLCPP_INFO(node_->get_logger(),
              "Panel initialized. Publisher created on topic: %s",
              topic_name_.c_str());
}

// --- Configuration Saving/Loading ---

void RobotSelectorPanel::save(rviz_common::Config config) const {
  // Call the base class save method first
  rviz_common::Panel::save(config);

  // Save state variables
  config.mapSetValue("YamlFilePath", QString::fromStdString(yaml_file_path_));
  config.mapSetValue("CurrentSetupType",
                     QString::fromStdString(current_setup_type_));
  config.mapSetValue("CurrentRobotIndex", robotDropdown_->currentIndex());

  RCLCPP_INFO(node_->get_logger(), "Saved config. Path: %s, Setup: %s",
              yaml_file_path_.c_str(), current_setup_type_.c_str());
}

void RobotSelectorPanel::load(const rviz_common::Config &config) {
  // Call the base class load method first
  rviz_common::Panel::load(config);

  QString path_qstring;
  QString setup_qstring;
  int saved_robot_index = -1;

  // 1. Retrieve the saved file path
  if (config.mapGetString("YamlFilePath", &path_qstring)) {
    yaml_file_path_ = path_qstring.toStdString();

    if (!yaml_file_path_.empty()) {
      RCLCPP_INFO(node_->get_logger(),
                  "Found saved config. Attempting to load: %s",
                  yaml_file_path_.c_str());

      // 2. Load the entire config first, but don't try to publish yet
      parseAndStoreYaml(yaml_file_path_);

      // 3. Restore the saved setup type
      if (config.mapGetString("CurrentSetupType", &setup_qstring)) {
        current_setup_type_ = setup_qstring.toStdString();
        // Find and set the selected text on the dropdown. This will trigger
        // handleSetupTypeChanged.
        setupTypeDropdown_->setCurrentText(setup_qstring);
      }

      // 4. Restore the saved robot index
      config.mapGetInt("CurrentRobotIndex", &saved_robot_index);

      if (saved_robot_index >= 0 &&
          saved_robot_index < robotDropdown_->count()) {
        // Set the robot index. This will trigger publishSelection.
        robotDropdown_->setCurrentIndex(saved_robot_index);
      }
    }
  }
}
// --- File Handling ---

void RobotSelectorPanel::handleLoadFileButtonClicked() {
  QString default_dir =
      QStandardPaths::writableLocation(QStandardPaths::HomeLocation);

  QString file_path = QFileDialog::getOpenFileName(
      this, tr("Open Robot Configuration YAML File"), default_dir,
      tr("YAML Files (*.yaml *.yml)"));

  if (file_path.isEmpty())
    return;

  // Update the state variable and call the core loading function
  yaml_file_path_ = file_path.toStdString();
  parseAndStoreYaml(yaml_file_path_);
}

void RobotSelectorPanel::parseAndStoreYaml(const std::string &file_path) {
  // Clear previous data
  all_robot_setups_.clear();
  setupTypeDropdown_->clear();
  robotDropdown_->clear();

  try {
    YAML::Node config = YAML::LoadFile(file_path);
    const std::string setups_key = "setups";

    if (config[setups_key] && config[setups_key].IsMap()) {

      // Loop through each setup (e.g., "real", "simulation")
      for (YAML::const_iterator setup_it = config[setups_key].begin();
           setup_it != config[setups_key].end(); ++setup_it) {

        std::string setup_name = setup_it->first.as<std::string>();
        const YAML::Node &setup_data = setup_it->second;

        // Check for the required 'team1' and 'team2' keys
        if (setup_data["first_end_robot_names"] &&
            setup_data["last_end_robot_names"] &&
            setup_data["first_end_robot_names"].IsSequence() &&
            setup_data["last_end_robot_names"].IsSequence()) {
          RobotConfig rc;

          // Process Team 1 (list of robot names)
          for (const auto &robot_node : setup_data["first_end_robot_names"]) {
            if (robot_node.IsScalar()) {
              rc.first_end["robots"].push_back(robot_node.as<std::string>());
            }
          }

          // Process Team 2 (list of robot names)
          for (const auto &robot_node : setup_data["last_end_robot_names"]) {
            if (robot_node.IsScalar()) {
              rc.last_end["robots"].push_back(robot_node.as<std::string>());
            }
          }

          // Store the config and add to the setup dropdown
          all_robot_setups_[setup_name] = rc;
          setupTypeDropdown_->addItem(QString::fromStdString(setup_name));
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "Setup '%s' missing required 'first_end_robot_names' or "
                      "'last_end_robot_names' list.",
                      setup_name.c_str());
        }
      }

      if (setupTypeDropdown_->count() > 0) {
        setupTypeDropdown_->setEnabled(true);
        statusLabel_->setText(QString("Loaded **%1** setup types.")
                                  .arg(setupTypeDropdown_->count()));
        statusLabel_->setStyleSheet("color: green; font-weight: bold;");

        // If a setup was not previously saved, manually trigger the change for
        // the first item
        if (current_setup_type_.empty() ||
            setupTypeDropdown_->findText(
                QString::fromStdString(current_setup_type_)) == -1) {
          handleSetupTypeChanged(setupTypeDropdown_->currentText());
        }
      } else {
        setupTypeDropdown_->addItem("--- No valid setups found ---");
        statusLabel_->setText("File loaded, but no valid 'setups' found.");
        statusLabel_->setStyleSheet("color: orange;");
      }
    } else {
      statusLabel_->setText("YAML format error: Missing 'setups' map.");
      statusLabel_->setStyleSheet("color: red;");
      RCLCPP_ERROR(node_->get_logger(),
                   "YAML file %s format error: Missing 'setups' map.",
                   file_path.c_str());
    }

  } catch (const YAML::Exception &e) {
    statusLabel_->setText("Error parsing file.");
    statusLabel_->setStyleSheet("color: red;");
    RCLCPP_ERROR(node_->get_logger(), "Failed to load/parse YAML file %s: %s",
                 file_path.c_str(), e.what());
  }
}

// --- Dropdown Change Handlers ---

void RobotSelectorPanel::handleSetupTypeChanged(
    const QString &setup_type_qstr) {
  std::string setup_type = setup_type_qstr.toStdString();
  current_setup_type_ = setup_type; // Update state variable for saving

  if (all_robot_setups_.count(setup_type) == 0) {
    robotDropdown_->clear();
    robotDropdown_->addItem("--- Invalid Setup ---");
    robotDropdown_->setEnabled(false);
    return;
  }

  // Call the function to populate the second dropdown
  populateRobotDropdown(setup_type);
}

void RobotSelectorPanel::populateRobotDropdown(const std::string &setup_type) {
  robotDropdown_->clear();

  // Check if the setup type exists in the stored map
  if (all_robot_setups_.find(setup_type) == all_robot_setups_.end()) {
    robotDropdown_->addItem("Error: Setup not found.");
    robotDropdown_->setEnabled(false);
    return;
  }

  const RobotConfig &rc = all_robot_setups_.at(setup_type);
  bool added_any = false;

  // 1. Add Team 1 robots
  if (rc.first_end.count("robots") && !rc.first_end.at("robots").empty()) {
    robotDropdown_->addItem("--- First End Robots ---");
    for (const std::string &robot_name : rc.first_end.at("robots")) {
      robotDropdown_->addItem(QString::fromStdString(robot_name));
    }
    added_any = true;
  }

  // 2. Add Team 2 robots
  if (rc.last_end.count("robots") && !rc.last_end.at("robots").empty()) {
    robotDropdown_->addItem("--- Last End Robots ---");
    for (const std::string &robot_name : rc.last_end.at("robots")) {
      robotDropdown_->addItem(QString::fromStdString(robot_name));
    }
    added_any = true;
  }

  if (added_any) {
    robotDropdown_->setEnabled(true);
    // Trigger publish for the current item
    publishSelection(robotDropdown_->currentIndex());
  } else {
    robotDropdown_->addItem("--- No Robots Found for this Setup ---");
    robotDropdown_->setEnabled(false);
  }
}

// --- ROS 2 Publishing ---

void RobotSelectorPanel::publishSelection(int index) {
  // Ignore if no item is selected, or if the selection is a separator (e.g.,
  // "--- Team 1 ---")
  if (index < 0 || !robotDropdown_->isEnabled())
    return;

  QString selected_text = robotDropdown_->currentText();
  if (selected_text.startsWith("---"))
    return; // Don't publish separators

  std_msgs::msg::String msg;
  msg.data = selected_text.toStdString();

  // Publish the message
  publisher_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "Published selection: '%s' (Setup: %s)",
              msg.data.c_str(), current_setup_type_.c_str());
}
} // namespace robot_selector_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_selector_rviz::RobotSelectorPanel,
                       rviz_common::Panel)