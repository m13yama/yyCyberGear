#include "yy_cybergear_app/main_window.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QStringList>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <algorithm>

#include "yy_cybergear/logging.hpp"

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent),
  m_centralWidget(nullptr),
  m_mainLayout(nullptr),
  m_cyberGear(nullptr),
  m_isConnected(false),
  m_motorEnabled(false)
{
  setupUI();
  updateConnectionStatus();
}

void MainWindow::setupUI()
{
  setWindowTitle("CyberGear Control Panel");
  setMinimumSize(800, 900);

  m_centralWidget = new QWidget;
  setCentralWidget(m_centralWidget);
  m_mainLayout = new QVBoxLayout(m_centralWidget);

  // Connection group
  m_connectionGroup = new QGroupBox("Connection");
  QGridLayout * connLayout = new QGridLayout(m_connectionGroup);

  connLayout->addWidget(new QLabel("Interface:"), 0, 0);
  m_interfaceEdit = new QLineEdit("can0");
  connLayout->addWidget(m_interfaceEdit, 0, 1);

  connLayout->addWidget(new QLabel("Host ID:"), 0, 2);
  m_hostIdSpin = new QSpinBox();
  m_hostIdSpin->setRange(1, 255);
  m_hostIdSpin->setValue(1);
  connLayout->addWidget(m_hostIdSpin, 0, 3);

  connLayout->addWidget(new QLabel("Motor ID:"), 1, 0);
  m_motorIdSpin = new QSpinBox();
  m_motorIdSpin->setRange(1, 255);
  m_motorIdSpin->setValue(1);
  connLayout->addWidget(m_motorIdSpin, 1, 1);

  m_connectBtn = new QPushButton("Connect");
  connect(m_connectBtn, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
  connLayout->addWidget(m_connectBtn, 1, 2);

  m_disconnectBtn = new QPushButton("Disconnect");
  connect(m_disconnectBtn, &QPushButton::clicked, this, &MainWindow::onDisconnectClicked);
  connLayout->addWidget(m_disconnectBtn, 1, 3);

  m_connectionStatus = new QLabel("Disconnected");
  connLayout->addWidget(m_connectionStatus, 2, 0, 1, 4);

  m_mainLayout->addWidget(m_connectionGroup);

  // Control group
  m_controlGroup = new QGroupBox("Motor Control");
  QHBoxLayout * controlLayout = new QHBoxLayout(m_controlGroup);

  m_enableBtn = new QPushButton("Enable Motor");
  connect(m_enableBtn, &QPushButton::clicked, this, &MainWindow::onEnableMotorClicked);
  controlLayout->addWidget(m_enableBtn);

  m_stopBtn = new QPushButton("Stop Motor");
  connect(m_stopBtn, &QPushButton::clicked, this, &MainWindow::onStopMotorClicked);
  controlLayout->addWidget(m_stopBtn);

  m_clearFaultsBtn = new QPushButton("Clear Faults");
  connect(m_clearFaultsBtn, &QPushButton::clicked, this, &MainWindow::onClearFaultsClicked);
  controlLayout->addWidget(m_clearFaultsBtn);

  m_setZeroBtn = new QPushButton("Set Mechanical Zero");
  connect(m_setZeroBtn, &QPushButton::clicked, this, &MainWindow::onSetMechanicalZeroClicked);
  controlLayout->addWidget(m_setZeroBtn);

  m_getMcuIdBtn = new QPushButton("Get MCU ID");
  connect(m_getMcuIdBtn, &QPushButton::clicked, this, &MainWindow::onGetMcuIdClicked);
  controlLayout->addWidget(m_getMcuIdBtn);

  m_verboseCheck = new QCheckBox("Verbose CAN");
  m_verboseCheck->setChecked(true);
  controlLayout->addWidget(m_verboseCheck);

  m_mainLayout->addWidget(m_controlGroup);

  // Run Mode group
  m_runModeGroup = new QGroupBox("Control Mode");
  QGridLayout * modeLayout = new QGridLayout(m_runModeGroup);
  modeLayout->addWidget(new QLabel("Mode:"), 0, 0);
  m_runModeCombo = new QComboBox();
  m_runModeCombo->addItem(
    "Operation", static_cast<int>(yy_cybergear::CyberGear::RunMode::Operation));
  m_runModeCombo->addItem(
    "Position", static_cast<int>(yy_cybergear::CyberGear::RunMode::Position));
  m_runModeCombo->addItem("Speed", static_cast<int>(yy_cybergear::CyberGear::RunMode::Speed));
  m_runModeCombo->addItem("Current", static_cast<int>(yy_cybergear::CyberGear::RunMode::Current));
  modeLayout->addWidget(m_runModeCombo, 0, 1);

  m_applyRunModeBtn = new QPushButton("Apply");
  connect(m_applyRunModeBtn, &QPushButton::clicked, this, &MainWindow::onApplyRunModeClicked);
  modeLayout->addWidget(m_applyRunModeBtn, 0, 2);

  m_refreshRunModeBtn = new QPushButton("Refresh");
  connect(m_refreshRunModeBtn, &QPushButton::clicked, this, &MainWindow::onRefreshRunModeClicked);
  modeLayout->addWidget(m_refreshRunModeBtn, 0, 3);

  m_mainLayout->addWidget(m_runModeGroup);

  // Position group (Operation Control)
  m_operationGroup = new QGroupBox("Operation Control");
  QGridLayout * posLayout = new QGridLayout(m_operationGroup);

  posLayout->addWidget(new QLabel("Target position [rad] :"), 0, 0);
  m_opPosSpin = new QDoubleSpinBox();
  m_opPosSpin->setRange(-50.0, 50.0);
  m_opPosSpin->setDecimals(kDispDecimalsAngle);
  m_opPosSpin->setSingleStep(0.01);
  m_opPosSpin->setValue(0.0);
  connect(
    m_opPosSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &MainWindow::onOpPositionChanged);
  posLayout->addWidget(m_opPosSpin, 0, 1);

  posLayout->addWidget(new QLabel("Position Gain [Nm/rad]:"), 0, 2);
  m_kpSpin = new QDoubleSpinBox();
  m_kpSpin->setRange(0.0, 500.0);
  m_kpSpin->setDecimals(2);
  m_kpSpin->setSingleStep(1.0);
  m_kpSpin->setValue(10.0);
  posLayout->addWidget(m_kpSpin, 0, 3);

  posLayout->addWidget(new QLabel("Target vel [rad/s] :"), 1, 0);
  m_opVelSpin = new QDoubleSpinBox();
  m_opVelSpin->setRange(-50.0, 50.0);
  m_opVelSpin->setDecimals(kDispDecimalsVelocity);
  m_opVelSpin->setSingleStep(0.01);
  m_opVelSpin->setValue(0.0);
  connect(
    m_opVelSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &MainWindow::onOpVelChanged);
  posLayout->addWidget(m_opVelSpin, 1, 1);

  posLayout->addWidget(new QLabel("Velocity Gain [Nm/(rad/s)]:"), 1, 2);
  m_kdSpin = new QDoubleSpinBox();
  m_kdSpin->setRange(0.0, 5.0);
  m_kdSpin->setDecimals(2);
  m_kdSpin->setSingleStep(0.1);
  m_kdSpin->setValue(1.0);
  posLayout->addWidget(m_kdSpin, 1, 3);

  posLayout->addWidget(new QLabel("Offset tau [Nm] :"), 2, 0);
  m_opTauSpin = new QDoubleSpinBox();
  m_opTauSpin->setRange(-30.0, 30.0);
  m_opTauSpin->setDecimals(kDispDecimalsTorque);
  m_opTauSpin->setSingleStep(0.01);
  m_opTauSpin->setValue(0.0);
  connect(
    m_opTauSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &MainWindow::onOpTorqueChanged);
  posLayout->addWidget(m_opTauSpin, 2, 1);

  m_mainLayout->addWidget(m_operationGroup);

  // Position group (Position Control)
  m_positionGroup = new QGroupBox("Position Control");
  {
    QGridLayout * posCtlLayout = new QGridLayout(m_positionGroup);
    posCtlLayout->addWidget(new QLabel("Target position [rad] :"), 0, 0);
    m_posRefSpin = new QDoubleSpinBox();
    m_posRefSpin->setRange(-50.0, 50.0);
    m_posRefSpin->setDecimals(kDispDecimalsAngle);
    m_posRefSpin->setSingleStep(0.01);
    m_posRefSpin->setValue(0.0);
    connect(
      m_posRefSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      &MainWindow::onTargetPositionChanged);
    posCtlLayout->addWidget(m_posRefSpin, 0, 1);
  }
  m_mainLayout->addWidget(m_positionGroup);

  // Command group (Speed only)
  m_commandGroup = new QGroupBox("Speed Control");
  QGridLayout * cmdLayout = new QGridLayout(m_commandGroup);

  cmdLayout->addWidget(new QLabel("Target speed [rad/s]:"), 0, 0);
  m_speedSpin = new QDoubleSpinBox();
  m_speedSpin->setRange(-50.0, 50.0);
  m_speedSpin->setDecimals(kDispDecimalsVelocity);
  m_speedSpin->setSingleStep(0.1);
  m_speedSpin->setValue(2.0);
  connect(
    m_speedSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &MainWindow::onTargetSpeedChanged);
  cmdLayout->addWidget(m_speedSpin, 0, 1);

  // Run is controlled by Enable/Stop only

  m_mainLayout->addWidget(m_commandGroup);

  // Current Command group
  m_currentGroup = new QGroupBox("Current Control");
  QGridLayout * curLayout = new QGridLayout(m_currentGroup);

  curLayout->addWidget(new QLabel("Target current Iq [A]:"), 0, 0);
  m_iqSpin = new QDoubleSpinBox();
  m_iqSpin->setRange(-40.0, 40.0);
  m_iqSpin->setDecimals(kDispDecimalsTorque);
  m_iqSpin->setSingleStep(0.1);
  m_iqSpin->setValue(0.0);
  connect(
    m_iqSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &MainWindow::onTargetCurrentChanged);
  curLayout->addWidget(m_iqSpin, 0, 1);

  m_mainLayout->addWidget(m_currentGroup);

  // Status group
  m_statusGroup = new QGroupBox("Motor Status");
  QGridLayout * statusLayout = new QGridLayout(m_statusGroup);

  statusLayout->addWidget(new QLabel("Position [rad]:"), 0, 0);
  m_positionLabel = new QLabel("N/A");
  statusLayout->addWidget(m_positionLabel, 0, 1);

  statusLayout->addWidget(new QLabel("Velocity [rad/s]:"), 0, 2);
  m_velocityLabel = new QLabel("N/A");
  statusLayout->addWidget(m_velocityLabel, 0, 3);

  statusLayout->addWidget(new QLabel("Torque [Nm]:"), 1, 0);
  m_currentLabel = new QLabel("N/A");
  statusLayout->addWidget(m_currentLabel, 1, 1);

  statusLayout->addWidget(new QLabel("Temperature [°C]:"), 1, 2);
  m_temperatureLabel = new QLabel("N/A");
  statusLayout->addWidget(m_temperatureLabel, 1, 3);

  statusLayout->addWidget(new QLabel("Motor CAN ID:"), 2, 0);
  m_motorIdLabel = new QLabel("N/A");
  statusLayout->addWidget(m_motorIdLabel, 2, 1);

  statusLayout->addWidget(new QLabel("Mode:"), 2, 2);
  m_modeLabel = new QLabel("N/A");
  statusLayout->addWidget(m_modeLabel, 2, 3);

  statusLayout->addWidget(new QLabel("Faults:"), 3, 0);
  m_faultsLabel = new QLabel("N/A");
  statusLayout->addWidget(m_faultsLabel, 3, 1, 1, 3);

  m_mainLayout->addWidget(m_statusGroup);

  // Log display
  m_logEdit = new QTextEdit();
  m_logEdit->setMaximumHeight(150);
  m_logEdit->setReadOnly(true);
  m_mainLayout->addWidget(m_logEdit);

  logMessage("Application started");

  // Setup control+monitor timer (fixed interval)
  m_monitorTimer = new QTimer(this);
  connect(m_monitorTimer, &QTimer::timeout, this, &MainWindow::onTimerTick);
  m_monitorTimer->setInterval(10);  // 10 ms (100 Hz)
}

void MainWindow::updateStatusFrom(const yy_cybergear::Status & st)
{
  m_positionLabel->setText(QString::number(st.angle_rad, 'f', kDispDecimalsAngle));
  m_velocityLabel->setText(QString::number(st.vel_rad_s, 'f', kDispDecimalsVelocity));
  m_currentLabel->setText(QString::number(st.torque_Nm, 'f', kDispDecimalsTorque));
  m_temperatureLabel->setText(QString::number(st.temperature_c, 'f', kDispDecimalsTemperature));
  m_motorIdLabel->setText(QString::number(st.motor_can_id));
  m_modeLabel->setText(
    QString::fromStdString(yy_cybergear::logging::statusModeToString(st.status_mode)));
  const auto faults = yy_cybergear::logging::faultBitsToString(st.fault_bits);
  if (faults.empty()) {
    m_faultsLabel->setText("None");
  } else {
    QStringList list;
    for (const auto & f : faults) list << QString::fromStdString(f);
    m_faultsLabel->setText(list.join(", "));
  }
}

void MainWindow::resetStatusLabels()
{
  m_positionLabel->setText("N/A");
  m_velocityLabel->setText("N/A");
  m_currentLabel->setText("N/A");
  m_temperatureLabel->setText("N/A");
  m_motorIdLabel->setText("N/A");
  m_modeLabel->setText("N/A");
  m_faultsLabel->setText("N/A");
}

void MainWindow::onConnectClicked()
{
  try {
    QString interface = m_interfaceEdit->text();
    uint8_t hostId = static_cast<uint8_t>(m_hostIdSpin->value());
    uint8_t motorId = static_cast<uint8_t>(m_motorIdSpin->value());

    m_cyberGear = std::make_unique<yy_cybergear::CyberGear>(
      interface.toStdString(), hostId, motorId, m_verboseCheck->isChecked());

    m_cyberGear->open();
    m_isConnected = true;

    // Initial safe setup: clear faults and set speed mode
    (void)m_cyberGear->clearFaults();
    (void)m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Speed);

    logMessage(
      QString("Connected to %1 (Host: %2, Motor: %3)").arg(interface).arg(hostId).arg(motorId));

  } catch (const std::exception & e) {
    QMessageBox::critical(
      this, "Connection Error", QString("Failed to connect: %1").arg(e.what()));
    logMessage(QString("Connection failed: %1").arg(e.what()));
    m_isConnected = false;
  }

  updateConnectionStatus();
}

void MainWindow::onDisconnectClicked()
{
  if (m_cyberGear) {
    m_cyberGear->close();
    m_cyberGear.reset();
  }

  m_isConnected = false;
  m_motorEnabled = false;
  updateConnectionStatus();
  logMessage("Disconnected");
  resetStatusLabels();
}

void MainWindow::onEnableMotorClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->enableMotor();
    if (result.ok()) {
      m_motorEnabled = true;
      logMessage("Motor enabled successfully");
      // Auto-start according to selected RunMode
      try {
        const auto selectedMode =
          static_cast<yy_cybergear::CyberGear::RunMode>(m_runModeCombo->currentData().toInt());
        (void)m_cyberGear->setRunMode(selectedMode);
        if (selectedMode == yy_cybergear::CyberGear::RunMode::Operation) {
          m_mode = ControlMode::Operation;
          m_running = true;
          m_monitorTimer->start();
          logMessage("Run started (Operation)");
        } else if (selectedMode == yy_cybergear::CyberGear::RunMode::Position) {
          m_mode = ControlMode::Position;
          m_running = true;
          m_monitorTimer->start();
          logMessage("Run started (Position)");
        } else if (selectedMode == yy_cybergear::CyberGear::RunMode::Speed) {
          m_mode = ControlMode::Speed;
          m_running = true;
          m_monitorTimer->start();
          logMessage("Run started (Speed)");
        } else if (selectedMode == yy_cybergear::CyberGear::RunMode::Current) {
          m_mode = ControlMode::Current;
          m_running = true;
          m_monitorTimer->start();
          logMessage("Run started (Current)");
        } else {
          m_mode = ControlMode::None;
          m_running = false;
          logMessage("RunMode not auto-looping (set Speed or Operation for loop)");
        }
      } catch (const std::exception & e) {
        logMessage(QString("Start loop error: %1").arg(e.what()));
      }
    } else {
      logMessage("Failed to enable motor");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Enable motor error: %1").arg(e.what()));
  }
}

void MainWindow::onStopMotorClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->stopMotor();
    if (result.ok()) {
      m_motorEnabled = false;
      m_running = false;
      m_mode = ControlMode::None;
      m_monitorTimer->stop();
      // Update RunMode combo to Operation
      for (int i = 0; i < m_runModeCombo->count(); ++i) {
        if (
          m_runModeCombo->itemData(i).toInt() ==
          static_cast<int>(yy_cybergear::CyberGear::RunMode::Operation)) {
          m_runModeCombo->setCurrentIndex(i);
          break;
        }
      }
      logMessage("Motor stopped successfully");
      updateStatusDisplay();
    } else {
      logMessage("Failed to stop motor");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Stop motor error: %1").arg(e.what()));
  }
}

void MainWindow::onClearFaultsClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->clearFaults(200);
    if (result.ok()) {
      logMessage("Faults cleared successfully");
      updateStatusDisplay();
    } else {
      logMessage("Failed to clear faults");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Clear faults error: %1").arg(e.what()));
  }
}

void MainWindow::onSetMechanicalZeroClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->setMechanicalZero();
    if (result.ok()) {
      logMessage("Mechanical zero set successfully");
    } else {
      logMessage("Failed to set mechanical zero");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Set mechanical zero error: %1").arg(e.what()));
  }
}

void MainWindow::onGetMcuIdClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->getMcuId(2000);
    if (result.ok()) {
      const auto & uid = result.value().value();
      QString uidStr;
      for (const auto & byte : uid) {
        uidStr += QString("%1 ").arg(byte, 2, 16, QChar('0')).toUpper();
      }
      logMessage(QString("MCU ID: %1").arg(uidStr.trimmed()));
    } else {
      logMessage("Failed to get MCU ID");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Get MCU ID error: %1").arg(e.what()));
  }
}

void MainWindow::onTargetSpeedChanged(double value)
{
  // If connected and not currently running the loop, apply the new target immediately
  if (!m_cyberGear || !m_isConnected || m_running) return;
  try {
    auto modeResult = m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Speed);
    if (!modeResult.ok()) {
      logMessage("Failed to set speed mode");
      return;
    }
    auto r = m_cyberGear->setSpeedReference(static_cast<float>(value));
    if (r.ok() && r.value().has_value()) {
      updateStatusFrom(*r.value());
    } else if (!r.ok()) {
      logMessage("Failed to set speed");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply speed error: %1").arg(e.what()));
  }
}

void MainWindow::onTargetPositionChanged(double value)
{
  // If connected and not currently running the loop, apply the new target immediately
  if (!m_cyberGear || !m_isConnected || m_running) return;
  try {
    auto modeResult = m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Position);
    if (!modeResult.ok()) {
      logMessage("Failed to set position mode");
      return;
    }
    auto r = m_cyberGear->setPositionReference(static_cast<float>(value));
    if (r.ok() && r.value().has_value()) {
      updateStatusFrom(*r.value());
    } else if (!r.ok()) {
      logMessage("Failed to set position");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply position ref error: %1").arg(e.what()));
  }
}

void MainWindow::onTargetCurrentChanged(double value)
{
  // If connected and not currently running the loop, apply the new target immediately
  if (!m_cyberGear || !m_isConnected || m_running) return;
  try {
    auto modeResult = m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Current);
    if (!modeResult.ok()) {
      logMessage("Failed to set current mode");
      return;
    }
    auto r = m_cyberGear->setIqReference(static_cast<float>(value));
    if (r.ok() && r.value().has_value()) {
      updateStatusFrom(*r.value());
    } else if (!r.ok()) {
      logMessage("Failed to set current");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply current error: %1").arg(e.what()));
  }
}

void MainWindow::onOpPositionChanged(double value)
{
  // Apply immediately if connected and not running (one-shot op command with zero torque)
  if (!m_cyberGear || !m_isConnected || m_running) return;
  try {
    auto modeResult = m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Operation);
    if (!modeResult.ok()) {
      logMessage("Failed to set operation mode");
      return;
    }
    yy_cybergear::OpCommand cmd{};
    cmd.pos_rad = static_cast<float>(value);
    cmd.vel_rad_s = static_cast<float>(m_opVelSpin ? m_opVelSpin->value() : 0.0);
    cmd.kp = static_cast<float>(m_kpSpin ? m_kpSpin->value() : 0.0);
    cmd.kd = static_cast<float>(m_kdSpin ? m_kdSpin->value() : 0.0);
    cmd.torque_Nm = static_cast<float>(m_opTauSpin ? m_opTauSpin->value() : 0.0);
    auto r = m_cyberGear->sendOperationCommand(cmd, 50);
    if (r.ok() && r.value().has_value()) {
      updateStatusFrom(*r.value());
    } else if (!r.ok()) {
      logMessage("Failed to send op command");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply position error: %1").arg(e.what()));
  }
}

void MainWindow::onOpVelChanged(double)
{
  // Apply immediately if connected and not running
  if (!m_cyberGear || !m_isConnected || m_running) return;
  try {
    auto modeResult = m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Operation);
    if (!modeResult.ok()) {
      logMessage("Failed to set operation mode");
      return;
    }
    yy_cybergear::OpCommand cmd{};
    cmd.pos_rad = static_cast<float>(m_opPosSpin->value());
    cmd.vel_rad_s = static_cast<float>(m_opVelSpin->value());
    cmd.kp = static_cast<float>(m_kpSpin->value());
    cmd.kd = static_cast<float>(m_kdSpin->value());
    cmd.torque_Nm = static_cast<float>(m_opTauSpin->value());
    (void)m_cyberGear->sendOperationCommand(cmd, 50);
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply op vel error: %1").arg(e.what()));
  }
}

void MainWindow::onOpTorqueChanged(double)
{
  // Apply immediately if connected and not running
  if (!m_cyberGear || !m_isConnected || m_running) return;
  try {
    auto modeResult = m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Operation);
    if (!modeResult.ok()) {
      logMessage("Failed to set operation mode");
      return;
    }
    yy_cybergear::OpCommand cmd{};
    cmd.pos_rad = static_cast<float>(m_opPosSpin->value());
    cmd.vel_rad_s = static_cast<float>(m_opVelSpin->value());
    cmd.kp = static_cast<float>(m_kpSpin->value());
    cmd.kd = static_cast<float>(m_kdSpin->value());
    cmd.torque_Nm = static_cast<float>(m_opTauSpin->value());
    (void)m_cyberGear->sendOperationCommand(cmd, 50);
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply op torque error: %1").arg(e.what()));
  }
}

void MainWindow::updateConnectionStatus()
{
  bool connected = m_isConnected && m_cyberGear && m_cyberGear->isOpen();

  m_connectionStatus->setText(connected ? "Connected" : "Disconnected");
  m_connectionStatus->setStyleSheet(connected ? "color: green;" : "color: red;");

  m_connectBtn->setEnabled(!connected);
  m_disconnectBtn->setEnabled(connected);
  m_interfaceEdit->setEnabled(!connected);
  m_hostIdSpin->setEnabled(!connected);
  m_motorIdSpin->setEnabled(!connected);

  m_operationGroup->setEnabled(connected);
  m_positionGroup->setEnabled(connected);
  m_commandGroup->setEnabled(connected);
  m_controlGroup->setEnabled(connected);
  m_runModeGroup->setEnabled(connected);
  m_currentGroup->setEnabled(connected);
}

void MainWindow::updateStatusDisplay()
{
  if (!m_cyberGear || !m_isConnected) {
    resetStatusLabels();
    return;
  }
}

void MainWindow::onTimerTick()
{
  // Control loop tick + status update
  if (m_running && m_cyberGear && m_isConnected) {
    if (m_mode == ControlMode::Operation) {
      yy_cybergear::OpCommand cmd{};
      cmd.pos_rad = static_cast<float>(m_opPosSpin ? m_opPosSpin->value() : 0.0);
      cmd.vel_rad_s = static_cast<float>(m_opVelSpin ? m_opVelSpin->value() : 0.0);
      cmd.kp = static_cast<float>(m_kpSpin ? m_kpSpin->value() : 0.0);
      cmd.kd = static_cast<float>(m_kdSpin ? m_kdSpin->value() : 0.0);
      cmd.torque_Nm = static_cast<float>(m_opTauSpin ? m_opTauSpin->value() : 0.0);
      auto r = m_cyberGear->sendOperationCommand(cmd, std::max(1, m_monitorTimer->interval() - 1));
      if (r.ok() && r.value().has_value()) {
        const auto & st = *r.value();
        if (st.fault_bits != 0) {
          logMessage("Fault detected during operation control. Stopping.");
          m_running = false;
          m_mode = ControlMode::None;
          m_monitorTimer->stop();
          (void)m_cyberGear->stopMotor();
          return;
        }
        updateStatusFrom(st);
      } else {
        logMessage("Failed to send operation command. Stopping.");
        m_running = false;
        m_mode = ControlMode::None;
        m_monitorTimer->stop();
        (void)m_cyberGear->stopMotor();
        return;
      }
    } else if (m_mode == ControlMode::Position) {
      auto r = m_cyberGear->setPositionReference(static_cast<float>(m_posRefSpin->value()));
      if (!r.ok()) {
        logMessage("Failed to set position reference. Stopping run.");
        m_running = false;
        m_mode = ControlMode::None;
        m_monitorTimer->stop();
        (void)m_cyberGear->stopMotor();
        return;
      }
      if (r.ok() && r.value().has_value()) {
        const auto & st = *r.value();
        if (st.fault_bits != 0) {
          logMessage("Fault detected during position control. Stopping.");
          m_running = false;
          m_mode = ControlMode::None;
          m_monitorTimer->stop();
          (void)m_cyberGear->stopMotor();
          return;
        }
        updateStatusFrom(st);
      }
    } else if (m_mode == ControlMode::Speed) {
      auto r = m_cyberGear->setSpeedReference(static_cast<float>(m_speedSpin->value()));
      if (!r.ok()) {
        logMessage("Failed to set speed reference. Stopping run.");
        m_running = false;
        m_mode = ControlMode::None;
        m_monitorTimer->stop();
        (void)m_cyberGear->stopMotor();
        return;
      }
      if (r.ok() && r.value().has_value()) {
        updateStatusFrom(*r.value());
      }
    } else if (m_mode == ControlMode::Current) {
      auto r = m_cyberGear->setIqReference(static_cast<float>(m_iqSpin->value()));
      if (!r.ok()) {
        logMessage("Failed to set current reference. Stopping run.");
        m_running = false;
        m_mode = ControlMode::None;
        m_monitorTimer->stop();
        (void)m_cyberGear->stopMotor();
        return;
      }
      if (r.ok() && r.value().has_value()) {
        const auto & st = *r.value();
        if (st.fault_bits != 0) {
          logMessage("Fault detected during current control. Stopping.");
          m_running = false;
          m_mode = ControlMode::None;
          m_monitorTimer->stop();
          (void)m_cyberGear->stopMotor();
          return;
        }
        updateStatusFrom(st);
      }
    }
  }

  // Always refresh status view at timer rate
  if (!m_running) updateStatusDisplay();
}

void MainWindow::logMessage(const QString & message)
{
  QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
  m_logEdit->append(QString("[%1] %2").arg(timestamp, message));

  QScrollBar * scrollBar = m_logEdit->verticalScrollBar();
  scrollBar->setValue(scrollBar->maximum());
}

void MainWindow::onApplyRunModeClicked()
{
  if (!m_cyberGear) return;
  try {
    // Stop running loops before switching
    if (m_running) {
      m_running = false;
      m_mode = ControlMode::None;
      m_monitorTimer->stop();
    }

    const int idx = m_runModeCombo->currentIndex();
    if (idx < 0) return;
    const auto modeVal =
      static_cast<yy_cybergear::CyberGear::RunMode>(m_runModeCombo->currentData().toInt());
    auto res = m_cyberGear->setRunMode(modeVal);
    if (res.ok()) {
      logMessage(QString("RunMode set to %1").arg(m_runModeCombo->currentText()));
      // If motor enabled and mode is Speed, we can resume speed loop automatically
      if (m_motorEnabled && modeVal == yy_cybergear::CyberGear::RunMode::Speed) {
        m_mode = ControlMode::Speed;
        m_running = true;
        m_monitorTimer->start();
      } else if (m_motorEnabled && modeVal == yy_cybergear::CyberGear::RunMode::Position) {
        m_mode = ControlMode::Position;
        m_running = true;
        m_monitorTimer->start();
      } else if (m_motorEnabled && modeVal == yy_cybergear::CyberGear::RunMode::Current) {
        m_mode = ControlMode::Current;
        m_running = true;
        m_monitorTimer->start();
      }
    } else {
      logMessage("Failed to set RunMode");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Apply RunMode error: %1").arg(e.what()));
  }
}

void MainWindow::onRefreshRunModeClicked()
{
  if (!m_cyberGear) return;
  try {
    auto r = m_cyberGear->getRunMode(200);
    if (r.ok() && r.value().has_value()) {
      const int mode_int = static_cast<int>(*r.value());
      // Find combo item with matching data
      for (int i = 0; i < m_runModeCombo->count(); ++i) {
        if (m_runModeCombo->itemData(i).toInt() == mode_int) {
          m_runModeCombo->setCurrentIndex(i);
          break;
        }
      }
      logMessage(QString("RunMode is %1").arg(m_runModeCombo->currentText()));
    } else {
      logMessage("Failed to get RunMode");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Refresh RunMode error: %1").arg(e.what()));
  }
}
