#include "yy_cybergear_app/main_window.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QStringList>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <algorithm>

#include "yy_cybergear/protocol_types.hpp"
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
  setWindowTitle("CyberGear Speed Control");
  setMinimumSize(800, 600);

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

  // Command group (Speed only)
  m_commandGroup = new QGroupBox("Speed Control");
  QGridLayout * cmdLayout = new QGridLayout(m_commandGroup);

  cmdLayout->addWidget(new QLabel("Target speed [rad/s]:"), 0, 0);
  m_speedSpin = new QDoubleSpinBox();
  m_speedSpin->setRange(-50.0, 50.0);
  m_speedSpin->setDecimals(3);
  m_speedSpin->setSingleStep(0.1);
  m_speedSpin->setValue(2.0);
  connect(
    m_speedSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &MainWindow::onTargetSpeedChanged);
  cmdLayout->addWidget(m_speedSpin, 0, 1);

  // Set button removed; speed changes auto-apply

  cmdLayout->addWidget(new QLabel("Rate [Hz]:"), 1, 0);
  m_rateSpin = new QSpinBox();
  m_rateSpin->setRange(1, 200);
  m_rateSpin->setValue(100);
  connect(
    m_rateSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onRateChanged);
  cmdLayout->addWidget(m_rateSpin, 1, 1);

  // Duration removed

  // Run is controlled by Enable/Stop only

  m_mainLayout->addWidget(m_commandGroup);

  // Limits group
  m_limitsGroup = new QGroupBox("Limits");
  QGridLayout * limitsLayout = new QGridLayout(m_limitsGroup);

  limitsLayout->addWidget(new QLabel("Torque limit [Nm]:"), 0, 0);
  m_torqueLimitSpin = new QDoubleSpinBox();
  m_torqueLimitSpin->setRange(0.0, 30.0);
  m_torqueLimitSpin->setDecimals(2);
  m_torqueLimitSpin->setSingleStep(0.1);
  m_torqueLimitSpin->setValue(6.0);
  // Run is controlled by Enable/Stop only
  m_speedLimitSpin = new QDoubleSpinBox();
  m_speedLimitSpin->setValue(10.0);
  limitsLayout->addWidget(m_speedLimitSpin, 0, 3);

  limitsLayout->addWidget(new QLabel("Current limit [A]:"), 1, 0);
  m_currentLimitSpin = new QDoubleSpinBox();
  m_currentLimitSpin->setRange(0.0, 40.0);
  m_currentLimitSpin->setDecimals(2);
  m_currentLimitSpin->setSingleStep(0.5);
  m_currentLimitSpin->setValue(10.0);
  limitsLayout->addWidget(m_currentLimitSpin, 1, 1);

  m_applyLimitsBtn = new QPushButton("Apply Limits");
  connect(m_applyLimitsBtn, &QPushButton::clicked, this, &MainWindow::onApplyLimitsClicked);
  limitsLayout->addWidget(m_applyLimitsBtn, 1, 3);

  m_mainLayout->addWidget(m_limitsGroup);

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

  // Setup control+monitor timer
  m_monitorTimer = new QTimer(this);
  connect(m_monitorTimer, &QTimer::timeout, this, &MainWindow::onTimerTick);
  onRateChanged(m_rateSpin->value());
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

    // Initial safe setup: clear faults, speed mode, limits
    (void)m_cyberGear->clearFaults();
    (void)m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Speed);
    onApplyLimitsClicked();

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

  m_positionLabel->setText("N/A");
  m_velocityLabel->setText("N/A");
  m_currentLabel->setText("N/A");
  m_temperatureLabel->setText("N/A");
  m_motorIdLabel->setText("N/A");
  m_modeLabel->setText("N/A");
  m_faultsLabel->setText("N/A");
}

void MainWindow::onEnableMotorClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->enableMotor();
    if (result.ok()) {
      m_motorEnabled = true;
      logMessage("Motor enabled successfully");
      // Immediately start the control loop (enable -> motor starts)
      try {
        (void)m_cyberGear->setRunMode(yy_cybergear::CyberGear::RunMode::Speed);
        m_running = true;
        m_monitorTimer->start();
        logMessage("Run started (auto-start on enable)");
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

// onRateChanged is defined earlier

void MainWindow::onStopMotorClicked()
{
  if (!m_cyberGear) return;

  try {
    auto result = m_cyberGear->stopMotor();
    if (result.ok()) {
      m_motorEnabled = false;
      m_running = false;
      m_monitorTimer->stop();
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

// onSetSpeedClicked removed; using auto-apply on target change

void MainWindow::onApplyLimitsClicked()
{
  if (!m_cyberGear) return;
  try {
    bool okAll = true;
    if (!m_cyberGear->setTorqueLimit(static_cast<float>(m_torqueLimitSpin->value())).ok())
      okAll = false;
    if (!m_cyberGear->setSpeedLimit(static_cast<float>(m_speedLimitSpin->value())).ok())
      okAll = false;
    if (!m_cyberGear->setCurrentLimit(static_cast<float>(m_currentLimitSpin->value())).ok())
      okAll = false;
    logMessage(okAll ? "Limits applied" : "Some limits failed to apply");
  } catch (const std::exception & e) {
    logMessage(QString("Apply limits error: %1").arg(e.what()));
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
      const auto & st = *r.value();
      m_positionLabel->setText(QString::number(st.angle_rad, 'f', 3));
      m_velocityLabel->setText(QString::number(st.vel_rad_s, 'f', 3));
      m_currentLabel->setText(QString::number(st.torque_Nm, 'f', 3));
      m_temperatureLabel->setText(QString::number(st.temperature_c, 'f', 1));
      m_motorIdLabel->setText(QString::number(st.motor_can_id));
      m_modeLabel->setText(QString::fromStdString(yy_cybergear::mode_to_string(st.mode)));
      const auto faults = yy_cybergear::fault_bits_to_string(st.fault_bits);
      if (faults.empty()) {
        m_faultsLabel->setText("None");
      } else {
        QStringList list;
        for (const auto & f : faults) list << QString::fromStdString(f);
        m_faultsLabel->setText(list.join(", "));
      }
    } else if (!r.ok()) {
      logMessage("Failed to set speed");
    }
  } catch (const std::exception & e) {
    logMessage(QString("Auto apply speed error: %1").arg(e.what()));
  }
}

void MainWindow::onRateChanged(int hz)
{
  int interval_ms = 1000 / std::max(1, hz);
  m_monitorTimer->setInterval(interval_ms);
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

  m_controlGroup->setEnabled(connected);
  m_commandGroup->setEnabled(connected);
  m_limitsGroup->setEnabled(connected);
}

void MainWindow::updateStatusDisplay()
{
  if (!m_cyberGear || !m_isConnected) {
    m_positionLabel->setText("N/A");
    m_velocityLabel->setText("N/A");
    m_currentLabel->setText("N/A");
    m_temperatureLabel->setText("N/A");
    m_motorIdLabel->setText("N/A");
    m_modeLabel->setText("N/A");
    m_faultsLabel->setText("N/A");
    return;
  }
}

void MainWindow::onTimerTick()
{
  // Control loop tick + status update
  if (m_running && m_cyberGear && m_isConnected) {
    // Send speed reference
    auto r = m_cyberGear->setSpeedReference(static_cast<float>(m_speedSpin->value()));
    if (!r.ok()) {
      logMessage("Failed to set speed reference. Stopping run.");
      m_running = false;
      m_monitorTimer->stop();
      (void)m_cyberGear->stopMotor();
      return;
    }
    // Update UI from reply status to avoid additional CAN transactions
    if (r.ok() && r.value().has_value()) {
      const auto & st = *r.value();
      m_positionLabel->setText(QString::number(st.angle_rad, 'f', 3));
      m_velocityLabel->setText(QString::number(st.vel_rad_s, 'f', 3));
      m_currentLabel->setText(QString::number(st.torque_Nm, 'f', 3));
      m_temperatureLabel->setText(QString::number(st.temperature_c, 'f', 1));
      m_motorIdLabel->setText(QString::number(st.motor_can_id));
      m_modeLabel->setText(QString::fromStdString(yy_cybergear::mode_to_string(st.mode)));
      const auto faults = yy_cybergear::fault_bits_to_string(st.fault_bits);
      if (faults.empty()) {
        m_faultsLabel->setText("None");
      } else {
        QStringList list;
        for (const auto & f : faults) list << QString::fromStdString(f);
        m_faultsLabel->setText(list.join(", "));
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
