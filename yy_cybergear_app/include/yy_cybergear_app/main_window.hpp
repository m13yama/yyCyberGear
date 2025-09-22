#ifndef YY_CYBERGEAR_APP__MAIN_WINDOW_HPP_
#define YY_CYBERGEAR_APP__MAIN_WINDOW_HPP_

#include <QtCore/QTimer>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/protocol_types.hpp"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() = default;

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
  void onEnableMotorClicked();
  void onStopMotorClicked();
  void onClearFaultsClicked();
  void onSetMechanicalZeroClicked();
  void onGetMcuIdClicked();
  void onApplyLimitsClicked();
  void onTimerTick();
  void onOpPositionChanged(double value);
  void onOpVelChanged(double value);
  void onOpTorqueChanged(double value);
  void onTargetPositionChanged(double value);
  void onTargetSpeedChanged(double value);
  void onTargetCurrentChanged(double value);
  void onApplyRunModeClicked();
  void onRefreshRunModeClicked();

private:
  void setupUI();
  void updateConnectionStatus();
  void updateStatusDisplay();
  void logMessage(const QString & message);
  void updateStatusFrom(const yy_cybergear::Status & st);
  void resetStatusLabels();

  // Fixed display precision (digits after decimal point)
  static constexpr int kDispDecimalsAngle = 3;
  static constexpr int kDispDecimalsVelocity = 3;
  static constexpr int kDispDecimalsTorque = 3;
  static constexpr int kDispDecimalsTemperature = 1;

  // UI Elements
  QWidget * m_centralWidget;
  QVBoxLayout * m_mainLayout;

  // Connection group
  QGroupBox * m_connectionGroup;
  QLineEdit * m_interfaceEdit;
  QSpinBox * m_hostIdSpin;
  QSpinBox * m_motorIdSpin;
  QPushButton * m_connectBtn;
  QPushButton * m_disconnectBtn;
  QLabel * m_connectionStatus;

  // Control group
  QGroupBox * m_controlGroup;
  QPushButton * m_enableBtn;
  QPushButton * m_stopBtn;  // Stop Motor (hard stop)
  QPushButton * m_clearFaultsBtn;
  QPushButton * m_setZeroBtn;
  QPushButton * m_getMcuIdBtn;
  QCheckBox * m_verboseCheck;

  // Run mode switch
  QGroupBox * m_runModeGroup;
  QComboBox * m_runModeCombo;
  QPushButton * m_applyRunModeBtn;
  QPushButton * m_refreshRunModeBtn;

  // Operation control
  QGroupBox * m_operationGroup;
  QDoubleSpinBox * m_opPosSpin;  // target position [rad]
  QDoubleSpinBox * m_kpSpin;     // position gain
  QDoubleSpinBox * m_kdSpin;     // velocity gain
  QDoubleSpinBox * m_opVelSpin;  // feedforward velocity [rad/s]
  QDoubleSpinBox * m_opTauSpin;  // feedforward torque [Nm]

  // Position control
  QGroupBox * m_positionGroup;
  QDoubleSpinBox * m_posRefSpin;  // target position [rad]

  // Speed control
  QGroupBox * m_commandGroup;
  QDoubleSpinBox * m_speedSpin;  // target speed [rad/s]

  // Current control
  QGroupBox * m_currentGroup;
  QDoubleSpinBox * m_iqSpin;  // target q-axis current [A]

  // Limits
  QGroupBox * m_limitsGroup;
  QDoubleSpinBox * m_torqueLimitSpin;
  QDoubleSpinBox * m_speedLimitSpin;
  QDoubleSpinBox * m_currentLimitSpin;
  QPushButton * m_applyLimitsBtn;

  // Status display
  QGroupBox * m_statusGroup;
  QLabel * m_positionLabel;
  QLabel * m_velocityLabel;
  QLabel * m_currentLabel;
  QLabel * m_temperatureLabel;
  QLabel * m_motorIdLabel;
  QLabel * m_modeLabel;
  QLabel * m_faultsLabel;

  // Log display
  QTextEdit * m_logEdit;

  // Timer for control & monitoring
  QTimer * m_monitorTimer;
  bool m_running = false;

  enum class ControlMode { None, Operation, Position, Speed, Current };
  ControlMode m_mode = ControlMode::None;

  // CyberGear instance
  std::unique_ptr<yy_cybergear::CyberGear> m_cyberGear;
  bool m_isConnected;
  bool m_motorEnabled;
};

#endif  // YY_CYBERGEAR_APP__MAIN_WINDOW_HPP_