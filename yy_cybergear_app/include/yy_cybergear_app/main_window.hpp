#ifndef YY_CYBERGEAR_APP__MAIN_WINDOW_HPP_
#define YY_CYBERGEAR_APP__MAIN_WINDOW_HPP_

#include <QtCore/QElapsedTimer>
#include <QtCore/QTimer>
#include <QtWidgets/QCheckBox>
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
  void onRateChanged(int hz);
  void onTimerTick();
  void onTargetSpeedChanged(double value);

private:
  void setupUI();
  void updateConnectionStatus();
  void updateStatusDisplay();
  void logMessage(const QString & message);

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

  // Speed control
  QGroupBox * m_commandGroup;
  QDoubleSpinBox * m_speedSpin;  // target speed [rad/s]
  // Removed Set button; speed changes auto-apply
  QSpinBox * m_rateSpin;            // control/monitor rate [Hz]
  QDoubleSpinBox * m_durationSpin;  // run duration [s] (0 = infinite)

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
  QLabel * m_voltageLabel;

  // Log display
  QTextEdit * m_logEdit;

  // Timer for control & monitoring
  QTimer * m_monitorTimer;
  QElapsedTimer m_elapsed;
  bool m_running = false;
  double m_targetDurationSec = 0.0;

  // CyberGear instance
  std::unique_ptr<yy_cybergear::CyberGear> m_cyberGear;
  bool m_isConnected;
  bool m_motorEnabled;
};

#endif  // YY_CYBERGEAR_APP__MAIN_WINDOW_HPP_