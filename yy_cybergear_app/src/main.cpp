#include <QtCore/QCoreApplication>
#include <QtWidgets/QApplication>
#include <iostream>

#include "yy_cybergear_app/main_window.hpp"

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);

  QCoreApplication::setApplicationName("CyberGear Speed Control");
  QCoreApplication::setApplicationVersion("1.0.0");
  QCoreApplication::setOrganizationName("YY CyberGear");

  try {
    MainWindow window;
    window.show();

    return app.exec();
  } catch (const std::exception & e) {
    std::cerr << "Application error: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown application error" << std::endl;
    return 1;
  }
}