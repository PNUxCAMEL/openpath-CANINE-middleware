#include "gui/mainwindow.h"

#include <QApplication>

std::string AL_NAME = "CAMEL-GUI";

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
