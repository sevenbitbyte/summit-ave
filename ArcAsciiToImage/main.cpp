#include <QApplication>
#include <QtCore>
#include <QtGui>

#include "mainwindow.h"
#include "arcasciiparser.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QStringList args = a.arguments();

    MainWindow window(args);
    window.show();

    return a.exec();
}
