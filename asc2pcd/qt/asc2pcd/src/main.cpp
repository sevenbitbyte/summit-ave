#include <QCoreApplication>

#include <iostream>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "asctopcd.h"

int main(int argc, char** argv) {

  QCoreApplication app(argc, argv);

  AscToPcd task(&app);

  return app.exec();
}
