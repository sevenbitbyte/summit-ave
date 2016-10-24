#include "asctopcd.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

AscToPcd::AscToPcd(QCoreApplication* parent) : QObject(parent)
{
  qDebug() << parent->arguments();

  parser = new ArcAsciiParser(this);

  for(int i=1; i < parent->arguments().size(); i++){
    parser->queueFile(parent->arguments().at(i));
  }

  qDebug() << parser->_filesToParse.size();
  connect(parser, SIGNAL(finished()), this, SLOT(parserDone()));
  connect(parser, SIGNAL(finished()), parent, SLOT(quit()));
  connect(parser, SIGNAL(dataReady(ArcAsciiData*)), this, SLOT(writePCD(ArcAsciiData*)));
  connect(parser, SIGNAL(parseError(QString)), this, SLOT(reportError(QString)));
  connect(parser, SIGNAL(parsingFile(QString)), this, SLOT(reportParsingStart(QString)));
  connect(parser, SIGNAL(totalProgress(qreal)), this, SLOT(reportTotalProgress(qreal)));
  connect(parser, SIGNAL(currentFileProgress(qreal,ArcAsciiData*)), this, SLOT(reportCurrentFileProgress(qreal,ArcAsciiData*)));

  QTimer::singleShot(10, parser, SLOT(startParsing()));
}

AscToPcd::~AscToPcd(){
  parser->stopParsing();
  delete parser;
}

void AscToPcd::parserDone(){
  qDebug() << "Parsing finished";
}

void AscToPcd::writePCD(ArcAsciiData* data){
  qDebug() << "Parsing complete with " << data->cloud.points.size() << " points";

  QString pcdPath = data->info.path();
  pcdPath.append("/");
  pcdPath.append(data->info.baseName());
  pcdPath.append(".pcd");

  qDebug() << "Writing:" << pcdPath;
  pcl::io::savePCDFileBinary(pcdPath.toAscii().data(), data->cloud);
  qDebug() << "Wrote:" << pcdPath;
}

void AscToPcd::reportError(QString error){
  qWarning() << error;
}

void AscToPcd::reportParsingStart(QString file){
  qDebug() << "Parsing: " << file;
}

void AscToPcd::reportTotalProgress(qreal percent) {
  qDebug() << "Total progress: " << percent;
}

void AscToPcd::reportCurrentFileProgress(qreal percent, ArcAsciiData* data) {
  qDebug() << "Current file progress: " << percent;
}
