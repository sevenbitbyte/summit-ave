#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "asctopcd.h"

AscToPcd::AscToPcd(QCoreApplication* parent) : QObject(parent)
{
  qDebug() << parent->arguments();

  mergedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  parserThread = new QThread();
  parser = new ArcAsciiParser();
  parser->moveToThread(parserThread);


  for(int i=1; i < parent->arguments().size(); i++){
    parser->queueFile(parent->arguments().at(i));
  }

  qDebug() << parser->_filesToParse.size();
  connect(parser, SIGNAL(finished()), this, SLOT(parserDone()));
  connect(this, SIGNAL(done()), parent, SLOT(quit()));
  connect(parser, SIGNAL(dataReady(ArcAsciiData*)), this, SLOT(writePCD(ArcAsciiData*)));
  connect(parser, SIGNAL(parseError(QString)), this, SLOT(reportError(QString)));
  connect(parser, SIGNAL(parsingFile(QString)), this, SLOT(reportParsingStart(QString)));
  connect(parser, SIGNAL(totalProgress(qreal)), this, SLOT(reportTotalProgress(qreal)));
  connect(parser, SIGNAL(currentFileProgress(qreal,ArcAsciiData*)), this, SLOT(reportCurrentFileProgress(qreal,ArcAsciiData*)));
  connect(parserThread, SIGNAL(destroyed()), parser, SLOT(stopParsing()));
  connect(parserThread, SIGNAL(finished()), parserThread, SLOT(deleteLater()));

  parserThread->start(QThread::LowPriority);
  QTimer::singleShot(10, parser, SLOT(startParsing()));
}

AscToPcd::~AscToPcd(){
  parser->stopParsing();
  delete parser;
}

void AscToPcd::parserDone(){
  qDebug() << "Parsing finished";

  qDebug() << "Writing: merged.pcd with " << mergedCloud->points.size() << " points";
  pcl::io::savePCDFileBinary("merged.pcd", *mergedCloud);
  qDebug() << "Wrote: merged.pcd";

  emit done();
}

void AscToPcd::writePCD(ArcAsciiData* data){
  qDebug() << "Parsing complete with " << data->cloud->points.size() << " points";

  QString pcdPath = data->info.path();
  pcdPath.append("/");
  pcdPath.append(data->info.baseName());
  pcdPath.append(".pcd");
  QFileInfo pcdInfo(pcdPath);

  if(pcdInfo.exists()){
    //Load existing PCD
    qDebug() << "Not writing: " << pcdPath << " already exists";
  }
  else{
    qDebug() << "Writing:" << pcdPath;
    pcl::io::savePCDFileBinary(pcdPath.toAscii().data(), *data->cloud);
    qDebug() << "Wrote:" << pcdPath;
  }

  qDebug() << "Creating pyramids of " << pcdPath << "...";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1000 = downsampleOrLoad(1000.0f, data->info, data->cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5000 = downsampleOrLoad(5000.0f, data->info, cloud1000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud10000 = downsampleOrLoad(10000.0f, data->info, cloud5000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud50000 = downsampleOrLoad(50000.0f, data->info, cloud10000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud100000 = downsampleOrLoad(100000.0f, data->info, cloud10000);

  for(int i=0; i<cloud50000->points.size(); i++){
    mergedCloud->push_back( cloud50000->points[i] );
  }

  delete data;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AscToPcd::downsampleOrLoad(float leafSize, QFileInfo info, pcl::PointCloud<pcl::PointXYZ>::Ptr input){

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

  QString pcdDownsampledPath = QString("%1/%2_voxel_%3.pcd")
      .arg(info.path())
      .arg(info.baseName())
      .arg(leafSize);
  QFileInfo pcdInfo(pcdDownsampledPath);

  if(pcdInfo.exists()){
    //Load existing PCD
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcdDownsampledPath.toAscii().data(), *downsampledCloudPtr) != -1 && downsampledCloudPtr->size() > 0){
      //Loaded existing PCD
      qDebug() << "Loaded existing pointCloud" << pcdInfo.fileName();
      return downsampledCloudPtr;
    }
  }

  pcl::VoxelGrid<pcl::PointXYZ> downsample;
  downsample.setInputCloud(input);
  downsample.setLeafSize(leafSize, leafSize, 90.0f);
  downsample.filter(*downsampledCloudPtr);

  qDebug() << "Downsample complete with " << downsampledCloudPtr->points.size() << " points";
  qDebug() << "Writing:" << pcdDownsampledPath;
  pcl::io::savePCDFileBinary(pcdDownsampledPath.toAscii().data(), *downsampledCloudPtr);
  qDebug() << "Wrote:" << pcdDownsampledPath;

  return downsampledCloudPtr;
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
