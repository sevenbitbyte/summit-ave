#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "asctopcd.h"

AscToPcd::AscToPcd(QCoreApplication* parent) : QObject(parent)
{
  qDebug() << parent->arguments();

  mergedCloud5k = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  mergedCloud10k = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  mergedCloud20k = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  parserThread = new QThread();
  parser = new ArcAsciiParser(40);
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

  qDebug() << "Writing: merged20k.pcd with " << mergedCloud20k->points.size() << " points";
  pcl::io::savePCDFileBinary("merged20k.pcd", *mergedCloud20k);

  qDebug() << "Writing: merged10k.pcd with " << mergedCloud10k->points.size() << " points";
  pcl::io::savePCDFileBinary("merged10k.pcd", *mergedCloud10k);

  /*qDebug() << "Writing: merged5k.pcd with " << mergedCloud5k->points.size() << " points";
  pcl::io::savePCDFileBinary("merged5k.pcd", *mergedCloud5k);*/
  qDebug() << "Wrote: merged PCDs";

  emit done();
}

void AscToPcd::writePCD(ArcAsciiData* data){
  qDebug() << "Parsing complete with " << data->cloud->points.size() << " points";

  QString pcdPath = data->info.path();
  pcdPath.append("/");
  pcdPath.append(data->info.baseName());
  pcdPath.append("_x");
  pcdPath.append(QString::number(parser->elevationCoefficient()));
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
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1000 = downsampleOrLoad(1000.0f, data->info, data->cloud, 500);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5000 = downsampleOrLoad(5000.0f, data->info, data->cloud, 2000.0f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud10000 = downsampleOrLoad(10000.0f, data->info, data->cloud, 3000.0f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud20000 = downsampleOrLoad(20000.0f, data->info, data->cloud, 3500.0f);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud100000 = downsampleOrLoad(100000.0f, data->info, data->cloud, 5000.0f);

  for(int i=0; i<cloud10000->points.size(); i++){
    mergedCloud10k->push_back( cloud10000->points[i] );
  }

  for(int i=0; i<cloud20000->points.size(); i++){
    mergedCloud20k->push_back( cloud20000->points[i] );
  }/*

  for(int i=0; i<cloud100000->points.size(); i++){
    mergedCloud100k->push_back( cloud100000->points[i] );
  }*/

  delete data;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AscToPcd::downsampleOrLoad(float leafSize, QFileInfo info, pcl::PointCloud<pcl::PointXYZ>::Ptr input, float leafSizeV){

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

  QString pcdDownsampledPath = QString("%1/%2_voxel_%3_x%4.pcd")
      .arg(info.path())
      .arg(info.baseName())
      .arg(leafSize)
      .arg(parser->elevationCoefficient());
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
  downsample.setLeafSize(leafSize, leafSize, leafSizeV);
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
