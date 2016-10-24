#include "arcasciiparser.h"

LLA::LLA(double lat, double lon, double alt){
  this->latitude = lat;
  this->longitude = lon;
  this->altitude = alt;
}

Point3D::Point3D(){
  this->x = 0.0;
  this->y = 0.0;
  this->z = 0.0;
}

Point3D::Point3D(LLA latLong){
  double cosLat = cos(latLong.latitude * M_PI/180.0);
  double sinLat = sin(latLong.latitude * M_PI/180.0);
  double cosLon = cos(latLong.longitude * M_PI/180.0);
  double sinLon = sin(latLong.longitude * M_PI/180.0);

  double radius = 6378137.0;
  double f = 1.0 / 298.257224;
  double C = 1.0 / sqrt( (cosLat * cosLat) + (((1-f) * (1-f)) * (sinLat * sinLat)));
  double S = (1.0 - f) * (1.0 -f) * C;
  double h = latLong.altitude;

  this->x = ((radius * C) + h) * cosLat * cosLon;
  this->y = ((radius * C) + h) * cosLat * sinLon;
  this->z = ((radius * S) + h) * sinLat;
}

LLA ArcAsciiHeader::rowColToLLA(int row, int col){
  return LLA((((this->height-1)-row)*this->cellSize) + this->yCorner, (col*this->cellSize)+this->xCorner);
}

ArcAsciiData::ArcAsciiData(QObject* parent) :
    QObject(parent)
{
  this->file = NULL;
  this->cloud.is_dense = true;
}


ArcAsciiParser::ArcAsciiParser(QObject* parent) :
    QObject(parent)
{
  _stopped = true;
  _parsing = false;
}


bool ArcAsciiParser::queueFile(QString file){

    QFileInfo info(file);

    if(!info.exists()){
        return false;
    }

    _filesToParse.push_back(info);

    return true;
}

void ArcAsciiParser::stopParsing(){
    _stopped = true;
}

bool ArcAsciiParser::parsing(){
    return _parsing;
}

void ArcAsciiParser::startParsing(){
  _parsing = true;
  _stopped = false;

  qreal bytesToRead = 0.0f;
  qreal bytesRead = 0.0f;

  for(int i=0; i<_filesToParse.size(); i++){
      bytesToRead += _filesToParse[i].size();
  }

  while( !_filesToParse.empty() && !_stopped ){
      QFileInfo info = _filesToParse.front();
      _filesToParse.pop_front();

      //emit parsingFile(info.filePath());
      ArcAsciiData* data = loadData(info);

      if(data == NULL){
          emit parseError(info.filePath());
          continue;
      }

      bytesRead += info.size();

      qreal percentProgress = bytesRead / bytesToRead;
      emit totalProgress(percentProgress);
      emit dataReady(data);
  }

  _parsing = false;
  emit finished();
}


ArcAsciiData* ArcAsciiParser::loadData(QFileInfo file){
    ArcAsciiData* data = new ArcAsciiData(this);

    data->info = file;
    data->file = new QFile(file.filePath(), data);
    if( !data->file->open(QFile::ReadOnly) ){
        qWarning() << "Failed to open data file ["<<file.filePath()<<"]";
        delete data;
        return NULL;
    }

    if(! loadHeader(data) ){
        qWarning() << "Failed to load header ["<<file.filePath()<<"]";
        delete data;
        return NULL;
    }

    if(data->cloud.points.empty()){
        if(!loadPointCloud(data)){
            qWarning() << "Failed to load elevation ["<<file.filePath()<<"]";
            delete data;
            return NULL;
        }
    }

    if(data->file != NULL && data->file->isOpen()){
        //Close file
        delete data->file;
        data->file = NULL;
    }

    return data;
}



bool ArcAsciiParser::loadHeader(ArcAsciiData* data){
    data->file->reset();
    int lineCount = 0;

    emit parsingFile(data->file->fileName());

    while( !data->file->atEnd() && lineCount < 6 ){

        QString lineData = data->file->readLine();
        lineCount++;

        if(lineCount<=6){
            QStringList tokens = lineData.simplified().toLower().split(' ');
            if(tokens.size() != 2){
                qCritical() << "Expected 2 tokens but found " << tokens.size() << " on line " << lineCount;
                return false;
            }

            if(tokens[0] == "ncols"){
                data->header.width = tokens[1].toInt();
            }
            else if(tokens[0] == "nrows"){
                data->header.height = tokens[1].toInt();
            }
            else if(tokens[0] == "xllcorner"){
                data->header.xCorner = tokens[1].toDouble();
            }
            else if(tokens[0] == "yllcorner"){
                data->header.yCorner = tokens[1].toDouble();
            }
            else if(tokens[0] == "cellsize"){
                data->header.cellSize = tokens[1].toDouble();
            }
            else if(tokens[0] == "nodata_value"){
                data->header.noData = tokens[1].toInt();
            }
        }
    }

    //Read the expected number of lines?
    return (lineCount >= 6);
}


bool ArcAsciiParser::loadPointCloud(ArcAsciiData* data){
    data->file->reset();
    int lineCount = 0;
    int fileRow = 0;

    //data->elevation.resize(data->header.height * data->header.width);

    emit parsingFile(data->file->fileName());

    while( !data->file->atEnd() && !_stopped){

        QString lineData = data->file->readLine();
        lineCount++;

        if(lineCount<=6){
            QStringList tokens = lineData.simplified().toLower().split(' ');
            if(tokens.size() != 2){
                qCritical() << "Expect 2 tokens but found " << tokens.size() << " on line " << lineCount;
                return false;
            }

        }
        else{
            int elevationIdx = (fileRow - 6) * data->header.width;
            int lineIdx = 0;
            int col=0;
            while(lineIdx < lineData.size() - 1 && elevationIdx < (((fileRow - 5) * data->header.width)) && !_stopped){
                int sep = lineData.indexOf(' ', lineIdx);


                QString subStr(&lineData.data()[lineIdx], sep - lineIdx);

                double elevation = subStr.toDouble();

                if(elevation != data->header.noData){
                    LLA lla = data->header.rowColToLLA(fileRow-6, col);
                    lla.altitude = elevation*5;
                    Point3D pt(lla);
                    data->cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
                }

                elevationIdx++;
                lineIdx += (sep - lineIdx) + 1;

                col++;
            }

            //Emit progress
            if(fileRow % 20 == 0){
                double progressPercent = ((qreal)data->file->pos()) / ((qreal) data->info.size());
                emit currentFileProgress(progressPercent, data);
            }
        }
        fileRow++;
    }

    return true;
}



