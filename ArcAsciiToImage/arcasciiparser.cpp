#include "arcasciiparser.h"
#include <QImage>

ArcAsciiData::ArcAsciiData(QObject* parent) :
    QObject(parent)
{
    stats.maxElevation = -9999;
    stats.minElevation = 99999;
    file = NULL;
}

QVariantMap ArcAsciiStatistics::toVariantMap(){
    QVariantMap map;

    map.insert("maxElevation", QVariant(maxElevation));
    map.insert("minElevation", QVariant(minElevation));

    QVariantMap histogramMap;

    QMap<qint32,qint32>::iterator histIter = histogram.begin();
    for(; histIter != histogram.end(); histIter++){
        histogramMap.insert( QString::number(histIter.key()), QVariant(histIter.value()) );
    }

    map.insert("histogram", histogramMap);

    return map;
}

bool ArcAsciiStatistics::loadFromVariant(QVariant variant){
    QMap<QString,QVariant> map = variant.toMap();

    if( !map.contains("maxElevation") ){
        qCritical() << "No field named maxElevation";
        return false;
    }

    if( !map.contains("minElevation") ){
        qCritical() << "No field named minElevation";
        return false;
    }

    if( !map.contains("histogram") ){
        qCritical() << "No field named histogram";
        return false;
    }


    if(!map["maxElevation"].canConvert(QMetaType::Int)){
        qCritical() << "Conversion failure maxElevation[" << map["maxElevation"].toString() << "]";
        return false;
    }

    if(!map["minElevation"].canConvert(QMetaType::Int)){
        qCritical() << "Conversion failure minElevation[" << map["minElevation"].toString() << "]";
        return false;
    }

    maxElevation = map["maxElevation"].toInt();
    minElevation = map["maxElevation"].toInt();

    QMap<QString,QVariant> histogramMap = map["histogram"].toMap();

    QMap<QString,QVariant>::iterator histIter = histogramMap.begin();
    for(; histIter != histogramMap.end(); histIter++){
        bool success = true;

        int bucket = histIter.key().toInt(&success);
        if(!success){ qCritical() << "Malformed histogram"; return false; }

        int count = histIter.value().toInt(&success);
        if(!success){ qCritical() << "Malformed histogram"; return false; }

        histogram.insert(bucket, count);
    }

    return true;
}

void ArcAsciiStatistics::merge(ArcAsciiStatistics& other){
    if(other.minElevation < minElevation){
        minElevation = other.minElevation;
    }

    if(other.maxElevation > maxElevation){
        maxElevation = other.maxElevation;
    }

    QMap<qint32,qint32>::iterator otherHistIter = other.histogram.begin();
    for(; otherHistIter != other.histogram.end(); otherHistIter++){
        if( histogram.contains(otherHistIter.key()) ){
            histogram[otherHistIter.key()] += otherHistIter.value();
        }
        else{
            histogram.insert( otherHistIter.key(), otherHistIter.value() );
        }
    }
}


ArcAsciiParser::ArcAsciiParser(QObject* parent) :
    QObject(parent)
{
    _stopped = true;
    _parsing = false;
    _loadElevation = false;
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

void ArcAsciiParser::setLoadElevation(bool value){
    _loadElevation = value;
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

    if( !loadStatistics(data) ){
        if(!loadElevation(data)){
            qWarning() << "Failed to load elevation ["<<file.filePath()<<"]";
            delete data;
            return NULL;
        }

        if(!saveStatistics(data)){
            qWarning() << "Failed to load save statistics for ["<<file.filePath()<<"]";
            delete data;
            return NULL;
        }

        if(!_loadElevation){
            data->elevation.clear();
        }
    }

    if(_loadElevation && data->elevation.empty()){
        if(!loadElevation(data)){
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


bool ArcAsciiParser::loadElevation(ArcAsciiData* data){
    data->file->reset();
    int lineCount = 0;
    int fileRow = 0;

    data->elevation.resize(data->header.height * data->header.width);

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
            while(lineIdx < lineData.size() - 1 && elevationIdx < (((fileRow - 5) * data->header.width)) && !_stopped){
                int sep = lineData.indexOf(' ', lineIdx);


                QString subStr(&lineData.data()[lineIdx], sep - lineIdx);

                float elevation = subStr.toFloat();
                data->elevation[elevationIdx] = elevation;

                if(elevation != data->header.noData){
                    //Update min/max elevation
                    if( elevation > data->stats.maxElevation ){
                        data->stats.maxElevation = elevation;
                    }

                    if( elevation < data->stats.minElevation ){
                        data->stats.minElevation = elevation;
                    }

                    //Update histogram
                    int bucket = (elevation/10);
                    if( !data->stats.histogram.contains(bucket) ){
                        data->stats.histogram.insert(bucket, 0);
                    }

                    data->stats.histogram[bucket] = data->stats.histogram[bucket] + 1;
                }

                elevationIdx++;
                lineIdx += (sep - lineIdx) + 1;

                //Emit progress
                if(fileRow % 20 == 0){
                    double progressPercent = ((qreal)data->file->pos()) / ((qreal) data->info.size());
                    emit currentFileProgress(progressPercent, data);
                }
            }
        }
        fileRow++;
    }

    return true;
}


bool ArcAsciiParser::loadStatistics(ArcAsciiData* data){
    QFile input( QString(data->info.filePath()).append(".stats") );

    if(!input.exists()){
        return false;
    }

    if(!input.open(QFile::ReadOnly)){
        qCritical() << "Failed to open file for reading[" << input.fileName() << "]";
        return false;
    }
    emit parsingFile(input.fileName());

    qDebug() << "Reading stats: " << input.fileName();

    QJsonDocument doc = QJsonDocument::fromJson( input.readAll() );

    return data->stats.loadFromVariant(doc.toVariant());
}


bool ArcAsciiParser::saveStatistics(ArcAsciiData* data){
    QVariantMap map = data->stats.toVariantMap();

    QJsonDocument doc = QJsonDocument::fromVariant(map);

    QString outputPath = QString(data->info.filePath()).append(".stats");

    QFile output(outputPath);

    if(!output.open(QFile::WriteOnly)){
        qCritical() << "Failed to open file for writing[" << output.fileName() << "]";
        return false;
    }

    if( output.write( doc.toJson() ) < 1){
        qCritical() << "Failed to write to file[" << output.fileName() << "]";
        return false;
    }

    qDebug() << "Saving stats: " << outputPath;

    output.close();
    return true;
}
