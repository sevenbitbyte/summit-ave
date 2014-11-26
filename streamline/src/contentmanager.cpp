#include "contentmanager.h"
#include "gps/trackpoint.h"

ContentManager::ContentManager(QObject *parent) :
	QObject(parent)
{

}


bool ContentManager::isMediaSuffix(QString suffix){
	if(suffix == "jpeg" || suffix == "jpg" || suffix == "png"){
		return true;	//Image
	}
	else if(suffix == "m4v" || suffix == "mpg" || suffix == "mpeg" || suffix == "ogg"){
		return true;
	}

	return false;
}

bool ContentManager::isPositionSuffix(QString suffix){
	if(suffix == "gpx"){
		return true;
	}

	return false;
}

QMultiMap<QUrl, TrackSegment*>	ContentManager::locationData() const {
	/*TODO:	Threading
	 *	Consider a mutex on location data here and where ever it is written.
	 **/
	return _locationData;
}

QList<TrackSegment*> ContentManager::getLocationByTime(QDateTime start, QDateTime end) const {
    Q_ASSERT(start < end);

    QList<TrackSegment*> locationList;

    QMap<QDateTime,TrackSegment*>::const_iterator first = _locationEndTimes.lowerBound(start);

    if(first != _locationEndTimes.end()){
        TrackSegment* segment = first.value();

        if( (segment->start > start && segment->start < end) ||
            (segment->end > start && segment->end <end) ){
            //All's well
        }
        else{
            return locationList;
        }
    }

    QMap<QDateTime,TrackSegment*>::const_iterator segIter = first;

    while(segIter != _locationEndTimes.end()){
        TrackSegment* segment = segIter.value();

        if( (segment->start > start && segment->start < end) ||
            (segment->end > start && segment->end <end) ){

            locationList.append(segment);
        }
        else{
            break;
        }

        /*if(segment->end > end){
            break;
        }*/

        segIter++;
    }

    return locationList;
}

void ContentManager::indexFile(QUrl path){
	Q_ASSERT(path.isLocalFile());

	QString localFilePath = path.toLocalFile();

	emit importStatus(path, ATTRIBUTES, 0.0f);

	QFileInfo info(localFilePath);

	QString suffix = info.suffix().toLower();

	if(isPositionSuffix(suffix)){
		//parse locations tracks

		_filesByType.insert("location", path);

		QList<TrackSegment*> segments = TrackSegment::parseXml(path.toLocalFile());

        for(int i=0; i<segments.size(); i++){
            QDateTime start = info.created();
            QDateTime end = start;

            QList<TrackPoint*>* firstTrack = &segments[i]->points;

            //Grab the start time
            if(!firstTrack->isEmpty()){
                Q_ASSERT(firstTrack->first() != NULL);
                start = firstTrack->first()->time();
            }

            //Grab the end time
            QList<TrackPoint*>* lastTrack = &segments[i]->points;
            if(!lastTrack->isEmpty()){
                Q_ASSERT(lastTrack->last() != NULL);
                end = lastTrack->last()->time();
            }
            else{
                end = start;
            }

            segments[i]->start = start;
            segments[i]->end = end;

            _locationData.insert(path, segments[i]);
            _locationEndTimes.insert(end, segments[i]);
            _locationStartTimes.insert(start, segments[i]);
        }

		emit importStatus(path, INDEX_COMPLETE);
	}
	else if(suffix == "jpeg" || suffix == "jpg" || suffix == "png"){
		//parse picture
		_filesByType.insert("image", path);
	}
	else if(suffix == "m4v" || suffix == "mpg" || suffix == "mpeg" || suffix == "ogg"){
		_filesByType.insert("movie", path);
		//parse video / audio
	}

	//Examine type
	//Read file timestamp
	//Read content start, end time(start + duration)
}

void ContentManager::indexFiles(QList<QUrl> paths) {
	for(int i = 0; i < paths.size(); i++) {
		indexFile(paths[i]);
	}
}
