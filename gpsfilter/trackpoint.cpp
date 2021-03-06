#include "trackpoint.h"
#include "UTM/conversion.h"

#include <qjson/qobjecthelper.h>
#include <qjson/serializer.h>

using namespace QJson;

TrackPoint::TrackPoint(QObject *parent) :
	QObject(parent)
{
	_hdop = 0.0f;
	_device_id = 0;
	_source = Source_Unknown;
	_latitude = 0.0f;
	_longitude = 0.0f;
	_utm_x = 0.0f;
	_utm_y = 0.0f;
	_elevation = 0.0f;
	_course = 0.0f;
	_speed = 0.0f;
	_satelites = 0;
}

TrackPoint* TrackPoint::parseXml(QDomNode& trkPtNode){
	TrackPoint* trkPt = NULL;
	QDomNodeList children = trkPtNode.childNodes();

	//Parse attributes
	QDomNamedNodeMap attributes = trkPtNode.attributes();

	if(attributes.count() > 0 || trkPtNode.nodeName() != QString("trkpt")){
		trkPt = new TrackPoint;
	}
	else{
		qDebug() << "Not expected node read [" << trkPtNode.nodeName() << "]";
		return trkPt;
	}

	QDomNode latNode = attributes.namedItem("lat");
	QDomNode lonNode = attributes.namedItem("lon");

	if(!latNode.isNull()){
		trkPt->setLatitude( latNode.nodeValue().toDouble() );
	}

	if(!lonNode.isNull()){
		trkPt->setLongitude( lonNode.nodeValue().toDouble() );
	}


	for(int i=0; i<children.count(); i++){
		QDomNode child = children.at(i);

		QString name = child.nodeName();

		if(name == QString("ele") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			trkPt->setElevation( valueNode.nodeValue().toDouble() );
		}
		else if(name == QString("course") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			trkPt->setCourse( valueNode.nodeValue().toDouble() );
		}
		else if(name == QString("speed") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			trkPt->setSpeed( valueNode.nodeValue().toDouble() );
		}
		else if(name == QString("hdop") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			trkPt->setHdop( valueNode.nodeValue().toDouble() );
		}
		else if(name == QString("src") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			QString sourceStr = valueNode.nodeValue();

			if(sourceStr == "gps"){
				trkPt->setSource(Source_Gps);
			}
			else if(sourceStr == "network"){
				trkPt->setSource(Source_Network);
			}
			else{
				trkPt->setSource(Source_Unknown);
				//qWarning() << "src = " << sourceStr.toAscii().data() << endl;
			}
		}
		else if(name == QString("sat") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			trkPt->setSatelites( valueNode.nodeValue().toInt() );
		}
		else if(name == QString("time") && child.hasChildNodes()){
			QDomNode valueNode = child.childNodes().at(0);
			trkPt->setTime( QDateTime::fromString(valueNode.nodeValue(), Qt::ISODate) );
		}
		else{
			qWarning() << "Unsupported trkpt child[" << name << "]";
			//cout << "skipped" << name.toAscii().data() <<  endl;
		}
	}

	trkPt->updateUTM();

	return trkPt;
}


double TrackPoint::latitude() const {
	return _latitude;
}

void TrackPoint::setLatitude(double latitude){
	_latitude = latitude;
}


double TrackPoint::longitude() const {
	return _longitude;
}

void TrackPoint::setLongitude(double longitude){
	_longitude = longitude;
}


double TrackPoint::elevation() const {
	return _elevation;
}

void TrackPoint::setElevation(double elevation){
	_elevation = elevation;
}


double TrackPoint::course() const {
	return _course;
}

void TrackPoint::setCourse(double course){
	_course = course;
}


double TrackPoint::speed() const {
	return _speed;
}

void TrackPoint::setSpeed(double speed){
	_speed = speed;
}


double TrackPoint::hdop() const {
	return _hdop;
}

void TrackPoint::setHdop(double hdop){
	_hdop = hdop;
}



TrackPoint::TrackSource TrackPoint::source() const {
	return _source;
}

void TrackPoint::setSource(TrackSource source){
	_source = source;
}


int TrackPoint::satelites() const {
	return _satelites;
}

void TrackPoint::setSatelites(int satelites){
	_satelites = satelites;
}

QDateTime TrackPoint::time() const {
	return _time;
}

void TrackPoint::setTime(const QDateTime& time){
	_time = time;
}


int TrackPoint::getDeviceId() const {
	return _device_id;
}

void TrackPoint::setDeviceId(int id) {
	_device_id = id;
}


double TrackPoint::utmX() const {
	return _utm_x;
}

double TrackPoint::utmY() const {
	return _utm_y;
}

QString TrackPoint::utmZone() const {
	return _utm_zone;
}

void TrackPoint::updateUTM() {
	LLtoUTM(23, _latitude, _longitude, _utm_y, _utm_x, _utm_zone);
}


void TrackPoint::updateLL() {
	UTMtoLL(23, _utm_x, _utm_y, _utm_zone, _latitude, _longitude);
}



TrackSegment::TrackSegment(QObject* parent) : QObject(parent) {
	//
}


TrackSegment::~TrackSegment() {
	while(!points.isEmpty()) {
		delete points.first();
		points.pop_front();
	}
}


TrackSegment* TrackSegment::parseXml(QDomNode& node){

	if(node.nodeName() == QString("trkseg")){

		TrackSegment* segment = new TrackSegment;

		QDomNode ptNode = node.firstChild();
		while( !ptNode.isNull() ){
			TrackPoint* point = TrackPoint::parseXml(ptNode);

			if(point != NULL){
				segment->points.push_back(point);
				//delete point;
			}

			ptNode = ptNode.nextSibling();
		}

		if(segment->points.count() > 0){
			return segment;
		}

		delete segment;
	}

	return (TrackSegment*) NULL;
}


QString TrackSegment::getJson(){
	Serializer serializer;

	QList<QVariant> pointVariantList;
	QString jsonString;
	QTextStream strStream(&jsonString);


	for(int i=0; i<points.count(); i++){
		TrackPoint* point = points.at(i);
		QVariantMap variant = QObjectHelper::qobject2qvariant( point );

		pointVariantList.append( variant );
	}

	QVariantMap trackSegment;

	trackSegment.insert(QString("points"), pointVariantList);

	strStream << serializer.serialize( trackSegment );

	return jsonString;
}

/*void TrackSegment::filterSegment(const QList<TrackPoint*> filterPoints){
	//
}

void TrackSegment::setPoints(const QList<TrackPoint> pointList){
	//_points = pointList;
}*/

/*QList<TrackPoint> TrackSegment::points() const {
	QList<TrackPoint> ptList;

	for(int i=0; i < _points.count(); i++){
		ptList.append( *_points[i] );
	}

	return ptList;
}*/
