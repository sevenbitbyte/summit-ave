#include "locationdb.h"

#include <QtCore>
#include <QtSql>

LocationDb::LocationDb(QString dbPath, QObject *parent) :
	QObject(parent)
{

	//db = new QSqlDatabase("QSQLITE");
	db = QSqlDatabase::addDatabase(QString("QSQLITE"));
	db.setDatabaseName(dbPath);

	if(!db.open()){
		qErrnoWarning("Failed to open database");
		return;
	}

	QStringList tables = db.tables(QSql::Tables);

	if(!tables.contains("filters")){
		constructFiltersTable();
	}

	if(!tables.contains("locations")){
		constructLocationsTable();
	}

	QString syncOff = "PRAGMA synchronous=OFF";
	QSqlQuery query(db);
	doQuery(query, syncOff);
}


bool LocationDb::doQuery(QSqlQuery query, QString queryString){
	bool success = query.exec(queryString);

	if(!success){
		qWarning() << "SqlQuery failed[type = " << query.lastError().type() << " text=" << query.lastError().text() << "] Offending string[" << queryString << "]";
	}
	Q_ASSERT_X(success, "sql query error", query.lastError().text().toAscii().data());

	return success;
}

QList<TrackFilter*> LocationDb::getFilters(int level){
	QString selectStr;
	QTextStream stringStream(&selectStr);

	stringStream << "SELECT level,lat,lon,utm_x,utm_y,hdop,timestamp_start,timestamp_end FROM filters WHERE level>=" << level;

	QSqlQuery query(db);
	doQuery(query, selectStr);

	QList<TrackFilter*> filterList;

	while(query.next()){
		TrackFilter* filter = new TrackFilter;

		filter->level = query.value(0).toInt();
		filter->latitude = query.value(1).toDouble();
		filter->longitude = query.value(2).toDouble();
		filter->utm_x = query.value(3).toDouble();
		filter->utm_y = query.value(4).toDouble();
		filter->hdop = query.value(5).toDouble();
		filter->timestamp_start = query.value(6).toDateTime();
		filter->timestamp_end = query.value(7).toDateTime();
	}

	return filterList;
}

QList<TrackFilter*> LocationDb::getFilters(QDateTime start, QDateTime end, int level){
	QString selectStr;
	QTextStream stringStream(&selectStr);

	qint64 startMs = start.toUTC().toMSecsSinceEpoch();
	qint64 endMs = end.toUTC().toMSecsSinceEpoch();

	stringStream << "SELECT level,lat,lon,utm_x,utm_y,hdop,timestamp_start,timestamp_end FROM filters"
				 << " WHERE level>=" << level
				 << " AND ((timestamp_start>=" << startMs << " AND timestamp_end<=" << startMs <<")"
				 << " OR (timestamp_start>=" << endMs << " AND timestamp_end<=" << endMs <<"))";


	QSqlQuery query(db);
	doQuery(query, selectStr);

	QList<TrackFilter*> filterList;

	while(query.next()){
		TrackFilter* filter = new TrackFilter;

		filter->level = query.value(0).toInt();
		filter->latitude = query.value(1).toDouble();
		filter->longitude = query.value(2).toDouble();
		filter->utm_x = query.value(3).toDouble();
		filter->utm_y = query.value(4).toDouble();
		filter->hdop = query.value(5).toDouble();
		filter->timestamp_start = query.value(6).toDateTime();
		filter->timestamp_end = query.value(7).toDateTime();
	}

	return filterList;
}

void LocationDb::removeFilterById(quint64 filterId){
	QString queryStr;
	QTextStream stringStream(&queryStr);


	stringStream << "DELETE FROM filters WHERE id="<<filterId;

	QSqlQuery query(db);
	doQuery(query, queryStr);
}

void LocationDb::removeFilters(int level){
	QString queryStr;
	QTextStream stringStream(&queryStr);


	stringStream << "DELETE FROM filters WHERE level>="<<level;

	QSqlQuery query(db);
	doQuery(query, queryStr);
}

void LocationDb::removeFilters(QDateTime start, QDateTime end, int level){
	QString queryStr;
	QTextStream stringStream(&queryStr);

	qint64 startMs = start.toUTC().toMSecsSinceEpoch();
	qint64 endMs = end.toUTC().toMSecsSinceEpoch();

	stringStream << "DELETE FROM filters WHERE level>="<<level
				 << " AND ((timestamp_start>=" << startMs << " AND timestamp_end<=" << startMs <<")"
				 << " OR (timestamp_start>=" << endMs << " AND timestamp_end<=" << endMs <<"))";

	QSqlQuery query(db);
	doQuery(query, queryStr);
}

void LocationDb::addFilter(TrackFilter& filterPoint){
	qint64 startMs = filterPoint.timestamp_start.toUTC().toMSecsSinceEpoch();
	qint64 endMs = filterPoint.timestamp_end.toUTC().toMSecsSinceEpoch();
	QString queryString;
	QTextStream stringStream(&queryString);

	stringStream.setRealNumberPrecision(22);

	stringStream << "INSERT INTO filters(level,lat,lon,utm_x,utm_y,hdop,timestamp_start,timestamp_end) VALUES("
				 << filterPoint.level << ", "
				 << filterPoint.latitude << ", "
				 << filterPoint.longitude << ", "
				 << filterPoint.utm_x << ", "
				 << filterPoint.utm_y << ", "
				 << filterPoint.hdop << ", "
				 << startMs << ", "
				 << endMs << ", "
				 << ")";

	QSqlQuery query(db);
	doQuery(query, queryString);
}

void LocationDb::addFilter(TrackPoint& point, QDateTime start, QDateTime end, int level){
	qint64 startMs = start.toUTC().toMSecsSinceEpoch();
	qint64 endMs = end.toUTC().toMSecsSinceEpoch();
	QString queryString;
	QTextStream stringStream(&queryString);

	stringStream.setRealNumberPrecision(22);

	stringStream << "INSERT INTO filters(level,lat,lon,utm_x,utm_y,hdop,timestamp_start,timestamp_end) VALUES("
				 << level << ", "
				 << point.latitude() << ", "
				 << point.longitude() << ", "
				 << point.utmX() << ", "
				 << point.utmY() << ", "
				 << point.hdop() << ", "
				 << startMs << ", "
				 << endMs << ", "
				 << ")";

	QSqlQuery query(db);
	doQuery(query, queryString);
}

void LocationDb::addFilters(QList<TrackFilter*> filterPointList){
	for(int i=0; i<filterPointList.size(); i++){
		addFilter(*filterPointList.at(i));
	}
}

void LocationDb::addFilters(TrackSegment& segment, QDateTime start, QDateTime end, int level){
	for(int i=0; i<segment.points.size(); i++){
		addFilter(*segment.points.at(i), start, end, level);
	}
}

bool LocationDb::addLocation(TrackPoint& point, bool filter, quint64 deviceId){
	if(filter){
		// TODO: Filter against db
		qWarning() << "addLocation() - Filtered location insert requested, feature not yet supported.";
	}

	qint64 timestampMS = point.time().toUTC().toMSecsSinceEpoch();
	QString queryString;
	QTextStream stringStream(&queryString);

	stringStream.setRealNumberPrecision(22);

	stringStream << "INSERT INTO locations(device_id,timestamp,source,lat,lon,utm_x,utm_y,utm_zone,hdop,elevation,course,speed,sats) VALUES("
				 << deviceId << ", "
				 << timestampMS << ", "
				 << (int)point.source() << ", "
				 << point.latitude() << ", "
				 << point.longitude() << ", "
				 << point.utmX() << ", "
				 << point.utmY() << ", "
				 << "'" << point.utmZone() << "', "
				 << point.hdop() << ", "
				 << point.elevation() << ", "
				 << point.course() << ", "
				 << point.speed() << ", "
				 << point.satelites()
				 << ")";

	QSqlQuery query(db);
	return doQuery(query, queryString);
}


bool LocationDb::addLocations(TrackDocument& document, bool filter){
	if(filter){
		// TODO: Filter against db
	}

	bool success = true;

	for(int i=0; i<document.segments.count(); i++){
		TrackSegment* segment = document.segments[i];
		for(int j=0; j<segment->points.count(); j++){
			success &= addLocation(*segment->points.at(j), filter, document.deviceId);
		}
	}

	return success;
}

bool LocationDb::constructFiltersTable(){
	QSqlQuery query(db);
	QString queryStr = "CREATE TABLE filters (id INTEGER PRIMARY KEY AUTOINCREMENT, "
					   "level int, "
					   "lat real, "
					   "lon real, "
					   "utm_x real, "
					   "utm_y real, "
					   "hdop real, "
					   "timestamp_start int, "
					   "timestamp_end int)";

	return doQuery(query, queryStr);
}


bool LocationDb::constructLocationsTable(){
	QSqlQuery query(db);
	QString queryStr = "CREATE TABLE locations (id INTEGER PRIMARY KEY AUTOINCREMENT, "
					   "device_id int, "
					   "timestamp int, "
					   "source int, "
					   "lat real, "
					   "lon real, "
					   "utm_x real, "
					   "utm_y real, "
					   "utm_zone char(2), "
					   "hdop real, "
					   "elevation real, "
					   "course real, "
					   "speed real, "
					   "sats real)";

	return doQuery(query, queryStr);
}
