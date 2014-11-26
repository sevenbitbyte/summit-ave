#ifndef TRACKPOINT_H
#define TRACKPOINT_H

#include <QtCore>
#include <QObject>
#include <QtXml>

struct TrackFilter{
	int level;
	double latitude;
	double longitude;
	double utm_x;
	double utm_y;
	double hdop;
	QDateTime timestamp_start;
	QDateTime timestamp_end;
};

class TrackPoint : public QObject
{
		Q_OBJECT
		Q_PROPERTY(double latitude READ latitude WRITE setLatitude)
		Q_PROPERTY(double longitude READ longitude WRITE setLongitude)
		Q_PROPERTY(double elevation READ elevation WRITE setElevation)
		Q_PROPERTY(double course READ course WRITE setCourse)
		Q_PROPERTY(double speed READ speed WRITE setSpeed)
		Q_PROPERTY(double hdop READ hdop WRITE setHdop)
		Q_PROPERTY(TrackSource source READ source WRITE setSource)
		Q_PROPERTY(int satelites READ satelites WRITE setSatelites)
		Q_PROPERTY(QDateTime time READ time WRITE setTime)
		Q_ENUMS(TrackSource)

	public:
		TrackPoint(QObject *parent = 0);

		static TrackPoint* parseXml(QDomNode& trkPtNode);

		double latitude() const;
		void setLatitude(double latitude);

		double longitude() const;
		void setLongitude(double longitude);

		double elevation() const;
		void setElevation(double elevation);

		bool filtered() const;
		void setFiltered(bool value);

		double course() const;
		void setCourse(double course);

		double speed() const;
		void setSpeed(double speed);

		double hdop() const;
		void setHdop(double hdop);

		enum TrackSource{ Source_Unknown=0, Source_Gps, Source_Network };
		TrackSource source() const;
		void setSource(TrackSource source);

		int satelites() const;
		void setSatelites(int satelites);

		QDateTime time() const;
		void setTime(const QDateTime& time);

		int getDeviceId() const;
		void setDeviceId(int id);

		double utmX() const;
		double utmY() const;
		QString utmZone() const;

		/**
		 * @brief Computes the UTM(x,y,zone) data from the latitude and longitude
		 */
		void updateUTM();

		/**
		 * @brief Computes the latitude and longitude from the UTM(x,y,zone) data
		 */
		void updateLL();

	private:
		double _latitude;
		double _longitude;
		double _utm_x;
		double _utm_y;
		QString _utm_zone;
		double _elevation;
		double _course;
		double _speed;
		double _hdop;
		TrackSource _source;
		int _satelites;
		QDateTime _time;
		int _device_id;
		bool _filtered;
};

/*Q_DECLARE_METATYPE(TrackPoint*);
Q_DECLARE_METATYPE(QList<TrackPoint*>);*/

class TrackSegment : public QObject {
	Q_OBJECT
	//Q_PROPERTY(QList points READ points WRITE setPoints)


	public:
		TrackSegment(QObject* parent = 0);
		~TrackSegment();

		static TrackSegment* parseNode(QDomNode& node);
		static QList<TrackSegment*> parseXml(QString documentPath);

		void filterSegment(const QList<TrackPoint*> filterPoints);



		/*void setPoints(const QList<TrackPoint> pointList);
		QList<TrackPoint> points() const;

	private:*/
		QList<TrackPoint*> points;
        QDateTime start;
        QDateTime end;
};




#endif // TRACKPOINT_H
