#ifndef CONTENTMANAGER_H
#define CONTENTMANAGER_H

#include <QtCore>
#include <QList>
#include <QObject>

#include "gps/trackpoint.h"

/*
 *	Types
 *		position	{GPX}
 *		media		{mp4, jpg, png, gif}
 *		log
 *
 */

class TimeLine : public QObject
{

	Q_OBJECT
	public:
		explicit TimeLine(QObject * parent = 0);

};

class ContentManager : public QObject
{
		Q_OBJECT
	public:

		enum ImportStatusCode { FETCHING, ATTRIBUTES, INDEX_COMPLETE, INDEX_FAIL };

		explicit ContentManager(QObject *parent = 0);
		

		static bool isMediaSuffix(QString suffix);
		static bool isPositionSuffix(QString suffix);

		QMultiMap<QUrl, TrackSegment*>	locationData() const;

		//QMultiMap<QString, QUrl> getActiveFiles(QDateTime start, QDateTime end, QList<QString> types, QRectF region, QVector3D viewpoint);
		//QMultiMap<QString, QUrl> getActiveFiles(QDateTime start, QDateTime end);

        QList<TrackSegment*> getLocationByTime(QDateTime start, QDateTime end) const;

	signals:

		/*TODO: Resolve
		 *		One or more operations can be reported at once however the meaning of
		 *	the import status code becomes ambiguous. I think an average(or equaly weighted scale)
		 *	of all mentioned completion types is fair.
		 */

		/**
		 * @brief importStatus
		 *
		 * @param	path		Object being reported about
		 * @param	code		Mask of operations being undertaken on the
		 * @param	percent		Operation percent completed
		 */
		void importStatus(QUrl path, ImportStatusCode code, float percent=1.0f);
		
	public slots:
		void indexFile(QUrl path);
		void indexFiles(QList<QUrl> paths);
		//bool indexDirectory(QList<QUrl> paths, bool recursive=true);

		//bool removeFile(QString);
		//bool removeDirctory(QUrl path, bool recursive=true);

		
	private:
		QMultiMap<QDateTime, QUrl>	_filesByStartTime;
		QMultiMap<QDateTime, QUrl>	_filesByEndTime;
		QMultiMap<QString, QUrl>	_filesByType;


		QMultiMap<QUrl, TrackSegment*>	_locationData;
        QMap<QDateTime, TrackSegment*> _locationStartTimes;
        QMap<QDateTime, TrackSegment*> _locationEndTimes;
		//QMultiMap<QUrl, QPixmap*>		_imageThumb;
		//QMultiMap<QUrl, QPixmap*>		_movieThumb;
		//QMap<QUrl, >
};

#endif // CONTENTMANAGER_H
