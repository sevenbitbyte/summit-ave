#ifndef LOCATIONDB_H
#define LOCATIONDB_H

#include <QtCore>
#include <QObject>
#include <QtSql>

#include "trackpoint.h"

class LocationDb : public QObject
{
		Q_OBJECT
	public:
		LocationDb(QString dbPath, QObject* parent=NULL);

		/**
		 * @brief	Returns a list of filters with a level greater than or equal
		 *			to that specified.
		 * @param	level	Minimum level of filters to return
		 * @return	List of TrackFilter with level defined greater than that specified
		 */
		QList<TrackFilter*> getFilters(int level = -1);

		/**
		 * @brief	Returns a list of filters with start or end values within
		 *			the spefied range and with level defined greater than or
		 *			equal to the specified level
		 * @param	start	Start timestamp of window to select filters from
		 * @param	end		End timestamp of window to select filters from
		 * @param	level	Minimum level of filters to return
		 * @return
		 */
		QList<TrackFilter*> getFilters(QDateTime start, QDateTime end, int level = -1);
		
		/**
		 * @brief	Delete all filters with matching filterId
		 * @param	filterId	Filters with matching id will be deleted from Db
		 */
		void removeFilterById(quint64 filterId);

		/**
		 * @brief	Deletes all filters with level greater than or equal to the
		 *			specified minimum filter level.
		 * @param	level	Mimimum filter level
		 */
		void removeFilters(int level = -1);

		/**
		 * @brief	Deletes all filters with start or end values within
		 *			the spefied range and with level defined greater than or
		 *			equal to the specified level
		 * @param	start	Start timestamp of window to select filters from
		 * @param	end		End timestamp of window to select filters from
		 * @param	level	Minimum level of filters to return
		 */
		void removeFilters(QDateTime start, QDateTime end, int level = -1);


		void addFilter(TrackFilter& filterPoint);
		void addFilter(TrackPoint& point, QDateTime start, QDateTime end, int level=0);

		void addFilters(QList<TrackFilter*> filterPointList);
		void addFilters(TrackSegment& segment, QDateTime start, QDateTime end, int level=0);

		bool addLocation(TrackPoint& point, bool filter=false, quint64 deviceId=0);
		bool addLocations(TrackDocument& document, bool filter=false);

	protected:
		bool doQuery(QSqlQuery query, QString queryString);

		bool constructLocationsTable();
		bool constructFiltersTable();


	signals:
		
	public slots:

	private:
		QSqlDatabase db;
		
};

#endif // LOCATIONDB_H
