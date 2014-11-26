#ifndef DUPLOCATOR_H
#define DUPLOCATOR_H

#include<QtCore>



namespace duplocator
{
	struct FileRecord {
		QDateTime read;
		QDateTime accessed;
		QDateTime modified;
		qint64 size;
		QString path;
		QString host;
		QChar hash[32];
	};


	class FileCrawler : public QThread {
		Q_OBJECT

		public:
			FileCrawler();
			~FileCrawler();

		public slots:
			void crawlRequestedDir(QString crawlPath);
			void crawlRequestedDirs(QList<QString> crawlPaths);

		signals:
			void filesDiscovered(QList<FileRecord*> records);
	};

	class DataStore : public QThread {
			Q_OBJECT

			public:
				DataStore(QString connectStr=QString("duplocator.sqlite"), QString type = QString("QSQLITE"));
				~DataStore();

			public slots:
				void updateRecords(QList<FileRecord*> records);
	};
}

#endif // DUPLOCATOR_H
