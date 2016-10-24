#ifndef ARCASCIIPARSER_H
#define ARCASCIIPARSER_H

#include <QtCore>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct LLA {
  double latitude;
  double longitude;
  double altitude;

  LLA(double lat=0.0, double lon=0.0, double alt=0.0);
};

struct Point3D {
  double x;
  double y;
  double z;

  Point3D();
  Point3D(LLA latLong);
};

struct ArcAsciiHeader {
    qint16 width;
    qint16 height;
    qreal xCorner;
    qreal yCorner;
    qreal cellSize;
    qint16 noData;

    LLA rowColToLLA(int row, int col);
};

class ArcAsciiData : public QObject {
  Q_OBJECT

  public:
    explicit ArcAsciiData(QObject* parent = 0);

    ArcAsciiHeader header;
    QFile* file;
    QFileInfo info;
    pcl::PointCloud<pcl::PointXYZ> cloud;
};

class ArcAsciiParser : public QObject
{
    Q_OBJECT
    public:
        explicit ArcAsciiParser(QObject* parent = 0);

        bool queueFile(QString file);
        bool parsing();

    protected:
        /**
         * @brief   Parses all file data
         * @param   File to read
         * @return
         */
        ArcAsciiData* loadData(QFileInfo file);

        bool loadHeader(ArcAsciiData* data);
        bool loadPointCloud(ArcAsciiData* data);

    signals:
        void dataReady(ArcAsciiData* data);
        void totalProgress(qreal percent);
        void currentFileProgress(qreal percent, ArcAsciiData* data);
        void parsingFile(QString file);
        void parseError(QString file);
        void finished();

    public slots:
        void startParsing();
        void stopParsing();

    private:
        bool _stopped;
        bool _parsing;
        bool _loadElevation;
        QList<QFileInfo> _filesToParse;
};

#endif // ARCASCIIPARSER_H
