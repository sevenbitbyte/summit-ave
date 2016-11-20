#ifndef ASCTOPCD_H
#define ASCTOPCD_H

#include <QtCore>
#include <QObject>

#include "arcasciiparser.h"

class AscToPcd : public QObject
{
    Q_OBJECT
  public:
    explicit AscToPcd(QCoreApplication* parent = 0);
    ~AscToPcd();

  signals:
    void done();

  protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleOrLoad(float leafSize, QFileInfo info, pcl::PointCloud<pcl::PointXYZ>::Ptr input, float leafSizeV=45.0f);

  public slots:
    void writePCD(ArcAsciiData* data);
    void reportError(QString error);
    void reportParsingStart(QString file);
    void parserDone();
    void reportTotalProgress(qreal percent);
    void reportCurrentFileProgress(qreal percent, ArcAsciiData* data);

  private:
    ArcAsciiParser* parser;
    QThread* parserThread;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud5k;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud10k;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud20k;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud100k;
};

#endif // ASCTOPCD_H
