#ifndef ARCASCIIPARSER_H
#define ARCASCIIPARSER_H

#include <QtCore>

struct ArcAsciiHeader {
    qint16 width;
    qint16 height;
    qreal xCorner;
    qreal yCorner;
    qreal cellSize;
    qint16 noData;
};

struct ArcAsciiStatistics {
    qint32 maxElevation;
    qint32 minElevation;

    QMap<qint32, qint32> histogram;

    QVariantMap toVariantMap();
    bool loadFromVariant(QVariant variant);
    void merge(ArcAsciiStatistics& other);
};


class ArcAsciiData : public QObject {
    Q_OBJECT

    public:
        explicit ArcAsciiData(QObject* parent = 0);

        ArcAsciiHeader header;
        ArcAsciiStatistics stats;
        QVector<qint16> elevation;
        QFile* file;
        QFileInfo info;
};

class ArcAsciiParser : public QObject
{
    Q_OBJECT
    public:
        explicit ArcAsciiParser(QObject* parent = 0);

        bool queueFile(QString file);
        void setLoadElevation(bool value);
        bool parsing();

    protected:
        /**
         * @brief   Parses at least the 6 line header. If _loadElevation is true
         *          reads in all file data, computes statistics and saves. If
         *          _loadElevation is false attempts to load statistics, if stats
         *          could not be loaded elevation data is read and discarded after
         *          stats are computed.
         * @param   File to read
         * @return
         */
        ArcAsciiData* loadData(QFileInfo file);

        bool loadHeader(ArcAsciiData* data);
        bool loadElevation(ArcAsciiData* data);
        bool loadStatistics(ArcAsciiData* data);

        bool saveStatistics(ArcAsciiData* data);

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
