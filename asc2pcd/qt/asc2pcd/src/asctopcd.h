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

  public slots:
    void writePCD(ArcAsciiData* data);
    void reportError(QString error);
    void reportParsingStart(QString file);
    void parserDone();
    void reportTotalProgress(qreal percent);
    void reportCurrentFileProgress(qreal percent, ArcAsciiData* data);

  private:
    ArcAsciiParser* parser;
};

#endif // ASCTOPCD_H
