#ifndef ARCASCIIRENDERER_H
#define ARCASCIIRENDERER_H

#include <QtCore>
#include <QtGui>
#include "arcasciiparser.h"

class ArcAsciiRenderer : public QObject
{
        Q_OBJECT
    public:
        explicit ArcAsciiRenderer(QList<ArcAsciiData*> headers, QObject *parent = 0);

        QPixmap getPixmap();

    protected:
        QRect computeBounds(ArcAsciiHeader& header);

    signals:
        //void rendering(QString file, QRect bounds);
        void renderProgress(QRect bounds, qreal progress);
        void tileComplete(QRect bounds);

    public slots:
        void processData(ArcAsciiData* data);

    private:
        ArcAsciiStatistics _stats;
        QPixmap _pixmap;

        qreal _pixelSize;
        qreal _scaleFactor;

        QRectF _imageBoundsLL;
        QSizeF _imageSizePx;
        QSizeF _renderSizePx;
};

#endif // ARCASCIIRENDERER_H
