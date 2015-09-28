#include "arcasciirenderer.h"

ArcAsciiRenderer::ArcAsciiRenderer(QList<ArcAsciiData*> headers, QObject *parent) :
    QObject(parent)
{

    _pixelSize = 0.0f;

    //Initialize bounds
    qreal imageLeftLL = -9999;
    qreal imageRightLL = -9999;
    qreal imageTopLL = -9999;
    qreal imageBottomLL = -9999;

    //Update bounds
    for(int i=0; i<headers.size(); i++){
        ArcAsciiData* d = headers[i];

        _stats.merge(d->stats);

        qreal left = d->header.xCorner;
        qreal right = left + (d->header.cellSize * (qreal)d->header.width);
        qreal bottom = d->header.yCorner;
        qreal top = bottom + (d->header.cellSize * (qreal)d->header.height);

        if(left < imageLeftLL || imageLeftLL == -9999){
            imageLeftLL = left;
        }

        if(right > imageRightLL || imageRightLL == -9999){
            imageRightLL = right;
        }

        if(top > imageTopLL || imageTopLL == -9999){
            imageTopLL = top;
        }

        if(bottom < imageBottomLL || imageBottomLL == -9999){
            imageBottomLL = bottom;
        }


        if(_pixelSize == 0.0f){
            _pixelSize = d->header.cellSize;
        }
    }

    _imageBoundsLL = QRectF(imageLeftLL, imageTopLL, qAbs(imageLeftLL - imageRightLL), qAbs(imageTopLL - imageBottomLL));


    qreal widthPx = _imageBoundsLL.width() / _pixelSize;
    qreal heightPx = _imageBoundsLL.height() / _pixelSize;


    _imageSizePx = QSizeF(widthPx, heightPx);

    qDebug() << "Real Dimensions: { " << widthPx << ", " << heightPx << " }";

    qreal xScale = 10000.0f / widthPx;
    qreal yScale = 10000.0f / heightPx;


    qreal renderWidth = widthPx;
    qreal renderHeight = heightPx;

    if(xScale > 1.0f && yScale > 1.0f){
        _scaleFactor = 1.0f;
    }
    else{

        _scaleFactor = xScale;

        if(yScale < xScale){
            _scaleFactor = yScale;
        }

        renderWidth = widthPx * _scaleFactor;
        renderHeight = heightPx * _scaleFactor;
    }

    qDebug() << "Render Dimensions: { " << renderWidth << ", " << renderHeight << " }";

    _renderSizePx = QSizeF(renderWidth, renderHeight);

    _pixmap = QPixmap((int)renderWidth, (int)renderHeight);
    _pixmap.fill(QColor(0,0,0,0));
}

QPixmap ArcAsciiRenderer::getPixmap(){
    return _pixmap;
}


void ArcAsciiRenderer::processData(ArcAsciiData* data){
    //Generate image
    int width = data->header.width;
    int height = data->header.height;

    QRect boundsRenderPx = computeBounds(data->header);
    QImage* image = new QImage(width, height, QImage::Format_ARGB32_Premultiplied);

    qDebug() << "Stats.minElevation: " << _stats.minElevation;
    qDebug() << "Stats.maxElevation: " << _stats.maxElevation;

    if(data->elevation.size() > 0){
        float shift = -_stats.minElevation;
        float divisor = (_stats.maxElevation + shift);

        qDebug() << "Shift: " << shift;
        qDebug() << "Divisor: " << divisor;
        qDebug() << "Alpha Channel: " << image->hasAlphaChannel();

        for(int j=0; j < (width * height); j++){
            QColor color(0,0,0,0);
            if(data->elevation[j] != data->header.noData){
                float value = ((data->elevation[j] + shift) / divisor);

                if(value > 1.0f || value < 0.0f){
                    qDebug() << "Out of range, value:" << value;
                    qDebug() << "Elevation: " << data->elevation[j];
                }

                Q_ASSERT(value >= 0.0f);
                Q_ASSERT(value <= 1.0f);

                //color = QColor::fromHsvF(value, value, 1.0);
                //color = QColor::fromHsvF(0.33, 1.0f, value);	//Green
                //color = QColor::fromHsvF(0.0, 0.0f, log(value+.1)+2);	//Black
                color = QColor::fromHsvF(0.0f, 0.0f, value);	//Black
                color.setAlpha(255);
            }

            image->setPixel( j % width, j / width, color.rgba() );

            if( j % width == 0){
                qreal progressPercent = ((qreal) j) / (qreal)(width * height);
                emit renderProgress( boundsRenderPx, progressPercent );
            }
        }

        QImage scaledImg = image->scaled( boundsRenderPx.width(), boundsRenderPx.height() );

        QPainter painter(&_pixmap);
        painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
        painter.drawImage(boundsRenderPx.topLeft().x(), boundsRenderPx.topLeft().y(), scaledImg);
        delete image;
        emit tileComplete( boundsRenderPx );
    }

    delete data;
}


QRect ArcAsciiRenderer::computeBounds(ArcAsciiHeader& header){

    qreal widthLL = header.cellSize * (qreal)header.width;
    qreal heightLL = header.cellSize * (qreal)header.height;

    qreal leftLL = header.xCorner;
    qreal topLL = header.yCorner + heightLL;

    QRectF boundsLL = QRectF(leftLL, topLL, widthLL, heightLL);

    qreal widthPx = boundsLL.width() / _pixelSize;
    qreal heightPx = boundsLL.height() / _pixelSize;


    qreal renderWidth = widthPx * _scaleFactor;
    qreal renderHeight = heightPx * _scaleFactor;

    qDebug() << "Tile Render Dimensions: { " << renderWidth << ", " << renderHeight << " }";

    qreal renderPxX = ((boundsLL.left() - _imageBoundsLL.left()) / _pixelSize) * _scaleFactor;
    qreal renderPxY = ( qAbs(_imageBoundsLL.top() - boundsLL.top()) / _pixelSize) * _scaleFactor;

    qDebug() << "Tile Render Position: {" << renderPxX << ", " << renderPxY << "}";

    return QRect(renderPxX, renderPxY, renderWidth, renderHeight);
}
