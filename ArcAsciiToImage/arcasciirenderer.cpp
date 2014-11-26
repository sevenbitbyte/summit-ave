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

    _scaleFactor = xScale;

    if(yScale < xScale){
        _scaleFactor = yScale;
    }

    qreal renderWidth = widthPx * _scaleFactor;
    qreal renderHeight = heightPx * _scaleFactor;

    qDebug() << "Render Dimensions: { " << renderWidth << ", " << renderHeight << " }";

    _renderSizePx = QSizeF(renderWidth, renderHeight);

    _pixmap = QPixmap((int)renderWidth, (int)renderHeight);
}

QPixmap ArcAsciiRenderer::getPixmap(){
    return _pixmap;
}


void ArcAsciiRenderer::processData(ArcAsciiData* data){
    //Generate image
    int width = data->header.width;
    int height = data->header.height;

    QRect boundsRenderPx = computeBounds(data->header);
    QImage* image = new QImage(width, height, QImage::Format_RGB32);

    if(data->elevation.size() > 0){
        float shift = -_stats.minElevation;
        float divisor = (_stats.maxElevation + shift);
        for(int j=0; j < (width * height); j++){
            QColor color(0,0,0);
            if(data->elevation[j] != data->header.noData){
                float value = ((data->elevation[j] + shift) / divisor);

                if(data->elevation[j] < 300){
                    //120/360 - 240/360
                    qreal maxVal = ((300.0 + shift) / divisor);
                    qreal scaledValue=value/maxVal;
                    qreal hue = (((240.0f-180.0f)/360.0f) * scaledValue) + 180.0f/360.0f;
                    //qreal hue = 360.0/360.0f - (((360.0f-240.0f)/360.0f) * (value/maxVal));


                    color = QColor::fromHsvF(hue, 1.0, 1.0 - (hue * 0.75));
                }
                else{

                    qreal maxVal = ((_stats.maxElevation + shift) / divisor);
                    qreal scaledValue=value/maxVal;
                    qreal hue = 240.0/360.0f - (((260.0f-160.0f)/360.0f) * (value/maxVal));

                    color = QColor::fromHsvF(hue, qPow(1.0-scaledValue, 2), 1.0-(hue * 0.75));

                    //color = QColor::fromHsvF(0.33, 1.0f, value);	//Green

                    //qreal maxVal = ((300.0 + shift) / divisor);
                    //qreal hue = 240.0/360.0f - (((260.0f-160.0f)/360.0f) * (value/maxVal));

                    //color = QColor::fromHsvF(0.0f, 1.0, 1.0-(hue * 0.75));

                    //color = QColor::fromHsvF(0.0, 0.0f, );	//Black
                }

                //color = QColor::fromHsvF(value, value, 1.0);
                //color = QColor::fromHsvF(0.33, 1.0f, value);	//Green
                //color = QColor::fromHsvF(0.0, 0.0f, log(value+.1)+2);	//Black
            }

            image->setPixel( j % width, j / width, color.rgb() );

            if( j % width == 0){
                qreal progressPercent = ((qreal) j) / (qreal)(width * height);
                emit renderProgress( boundsRenderPx, progressPercent );
            }
        }

        QImage scaledImg = image->scaled( boundsRenderPx.width(), boundsRenderPx.height() );

        QPainter painter(&_pixmap);
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
