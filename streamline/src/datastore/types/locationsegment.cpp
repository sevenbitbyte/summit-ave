#include "locationsegment.h"

LocationSegment::LocationSegment()
{
    initValues();
}


Datum* LocationSegment::alloc(){
    return new LocationSegment();
}


QString LocationSegment::getTypeName() const {
    return "LocationSegment";
}


QString LocationSegment::getFieldName(int index) const {
    switch(index){
        case StartTime_Pos:
            return QString("start_timestamp");
        case EndTime_Pos:
            return QString("end_timestamp");
        case DeviceId_Pos:
            return QString("device_id");
        case Bounds_Pos:
            return QString("bounds");
        case SourceFileId_Pos:
            return QString("source_file_id");
        default:
            break;
    }

    return QString("");
}


QVariant::Type LocationSegment::getFieldType(int index) const {
    switch(index){
        case StartTime_Pos:
        case EndTime_Pos:
        case DeviceId_Pos:
        case SourceFileId_Pos:
            return QVariant::LongLong;
        case Bounds_Pos:
            return QVariant::String;
        default:
            break;
    }

    return QVariant::Invalid;
}


int LocationSegment::deviceId() const {
    QVariant* devIdVariant = getValue(DeviceId_Pos);

    return devIdVariant->toInt();
}


void LocationSegment::setDeviceId(int id){
    QVariant* devIdVariant = new QVariant( (qlonglong) id);

    setValue( (int) DeviceId_Pos, devIdVariant);
}


QDateTime LocationSegment::startTime() const {
    QVariant* timeVariant = getValue(StartTime_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void LocationSegment::setStartTime(const QDateTime& time){
    QVariant* timeVariant = new QVariant( (qlonglong) time.toMSecsSinceEpoch() );

    setValue( (int) StartTime_Pos, timeVariant);
}


QDateTime LocationSegment::endTime() const {
    QVariant* timeVariant = getValue(EndTime_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void LocationSegment::setEndTime(const QDateTime& time){
    QVariant* timeVariant = new QVariant( (qlonglong) time.toMSecsSinceEpoch() );

    setValue( (int) EndTime_Pos, timeVariant);
}


QPolygonF LocationSegment::bounds() const {
    QVariant* boundsVariant = getValue(Bounds_Pos);

    QByteArray boundsByteArry = QByteArray::fromBase64( boundsVariant->toString().toLatin1() );
    QDataStream boundsDataStream(&boundsByteArry, QIODevice::ReadWrite);

    QPolygonF boundsPolygon;
    boundsDataStream >> boundsPolygon;

    return boundsPolygon;
}


void LocationSegment::setBounds(const QPolygonF& bound){
    QByteArray boundsByteArray;
    QDataStream boundsDataStream(&boundsByteArray, QIODevice::ReadWrite);

    boundsDataStream << bound;

    QVariant* boundsVariant = new QVariant( (const char*) boundsByteArray.toBase64().data() );
    setValue( (int) Bounds_Pos, boundsVariant );
}


void LocationSegment::setBounds(const QRectF& bound){
    QPolygonF boundsPoly(bound);
    setBounds(boundsPoly);
}


void LocationSegment::setBounds(const QVector<QPointF>& bound){
    QPolygonF boundsPoly(bound);
    setBounds( boundsPoly.boundingRect() );
}


qlonglong LocationSegment::sourceFileId() const {
    QVariant* fileIdVariant = getValue(SourceFileId_Pos);

    return fileIdVariant->toLongLong();
}


void LocationSegment::setSourceFileId(qlonglong id){
    QVariant* fileIdVariant = new QVariant( (qlonglong) id );

    setValue( (int) SourceFileId_Pos, fileIdVariant );
}
