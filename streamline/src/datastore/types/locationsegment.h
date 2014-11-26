#ifndef LOCATIONSEGMENT_H
#define LOCATIONSEGMENT_H

#include <QtCore>

#include "../database.h"

class LocationSegment : public Datum
{
    public:
        LocationSegment();

        virtual Datum* alloc();
        virtual QString getTypeName() const;
        virtual QString getFieldName(int index) const;
        virtual QVariant::Type getFieldType(int index) const;

        enum FieldPos{
            StartTime_Pos = 0,
            EndTime_Pos = 1,
            DeviceId_Pos = 2,
            Bounds_Pos = 3,
            SourceFileId_Pos = 4
        };

        int deviceId() const;
        void setDeviceId(int id);

        QDateTime startTime() const;
        void setStartTime(const QDateTime& time);

        QDateTime endTime() const;
        void setEndTime(const QDateTime& time);

        QPolygonF bounds() const;
        void setBounds(const QPolygonF& bound);
        void setBounds(const QRectF& bound);
        void setBounds(const QVector<QPointF>& bound);

        qlonglong sourceFileId() const;
        void setSourceFileId(qlonglong id);
};

#endif // LOCATIONSEGMENT_H
