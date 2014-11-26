#ifndef FILERECORD_H
#define FILERECORD_H

#include "../database.h"
#include <QtCore>

class FileRecord : public Datum
{
    public:
        FileRecord();

        virtual Datum* alloc();
        virtual QString getTypeName() const;
        virtual QString getFieldName(int index) const;
        virtual QVariant::Type getFieldType(int index) const;

        enum FieldPos{
            Size_Pos = 0,
            Perm_Pos = 1,
            Created_Pos = 2,
            Modified_Pos = 3,
            Accessed_Pos = 4,
            FirstCrawl_Pos = 5,
            LastCrawl_Pos = 6,
            HostId_Pos = 7,
            Path_Pos = 8,
            LocalPath_Pos = 9
        };

        qulonglong size() const;
        void setSize(qulonglong size);

        QFile::Permissions permissions() const;
        void setPermissions(QFile::Permissions& perm);

        QDateTime created() const;
        void setCreated(QDateTime& date);

        QDateTime modified() const;
        void setModified(QDateTime& time);

        QDateTime accessed() const;
        void setAccessed(QDateTime& time);

        QDateTime firstCrawl() const;
        void setFirstCrawl(QDateTime& time);

        QDateTime lastCrawl() const;
        void setLastCrawl(QDateTime& time);

        qlonglong hostId() const;
        void setHostId(qlonglong id);

        QUrl path() const;
        void setPath(QUrl& url);

        QString localPath() const;
        void setLocalPath(QString& local);
};

#endif // FILERECORD_H
