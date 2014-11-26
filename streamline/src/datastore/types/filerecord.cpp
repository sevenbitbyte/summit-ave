#include "filerecord.h"

FileRecord::FileRecord()
{
    initValues();
}


Datum* FileRecord::alloc(){
    return new FileRecord();
}


QString FileRecord::getTypeName() const {
    return QString("FileRecord");
}


QString FileRecord::getFieldName(int index) const {
    switch(index){
        case Size_Pos:
            return QString("size");
        case Perm_Pos:
            return QString("perm");
        case Created_Pos:
            return QString("created");
        case Modified_Pos:
            return QString("modified");
        case Accessed_Pos:
            return QString("accessed");
        case FirstCrawl_Pos:
            return QString("first_crawl");
        case LastCrawl_Pos:
            return QString("last_crawl");
        case HostId_Pos:
            return QString("host_id");
        case Path_Pos:
            return QString("path");
        case LocalPath_Pos:
            return QString("local_path");
        default:
            break;
    }

    return QString("");
}


QVariant::Type FileRecord::getFieldType(int index) const {
    switch(index){
        case Size_Pos:
            return QVariant::ULongLong;
        case Perm_Pos:
        case Created_Pos:
        case Modified_Pos:
        case Accessed_Pos:
        case FirstCrawl_Pos:
        case LastCrawl_Pos:
            return QVariant::LongLong;
        case HostId_Pos:
        case Path_Pos:
        case LocalPath_Pos:
            return QVariant::String;
        default:
            break;
    }

    return QVariant::Invalid;
}


qulonglong FileRecord::size() const {
    QVariant* sizeVariant = getValue(Size_Pos);

    return sizeVariant->toULongLong();
}


void FileRecord::setSize(qulonglong size){
    QVariant* sizeVariant = new QVariant( (qulonglong) size );

    setValue( (qulonglong) Size_Pos, sizeVariant );
}


QFile::Permissions FileRecord::permissions() const {

}


void FileRecord::setPermissions(QFile::Permissions& perm){

}


QDateTime FileRecord::created() const {
    QVariant* timeVariant = getValue(Created_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void FileRecord::setCreated(QDateTime& date){
    QVariant* timeVariant = new QVariant( (qlonglong) date.toMSecsSinceEpoch() );

    setValue( (int) Created_Pos, timeVariant);
}


QDateTime FileRecord::modified() const {
    QVariant* timeVariant = getValue(Modified_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void FileRecord::setModified(QDateTime& time){
    QVariant* timeVariant = new QVariant( (qlonglong) time.toMSecsSinceEpoch() );

    setValue( (int) Modified_Pos, timeVariant);
}


QDateTime FileRecord::accessed() const{
    QVariant* timeVariant = getValue(Accessed_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void FileRecord::setAccessed(QDateTime& time) {
    QVariant* timeVariant = new QVariant( (qlonglong) time.toMSecsSinceEpoch() );

    setValue( (int) Accessed_Pos, timeVariant);
}


QDateTime FileRecord::firstCrawl() const {
    QVariant* timeVariant = getValue(FirstCrawl_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void FileRecord::setFirstCrawl(QDateTime& time) {
    QVariant* timeVariant = new QVariant( (qlonglong) time.toMSecsSinceEpoch() );

    setValue( (int) FirstCrawl_Pos, timeVariant);
}


QDateTime FileRecord::lastCrawl() const {
    QVariant* timeVariant = getValue(LastCrawl_Pos);

    return QDateTime::fromMSecsSinceEpoch(timeVariant->toLongLong());
}


void FileRecord::setLastCrawl(QDateTime& time){
    QVariant* timeVariant = new QVariant( (qlonglong) time.toMSecsSinceEpoch() );

    setValue( (int) LastCrawl_Pos, timeVariant);
}


qlonglong FileRecord::hostId() const {

}


void FileRecord::setHostId(qlonglong id){

}


QUrl FileRecord::path() const {

}


void FileRecord::setPath(QUrl& url){

}


QString FileRecord::localPath() const {

}


void FileRecord::setLocalPath(QString& local){

}
