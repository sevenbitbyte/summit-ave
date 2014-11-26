#include <QtCore>
#include <QCoreApplication>

#include <QtSql>
#include <QDateTime>
#include <QCryptographicHash>
#include <QHostInfo>

#define MIN_FILE_SIZE (512*1024)

QString cleanPath(QString str){
	return str.replace("'", "''");
}


class ChecksumDb : public QObject {
		
	public:
		QString host;
		
		ChecksumDb(QString dbPath, QObject* parent=NULL) : QObject(parent){
			//db = new QSqlDatabase("QSQLITE");
			db = QSqlDatabase::addDatabase(QString("QSQLITE"));
			db.setDatabaseName(dbPath);
			
			
			
			if(!db.open()){
				qErrnoWarning("Failed to open database");
				return;
			}
		
			QStringList tables = db.tables(QSql::Tables);
			
			if(!tables.contains("checksums")){
				//Recreate checksums table
				QSqlQuery query(db);
				
				bool success = query.exec("CREATE TABLE checksums (id INTEGER PRIMARY KEY AUTOINCREMENT, "
													"hash char(32), "
													"size int,"
													"timestamp int, "
													"host varchar(64), "
													"path varchar(1024))");
				
				if(!success){
					qCritical("Creating checksums table failed");
				}
			}

			if(!tables.contains("duplicates")){
				//Recreate duplicates table
				QSqlQuery query(db);
				
				bool success = query.exec("CREATE TABLE duplicates (id INTEGER PRIMARY KEY AUTOINCREMENT, hash char(32), count int, start int, end int)");
				
				if(!success){
					qCritical("Creating duplicates table failed");
				}
			}
		}
		
		~ChecksumDb(){
			
		}

		void doQuery(QSqlQuery& query, QString queryStr){
			//query.prepare(queryStr);
			if(!query.exec(queryStr)){
				throw query;
			}
		}
		
		bool isKnownByHashPath(QString hash, QString path){
			QSqlQuery query(db);
			QString selectStr = "SELECT id FROM checksums WHERE hash='"+hash+"' AND host='"+host+"' AND path='"+path+"'";
			
			doQuery(query, selectStr);
			return (query.size() > 0);
		}
		
		bool isKnownFile(QString path){
			QSqlQuery query(db);
			QString selectStr = "SELECT id FROM checksums WHERE host='"+host+"' AND path='"+path+"'";
			
			doQuery(query, selectStr);
			return query.next();
		}
		
		bool insertSum(QString hash, QString path, QDateTime timestamp){

			if(!isKnownFile(path)){
				
				qint64 timestampMs = timestamp.toUTC().toMSecsSinceEpoch();
				QString queryString;
				QTextStream stringStream(&queryString);
				
				stringStream << "INSERT INTO checksums(hash,timestamp,host,path) VALUES('"
						  << hash << "',"
						  << timestampMs << ",'" 
						  << host << "','" 
						  << path+"')";
				
				QSqlQuery query(db);
				doQuery(query, queryString);
				
				//return true;
			}
			else{
				QString lastHash;
				QSqlQuery query;
				
				//Get last hash
				if(getFileInfo(query, host, path, "hash")){ 
					lastHash = query.value(0).toString();
				}
				
				
				//Update existing record
				qint64 timestampMs = timestamp.toUTC().toMSecsSinceEpoch();
				QString queryString;
				QTextStream stringStream(&queryString);
				
				stringStream << "UPDATE	checksums SET timestamp=" << timestampMs << 
								",hash='" << hash << "' WHERE path='" << path << "' AND host='" << host << "'";
				

				
				doQuery(query, queryString);
				
				if(lastHash.size() > 0){
					//Recount number of old hashes
					insertDuplicate(lastHash);
				}
			}
			
			insertDuplicate(hash);
			
			return false;
		}
		
		bool shouldUpdate(QFileInfo& info){
			QSqlQuery query;
			
			if(getFileInfo(query, host, cleanPath(info.absoluteFilePath()), "timestamp")){

				qint64 timestampMs = query.value(0).toLongLong();
				quint64 currentTimestampMs = info.lastModified().toUTC().toMSecsSinceEpoch();
				
				if(timestampMs < currentTimestampMs){
					return true;
				}
				
				return false;
			}
			
			return true;
		}
		
        bool getFileInfo(QSqlQuery& query, QString host, QString path, QString fields=QString("*")){
			QString queryStr = "SELECT "+fields+" FROM checksums WHERE host='" +host+ "' AND path='" +path+ "'";
			doQuery(query, queryStr);
			
			if(!query.isActive()){
				qWarning() << "Error: " << query.lastError().text() << " while executing " << query.lastQuery();
				return false;
			}
			
			return query.first();
		}
		
		QStringList getDuplicatePaths(QString hash){
			QString selectStr = "SELECT host,path FROM checksums WHERE hash='" +hash+ "'";
			
			QSqlQuery query;
			doQuery(query, selectStr);
			
			QStringList duplicateFiles;
			while(query.next()){
				QString host = query.value(0).toString();
				QString path = query.value(1).toString();
				
				duplicateFiles.push_back(host + ":" + path);
			}
			
			return duplicateFiles;
		}
		
		QDateTime lastHashed(QString path){
			QSqlQuery query;
			QString selectStr = "SELECT timestamp FROM checksums WHERE path='" +path+ "' AND host='" +host+ "'";
			
			doQuery(query, selectStr);
			
			int timestampMs = query.value(0).toInt();
			
			return QDateTime::fromMSecsSinceEpoch(timestampMs);
		}
		
		int duplicateCount(QString hash){
			QString selectStr = "SELECT id FROM checksums WHERE hash='"+hash+"'";
			
			QSqlQuery query(db);
			doQuery(query, selectStr);
			
			int count = 0;
			while(query.isActive() && query.next()){
				count++;
			}
			
			return count;
		}
		
		bool isKnownDuplicate(QString hash){
			QString selectStr = "SELECT id FROM duplicates WHERE hash='"+hash+"'";
			
			QSqlQuery query(db);
			doQuery(query, selectStr);
			
			return query.next();
		}
		
		int knownDuplicateCount(QString hash){
			QString selectStr = "SELECT count FROM duplicates WHERE hash='"+hash+"'";
			
			QSqlQuery query(db);
			doQuery(query, selectStr);
			
			if(query.isActive() && query.first()){
				return query.value(0).toInt();
			}
			
			return 0;
		}
		
		void insertDuplicate(QString hash){
			int knownCount = knownDuplicateCount(hash);
			int count = duplicateCount(hash);
			
			
			if(knownCount != count && knownCount > 0){
				//Update current row
				QSqlQuery query;
				QString queryString;
				QTextStream stringStream(&queryString);
				
				stringStream << "UPDATE	duplicates SET hash='" << hash << "',count=" << count << " WHERE hash='" << hash << "'";
								
				doQuery(query, queryString);
			}
			else if(knownCount == 0 && count > 1){
				//Uknown duplicate insert new row
				QSqlQuery query;
				QString queryString;
				QTextStream stringStream(&queryString);
				
				stringStream << "INSERT INTO duplicates(hash,count) VALUES('" << hash << "'," << count << ")";
								
				doQuery(query, queryString);
			}
		}

	private:
		QSqlDatabase db;
};



QString hashFile(QString path){
	QFile file(path);
	file.open(QIODevice::ReadOnly);
	
	const int block_size = (file.size() > 1024*1024) ? 10*1024 : 1024;
	char buffer[block_size];
	int bytes_read;

	QCryptographicHash hash(QCryptographicHash::Md5);
	
	while( (bytes_read = file.read(buffer, block_size)) > 0 ){
		  hash.addData(buffer, bytes_read);
	}
	
	return hash.result().toHex();

}

class Duplocator : public QObject {
   
    public:
		Duplocator(QObject* parent=NULL) : QObject(parent) {
			//value = (unsigned long) parent;
			QString path = QString(/*QCoreApplication::applicationDirPath() + QDir::separator() +*/ "duplocator.db.sqlite");

			qDebug() << path;
			
			sumDb = new ChecksumDb(path, parent);
			//ChecksumDb sum(QCoreApplication::applicationDirPath() + QDir::separator() + "duplocator.sqlite");
		}
		
		~Duplocator(){
			qDebug("destructor");
		}
		
    /*public slots:
        void run(){
			//
			
			
			
			//QCryptographicHash hash;
			//hash.result().toHex()s
			
			//QString path = QCoreApplication::applicationDirPath();
			
			
			
			emit finished();
        }
		
	signals:
		void finished();
		*/
	private:
		ChecksumDb* sumDb;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    
	//QString databasePath( (argc>2) ? argv[1] : "duplocator.sqlite");
	QString databasePath("duplocator.sqlite");
	
	QDir dir = QDir((argc>1) ? argv[1] : QDir::currentPath());
	ChecksumDb checksumDb(databasePath, &a);
	checksumDb.host = QHostInfo::localHostName();
	
	QFileInfoList files;
	QFileInfoList dirs;
	dirs.push_back( QFileInfo(dir.path()) );

	qDebug() << "Hostname: " << checksumDb.host;

	while(!files.empty() || !dirs.empty()){
		qDebug() << "Files.size:" << files.size() <<" Dirs.size:" << dirs.size();
		if(dirs.size() > 0){
			QFileInfo currentDir = dirs.takeFirst();
			dir = QDir(currentDir.absoluteFilePath());	
	
			qDebug() << "Updating dir:" << currentDir.absoluteFilePath();
			
			files.append(dir.entryInfoList(QDir::Files|QDir::NoSymLinks|QDir::NoDotAndDotDot));
			dirs.append(dir.entryInfoList(QDir::Dirs|QDir::NoSymLinks|QDir::NoDotAndDotDot));
		}
		
		while(!files.empty()){
			QFileInfo currentFile = files.takeLast();

			qDebug() << "Testing file " << currentFile.absoluteFilePath();
			if(currentFile.size() > MIN_FILE_SIZE && checksumDb.shouldUpdate(currentFile)){
				QString hash = hashFile(currentFile.absoluteFilePath());
				qDebug() << "\t\tHash:"<<hash;
				
				checksumDb.insertSum(hash, cleanPath(currentFile.absoluteFilePath()), currentFile.lastModified());
			}
			else{
				qDebug() << "\t\tSkipping";
			}
		}
	}
	
	//QObject::connect(&duptask, SIGNAL(finished()), &a, SLOT(quit()));
	//QTimer::singleShot(0, &duptask, SLOT(run()));
	
    //return a.exec();
}
