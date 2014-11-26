#include <QtCore>
#include <QtCore/QCoreApplication>
#include <QtXml>


#include <qjson/qobjecthelper.h>
#include <qjson/serializer.h>

#include <iostream>

#include <trackpoint.h>
#include <locationdb.h>
#include <UTM/conversion.h>

using namespace std;
using namespace QJson;


QString getTabs(int count){
	QString outputStr;
	for(int i=0; i<count; i++){
		outputStr.append("\t");
	}
	return outputStr;
}



void printNode(QDomNode& node, int depth=0){
	QString nodeStr;

	if(node.nodeName() == QString("trkpt")){
		TrackPoint* point = TrackPoint::parseXml(node);

		//if(point->hdop > 5.0){
			//cout << point->hdop << endl;
		//}

		delete point;

		return;
	}

	QDomElement element = node.toElement();

	nodeStr.append(element.tagName());

	//if(element.is()){
		//nodeStr.append("=" + element.nodeType());

	//}

	QDomNamedNodeMap nodeMap = element.attributes();
	for(int i=0; i<nodeMap.count(); i++){
		QDomNode attrNode = nodeMap.item(i);
		nodeStr.append(" " + attrNode.nodeName() + "=" + attrNode.nodeValue());
	}


	//cout << getTabs(depth).toAscii().data() << nodeStr.toAscii().data() << endl;

	QDomNodeList children = element.childNodes();
	for(int i=0; i<children.count(); i++){
		QDomNode child = children.at(i);

		if(!child.isText()){
			printNode(child, depth+1);
		}
		else{
			//cout << getTabs(depth+1).toAscii().data() << child.nodeValue().toAscii().data() << endl;
		}
	}
}


bool parseGPXDocument(QString documentPath, TrackDocument* track){
	QDomDocument doc;
	QFile file(documentPath);

	if(file.open(QIODevice::ReadOnly)) {
		if(!doc.setContent(&file)) {
			qCritical() << "Failed to set XML content for file [" << documentPath << "]" ;
			return false;
		}
	}
	else{
		qCritical() << "Failed to open GPX document [" << documentPath << "]";
		return false;
	}


	QDomElement docElem = doc.documentElement();

    QDomNode n = docElem.firstChild();
    while(!n.isNull()) {

		if(n.nodeName() == QString("trk")){
			n = n.firstChild();
		}

		if(n.nodeName() == QString("trkseg")){
			TrackSegment* segment = TrackSegment::parseXml(n);

			if(segment != NULL){
				track->segments.push_back( segment );
			}
			//qDebug() << "Read segment with " << segment->points.count() << " points";
		}
		else{
			//qDebug() << n.nodeName();
		}

		//printNode(n);
        /*QDomElement e = n.toElement(); // try to convert the node to an element.
        if(!e.isNull()) {
            cout << qPrintable(e.tagName()) << endl; // the node really is an element.
		}*/
        n = n.nextSibling();
    }

	return true;
}

struct RunConfig{
	QString locationDbPath;
	QString filterDbPath;
	int deviceId;
	QStringList filterFiles;
	QStringList locationFiles;
};

struct RunState{
	LocationDb* locationDb;
	LocationDb* filterDb;
	QList<QString> parsedFiles;
	QList<TrackDocument*> locationTracks;
	QList<TrackDocument*> filterTracks;
	QDateTime filterStart;
	QDateTime filterEnd;
	int newFilterLevel;
};


void printUsage(int argc, char** argv){
	qErrnoWarning("Usage: %s df [filename]\n"
				  "\t-d [database]\tOpen the specified database\n"
				  "\t-f <gpx_file>\tImport a filter file in GPX format\n"
				  "\t-i [device_id]\tDevice id"
				  "\t"
				  , argv[0]);
}

int main(int argc, char** argv)
{
	RunState state;
	RunConfig config;

	config.deviceId = 0;

	QCoreApplication a(argc, argv);
	
	QStringList argStrings = a.arguments();

	for(int i=0; i<argStrings.length(); i++){
		qDebug() <<"arg[" << i << "] = " << argStrings[i];
	}

	if(argc < 2){
		printUsage(argc, argv);
		a.exit(-1);
	}

	argStrings.pop_front();

	//Parse input arguments
	while(!argStrings.isEmpty()) {
		QString arg = argStrings.first();
		argStrings.pop_front();

		if(arg.startsWith("-d")){
			if(!argStrings.isEmpty()){
				//Read database path
				arg = argStrings.first();
				argStrings.pop_front();

				config.locationDbPath = arg;
			}
			else{
				qCritical() << "No database specified";
				printUsage(argc, argv);
				a.exit(-1);
			}
		}
		else if(arg.startsWith("-i")){
			if(!argStrings.isEmpty()){
				//Read device id
				arg = argStrings.first();
				argStrings.pop_front();

				bool conversionSuccess=true;
				config.deviceId = arg.toInt(&conversionSuccess);

				if(!conversionSuccess){
					qCritical() << "Invalid device id specified";
					printUsage(argc, argv);
					a.exit(-1);
				}
			}
			else{
				qCritical() << "No device id specified";
				printUsage(argc, argv);
				a.exit(-1);
			}
		}
		else if(arg.startsWith("-f")){
			if(!argStrings.isEmpty()){
				//Read filter database path
				arg = argStrings.first();
				argStrings.pop_front();

				config.filterFiles.push_back( arg );
			}
			else{
				qCritical() << "No filter database specified";
				printUsage(argc, argv);
				a.exit(-1);
			}
		}
		else if(arg.startsWith("-")){
			qCritical() << "Unknown command flag [" << arg << "]";
			printUsage(argc, argv);
			a.exit(-1);
		}
		else{
			config.locationFiles.push_back( arg );
		}
	}

	//Setup location database
	if(!config.locationDbPath.isEmpty()) {
		state.locationDb = new LocationDb(config.locationDbPath);
	}

	//Setup filter database
	if(config.filterDbPath == config.locationDbPath) {
		state.filterDb = state.locationDb;
	}
	else if(!config.filterDbPath.isEmpty()) {
		state.filterDb = new LocationDb(config.filterDbPath);
	}


	//Parse filter files
	for(int i=0; i<config.filterFiles.length(); i++){
		TrackDocument* track = new TrackDocument;

		qDebug() << "Parsing filter: " << config.filterFiles[i];
		if(parseGPXDocument(config.filterFiles[i], track)){
			state.filterTracks.push_back(track);
		}
		else{
			delete track;
		}
	}


	if(state.filterDb != NULL){
		//Insert filters into database
		while(!state.filterTracks.isEmpty()){
			TrackDocument* track = state.filterTracks.front();

			for(int i=0; i<i<track->segments.size(); i++){
				state.filterDb->addFilters(*track->segments[i], state.filterStart, state.filterEnd, state.newFilterLevel);
			}

			delete track;
			state.filterTracks.pop_front();
		}
	}


	//Read location files
	for(int i=0; i<config.locationFiles.size(); i++){
		TrackDocument* track = new TrackDocument;
		track->deviceId = config.deviceId;
		//qDebug() << "Parsing location: " << config.locationFiles[i];
		if(parseGPXDocument(config.locationFiles[i], track)){
			if(!state.filterTracks.isEmpty()){
				//Filter track
			}


			if(state.locationDb != NULL){
				//Insert locations into database
				//for(int j=0; j < track->segments.length(); j++){
					state.locationDb->addLocations(*track);
				//}

			}
			else{
				//Filter tracks
				//Output track as JSON object
			}
		}

		delete track;
	}

	//Close database

	/*
	for(int i=1; i < argc; i++){
		QDomDocument doc;
		QFile file(argv[i]);

		if(file.open(QIODevice::ReadOnly)){
			if(doc.setContent(&file)){
				parseDocument(&doc);

				//for(int i = 0; i < gpsDocument.segments[0]->points().count(); i++){
					TrackSegment* segment = gpsDocument.segments[0];

					cout << segment->getJson().toAscii().data();

					//Serializer serializer;
					//QVariantMap variant = QObjectHelper::qobject2qvariant( segment );
					//qDebug() << serializer.serialize( variant );
				//}
			}
			file.close();
		}
	}*/

	//return a.exec();
}
