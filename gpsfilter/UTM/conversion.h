//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions
#include <QString>

#ifndef LATLONGCONV
#define LATLONGCONV

void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long, double &UTMNorthing, double &UTMEasting, QString& UTMZone);
void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const QString& UTMZone, double& Lat, double& Long);
char UTMLetterDesignator(double Lat);

class Ellipsoid
{
public:
	Ellipsoid(){};
	Ellipsoid(int Id, const char* name, double radius, double ecc)
	{
		id = Id; ellipsoidName = (char*) name;
		EquatorialRadius = radius; eccentricitySquared = ecc;
	}
	int id;
	char* ellipsoidName;
	double EquatorialRadius; 
	double eccentricitySquared;  
};

#endif
