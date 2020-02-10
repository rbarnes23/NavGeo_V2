#ifndef __GEO_H__
#define __GEO_H__


#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PVector.h>
//#include "Motor.h"

#define ALLOW_DOUBLE_MATH_FUNCTIONS;

class Geo {

  public:
    // Constructor
    Geo(double maxSpeed, double _maxForce, bool displayMsg);

    
    // Methods
    PVector convertV3ftoV2f(PVector convert);
    int radToDeg(double x);
    double degToRad(double x);
    double calculateAcceleration(double distance, int seconds);
    double calculateTimeFromVelocityAndAcceleration(double Vf, double Vi, double acceleration);
    double distanceFromAccelerationAndTime(int seconds, double acceleration);
    PVector convertSphericalToCartesian(PVector latlng);
    PVector convertCartesianToSpherical(PVector cartesian);
    PVector distanceToObject(PVector origin, PVector destination);
    double haversineDistance(PVector latlong1, PVector latlong2);
    double arcLength(PVector latlong1, PVector latlong2);
    double courseTo(double lat1, double long1, double lat2, double long2);  //Courtesty TinyGPS++
    double distanceBetween(double lat1, double long1, double lat2, double long2);  //Courtesty TinyGPS++
    double calculateBearing(PVector latlong1, PVector latlong2);
    int convertBearingToAngle(int bearing);
    PVector calcXYCoordinates(double distance, double bearing);
    PVector calculateCoord(PVector origin, double brng, double arcLength);
    PVector midpoint(PVector latlong1, PVector latlong2);
    double angleBetweenPoints(PVector latlongA, PVector latlongB, PVector latlongC, String angleType);
    double angleBetweenHeadings(int headingBA, int headingBC, String angleType);
    PVector triangulate(PVector latlongA, PVector latlongB, int bearingAC, int bearingBC);
    double gravitationalAcceleration(double latitude, double altitude);
    double convertDistance(double distance, String conversionType);
    //Mapping Stuff
    PVector setMapCenterPoint(PVector cPoint, int zoom, int scale);
    double mercX(double lon, int zoom, int scale);
    double mercY(double lat, int zoom, int scale);
    size_t addPoint(PVector point);
    void follow(PVector position, double speed, double lookAhead);
    PVector closestPointOnPath(PVector currentPos, PVector velocity, int lookAhead);
    PVector getStart();
    int getEnd();
    PVector getPoint(int i);
    std::vector<PVector> getPath();
    PVector getNormalPoint(PVector p, PVector a, PVector b);
    void applyForce(PVector force);
    void seek(PVector target);
    double get_horizontal_distance_cm(const PVector origin, PVector destination);
    double get_bearing_cd(PVector origin,  PVector destination);
    // Converts from WGS84 geodetic coordinates (lat, lon, height)
    // into WGS84 Earth Centered, Earth Fixed (ECEF) coordinates
    // (X, Y, Z)
    void wgsllh2ecef(const PVector &llh, PVector &ecef);

    // Converts from WGS84 Earth Centered, Earth Fixed (ECEF)
    // coordinates (X, Y, Z), into WHS84 geodetic
    // coordinates (lat, lon, height)
    void wgsecef2llh(const PVector &ecef, PVector &llh);
    PVector convertGpsToECEF(double lat, double lon, double alt);
    PVector distanceBearingVector(PVector gpsLocal, PVector gpsRemote);
    uint16_t servoPointCommand(PVector gpsLocal, PVector gpsRemote);
    // LatLonToUTMXY
    // Converts a latitude/longitude pair to x and y coordinates in the
    // Universal Transverse Mercator projection.
    //
    // Inputs:
    //   lat - Latitude of the point, in radians.
    //   lon - Longitude of the point, in radians.
    //   zone - UTM zone to be used for calculating values for x and y.
    //          If zone is less than 1 or greater than 60, the routine
    //          will determine the appropriate zone from the value of lon.
    //
    // Outputs:
    //   x - The x coordinate (easting) of the computed point. (in meters)
    //   y - The y coordinate (northing) of the computed point. (in meters)
    //
    // Returns:
    //   The UTM zone used for calculating the values of x and y.
    int LatLonToUTMXY (double lat, double lon, int zone, double& x, double& y);

    double UTMCentralMeridian(int zone);

    void updatePosition();  //Move position in purepursuit
    void purePursuit(PVector gpsLocal);  //Pure Pursuit Algorithm
    void calculateWheelVelocity( double speed, double direction);
    void calculateMotorSpeeds(double direction, double speed, int source = 0);
    int DecDeg2UTM(double latitude, double longitude, int zone, double &x, double &y);
    double getMaxSpeed();
    void setLookAhead(double lookAhead);
    void setMaxSpeed(double maxSpeed);
    void followPath(PVector position, double speed);
    void followMe(PVector gpsLocal,PVector gpsRemote,double speed);
  private:
    bool _displayMsg;
    PVector _cPos;  //Central position of map
    PVector _position;
    PVector _acceleration;
    PVector _velocity;
    PVector _predictPos;
    std::vector<PVector> _points;
    double _lookAhead = 10;
    double _pathRadius = 5; //meters
    double _maxSpeed;
    double _currSpeed = 0;
    double _maxForce = 1;
    float _distanceBeforeTurn=1;  //IN METERS
#define UTMScaleFactor 0.9996
    /* Ellipsoid model constants (actual values here are for WGS84) */
#define sm_a 6378137.0
#define sm_b 6356752.314
#define sm_EccSquared 6.69437999013e-03

    // Semi-major axis of the Earth, in kilometers.
    // static const double WGS84_A = 6378.1370;
    const double earthRadius = 6378.1370;
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
    const double DEG_TO_RAD_DOUBLE = asin(1) / 90;
    const double RAD_TO_DEG_DOUBLE = 1 / DEG_TO_RAD_DOUBLE;
#endif
    // Centi-degrees to radians
#define DEGX100 5729.57795f
#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
    //#define EARTH_RADIUS  6371e3;
#endif
#define PI_2 (PI*2)
    double ArcLengthOfMeridian(double phi);
    float ArcLengthOfMeridian (float phi);
    void MapLatLonToXY(float phi, float lambda, float lambda0, float &x, float &y);
    void MapLatLonToXY (double phi, double lambda, double lambda0, double &x, double &y);
    static double dDistance( double east1, double north1, double east2, double north2 );
    static double distanceSquared( double east1, double north1, double east2, double north2 );
};
#endif
