#include "Geo.h"

Geo::Geo(double maxSpeed, double maxForce, bool displayMsg) {
  _displayMsg = displayMsg;
  _maxSpeed = maxSpeed;
  _maxForce = maxForce;
}
// Get maxSpeed
double Geo::getMaxSpeed() {
  return _maxSpeed;
}

void Geo::setLookAhead(double lookAhead) {
  _lookAhead = lookAhead;
}

void Geo::setMaxSpeed(double maxSpeed) {
  _maxSpeed = maxSpeed;
}

//Convert PVector to PVector
PVector Geo::convertV3ftoV2f(PVector convert) {
  PVector converted(convert.getX(), convert.getY());
  //PVector converted(convert);
  return converted;
}

//Calculate Acceleration i.e. 2(402 meters)/5.5 seconds is time taken=27 m/s squared
double Geo::calculateAcceleration(double distance, int seconds) {
  return (2 * distance) / (seconds ^ 2)  ; //meters per second
}

//Distance to accelerate to a given speed  Vf-Vi (final velocity and initial velocity)/acceleration.  Vi is normally 0
double Geo::calculateTimeFromVelocityAndAcceleration(double Vf, double Vi, double acceleration) {
  return (Vf - Vi) / acceleration  ; //i.e (146.3ms-0 ms)/26.6ms=5.5mseconds
}

//Calcualte distance from Accleration and time 1/2 * 26.6m/s squared * 5.5s squared=402 meters
double Geo::distanceFromAccelerationAndTime(int seconds, double acceleration) {
  return .5 * (acceleration * acceleration) * (seconds ^ 2);
}

//Convert Degrees to Radians
double Geo::degToRad(double x) {
  return x * PI / 180;
}

//Convert Radians to Degrees
int Geo::radToDeg(double x) {
  return x * 180 / PI;
}
//Create a real vector i.e. magnitude , direction based on two latlng
PVector Geo::distanceBearingVector(PVector gpsLocal, PVector gpsRemote) {
  PVector L = gpsLocal;
  PVector R = gpsRemote;
  double dist = haversineDistance(L, R) ;
  double bearing = calculateBearing(L, R) ;
  PVector v(dist, bearing);
  return v;
}


PVector Geo::convertSphericalToCartesian(PVector latlng) {
  double lat = degToRad (latlng.getX());
  double lon = degToRad (latlng.getY());
  double x = earthRadius * cos(lat) * cos(lon);
  double y = earthRadius * cos(lat) * sin(lon);
  double z = earthRadius * sin(lat);
  PVector coord(x, y, z);
  return coord;
}

PVector convertCartesianToSpherical(PVector cartesian)
{
  double r = sqrt(cartesian.getX() * cartesian.getX() + cartesian.getY() * cartesian.getY() + cartesian.getZ() * cartesian.getZ());
  double lat = degrees(asin(cartesian.getZ() / r));
  double lon = degrees(atan2(cartesian.getY(), cartesian.getX()));
  PVector coord(lat, lon, 0);
  return coord;
}

PVector Geo::distanceToObject(PVector origin, PVector destination) {
  // Common values

  double b        = earthRadius + destination.getZ(); //Altitude
  double c        = earthRadius + origin.getZ();

  double b2       = b * b;
  double c2       = c * c;
  double bc2      = 2 * b * c;

  // Longitudinal calculations
  double alpha    = destination.getY() - origin.getY();
  // Conversion to radian
  alpha = alpha * PI / 180;
  // Small-angle approximation
  double cosine      =  cos(alpha);//1 - alpha*alpha/2;
  // Use the law of cosines / Al Kashi theorem
  double x        = sqrt(b2 + c2 - bc2 * cosine);

  // Repeat for latitudinal calculations
  alpha      = destination.getX() - origin.getX();
  alpha      = alpha * PI / 180;
  cosine =  cos(alpha);//1 - alpha*alpha/2;
  double y   = sqrt(b2 + c2 - bc2 * cosine);

  // Obtain vertical difference, too
  double z   = destination.getZ() - origin.getZ();
  PVector ret;
  ret.set(x, y, z);
  printf("distanceToObject:  %.6lf %.6lf %.6lf\n", ret.getX(), ret.getY(), ret.getZ());
  return ret;
}


//Distance based on haversine formula based on 2 coordinates
double Geo::haversineDistance(PVector latlong1, PVector latlong2) {
  double lat1 = degToRad(latlong1.getX());
  double lon1 = degToRad(latlong1.getY());
  double lat2 = degToRad(latlong2.getX());
  double lon2 = degToRad(latlong2.getY());
  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;
  double cordLength = pow(sin(dLat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dLon / 2), 2);
  double centralAngle = 2 * atan2(sqrt(cordLength), sqrt(1 - cordLength));
  return earthRadius * centralAngle;
}

// Calculate Arc length between 2 points  - This should be about same as haverineDistance
double Geo::arcLength(PVector latlong1, PVector latlong2) {
  PVector point1 = convertSphericalToCartesian(latlong1);
  PVector point2 = convertSphericalToCartesian(latlong2);
  double cordLength = sqrt(pow(point1.getX() - point2.getX(), 2) + pow(point1.getY() - point2.getY(), 2) + pow(point1.getZ() - point2.getZ(), 2));
  double centralAngle = 2 * asin(cordLength / 2 / earthRadius);
  return earthRadius * centralAngle;
}

//Convert Navigational Bearing to Trig Angle
int Geo::convertBearingToAngle(int bearing) {
  //To convert nav bearing to trig angle start by moving start to 90
  int a = 90 - bearing;
  // if result is negative add 360
  int angle = ( a > 0 ) ? a : 360 + a;
  // if angle is 360 make it 0 which is the same
  (angle == 360) ? angle = 0 : angle;
  return angle;
}

/* static */
double Geo::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double Geo::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

double Geo::calculateBearing(PVector latlong1, PVector latlong2) {
  double lat1 = degToRad(latlong1.getX());
  double lon1 = latlong1.getY();
  double lat2 = degToRad(latlong2.getX());
  double lon2 = latlong2.getY();
  double dLon = degToRad(lon2 - lon1);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double brng = (radToDeg(atan2(y, x)) + 360) % 360;
  return brng;
}

//Calculate coordinates from distance and bearing
PVector Geo::calcXYCoordinates(double distance, double bearing) {
  double angle = convertBearingToAngle(bearing); //Nav bearing to Angle;
  Serial.println("ANGLE: " + String(angle));
  angle = degToRad(angle);
  PVector pos;
  pos.set(distance * cos(angle), distance * sin(angle));
  printf("XY Vector2 %.2lf %.2lf\n", pos.getX(), pos.getY());
  return pos;
}

//Calculate a new coordinate from an origin
PVector Geo::calculateCoord(PVector origin, double brng, double arcLength) {
  double lat1 = degToRad(origin.getX());
  double lon1 = degToRad(origin.getY());
  double centralAngle = arcLength / earthRadius;
  double lat2 = asin( sin(lat1) * cos(centralAngle) + cos(lat1) * sin(centralAngle) * cos(degToRad(brng)));
  double lon2 = lon1 + atan2(sin(degToRad(brng)) * sin(centralAngle) * cos(lat1), cos(centralAngle) - sin(lat1) * sin(lat2));
  PVector coord;
  coord.set(radToDeg(lat2), radToDeg(lon2));
  return coord;
}

PVector Geo::midpoint(PVector latlong1, PVector latlong2) {
  double arcLength = haversineDistance(latlong1, latlong2);
  double brng = calculateBearing(latlong1, latlong2);
  return calculateCoord(latlong1, brng, arcLength / 2);
}

//Angle between 3 points
double Geo::angleBetweenPoints(PVector latlongA, PVector latlongB, PVector latlongC, String angleType)
{
  double headingBA = calculateBearing(latlongB, latlongA);
  double headingBC = calculateBearing(latlongB, latlongC);

  return angleBetweenHeadings(headingBA, headingBC, angleType);
}

//Angle between 2 headings
double Geo::angleBetweenHeadings(int headingBA, int headingBC, String angleType)
{
  double angle = ((headingBA - headingBC) + 360) % 360;

  if (angleType == "inner")
    if (angle > 180)
      return 360 - angle;
    else
      return angle;

  if (angleType == "outer")
    if (angle < 180)
      return 360 - angle;
    else
      return angle;
}


PVector Geo::triangulate(PVector latlongA, PVector latlongB, int bearingAC, int bearingBC)
{
  double AB = haversineDistance(latlongA, latlongB);
  double bearingAB = calculateBearing(latlongA, latlongB);
  double angleBAC = angleBetweenHeadings(bearingAB, bearingAC, "inner");
  double angleABC = angleBetweenHeadings(180 - bearingAB, bearingBC, "inner");
  double angleBCA = 180 - angleBAC - angleABC;
  double AC = AB * sin(degToRad(angleABC)) / sin(degToRad(angleBCA));
  return calculateCoord(latlongA, bearingAC, AC);
}

//This function will take in a latitude and an altitude in meters. It will return a number representing the gravitational accelerations whose units are in clip_image006
double Geo::gravitationalAcceleration(double latitude, double altitude)
{
  double lat = degToRad(latitude);
  return 9.780327 * (1 + 0.0053024 * pow(sin(lat), 2) - 0.0000058 * pow(sin(2 * lat), 2)) - 0.000003086 * altitude;
}

double Geo::convertDistance(double distance, String conversionType)
{
  //meters to kilometers
  if (conversionType == "m2km" || conversionType == "m2mi" || conversionType == "m2ft")
    distance /= 1000;

  //kilometers to meters
  if (conversionType == "ft2m" || conversionType == "mi2m" || conversionType == "km2m")
    distance *= 1000;

  //feet to miles
  if (conversionType == "ft2mi" || conversionType == "ft2m" || conversionType == "ft2km")
    distance /= 5280;

  //miles to feet
  if (conversionType == "mi2ft" || conversionType == "m2ft" || conversionType == "km2ft")
    distance *= 5280;

  //kilometers to miles
  if (conversionType == "m2mi" || conversionType == "m2ft" || conversionType == "km2ft" || conversionType == "km2mi")
    distance /= 1.609344;

  //miles to kilometers
  if (conversionType == "mi2m" || conversionType == "ft2m" || conversionType == "ft2km" || conversionType == "mi2km")
    distance *= 1.609344;

  return distance;
}

//Set Center point of map based on a coordinate and the zoom and scale of the map
PVector Geo::setMapCenterPoint(PVector cPoint, int zoom, int scale) {
  //double clat = 27.9506;  //EXAMPLES zoom default =10 and scale =256
  //double clon = -82.4572;
  _cPos.set( mercX(cPoint.getY(), zoom, scale), mercY(cPoint.getX(), zoom, scale));
  return _cPos;
}

double Geo::mercX(double lon, int zoom, int scale) {
  lon = radians(lon);
  double a = (scale / PI) * pow(2, zoom);
  double b = lon + PI;
  return a * b;
}

double Geo::mercY(double lat, int zoom, int scale) {
  lat = radians(lat);
  double a = (scale / PI) * pow(2, zoom);
  double b = tan(PI / 4 + lat / 2);
  double c = PI - log(b);
  return a * c;
}
PVector Geo::getPoint(int i) {
  return _points[i];
}

//Distance calcs of all kinds
double Geo::dDistance( double east1, double north1, double east2, double north2 ) {
  return sqrt(
           pow( east1 - east2, 2 )
           + pow( north1 - north2, 2 ) );
}


//not normalized distance, no square root
double Geo::distanceSquared( double east1, double north1, double east2, double north2 ) {
  return pow( east1 - east2, 2 ) + pow( north1 - north2, 2 );
}


size_t Geo::addPoint(PVector point) {
  PVector gpsLocal(27.911081314086914, -82.857466125488281);
  PVector vPoint = convertV3ftoV2f(point);

  double distance, distance2 = 0;
  double heading = 0;
  double bearing = 0;
  int end;
  if (_points.empty()) {
    end = 0;
  } else {
    end = getEnd();
  }
  printf("END %i\n", end);

  double d;
  double x, x1, y, y1;
  int zone;

  if (end == 0) {
    zone = LatLonToUTMXY (gpsLocal.getX(), gpsLocal.getY() , 0,  x,  y);//gpsLocal
    zone = LatLonToUTMXY (point.getX(), point.getY() , 0,  x1,  y1);//Point on Path
    distance = haversineDistance(vPoint, convertV3ftoV2f(gpsLocal));
    //distance = get_horizontal_distance_cm(point, gpsLocal);
    distance2 = arcLength(gpsLocal, point);
    heading = degrees(gpsLocal.angleBetween(gpsLocal, point)) + 90;
    bearing = calculateBearing(gpsLocal, point);
  } else {
    //dist = point.distance(_points[end]);
    heading = degrees(point.angleBetween(point, _points[end - 1])) + 90;
    bearing = calculateBearing(vPoint, _points[end - 1]);
    distance2 = get_horizontal_distance_cm(point, _points[end - 1]);
    distance2 = arcLength(point, _points[end - 1]);
    distance = haversineDistance(vPoint, convertV3ftoV2f(_points[end - 1]));
    zone = LatLonToUTMXY (point.getX(), point.getY() , 0,  x,  y);//gpsLocal
    zone = LatLonToUTMXY (_points[end - 1].getX(), _points[end - 1].getY() , 0,  x1,  y1);//Point on Path
    //bearing=get_bearing_cd(point, _points[end-1]);
  }
  d = dDistance(x, y, x1, y1 );  //UTM Distance between two points
  //d = distanceSquared( x, y, x1, y1 );  This can be used to do dot product as it is faster more relative

  printf( "The point Origin zone is %i and the gUTM Easting is %f and Northing is %f Destination dUTM Easting is %f and Northing is %f\n", zone, x, y, x1, y1);
  printf( "Bearing  %f DistanceHav %f dDistance %lf DistanceCM %f Heading %f\n", bearing, distance, d, distance2, heading);

  point.setZ(bearing);
  _points.push_back(point);
  return _points.size();
}


// Get the first item on the path
PVector Geo::getStart() {
  return _points[0];
}

//Find the last item on the path
int Geo::getEnd() {
  //return (sizeof(_points) / sizeof(_points[0]));
  return _points.size();
}
//Get the current path
std::vector<PVector> Geo::getPath() {
  return _points;
}

// A function to get the normal point from a point (p) to a line segment (a-b)
// This function could be optimized to make fewer new Vector objects
PVector Geo::getNormalPoint(PVector p, PVector a, PVector b) {
  // Vector from a to p
  PVector ap = p.sub(p , a);
  // Vector from a to b
  PVector ab = b.sub(b , a);
  ab.normalize(); // Normalize the line
  // Project vector "diff" onto line by using the dot product
  ab.mult(ab, ap.dot(ab));

  printf("SCALE %lf %lf\n", ab.getX(), ab.getY());
  PVector normalPoint = a.add(a , ab);
  return normalPoint;
}

void Geo::applyForce(PVector force) {
  // We could add mass here if we want A = F / M
  _acceleration = _acceleration.add(_acceleration, force);
}

PVector Geo::closestPointOnPath(PVector currentPos, PVector predictPos, int lookAhead) {
  PVector pop(0, 0, 0);
  PVector normal(0, 0);
  PVector target(0, 0);
  double worldRecord = 1e6;  // Start with a very high record distance that can easily be beaten
  if (getEnd() < 1) {
    return pop;
  }


  // Loop through all points of the path
  for (int i = 0; i < getEnd() - 1; i++) {

    // Look at a line segment
    PVector a(convertV3ftoV2f(getPoint(i)));//Current Point
    PVector b(convertV3ftoV2f(getPoint(i + 1))); //Next Point

    // Get the normal point to that line
    PVector normalPoint = getNormalPoint(predictPos, a, b);
    // This only works because we know our path goes from left to right
    // We could have a more sophisticated test to tell if the point is in the line segment or not
    if (normalPoint.getX() < a.getX() || normalPoint.getX() > b.getX()) {
      // This is something of a hacky solution, but if it's not within the line segment
      // consider the normal to just be the end of the line segment (point b)
      normalPoint = b;
    }

    // How far away are we from the path?
    double distance = predictPos.distanceBetween(predictPos, normalPoint ); // distance is x
    printf("Distance: %10.8lf\n", distance);

    //We need to estimate time to
    double estimated = calculateTimeFromVelocityAndAcceleration(_maxSpeed, _currSpeed, _maxSpeed);
    //If we know the distance and velocity we can calculate acceleration
    double acceleration = calculateAcceleration(distance, estimated);
    printf("Estimated Time and Acceleration: %10.8lf %10.8lf\n", estimated, acceleration);

    // Did we beat the record and find the closest line segment?
    if (distance < worldRecord) {
      worldRecord = distance;
      pop.set(normalPoint.getX(), normalPoint.getY(), i);
      printf("Point On Path: %20.14lf %20.14lf\n", normalPoint.getX(), normalPoint.getY());
    }
  }
  return pop;
}

// return horizontal distance between two positions in cm
double Geo::get_horizontal_distance_cm(PVector origin, PVector destination)
{
  double distance = origin.distanceBetween(origin, destination ); // distance
  return distance;
}

// return bearing in centi-degrees between two positions
double Geo::get_bearing_cd(PVector  origin, PVector destination)
{
  PVector A = destination.sub(destination , origin);
  double bearing = atan2f( A.getY(), A.getX() ) * DEGX100;
  if (bearing < 0) {
    bearing += 36000.0f;
  }
  return bearing;
}

uint16_t Geo::servoPointCommand(PVector gpsLocal, PVector gpsRemote)
/* THis function will take the cordinates of the base station and the target object
    then calculate the angle between then using the inverse tangent function.
*/
{
  //Define Local variables
  double latDiff, lonDiff, theta;
  double servoPosition;

  //Calculate difference
  latDiff = (gpsLocal.getX() - gpsRemote.getX());
  lonDiff = (gpsLocal.getY() - gpsRemote.getY());
  //Calculate angle
  theta =   atan2 (latDiff, lonDiff);
  theta = theta * 180 / 3.145;

  //Print result to console
  printf("latDiff: %.6lf %.6lf %.6lf\n", latDiff, lonDiff, theta);

  //Map the calculated angle into a value the servo will understand
  servoPosition = map(theta, -180, 0, 0, 180);
  servoPosition = constrain(servoPosition, 0, 180);

  //servo1.write(servoPosition);
  //Serial.println("ServoPos " + String(servoPosition, 0));
  return (uint16_t) servoPosition;
}

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
int Geo::LatLonToUTMXY (double lat, double lon, int zone, double & x, double & y) {
  if ( (zone < 1) || (zone > 60) )
    zone = floor((lon + 180.0) / 6) + 1;

  MapLatLonToXY (degToRad(lat), degToRad(lon), UTMCentralMeridian(zone), x, y);

  /* Adjust easting and northing for UTM system. */
  x = x * UTMScaleFactor + 500000.0;
  y = y * UTMScaleFactor;
  if (y < 0.0)
    y = y + 10000000.0;

  return zone;
}

// MapLatLonToXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Transverse Mercator projection.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//    phi - Latitude of the point, in radians.
//    lambda - Longitude of the point, in radians.
//    lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//    x - The x coordinate of the computed point.
//    y - The y coordinate of the computed point.
//
// Returns:
//    The function does not return a value.
void Geo::MapLatLonToXY (float phi, float lambda, float lambda0, float & x, float & y) {
  double N, nu2, ep2, t, t2, l;
  double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
  //double tmp; // Unused

  /* Precalculate ep2 */
  ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);

  /* Precalculate nu2 */
  nu2 = ep2 * pow(cos(phi), 2.0);

  /* Precalculate N */
  N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));

  /* Precalculate t */
  t = tan(phi);
  t2 = t * t;
  //tmp = (t2 * t2 * t2) - pow(t, 6.0); // Unused

  /* Precalculate l */
  l = lambda - lambda0;

  /* Precalculate coefficients for l**n in the equations below
     so a normal human being can read the expressions for easting
     and northing
     -- l**1 and l**2 have coefficients of 1.0 */
  l3coef = 1.0 - t2 + nu2;

  l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

  l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2
           - 58.0 * t2 * nu2;

  l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2
           - 330.0 * t2 * nu2;

  l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

  l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

  /* Calculate easting (x) */
  x = N * cos(phi) * l
      + (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0))
      + (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0))
      + (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

  /* Calculate northing (y) */
  y = ArcLengthOfMeridian (phi)
      + (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0))
      + (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0))
      + (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0))
      + (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));

  return;
}

// UTMCentralMeridian
// Determines the central meridian for the given UTM zone.
//
// Inputs:
//     zone - An integer value designating the UTM zone, range [1,60].
//
// Returns:
//   The central meridian for the given UTM zone, in radians
//   Range of the central meridian is the radian equivalent of [-177,+177].
double Geo::UTMCentralMeridian(int zone) {
  double cmeridian;
  cmeridian = degToRad(-183.0 + ((double)zone * 6.0));

  return cmeridian;
}

// ArcLengthOfMeridian
// Computes the ellipsoidal distance from the equator to a point at a
// given latitude.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//     phi - Latitude of the point, in radians.
//
// Globals:
//     sm_a - Ellipsoid model major axis.
//     sm_b - Ellipsoid model minor axis.
//
// Returns:
//     The ellipsoidal distance of the point from the equator, in meters.
float Geo::ArcLengthOfMeridian (float phi) {
  double alpha, beta, gamma, delta, epsilon, n;
  double result;

  /* Precalculate n */
  n = (sm_a - sm_b) / (sm_a + sm_b);

  /* Precalculate alpha */
  alpha = ((sm_a + sm_b) / 2.0)
          * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

  /* Precalculate beta */
  beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0)
         + (-3.0 * pow(n, 5.0) / 32.0);

  /* Precalculate gamma */
  gamma = (15.0 * pow(n, 2.0) / 16.0)
          + (-15.0 * pow(n, 4.0) / 32.0);

  /* Precalculate delta */
  delta = (-35.0 * pow(n, 3.0) / 48.0)
          + (105.0 * pow(n, 5.0) / 256.0);

  /* Precalculate epsilon */
  epsilon = (315.0 * pow(n, 4.0) / 512.0);

  /* Now calculate the sum of the series and return */
  result = alpha
           * (phi + (beta * sin(2.0 * phi))
              + (gamma * sin(4.0 * phi))
              + (delta * sin(6.0 * phi))
              + (epsilon * sin(8.0 * phi)));

  return result;
}

PVector Geo::convertGpsToECEF(double lat, double lon, double alt) {

  double a = 6378.1;
  double b = 6356.8;
  double N;
  double e = 1 - (pow(b, 2) / pow(a, 2));
  N = a / (sqrt(1.0 - (e * pow(sin(radians(lat)), 2))));
  double cosLatRad = cos(radians(lat));
  double coslonRad = cos(radians(lon));
  double sinLatRad = sin(radians(lat));
  double sinlonRad = sin(radians(lon));
  double x = (N + 0.001 * alt) * cosLatRad * coslonRad;
  double y = (N + 0.001 * alt) * cosLatRad * sinlonRad;
  double z = ((pow(b, 2) / pow(a, 2)) * N + 0.001 * alt) * sinLatRad;
  PVector ecef(x, y, z);
  return ecef;
}

void Geo::followPath(PVector position, double speed) {
  //Get the vector to the 1st point on path
  //seek that vector from gpsLocal
  //When get close to point on path get the next point on path
  //Repeat

  // Loop through all points of the path
  for (int i = 0; i < getEnd() - 1; i++) {
    //Destination to travel to
    PVector destination(getPoint(i));//Current Point
    double distance = distanceBetween(position.getX(), position.getY(), destination.getX(), destination.getY());
    while (distance > _distanceBeforeTurn) {
      //get distance from gpslocal and destination
      distance = distanceBetween(position.getX(), position.getY(), destination.getX(), destination.getY());

      // Get the course from gpsLocal to destination
      double course = radians(courseTo(position.getX(), position.getY(), destination.getX(), destination.getY()));

      //Limits the direction between -180 and 180
      double e = degrees(atan2(sin(course), cos(course)));
      printf("AngleBetween: %f\n", e);
      calculateMotorSpeeds(e, _maxSpeed);
    }
  }
}

void Geo::followMe(PVector gpsLocal,PVector gpsRemote,double speed) {
    //Make this into a task
    //Destination to travel to
      // Get the course from gpsLocal to destination
      double course = radians(courseTo(gpsLocal.getX(), gpsLocal.getY(), gpsRemote.getX(), gpsRemote.getY()));
      //Limits the direction between -180 and 180
      double e = degrees(atan2(sin(course), cos(course)));
      printf("AngleBetween: %f\n", e);
      //Calculate motor speeds and drive to in Motor
      //calculateMotorSpeeds(e, _maxSpeed);
}

void Geo::follow(PVector position, double speed, double lookAhead) {
  _position = convertV3ftoV2f(position);
  _lookAhead = lookAhead;
  // Predict position 50 (arbitrary choice) frames ahead
  // This could be based on speed
  PVector predict = _velocity;
  predict.normalize();
  predict = predict.mult(predict, lookAhead); //look ahead
  PVector predictpos = _position.add(_position, predict);  //Need to add getPosition setPosition
  PVector target = closestPointOnPath(_position, predictpos, _lookAhead);

  printf("ClosestPointonPath: %.14lf  %.14lf %.0lf", target.getX(), target.getY(), target.getZ());
  PVector seekTarget = convertV3ftoV2f(target);
  // Only if the distance is greater than the path's radius do we bother to steer

  seek(seekTarget);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
void Geo::seek(PVector target) {
  printf("Target: %.14lf %.14lf\n", target.getX(), target.getY());

  PVector desired =  desired.sub(_position, target) ; // A vector pointing from the position to the target
  printf("Position: %.14lf %.14lf", _position.getX(), _position.getY());

  printf("Desired: %.14lf %.14lf", desired.getX(), desired.getY());


  // If the magnitude of desired equals 0, skip out of here
  // (We could optimize this to check if x and y are 0 to avoid mag() square root
  if (desired.mag() == 0) return;

  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(desired, _maxSpeed);
  printf("DesiredScaled: %.14lf %.14lf %lf\n", desired.getX(), desired.getY(), _maxSpeed);

  // Steering = Desired minus Velocity
  PVector steer = desired.sub(desired, _velocity);
  steer.limit(_maxForce);  // Limit to maximum steering force
  printf("Steer: %.14lf %.14lf\n", steer.getX(), steer.getY());
  printf("STEER Degrees: %.4lf\n", degrees(steer.getY()));
  //calculateWheelVelocity(desired.getY(), degrees(steer.getY()));
  //calculateWheelVelocity(_maxSpeed, degrees(steer.y));

  calculateMotorSpeeds(degrees(steer.getY()), _maxSpeed);
  applyForce(steer);
}

// Method to update position
void Geo::updatePosition() {
  // Update velocity
  _velocity = _velocity.add(_velocity, _acceleration);
  // Limit speed
  _velocity.limit(_maxSpeed);
  _position = _position.add(_position, _velocity);
  printf("PositionU: %.14lf %.14lf\n", _position.getX(), _position.getY());
  //Serial.println(buff);

  // Reset accelertion to 0 each cycle
  _acceleration = _acceleration.mult(_acceleration, 0);
}

void Geo::purePursuit(PVector gpsLocal) {
  //  config.readFile(SPIFFS, "/path.geojson");
  //PVector pos(0, 0, 0);
  if (gpsLocal.getX() == 0) {
    gpsLocal.set(27.9115465, -82.45734233);
  }
  //follow(gpsLocal, 10, 10);
  followPath(gpsLocal, 10);
}



void Geo::calculateMotorSpeeds(double direction, double speed, int source /*FlySky or actual*/)
{
  if (source == 1) {
    //direction must be between -180 and 180 converted to skyfy ranges of 1000 to 2000
    direction = map(direction, -180, 180, 1000, 2000);
    //speed is between 0 and _maxSpeed converted to skyfy ranges of 1000 to 2000  *****Need to do Something about speed when actual as between 1000 and 1500 is backwards and 1500 to 2000 forward
    speed = map(speed, -_maxSpeed, _maxSpeed, 1000, 2000);
  }
  // OUTPUTS
  double     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
  double     nMotMixR;           // Motor (right) mixed output           (-128..+127)

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  double fPivYLimit = 32.0;

  // TEMP VARIABLES
  double   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  double   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  double   nPivSpeed;      // Pivot Speed                          (-128..+127)
  double   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

  printf("Direction is %f and Speed is %f\n", direction, speed);

  // Map the X and Y axis to positive and negative numbers -  X for direction and Y for speed
  int x = map(direction, 1000, 2000, -128, 128); //map our speed to 0-255 range
  int y = map(speed, 1000, 2000, -128, 128); //map our speed to 0-255 range

  // Map the X and Y axis to positive and negative numbers -  X for direction and Y for speed
  //  int x = map(direction, 1000, 2000, -128, 127); //map our direction to 0-255 range
  //  int y = map(speed, 1000, 2000, -128, 127); //map our speed to 0-255 range
  printf( "x y: % i % i\n", x, y);

  //Serial.println(y);
  // Calculate Drive Turn output due to Joystick X input
  if (y >= 0) {
    // Forward
    nMotPremixL = (x >= 0) ? 127.0 : (127.0 + x);
    nMotPremixR = (x >= 0) ? (127.0 - x) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (x >= 0) ? (127.0 - x) : 127.0;
    nMotPremixR = (x >= 0) ? 127.0 : (127.0 + x);
  }
  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * y / 128.0;
  nMotPremixR = nMotPremixR * y / 128.0;



  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = (int) x;
  double absY = (y > 0) ? (y * 1.0) : (y * -1.0);
  fPivScale = (absY > fPivYLimit) ? 0.0 : (1.0 - absY / fPivYLimit);

  printf("nMotPremixL nMotPremixR, nPivSpeed, fPivScale: % lf % lf % lf %lf\n", nMotPremixL, nMotPremixR, nPivSpeed, fPivScale);

  // Calculate final mix of Drive and Pivot
  nMotMixL = lrint((1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed));
  nMotMixR = lrint((1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed));

  printf("Final nMotMixL nMotMixR, nPivSpeed, fPivYLimit: % lf % lf % lf % lf\n", nMotMixL, nMotMixR, nPivSpeed, fPivScale);

  //Set Motor Direction
  double lm, rm, leftMotorDir, rightMotorDir;
  if (nMotMixL > 0) {
    lm = map(nMotMixL, 1, 128, 1, 255);
    leftMotorDir = 1;
  } else if (nMotMixL < 0) {
    lm = map(abs(nMotMixL), 1, 128, 1, 255);
    leftMotorDir = 2;
  } else {
    lm = 0;
    leftMotorDir = 0;
  }

  if (nMotMixR > 0) {
    rm = map(nMotMixR, 1, 128, 1, 255);
    rightMotorDir = 1;
  } else if (nMotMixL < 0) {
    rm = map(abs(nMotMixR), 1, 128, 1, 255);
    rightMotorDir = 2;
  } else {
    rm = 0;
    rightMotorDir = 0;
  }

  //Make absolutely sure they are in range
  constrain(nMotMixL, 0, 255);
  constrain(nMotMixR, 0, 255);

  struct _motorDrive {
    uint8_t ldir;
    uint8_t lspeed;
    uint8_t rdir;
    uint8_t rspeed;
  } ;
  static _motorDrive mDrive;
  mDrive.ldir = leftMotorDir;
  mDrive.lspeed = lm;
  mDrive.rdir = rightMotorDir;
  mDrive.rspeed = rm;
  printf("mDrive: % i % i % i % i\n", mDrive.ldir, mDrive.lspeed, mDrive.rdir, mDrive.rspeed);
}




//Change this so it takes Current latlng and desired latlng and then creates a vector of magnitude and direction
void Geo::calculateWheelVelocity( double speed, double direction) {
  //direction must be between -90 and 90 degrees
  double R = .127; //10 inch tires in meters
  // Limit to max speed
  speed = (speed > _maxSpeed) ? _maxSpeed : speed;
  double V = speed;//Magnitude Desired Speed
  double O = radians(direction); //Desired Angle
  double L = .6604;  //Wheel Base in meters
  double V_r = ((V) + (O * L)) / (2 * R) * PI_2;
  double V_l = ((V) - (O * L)) / (2 * R) * PI_2;

  //V_l = map(V_l, 0, _maxSpeed, 0, 254);
  //V_r = map(V_r, 0, _maxSpeed, 0, 254);
  //V_l = map(V_l, 0, _maxSpeed * PI_2, 0, 254);
  //V_r = map(V_r, 0, _maxSpeed * PI_2, 0, 254);

  //V_l = constrain(V_l, 0, 254);
  //V_r = constrain(V_r, 0, 254);

  //  double V_r = V + (O * L) / (2 * R);
  //  double V_l = V - (O * L) / (2 * R);
  //double e = atan2(sin(desired.angleBetween(current)), cos(desired.angleBetween(current)));
  printf( "Geo Vl is % f and Vr is % f using speed % f and direction of % f\n", V_l, V_r, V, O);
}
//Double version of MapLatLonXY AgopenGPS Version conversion
void Geo::MapLatLonToXY(double phi, double lambda, double lambda0, double & x, double & y)
{
  double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
  double nu2 = ep2 * pow(cos(phi), 2.0);
  double n = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
  double t = tan(phi);
  double t2 = t * t;
  double l = lambda - lambda0;
  double l3Coef = 1.0 - t2 + nu2;
  double l4Coef = 5.0 - t2 + (9 * nu2) + (4.0 * (nu2 * nu2));
  double l5Coef = 5.0 - (18.0 * t2) + (t2 * t2) + (14.0 * nu2) - (58.0 * t2 * nu2);
  double l6Coef = 61.0 - (58.0 * t2) + (t2 * t2) + (270.0 * nu2) - (330.0 * t2 * nu2);
  double l7Coef = 61.0 - (479.0 * t2) + (179.0 * (t2 * t2)) - (t2 * t2 * t2);
  double l8Coef = 1385.0 - (3111.0 * t2) + (543.0 * (t2 * t2)) - (t2 * t2 * t2);

  /* Calculate easting (x) */
  x = (n * cos(phi) * l)
      + (n / 6.0 * pow(cos(phi), 3.0) * l3Coef * pow(l, 3.0))
      + (n / 120.0 * pow(cos(phi), 5.0) * l5Coef * pow(l, 5.0))
      + (n / 5040.0 * pow(cos(phi), 7.0) * l7Coef * pow(l, 7.0));

  /* Calculate northing (y) */
  y = ArcLengthOfMeridian(phi)
      + (t / 2.0 * n * pow(cos(phi), 2.0) * pow(l, 2.0))
      + (t / 24.0 * n * pow(cos(phi), 4.0) * l4Coef * pow(l, 4.0))
      + (t / 720.0 * n * pow(cos(phi), 6.0) * l6Coef * pow(l, 6.0))
      + (t / 40320.0 * n * pow(cos(phi), 8.0) * l8Coef * pow(l, 8.0));

  //  return xy;
}

//AgopenGPS Version conversion
double Geo::ArcLengthOfMeridian(double phi)
{
  const double n = (sm_a - sm_b) / (sm_a + sm_b);
  double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));
  double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) * 0.0625) + (-3.0 * pow(n, 5.0) / 32.0);
  double gamma = (15.0 * pow(n, 2.0) * 0.0625) + (-15.0 * pow(n, 4.0) / 32.0);
  double delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
  double epsilon = (315.0 * pow(n, 4.0) / 512.0);
  return alpha * (phi + (beta * sin(2.0 * phi))
                  + (gamma * sin(4.0 * phi))
                  + (delta * sin(6.0 * phi))
                  + (epsilon * sin(8.0 * phi)));
}

//AgopenGPS Version conversion of LatLonToUTMXY using doubles  - this is way off
int Geo::DecDeg2UTM(double latitude, double longitude, int zone, double & x, double & y)
{
  //only calculate the zone once!
  //RED if (!mf.isFirstFixPositionSet) zone = floor((longitude + 180.0) * 0.16666666666666666666666666666667) + 1;
  if ( (zone < 1) || (zone > 60) ) {
    zone = floor((longitude + 180.0) * 0.16666666666666666666666666666667) + 1;
    //zone = floor((longitude + 180.0) / 6) + 1;
  }



  MapLatLonToXY(latitude * 0.01745329251994329576923690766743,
                longitude * 0.01745329251994329576923690766743,
                (-183.0 + (zone * 6.0)) * 0.01745329251994329576923690766743, x, y);

  x = (x * UTMScaleFactor) + 500000.0;
  x *= UTMScaleFactor;
  if (y < 0.0) {
    y += 10000000.0;
  }
  return zone;
}
