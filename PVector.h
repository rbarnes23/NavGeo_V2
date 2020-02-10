#ifndef __PVECTOR_H__
#define __PVECTOR_H__


#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

struct pVector {
  double x;
  double y;
  double z;
};


class PVector {
  public:
    // Constructor
    /**
      Constructor for an empty vector: x, y, and z are set to 0.
    */
    //PVector();


    /**
       Constructor for a 3D vector.

       @param  x the x coordinate.
       @param  y the y coordinate.
       @param  z the y coordinate.
    */
    PVector(double x = 0, double y = 0, double z = 0) {
      _x = x;
      _y = y;
      _z = z;
    }



    /**
        Set x, y, and z coordinates.

        @param x the x coordinate.
        @param y the y coordinate.
        @param z the z coordinate.
    */
    void set(double x = 0, double y = 0, double z = 0) {
      _x = x;
      _y = y;
      _z = z;
    }


    /**
       Get a copy of this vector.
    */
    pVector get() {
      _thisVector.x = _x;
      _thisVector.y = _y;
      _thisVector.z = _z;
      return _thisVector;
    }

    /**
       Calculate the magnitude (length) of the vector
       @return the magnitude of the vector
    */
    double mag() {
      return (double) sqrt(_x * _x + _y * _y + _z * _z);
    }

    /**
       Add a vector to this vector
       @param v the vector to be added
    */
    void add(PVector v) {
      _x += v._x;
      _y += v._y;
      _z += v._z;
    }

    void add(double x, double y, double z) {
      _x += x;
      _y += y;
      _z += z;
    }

    /**
       Add two vectors into a target vector
       @param v1 a vector
       @param v2 another vector
       @param target the target vector (if null, a new vector will be created)
       @return a new vector that is the sum of v1 and v2
    */
    static PVector add(PVector v1, PVector v2, PVector target) {
      target.set(v1._x + v2._x, v1._y + v2._y, v1._z + v2._z);
      return target;
    }



    /**
      Add two vectors
      @param v1 a vector
      @param v2 another vector
      @return a new vector that is the sum of v1 and v2
    */

    static PVector add(PVector v1, PVector v2) {
      return add(v1, v2, 0);
    }

    /**
      Subtract a vector from this vector
      @param v the vector to be subtracted
    */
    void sub(PVector v) {
      _x -= v._x;
      _y -= v._y;
      _z -= v._z;
    }

    void sub(double x, double y, double z) {
      _x -= x;
      _y -= y;
      _z -= z;
    }

    /**
       Subtract one vector from another
       @param v1 a vector
       @param v2 another vector
       @return a new vector that is v1 - v2
    */
    static PVector sub(PVector v1, PVector v2) {
      return sub(v1, v2, 0);
    }

    static PVector sub(PVector v1, PVector v2, PVector target) {
      target.set(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
      return target;
    }

    /**
       Multiply this vector by a scalar
       @param n the value to multiply by
    */
    void mult(double n) {
      _x *= n;
      _y *= n;
      _z *= n;
    }

    /**
       Multiply a vector by a scalar, and write the result into a target PVector.
       @param v a vector
       @param n scalar
       @param target PVector to store the result
       @return the target vector, now set to v1 * n
    */
    static PVector mult(PVector v, double n, PVector target) {
      target.set(v._x * n, v._y * n, v._z * n);
      return target;
    }

    /**
       Multiply a vector by a scalar
       @param v a vector
       @param n scalar
       @return a new vector that is v1 * n
    */
    static PVector mult(PVector v, double n) {
      return mult(v, n, 0);
    }

    /**
       Multiply each element of one vector by the elements of another vector.
       @param v the vector to multiply by
    */
    void mult(PVector v) {
      _x *= v._x;
      _y *= v._y;
      _z *= v._z;
    }

    /**
       Multiply each element of one vector by the individual elements of another
       vector, and return the result as a new PVector.
    */
    static PVector mult(PVector v1, PVector v2) {
      return mult(v1, v2, 0);
    }

    /**
       Multiply each element of one vector by the individual elements of another
       vector, and write the result into a target vector.
       @param v1 the first vector
       @param v2 the second vector
       @param target PVector to store the result
    */
    static PVector mult(PVector v1, PVector v2, PVector target) {
      target.set(v1._x * v2._x, v1._y * v2._y, v1._z * v2._z);
      return target;
    }

    /**
       Divide this vector by a scalar
       @param n the value to divide by
    */
    void div(double n) {
      _x /= n;
      _y /= n;
      _z /= n;
    }

    /**
       Divide a vector by a scalar and return the result in a new vector.
       @param v a vector
       @param n scalar
       @return a new vector that is v1 / n
    */
    static PVector div(PVector v, double n) {
      return div(v, n, 0);
    }


    static PVector div(PVector v, double n, PVector target) {
      target.set(v._x / n, v._y / n, v._z / n);
      return target;
    }


    /**
       Divide each element of one vector by the elements of another vector.
    */
    void div(PVector v) {
      _x /= v._x;
      _y /= v._y;
      _z /= v._z;
    }

    /**
       Multiply each element of one vector by the individual elements of another
       vector, and return the result as a new PVector.
    */
    static PVector div(PVector v1, PVector v2) {
      return div(v1, v2, 0);
    }


    /**
       Divide each element of one vector by the individual elements of another
       vector, and write the result into a target vector.
       @param v1 the first vector
       @param v2 the second vector
       @param target PVector to store the result
    */
    static PVector div(PVector v1, PVector v2, PVector target) {
      target.set(v1._x / v2._x, v1._y / v2._y, v1._z / v2._z);
      return target;
    }

    /**
       Calculate the Euclidean distance between two points (considering a point as a vector object)
       @param v another vector
       @return the Euclidean distance between
    */
    double dist(PVector v) {
      double dx = _x - v._x;
      double dy = _y - v._y;
      double dz = _z - v._z;
      return (double) sqrt(dx * dx + dy * dy + dz * dz);
    }


    /**
       Calculate the Euclidean distance between two points (considering a point as a vector object)
       @param v1 a vector
       @param v2 another vector
       @return the Euclidean distance between v1 and v2
    */
    static double dist(PVector v1, PVector v2) {
      double dx = v1._x - v2._x;
      double dy = v1._y - v2._y;
      double dz = v1._z - v2._z;
      return (double) sqrt(dx * dx + dy * dy + dz * dz);
    }
    /**
       Calculate the dot product with another vector
       @return the dot product
    */
    double dot(PVector v) {
      return _x * v._x + _y * v._y + _z * v._z;
    }


    double dot(double x, double y, double z) {
      return _x * x + _y * y + _z * z;
    }


    static double dot(PVector v1, PVector v2) {
      return v1._x * v2._x + v1._y * v2._y + v1._z * v2._z;
    }

    /**
       Return a vector composed of the cross product between this and another.
    */
    PVector cross(PVector v) {
      return cross(v, 0);
    }


    /**
       Perform cross product between this and another vector, and store the
       result in 'target'. If target is null, a new vector is created.
    */
    PVector cross(PVector v, PVector target) {
      double crossX = _y * v._z - v._y * _z;
      double crossY = _z * v._x - v._z * _x;
      double crossZ = _x * v._y - v._x * _y;
      target.set(crossX, crossY, crossZ);
      return target;
    }

    static PVector cross(PVector v1, PVector v2, PVector target) {
      double crossX = v1._y * v2._z - v2._y * v1._z;
      double crossY = v1._z * v2._x - v2._z * v1._x;
      double crossZ = v1._x * v2._y - v2._x * v1._y;
      target.set(crossX, crossY, crossZ);
      return target;
    }


    /**
       Normalize the vector to length 1 (make it a unit vector)
    */
    void normalize() {
      double m = mag();
      if (m != 0 && m != 1) {
        div(m);
      }
    }


    /**
       Normalize this vector, storing the result in another vector.
       @param target Set to null to create a new vector
       @return a new vector (if target was null), or target
    */
    PVector normalize(PVector target) {
      double m = mag();
      if (m > 0) {
        target.set(_x / m, _y / m, _z / m);
      } else {
        target.set(_x, _y, _z);
      }
      return target;
    }
    /**
       Limit the magnitude of this vector
       @param max the maximum length to limit this vector
    */
    void limit(double max) {
      if (mag() > max) {
        normalize();
        mult(max);
      }
    }


    /**
       Calculate the angle of rotation for this vector (only 2D vectors)
       @return the angle of rotation
    */
    double heading2D() {
      double angle = (double) atan2(-_y, _x);
      return -1 * angle;
    }

    /**
       Calculate the angle between two vectors, using the dot product
       @param v1 a vector
       @param v2 another vector
       @return the angle between the vectors in radians
    */
    static double angleBetween(PVector v1, PVector v2) {
      double dot = v1._x * v2._x + v1._y * v2._y + v1._z * v2._z;
      double v1mag = sqrt(v1._x * v1._x + v1._y * v1._y + v1._z * v1._z);
      double v2mag = sqrt(v2._x * v2._x + v2._y * v2._y + v2._z * v2._z);
      double test = dot / (v1mag * v2mag);
      return (double)acos(dot / (v1mag * v2mag));
    }


    String toString() {
      return "[ " + String(_x) + ", " + String(_y) + ", " + String(_z) + " ]";
    }



    size_t addPoint(PVector point) {
      int end;
      if (_points.empty()) {
        end = 0;
      } else {
        end = getEnd();
      }
      double d;
      double x, x1, y, y1;
      int zone;
      _points.push_back(point);
      return _points.size();
    }

    // Get the first item on the path
    PVector getStart() {
      return _points[0];
    }

    PVector getPoint(int i) {
      return _points[i];
    }

    //Find the last item on the path
    int getEnd() {
      return _points.size();
    }

    double getX() {
      return _x;
    }

    double getY() {
      return _y;
    }

    double getZ() {
      return _z;
    }

    void setX(double x) {
      _x = x;
    }

    void setY(double y) {
      _y = y;
    }
    void setZ(double z) {
      _z = z;
    }


    //Get the current path
    std::vector<PVector> getPath() {
      return _points;
    }

    double distanceBetween(PVector v1, PVector v2)
    {
      // returns distance in meters between two positions, both specified
      // as signed decimal-degrees latitude and longitude. Uses great-circle
      // distance computation for hypothetical sphere of radius 6372795 meters.
      // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
      // Courtesy of Maarten Lamers
      double delta = radians(v1._y - v2._y);
      double sdlong = sin(delta);
      double cdlong = cos(delta);
      v1._x = radians(v1._x);
      v2._x = radians(v2._x);
      double slat1 = sin(v1._x);
      double clat1 = cos(v1._x);
      double slat2 = sin(v2._x);
      double clat2 = cos(v2._x);
      delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
      delta = sq(delta);
      delta += sq(clat2 * sdlong);
      delta = sqrt(delta);
      double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
      delta = atan2(delta, denom);
      return delta * 6372795;
    }

    double courseTo(PVector v1, PVector v2)
    {
      // returns course in degrees (North=0, West=270) from position 1 to position 2,
      // both specified as signed decimal-degrees latitude and longitude.
      // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
      // Courtesy of Maarten Lamers
      double dlon = radians(v2._y - v1._y);
      v1._x = radians(v1._x);
      v2._x = radians(v2._x);
      double a1 = sin(dlon) * cos(v2._x);
      double a2 = sin(v1._x) * cos(v2._x) * cos(dlon);
      a2 = cos(v1._x) * sin(v2._x) - a2;
      a2 = atan2(a1, a2);
      if (a2 < 0.0)
      {
        a2 += TWO_PI;
      }
      return degrees(a2);
    }

    //Check to see if vector is empty to prevent divide by zero errors
    int empty() {
      int empty = 0;
      if (_x == 0 || _y == 0) {
        empty = 1;
      }
      return empty;
    }

    PVector getNormalPoint(PVector p, PVector a, PVector b) {

      // Vector from a to p
      PVector ap = sub(p, a);
      // Vector from a to b
      PVector ab = sub(b, a);

      ab.normalize(); // Normalize the line


      // Project vector "diff" onto line by using the dot product
      ab = mult(ab, ap.dot(ab));
      printf("AB: %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", a._x, a._y, b._x, b._y, ap._x, ap._y, ab._x, ab._y);


      //ab.scale(ap.dot(ab));
      printf("SCALE %lf %lf\n", ab._x, ab._y);
      PVector normalPoint = add(a, ab);
      return normalPoint;
    }

    PVector closestPointOnPath(PVector currentPos, PVector predictPos, int lookAhead) {
      PVector pop(0, 0, 0);
      PVector normal(0, 0);
      PVector target(0, 0);
      float worldRecord = 1e6;  // Start with a very high record distance that can easily be beaten
      if (getEnd() < 1) {
        return pop;
      }


      // Loop through all points of the path
      for (int i = 0; i < getEnd() - 1; i++) {

        // Look at a line segment
        PVector a = getPoint(i); //Current Point
        PVector b = getPoint(i + 1); //Next Point


        // Get the normal point to that line
        PVector normalPoint = getNormalPoint(predictPos, a, b);
        printf("NP: %lf %lf %lf %lf\n", a._x, a._y, normalPoint._x, normalPoint._y);

        // This only works because we know our path goes from left to right
        // We could have a more sophisticated test to tell if the point is in the line segment or not
        if (normalPoint._x < a._x || normalPoint._x > b._x) {
          // This is something of a hacky solution, but if it's not within the line segment
          // consider the normal to just be the end of the line segment (point b)
          normalPoint = b;
        }


        // How far away are we from the path?
        //double distance = distanceBetween(predictPos, normalPoint ); // distance is x
        double distance = distanceBetween(a, b ); // distance is x
        printf("Distance: %.8lf\n", distance);
        //We need to estimate time to
        //float estimated = calculateTimeFromVelocityAndAcceleration(_maxSpeed, _currSpeed, _maxSpeed);
        //If we know the distance and velocity we can calculate acceleration
        //float acceleration = calculateAcceleration(distance, estimated);
        //printf("Estimated Time and Acceleration: %.8lf %.8lf\n", estimated, acceleration);
        // Did we beat the record and find the closest line segment?
        if (distance < worldRecord) {
          worldRecord = distance;
          pop.set(normalPoint._x, normalPoint._y, i);
          printf("Point On Path: %20.14lf %20.14lf\n", normalPoint._x, normalPoint._y);
        }
      }
      printf("Closest Point On Path: %.6lf %.6lf", pop._x, pop._y);
      return pop;
    }
    /**
       Return a representation of this vector as a float array. This is only for
       temporary use. If used in any other fashion, the contents should be copied
       by using the get() command to copy into your own array.
    */
    /*
      float[] array() {
        if (array == null) {
          array = new float[3];
        }
        array[0] = x;
        array[1] = y;
        array[2] = z;
        return array;
      }


    */
    //Variables

    // Methods

  private:
    //Variables
    double _x, _y, _z;
    pVector _thisVector;
    std::vector<PVector> _points;
    //Methods

};
#endif
