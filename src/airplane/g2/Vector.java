package airplane.g2;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.lang.Math;

import org.apache.log4j.Logger;

public class Vector
{
  public Vector(double xCoord, double yCoord)
  {
    x = xCoord;
    y = yCoord;
  }
  public Vector(Point2D point1, Point2D point2)
  {
    x = (float) (point2.getX() - point1.getX());
    y = (float) (point2.getY() - point1.getY());
  }
  public Vector(Point2D point)
  {
    x = (float) point.getX();
    y = (float) point.getY();
  }
  public Vector(double bearing) 
  {
	double bearingAngle = bearingToAng(bearing);
	x = Math.cos(Math.toRadians(bearingAngle));
	y = Math.sin(Math.toRadians(bearingAngle));
  }
  public void normalize()
  {
    float vectorLength = length();
    x = (float) (x/vectorLength);
    y = (float) (y/vectorLength);
  }
  public void multiply(double factor)
  {
    x = factor*x;
    y = factor*y;
  }
  public float length()
  {
    return (float) Point2D.distance(0, 0, x, y);
  }
  public Vector rotate90Clockwise()
  {
    return new Vector(y, -x);
  }
  public Vector rotate90AntiClockwise()
  {
    return new Vector(-y, x);
  }
  public Vector rotateOpposite()
  {
    return new Vector(-x, -y);
  }
  public double getAngle()
  {
    double angle = Math.toDegrees(Math.atan2(y, x));
    return angle;
  }

  public Vector rotate(double angleDegrees)
  {
    double angle = Math.toRadians(angleDegrees);
    double cos = Math.cos(angle);
    double sin = Math.sin(angle);
    return new Vector (x*cos - y*sin, x*sin + y*cos);
  }
  public Vector rotateToward(Vector v, double maxDegrees) 
  {
   double thisAngle = this.getAngle();
   double targetAngle = v.getAngle();
   double deltaAngle = targetAngle - thisAngle;
   if (deltaAngle > 180)
     deltaAngle = deltaAngle - 360;
   if (deltaAngle < -180)
     deltaAngle = deltaAngle + 360;
   if (Math.abs(deltaAngle) < maxDegrees) {
      return this.rotate(deltaAngle);
   } else if (Math.abs(deltaAngle - 360) < maxDegrees) {
      return this.rotate(deltaAngle - 360);
   } else if (deltaAngle >= 0 && deltaAngle <=180) {
      return this.rotate(maxDegrees);
   } else {
      return this.rotate(-maxDegrees);
   }
  }
  public double angleBetween(Vector v) 
  {
	  return Math.toDegrees(Math.acos(this.dotProduct(v) / (this.length() * v.length())));
  }
  public Point2D getPoint ()
  {
    return new Point2D.Double(x, y);
  }
  public static Vector addVectors(Vector v1, Vector v2)
  {
    double xCoord = v1.x + v2.x;
    double yCoord = v1.y + v2.y;
    return new Vector(xCoord, yCoord);
  }
  public static Vector subVectors(Vector v1, Vector v2)
  {
    double xCoord = v1.x - v2.x;
    double yCoord = v1.y - v2.y;
    return new Vector(xCoord, yCoord);
  }
  public double dotProduct(Vector v) 
  {
	return this.x * v.x + this.y * v.y;
  }
  public static double bearingToAng(double bearing) 
  {
    double b = (bearing + 270) % 360;
	return b < 0 ? b + 360 : b;
  }
  public static double angToBearing(double angle) 
  {
	double b = (angle + 90) % 360;
	return b < 0 ? b + 360 : b;
  }
  public double getBearing() {
	double ang = Math.toDegrees(Math.atan2(y, x));
	return angToBearing(ang);
  }
  public double x = 0;
  public double y = 0;
  private Logger logger = Logger.getLogger(this.getClass());
}

