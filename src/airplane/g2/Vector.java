package airplane.g2;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.lang.Math;

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
    x = (float) (.99*x/vectorLength);
    y = (float) (.99*y/vectorLength);
  }
  public void multiply(float factor)
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
  public Vector rotate(double angleDegrees)
  {
    double angle = Math.toRadians(angleDegrees);
    double cos = Math.cos(angle);
    double sin = Math.sin(angle);
    return new Vector (x*cos - y*sin, x*sin + y*cos);
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
}

