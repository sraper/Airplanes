package airplane.g2;

import java.awt.geom.Line2D;
import java.util.ArrayList;

import airplane.sim.Plane;

public class PlaneUtils {

	public static double xIntersect(Line2D.Double l1, Line2D.Double l2) {
    	double x1 = l1.getX1();
    	double x2 = l1.getX2();
    	double x3 = l2.getX1();
    	double x4 = l2.getX2();
    	
    	double y1 = l1.getY1();
    	double y2 = l1.getY2();
    	double y3 = l2.getY1();
    	double y4 = l2.getY2();
    	
    	double denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    	return ((x3-x4)*(x1*y2-y1*x2)-(x1-x2)*(x3*y4-y3*x4))/denom;
    }
    
    public static double yIntersect(Line2D.Double l1, Line2D.Double l2) {
    	double x1 = l1.getX1();
    	double x2 = l1.getX2();
    	double x3 = l2.getX1();
    	double x4 = l2.getX2();
    	
    	double y1 = l1.getY1();
    	double y2 = l1.getY2();
    	double y3 = l2.getY1();
    	double y4 = l2.getY2();
    	
    	double denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    	return ((y3-y4)*(x1*y2-y1*x2)-(y1-y2)*(x3*y4-y3*x4))/denom;
    }
    
	public static boolean planeTooClose(Plane p, ArrayList<Plane> planes) {
		for (Plane o : planes) {
			if (p != o && o.getBearing() >= 0 && p.getLocation().distance(o.getLocation()) < 6) return true;
		}
		return false;
	}
}
