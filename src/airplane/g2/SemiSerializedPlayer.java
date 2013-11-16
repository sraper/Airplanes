package airplane.g2;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Line2D.Double;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

import org.apache.log4j.Logger;
import airplane.sim.Plane;

import static airplane.g2.Vector.*;



public class SemiSerializedPlayer extends airplane.sim.Player {
	
	private static final int FINISHED = -2;
	private static final int WAITING = -1;
	private static final double ALLOWED_PROXIMITY = 8;
	
	private HashMap<Plane, Integer> startTimes;
	
	// Dials and Knobs
	private Logger logger = Logger.getLogger(this.getClass()); // for logging

	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		startTimes = new HashMap<Plane, Integer>();
		HashSet<Plane> finished = new HashSet<Plane>();
		
		ArrayList<Plane> planesCopy = new ArrayList<Plane>();
		planesCopy.addAll(planes);
		
		Collections.sort(planesCopy, new Comparator<Plane>() {
			public int compare(Plane pa, Plane pb) {
				Point2D.Double pal = pa.getLocation();
				Point2D.Double pad = pa.getDestination();
				Point2D.Double pbl = pb.getLocation();
				Point2D.Double pbd = pb.getDestination();
				return -(new java.lang.Double(pal.distance(pad)).compareTo(pbl.distance(pbd)));
			}
		});
		
		for (Plane p : planes) {
			int proposedTime = p.getDepartureTime();
			while (existsConflict(p, proposedTime, finished, startTimes)) {
				proposedTime++;
				logger.info("PROPOSED TIME: " + proposedTime);
			}
			
			startTimes.put(p, proposedTime);
			finished.add(p);
		}
		
		logger.info("Starting new game!");
	}

	private boolean existsConflict(Plane p, int proposedTime, HashSet<Plane> finished, 
			HashMap<Plane, Integer> startTimes) {
		Line2D.Double pPath = new Line2D.Double(p.getLocation(), p.getDestination());
		
		for (Plane o : finished) {
			// [TG]: Because planes move one unit per timestep, one can just use distances directly.
			Line2D.Double oPath = new Line2D.Double(o.getLocation(), o.getDestination());
			Point2D.Double intersect = new Point2D.Double(xIntersect(pPath, oPath), yIntersect(pPath, oPath)); 
			double pDistance = p.getLocation().distance(intersect) - proposedTime;
			double oDistance = o.getLocation().distance(intersect) - startTimes.get(o);
			double oGoalDistance =  o.getLocation().distance(o.getDestination());
			if (Math.abs(pDistance - oDistance) < ALLOWED_PROXIMITY) {
				return true;
			}
		}
		
		return false;
	}
	
	private double xIntersect(Line2D l1, Line2D l2) {
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
    
    private double yIntersect(Line2D l1, Line2D l2) {
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

	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {

		for (int i = 0; i < planes.size(); i++) {
		    Plane p  = planes.get(i);
		    if (planeTooClose(p, planes) || round <= startTimes.get(p) || bearings[i] == FINISHED) {
		    	continue;
		    } else {
		    	bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    }
		}

		return bearings;
	}
	
	// Determine if any planes are too close to the current plane to allow it to launch
	private boolean planeTooClose(Plane p, ArrayList<Plane> planes) {
		for (Plane o : planes) {
			if (p != o && o.getBearing() >= 0 && p.getLocation().distance(o.getLocation()) < 6) return true;
		}
		return false;
	}
	
	@Override
	public String getName() {
		return "Semi-Serialized Player";
	}
}
