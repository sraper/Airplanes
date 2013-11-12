package airplane.g2;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

import org.apache.log4j.Logger;
import airplane.sim.Plane;

import static airplane.g2.Vector.*;



public class SerialAvoiderPlayer extends airplane.sim.Player {
	
	private static final int FINISHED = -2;
	private static final int WAITING = -1;
	
	// Dials and Knobs
	private static final double PLANE_DIST_THRESHOLD = 15; // Distance before collision prevention occurs
	private static final double ALLOWED_PROXIMITY = 20;
	private static final double WALL_AVOIDANCE_THRESHOLD = 5;
	private static final float  WALL_AVOIDANCE_FORCE = 2;
	private static final double TURN_RADIUS = 9.5; // Max degrees turned per timestep
	private static final double YOLO_FACTOR = 1.4; // Scaling factor for tendency to ignore other planes near airport.

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private HashMap<Plane, Integer> startTimes;
	
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

	// TODO(TG): Generalize this to be a generic "Would I collide with this path" function
	private boolean existsConflict(Plane p, int proposedTime, HashSet<Plane> finished, 
			HashMap<Plane, Integer> startTimes) {
		Line2D.Double pPath = new Line2D.Double(p.getLocation(), p.getDestination());
		
		for (Plane o : finished) {
			// [TG]: Because planes move one unit per timestep, one can just use distances directly.
			Line2D.Double oPath = new Line2D.Double(o.getLocation(), o.getDestination());
			Point2D.Double intersect = new Point2D.Double(xIntersect(pPath, oPath), yIntersect(pPath, oPath)); 
			double pDistance = p.getLocation().distance(intersect) - proposedTime;
			double oDistance = o.getLocation().distance(intersect) - startTimes.get(o);
			double oGoalDistance =  o.getLocation().distance(o.getDestination()) - startTimes.get(o);
			// [TG]: Second clause here is to make sure that we don't freak out if 'o' would have reached its destination.
			if (Math.abs(pDistance - oDistance) < ALLOWED_PROXIMITY && oDistance < oGoalDistance) {
				return true;
			}
		}
		
		return false;
	}
	
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {

		for (int i = 0; i < planes.size(); i++) {
		    Plane p  = planes.get(i);
		    Vector currVec = new Vector(bearings[i]);
		    Vector goalVec = new Vector(calculateBearing(p.getLocation(), p.getDestination()));
		    
		    if (round <= startTimes.get(p)
		            || p.getBearing() == FINISHED 
		    		|| bearings[i] == WAITING && planeTooClose(p, planes)) { 
		    	continue;
			} else if (round > p.getDepartureTime() && bearings[i] == WAITING) {
		    	bearings[i] = goalVec.getBearing();
		    } else {
		    	Vector planeAvoidVec = addVectors(goalVec, planeAvoidanceVector(p, planes, bearings));
		    	Vector wallAvoidVec  = addVectors(planeAvoidVec, wallAvoidanceVector(p, planes));
		    	bearings[i] = currVec.rotateToward(wallAvoidVec, TURN_RADIUS).getBearing();
		    }
		}

		return bearings;
	}

	// Calculate avoidance vector for plane-plane collisions
	private Vector planeAvoidanceVector(Plane p, ArrayList<Plane> planes, double[] bearings) {
		Vector outVec = new Vector(0, 0);
		Vector currVec = new Vector(p.getBearing());
		
		Point2D.Double pl = p.getLocation();
		
		double avoidanceThreshold = Math.min(PLANE_DIST_THRESHOLD, pl.distance(p.getDestination()) * YOLO_FACTOR);
		
		// TODO(TG): We really only need to avoid planes that we will get closer to if we continue our same bearing.
		for (int i = 0; i < planes.size(); i++) {
			Plane o = planes.get(i);
			Vector diff = subVectors(new Vector(o.getLocation()), new Vector(p.getLocation()));
			if (p != o && pl.distance(o.getLocation()) < avoidanceThreshold && collisionPossible(p, o)) {
				outVec = subVectors(outVec, diff);
				// [TG]: Causes planes to turn in opposite directions in head-on collision cases, such as on 'Double'
				outVec = addVectors(outVec, new Vector(bearings[i]).rotateOpposite());
				//outVec = addVectors(outVec, new Vector(bearings[i]).rotateToward(currVec, -90));
			}
		}

		return outVec;
	}	
	
	private boolean collisionPossible(Plane p, Plane o) {
		// TODO(TG): Write this.
		return o.getBearing() >= 0;
	}

	private Vector wallAvoidanceVector(Plane p, ArrayList<Plane> planes) {
		Vector outVec = new Vector(0, 0);
		Point2D.Double loc = p.getLocation();
		if (loc.x < WALL_AVOIDANCE_THRESHOLD)       outVec = addVectors(outVec, new Vector(90));
		if (loc.y < WALL_AVOIDANCE_THRESHOLD)       outVec = addVectors(outVec, new Vector(180));
		if (loc.x > 100 - WALL_AVOIDANCE_THRESHOLD) outVec = addVectors(outVec, new Vector(270));
		if (loc.y > 100 - WALL_AVOIDANCE_THRESHOLD) outVec = addVectors(outVec, new Vector(0));
		
		//outVec.normalize();
		outVec.multiply(WALL_AVOIDANCE_FORCE);
		return outVec;
	}
	
	private boolean planeTooClose(Plane p, ArrayList<Plane> planes) {
		for (Plane o : planes) {
			if (p != o && o.getBearing() >= 0 && p.getLocation().distance(o.getLocation()) < 6) return true;
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
	public String getName() {
		return "Serial Avoider Player";
	}
	
}
