package airplane.g2;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import org.apache.log4j.Logger;
import airplane.sim.Plane;

import static airplane.g2.Vector.*;



public class SemiSerializedPlayer extends airplane.sim.Player {
	
	private static final int FINISHED = -2;
	private static final int WAITING = -1;
	
	// Dials and Knobs
	private static final double PLANE_DIST_THRESHOLD = 20; // Distance before collision prevention occurs
	private static final double WALL_AVOIDANCE_THRESHOLD = 10;
	private static final float  WALL_AVOIDANCE_FORCE = 2;
	private static final double TURN_RADIUS = 9.5; // Max degrees turned per timestep
	private static final double YOLO_FACTOR = 2; // Scaling factor for tendency to ignore other planes near airport.
	
	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	
	@Override
	public String getName() {
		return "Avoider Player";
	}

	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		logger.info("Starting new game!");

	}

	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {

		for (int i = 0; i < planes.size(); i++) {
		    Plane p  = planes.get(i);
		    Vector currVec = new Vector(bearings[i]);
		    Vector goalVec = new Vector(calculateBearing(p.getLocation(), p.getDestination()));
		    
		    if (round <= p.getDepartureTime() 
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
			}
		}

		return outVec;
	}	
	
	// Determine whether two planes close to one another are actually at risk for colliding (i.e. they are 
	// heading toward one another).
	private boolean collisionPossible(Plane p, Plane o) {
		return true;
	}

	// Calculate avoidance vector for plane-wall collisions
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
	
	// Determine if two planes are too close for one to launch.
	private boolean planeTooClose(Plane p, ArrayList<Plane> planes) {
		for (Plane o : planes) {
			if (p != o && o.getBearing() != -1 && p.getLocation().distance(o.getLocation()) < 6) return true;
		}
		return false;
	}
}
