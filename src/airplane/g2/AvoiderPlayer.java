package airplane.g2;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import org.apache.log4j.Logger;
import airplane.sim.Plane;

import static airplane.g2.Vector.*;



public class AvoiderPlayer extends airplane.sim.Player {
	
	private static final int FINISHED = -2;
	private static final int WAITING = -1;
	
	// Dials and Knobs
	private static final double PLANE_DIST_THRESHOLD = 20; // Distance before collision prevention occurs
	private static final double WALL_AVOIDANCE_THRESHOLD = 10;
	private static final float  WALL_AVOIDANCE_FORCE = 2;
	private static final double TURN_RADIUS = 9.5; // Max degrees turned per timestep
	private static final double YOLO_FACTOR = 0.5; // Scaling factor for tendency to ignore other planes near airport.

	private double[] bearings;
	
	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	
	@Override
	public String getName() {
		return "Avoider Player";
	}
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		logger.info("Starting new game!");

	}
	
	/*
	 * This is called at each step of the simulation.
	 * The List of Planes represents their current location, destination, and current
	 * bearing; the bearings array just puts these all into one spot. 
	 * This method should return an updated array of bearings.
	 */
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {

		this.bearings = bearings; // Make it a little bit easier to access bearings in helper methods.
		
		for (int i = 0; i < planes.size(); i++) {
		    Plane p  = planes.get(i);
		    
		    Vector currVec = new Vector(bearings[i]);
		    Vector goalVec = new Vector(calculateBearing(p.getLocation(), p.getDestination()));
		    
		    if (round <= p.getDepartureTime() || p.getBearing() == FINISHED 
		    		|| bearings[i] == WAITING && planeTooClose(p, planes)) { 
		    	continue;
			} else if (round > p.getDepartureTime() && bearings[i] == WAITING) {
		    	bearings[i] = goalVec.getBearing();
		    	continue;
		    } else {
		    	Vector planeAvoidVec = addVectors(goalVec, planeAvoidanceVector(p, planes));
		    	Vector wallAvoidVec  = addVectors(planeAvoidVec, wallAvoidanceVector(p, planes));

		    	bearings[i] = currVec.rotateToward(wallAvoidVec, TURN_RADIUS).getBearing();
		    }
		}
		
		
		return bearings;
	}

	private Vector planeAvoidanceVector(Plane p, ArrayList<Plane> planes) {
		Vector outVec = new Vector(0, 0);
		Point2D.Double pl = p.getLocation();
		
		double avoidanceThreshold = Math.min(PLANE_DIST_THRESHOLD, pl.distance(p.getDestination()) * YOLO_FACTOR);
		
		for (int i = 0; i < planes.size(); i++) {
			Plane o = planes.get(i);
			Vector diff = subVectors(new Vector(o.getLocation()), new Vector(p.getLocation()));
			if (p != o && pl.distance(o.getLocation()) < avoidanceThreshold) {
				outVec = subVectors(outVec, diff);
				// [TG]: Following line deviates from boid formula, causes planes to turn opposite ways.
				outVec = addVectors(outVec, new Vector(bearings[i]).rotateOpposite());
			}
		}

		return outVec;
	}	
	
	private Vector wallAvoidanceVector(Plane p, ArrayList<Plane> planes) {
		Vector outVec = new Vector(0, 0);
		Point2D.Double loc = p.getLocation();
		if (loc.x < WALL_AVOIDANCE_THRESHOLD) outVec = addVectors(outVec, new Vector(90));
		if (loc.y < WALL_AVOIDANCE_THRESHOLD) outVec = addVectors(outVec, new Vector(180));
		if (loc.x > 100 - WALL_AVOIDANCE_THRESHOLD) outVec = addVectors(outVec, new Vector(270));
		if (loc.y > 100 - WALL_AVOIDANCE_THRESHOLD) outVec = addVectors(outVec, new Vector(0));
		
		//outVec.normalize();
		outVec.multiply(WALL_AVOIDANCE_FORCE);
		logger.info("OUTVEC: " + outVec.x + " " + outVec.y);
		
		return outVec;
	}
	
	private boolean planeTooClose(Plane p, ArrayList<Plane> planes) {
		for (Plane o : planes) {
			if (p != o && o.getBearing() != -1 && p.getLocation().distance(o.getLocation()) < 6) return true;
		}
		
		return false;
	}
}
