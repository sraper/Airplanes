package airplane.g2;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import org.apache.log4j.Logger;
import airplane.sim.Plane;

import static airplane.g2.Vector.*;



public class AvoiderPlayer extends airplane.sim.Player {
	
	private static final int FINISHED = -2;
	private static final double COLLISION_AVOIDANCE_THRESHOLD = 2;

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

		logger.info("::: ROUND = " + round);
		for (int i = 0; i < planes.size(); i++) {
			logger.info("PLANE = " + i);
		    Plane p  = planes.get(i);
		    
		    if (round <= p.getDepartureTime() || p.getBearing() == FINISHED) {
		    	continue;
		    } else if (round == p.getDepartureTime() + 1) {
		    	bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    	continue;
		    } else {
		    
		    	Vector goalVec = new Vector(calculateBearing(p.getLocation(), p.getDestination()));
		    	//Vector planeAvoidanceVector = planeAvoidanceVector(p, planes);
		    	//Vector outVec = Vector.addVectors(goalVec, planeAvoidanceVector);
		    	
		    	//currVectors.put(p, addVectors(currVectors.get(p), new Vector(180)));
		    	bearings[i] = goalVec.getBearing();
		    }
		}
		
		return bearings;
	}

	private Vector planeAvoidanceVector(Plane p, ArrayList<Plane> planes) {
		Vector outVec = new Vector(0, 0);
		Point2D.Double pl = p.getLocation();
		
		for (Plane o : planes) {
			Vector diff = subVectors(new Vector(o.getLocation()), new Vector(p.getLocation()));
			if (p != o && pl.distance(o.getLocation()) < COLLISION_AVOIDANCE_THRESHOLD) {
				outVec = subVectors(outVec, diff);
			}
		}
		outVec.normalize();
		return outVec;
	}	
}
