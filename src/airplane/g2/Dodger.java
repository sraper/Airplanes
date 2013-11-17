package airplane.g2;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.apache.log4j.Logger;
import airplane.sim.Plane;

public class Dodger extends airplane.sim.Player {
	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	
	private HashMap<Plane, PlaneState> planeStates;

  // knobs
  static final double maxBearingDeg = 10;
  static final double collisionDistance = 5;
  static final double velocity = 1;

  private double safetyDistance;
	
	@Override
	public String getName() {
		return "Dodger";
	}
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
    planeStates = new HashMap<Plane, PlaneState>();
    double safetyDistanceHorizontal = 0;
    double safetyDistanceVertical = 0; // should increase to collisionDistance
    double safetyMoves = 0;
    // calculate safety distance
    for (double i = 0; i <= 90; i = i + maxBearingDeg) {
    	safetyDistanceHorizontal += velocity * Math.cos(Math.toRadians(i));
    	safetyDistanceVertical += velocity * Math.sin(Math.toRadians(i));
      safetyMoves++;
      if (safetyDistanceVertical > collisionDistance)
      {
        break;
      }
    }
    if (safetyDistanceVertical < collisionDistance) {
      safetyMoves += Math.ceil((collisionDistance - safetyDistanceVertical)/velocity);
      safetyDistanceHorizontal += (safetyMoves * velocity);
    }
    logger.info ("safetyMoves: " + safetyMoves 
        + " h: " + safetyDistanceHorizontal 
        + " v: " + safetyDistanceVertical); 

    safetyDistance = Math.ceil(safetyDistanceHorizontal) + (safetyMoves*velocity); // add distance covered by opposite plane

    logger.info ("safety distance: " + safetyDistance); 
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
		return bearings;
	}
}
