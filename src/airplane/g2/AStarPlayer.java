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

public class AStarPlayer extends airplane.sim.Player {
	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	
	private static int count;
	
	private HashMap<Plane, PlaneState> planeStates;
	
	@Override
	public String getName() {
		return "AStar P2: Feel the Algo-Rhythm";
	}
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
    planeStates = new HashMap<Plane, PlaneState>();
		count = 0;
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
		//TODO: order planes
		if(count % 5 == 0) {
			Set<Line2D> walls = new HashSet<Line2D>();
			for(Plane p : planes) {
        PlaneState state = planeStates.get(p);
        if (state == null) {
          state = new PlaneState();
        }
				AStar astar = new AStar(walls, 5);
				Deque<Waypoint> dq = astar.AStarPath(p.getLocation(), p.getDestination());
				
				// need to shorten paths to a small subsection
				Waypoint[] wp = dq.toArray(new Waypoint[0]);
				
				// this loop is incorrect
				Point2D start = p.getLocation();
				Point2D end;
				for(int j = 0; j < wp.length; j++) {
					end = wp[j].point;
					walls.add(new Line2D.Double(start, end));
					start = wp[j].point;
				}
				state.path = dq;
        planeStates.put(p, state);
			}
		} else {
			for (int i = 0; i < planes.size(); i++) {
				Plane p = planes.get(i);
        PlaneState state = planeStates.get(p);
				Deque<Waypoint> dq = state.path;
				
				if (p.getLocation().equals(dq.peekLast().point)) dq.removeLast();
				
				bearings[i] = calculateBearing(p.getLocation(), (Point2D.Double) dq.peekLast().point);
			}
		}
		
		count++;		
		return bearings;
	}
}
