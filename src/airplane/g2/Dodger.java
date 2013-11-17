package airplane.g2;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Logger;

import airplane.sim.*;

public class Dodger extends airplane.sim.Player {
	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	
	private Map<Integer, PlaneState> planeStates;
	private Map<Integer, PlaneState> simulatedPlaneStates;
  private Set<Line2D> walls;

  private static final int FINISHED = -2;
	private static final int WAITING = -1;

  // knobs
  private static final double maxBearingDeg = 9.8;
  private static final double collisionDistance = 5;
  private static final double velocity = 1;

  private double safetyDistance;
  private boolean simulating;
  private int currentPlane; // used while simulating
	
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
    simulating = false;
    planeStates = new HashMap<Integer, PlaneState>();
    walls = new HashSet<Line2D>();
    double safetyDistanceHorizontal = 0;
    double safetyDistanceVertical = 0; // should increase to collisionDistance
    double safetyMoves = 0;
    // calculate safety distance
    for (double i = 0; i <= 90; i = i + maxBearingDeg) {
    	safetyDistanceHorizontal += velocity * Math.cos(Math.toRadians(i));
    	safetyDistanceVertical += velocity * Math.sin(Math.toRadians(i));
      safetyMoves++;
      if (safetyDistanceVertical > collisionDistance) {
        break;
      }
    }
    if (safetyDistanceVertical < collisionDistance) {
      safetyMoves += Math.ceil((collisionDistance - safetyDistanceVertical)/velocity);
      safetyDistanceHorizontal += (safetyMoves * velocity);
    }
    logger.trace ("safetyMoves: " + safetyMoves 
        + " h: " + safetyDistanceHorizontal 
        + " v: " + safetyDistanceVertical); 

    safetyDistance = Math.ceil(safetyDistanceHorizontal) + (safetyMoves*velocity); // add distance covered by opposite plane

    logger.trace ("safety distance: " + safetyDistance); 
		logger.info("Starting new game!");
	}

  @Override
  public double[] simulateUpdate (ArrayList<Plane> planes, int round, double[] bearings) {
    simulating = true;
    bearings = updatePlanes(planes, round, bearings);
    simulating = false;
    return bearings;
  }
	
	/*
	 * This is called at each step of the simulation.
	 * The List of Planes represents their current location, destination, and current
	 * bearing; the bearings array just puts these all into one spot. 
	 * This method should return an updated array of bearings.
	 */
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {
    boolean allDone = true;
    boolean takeOff = false; // have to take-off one plane at a time
    boolean wait = false;

    if (simulating == false) {
      logger.trace("round: " + round);
    } else {
      logger.trace("simulating round: " + round);
    }

    // save ids
    if (round == 1 && simulating == false) {
      for (int i = 0; i < planes.size(); i++) {
        planes.get(i).id = i;
      }
    }

    // TODO: sort planes
    for (int i = 0; i < planes.size(); i++) {
      Plane plane = planes.get(i);
      PlaneState state;
      Deque<Waypoint> path = null;

      if (bearings[i] != FINISHED) {
        allDone = false;
      }

      if (round < plane.getDepartureTime() || bearings[i] == FINISHED) {
        // skip
        continue;
      } else if (bearings[i] == WAITING && simulating && i != currentPlane) {
        logger.trace("not taking off plane: " + i + " in simulation" + " current plane: " + currentPlane);
        // do not take-off any new planes in simulation except the currentPlane
        continue;
      }

      if (simulating) {
        logger.trace("get simulated state for plane: " + i);
        state = simulatedPlaneStates.get(plane.id);
      } else {
        logger.trace("get state for plane: " + i);
        state = planeStates.get(plane.id);
        currentPlane = i; // set currentPlane placeholder
        // reset walls, simulation runs with existing walls
        walls = new HashSet<Line2D>();
      }

      if (state == null) {
        if (simulating) {
          logger.trace ("create new simulated state for plane: " + i);
        } else {
          logger.trace ("create new state for plane: " + i);
        }
        state = new PlaneState();
      }

      path = state.path;

      if (path == null) { // need to choose path for this plane
        logger.trace ("path is null for plane: " + i);
        if (simulating == false) {
          logger.trace("start simulation plane: " + currentPlane);
          // detect collision points and place walls
          SimulationResult result;
          while(true) {
            // make copies
            simulatedPlaneStates = new HashMap<Integer, PlaneState>();
            for (int k = 0; k < planes.size(); k++) {
              PlaneState origState = planeStates.get(planes.get(k).id);
              if (origState != null) {
                PlaneState newState = new PlaneState();
                if (origState.path != null) {
                  logger.trace("copy to simulated state, path exists for plane: " + k);
                  newState.path = new ArrayDeque<Waypoint>(origState.path);
                }
                newState.target = origState.target;
                simulatedPlaneStates.put(planes.get(k).id, newState);
              }
            }
            result = startSimulation(planes, round);
            if (result.getReason() == SimulationResult.TOO_CLOSE) {
              logger.trace ("collision detected!");
              ArrayList<Plane> simulatedPlanes = result.getPlanes();
              // locate planes and add walls
              for (int j = 0; j < simulatedPlanes.size(); j++) {
                Plane simulatedPlane = simulatedPlanes.get(j);
                Plane simulatedSelfPlane = simulatedPlanes.get(currentPlane);
                if (currentPlane != j) {
                  double distance = simulatedSelfPlane.getLocation().distance(simulatedPlane.getLocation());
                  if (distance <= collisionDistance) {
                    PlaneState simulatedPlaneState = simulatedPlaneStates.get(j);
                    if (simulatedPlaneState != null) {
                      logger.trace("collision!" 
                          + " plane1: " + simulatedSelfPlane.getLocation()
                          + " plane2: " + simulatedPlane.getLocation());
                      // create wall here
                      Vector alongPath = new Vector (simulatedPlane.getLocation(), 
                          simulatedPlaneState.path.peekFirst().point);
                      alongPath.normalize();
                      alongPath.multiply(safetyDistance);
                      Vector planeVector = new Vector (simulatedPlane.getLocation());
                      Vector safetyPointVector = Vector.addVectors(planeVector, alongPath);
                      Line2D wall = new Line2D.Double(simulatedPlane.getLocation(), safetyPointVector.getPoint());
                      // check if we are getting the same collision again
                      for (Line2D wall2: walls) {
                        if ((wall.getP1().equals(wall2.getP1()) && wall.getP2().equals(wall2.getP2()))
                            || (wall.getP1().equals(wall2.getP2()) && wall.getP2().equals(wall2.getP1()))) {
                          logger.trace("detect same collision twice. skip plane.");
                          wait = true;
                        } 
                      }
                      walls.add(wall);
                    }
                  }
                }
              }
              if (wait == true)
                break;
            } else {
              logger.trace ("simulation end reason: " + result.getReason());
              break;
            }
          } 
        }

        if (wait == true) {
          logger.trace ("destination unreachable for plane " + i);
          continue;
        }

        takeOff = true;

        if (simulating)
          logger.trace("calculate a-star in simulation, plane " + i);
        else
          logger.trace("calculate a-star, plane " + i);

        AStar astar = new AStar(walls, collisionDistance + 1);
        path = astar.AStarPath(plane.getLocation(), plane.getDestination());
        if (path == null) {
           logger.trace("plane: " + i + "can't take off yet");
          if (simulating == true) {
            if (i == currentPlane) {
              logger.trace("simulated plane can't take off yet. stop simulation.");
              // can't take-off yet. try later
              stopSimulation();
            }
          }
          // destination is unreachable at this time. 
          // some other plane is landing/taking-off?
          continue;
        }
      }


      Waypoint firstWaypoint = path.peekFirst();
      if (plane.getLocation().distance(firstWaypoint.point) < collisionDistance) {
        logger.trace("plane: " + i + " reached waypoint: " + firstWaypoint.point);
        if (path.size() > 1) { // keep the last element
          path.removeFirst();
          firstWaypoint = path.peekFirst();
        }
      }

      // head to first waypoint
      logger.trace("plane " + i + " heading to: " + firstWaypoint.point
          + " current location: " + plane.getLocation());
	    Vector currVec;
      if (bearings[i] != WAITING) {
        currVec = new Vector(bearings[i]);
      } else {
        currVec = new Vector(calculateBearing(plane.getLocation(), (Point2D.Double) firstWaypoint.point));
      }
      Vector goalVec = new Vector(calculateBearing(plane.getLocation(), (Point2D.Double) firstWaypoint.point));
      bearings[i] = currVec.rotateToward(goalVec, maxBearingDeg).getBearing();

      state.path = path;
      if (simulating) {
        logger.trace("updating simulate state for plane: " + i);
        simulatedPlaneStates.put(plane.id, state);
      } else {
        logger.trace("updating state for plane: " + i);
        planeStates.put(plane.id, state);
      }
      if (takeOff) {
        logger.trace("take-off plane: " + i);
        break;
      }
    }
    if ((allDone || bearings[currentPlane] == FINISHED) && simulating) {
      logger.trace("simulation stopped in round: " + round);
      stopSimulation();
    }

		return bearings;
	}
}
