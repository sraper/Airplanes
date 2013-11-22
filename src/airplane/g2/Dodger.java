package airplane.g2;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.apache.log4j.Logger;

import airplane.sim.*;

public class Dodger extends airplane.sim.Player {
	private Logger logger = Logger.getLogger(this.getClass()); // for logging

	private Map<Integer, PlaneState> planeStates;
	private Map<Integer, PlaneState> simulatedPlaneStates;
	private Set<Line2D> walls;

  private ArrayList<Line2D> lines;

	private static final int FINISHED = -2;
	private static final int WAITING = -1;

	// knobs
	private static final double maxBearingDeg = 9.9;
	private static final double collisionDistance = 5;
	private static final double velocity = 1;
	private static final int maxSimulationRounds = 200; // prevent infinite
														// orbiting...

	private double safetyDistance = 7; // TODO: try tweaking this...
	private boolean simulating;
	private int currentPlane; // used while simulating
	private int simulationRound = 0;

  private HashSet<Integer> takenOff;
	
	private HashMap<PointTuple, Integer> flows;

	@Override
	public String getName() {
		return "Dodger";
	}

	/*
	 * This is called at the beginning of a new simulation. Each Plane object
	 * includes its current location (origin), destination, and current bearing,
	 * which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
    lines = new ArrayList<Line2D>();
		simulating = false;
		planeStates = new HashMap<Integer, PlaneState>();
		walls = new HashSet<Line2D>();
    takenOff = new HashSet<Integer>();
		
		for (int i = 0; i < planes.size(); i++) {
			Plane plane = planes.get(i);
			plane.id = i;
			logger.info("init plane " + i + ", location: "
					+ plane.getLocation() + " destination: "
					+ plane.getDestination() + " departure: "
					+ plane.getDepartureTime());
		}
		// initial naive sort by path distance
		Collections.sort(planes, new PlaneSorter());
		//Collections.sort(planes, new IdealIntersectionSorter(planes));

		flows = new HashMap<PointTuple, Integer>();
		for (Plane p : planes) {
			PointTuple pt = new PointTuple(p.getLocation(), p.getDestination());
			flows.put(pt, flows.containsKey(pt) ? flows.get(pt) + 1 : 1);
		}
		Set<Map.Entry<PointTuple, Integer>> myset = new HashSet<Map.Entry<PointTuple, Integer>>();
		myset.addAll(flows.entrySet());
		for (Entry<PointTuple, Integer> pt : myset) {
			if (pt.getValue() < 5) {
				flows.remove(pt.getKey());
			}
		}

		setFlowPaths(planes);
		
		
		
		logger.info(flows.toString());


		
		
		/*
		 * double safetyDistanceHorizontal = 0; double safetyDistanceVertical =
		 * 0; // should increase to collisionDistance double safetyMoves = 0; //
		 * calculate safety distance for (double i = 0; i <= 90; i = i +
		 * maxBearingDeg) { safetyDistanceHorizontal += velocity *
		 * Math.cos(Math.toRadians(i)); safetyDistanceVertical += velocity *
		 * Math.sin(Math.toRadians(i)); safetyMoves++; if
		 * (safetyDistanceVertical >= collisionDistance) { break; } } if
		 * (safetyDistanceVertical < collisionDistance) { safetyMoves +=
		 * Math.ceil((collisionDistance - safetyDistanceVertical)/velocity);
		 * safetyDistanceHorizontal += (safetyMoves * velocity); } logger.info
		 * ("safetyMoves: " + safetyMoves + " h: " + safetyDistanceHorizontal +
		 * " v: " + safetyDistanceVertical);
		 * 
		 * safetyDistance = Math.ceil(safetyDistanceHorizontal) +
		 * (safetyMoves*velocity); // add distance covered by opposite plane
		 */
		logger.info("safety distance: " + safetyDistance);
		logger.info("Starting new game!");
	}

	private void setFlowPaths(ArrayList<Plane> planes) {
		Set<Map.Entry<PointTuple, Integer>> myset2 = new HashSet<Map.Entry<PointTuple, Integer>>();
		myset2.addAll(flows.entrySet());
		double flowsafety = this.safetyDistance + .1;
		
		for (Entry<PointTuple, Integer> pt : myset2) {
			AStar as = new AStar(walls, flowsafety);
			Point2D p1 = pt.getKey().a;
			Point2D p2 = pt.getKey().b;
//			if(as.isInLineOfSight(p1.getX(),p1.getY(), p2.getX(), p2.getY())) {
//				for (Plane p : planes) {
//					if(p.getLocation().equals(p1) && p.getDestination().equals(p2)) {
//						PlaneState ps = new PlaneState();
//						ps.fullPath = as.AStarPath(p1, p2);
//						ps.path = new ArrayDeque<Waypoint>();
//						ps.path.addAll(ps.fullPath);
//						ps.target = p2;
//						planeStates.put(p.id, ps);
//					}
//				}
//				walls.add(new Line2D.Double(p1, p2));
//			} else {
			
			if (!myset2.contains(new PointTuple(p2, p1))) {
			
				Deque<Waypoint> dq = as.AStarPath(p1, p2);
				if (dq != null) {
					for (Plane p : planes) {
						if(p.getLocation().equals(p1) && p.getDestination().equals(p2)) {
							PlaneState ps = new PlaneState();
							ps.fullPath = dq;
							ps.path = new ArrayDeque<Waypoint>();
							ps.path.addAll(ps.fullPath);
							ps.target = p2;
							planeStates.put(p.id, ps);
						}
					}
					Point2D start = p1;
					Point2D end;
					for(Waypoint w : dq) {
						end = dq.removeFirst().point;
						walls.add(new Line2D.Double(start, end));
						start = end;
					}
				}
			}
		}
		logger.info("hello");
		for(Line2D l : walls) {
			logger.info(l.getP1() + ", " + l.getP2());
		}
	}
	
	

	@Override
	public double[] simulateUpdate(ArrayList<Plane> planes, int round,
			double[] bearings) {
		simulating = true;
		bearings = updatePlanes(planes, round, bearings);
		simulating = false;
		return bearings;
	}

	/*
	 * This is called at each step of the simulation. The List of Planes
	 * represents their current location, destination, and current bearing; the
	 * bearings array just puts these all into one spot. This method should
	 * return an updated array of bearings.
	 */
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round,
			double[] bearings) {
		for (Plane p : planes) {
			if (simulating == false)
				logger.trace("Start: " + p.getLocation() + ", End: "
						+ p.getDestination());
		}
		boolean allDone = true;
		boolean wait = false;

		if (simulating == false) {
      lines.clear();
			logger.trace("round: " + round);
		} else {
			simulationRound++;
			logger.trace("simulation round: " + simulationRound
					+ ", simulating round: " + round);
		}

		// save ids
//		if (round == 1 && simulating == false) {
//			for (int i = 0; i < planes.size(); i++) {
//				Plane plane = planes.get(i);
//				plane.id = i;
//				logger.info("init plane " + i + ", location: "
//						+ plane.getLocation() + " destination: "
//						+ plane.getDestination() + " departure: "
//						+ plane.getDepartureTime());
//			}
//		}

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
			} else if (bearings[i] == WAITING && simulating && !takenOff.contains(i) && i != currentPlane) {
        logger.trace("not taking off plane: " + i + " in simulation"
            + " current plane: " + currentPlane);
        // do not take-off any new planes in simulation except the
        // currentPlane
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
					logger.trace("create new simulated state for plane: " + i);
				} else {
					logger.trace("create new state for plane: " + i);
				}
				state = new PlaneState();
			}

			path = state.path;

			if (path == null) { // need to choose path for this plane
				logger.trace("path is null for plane: " + i);
				if (simulating == false) {
					logger.trace("start simulation plane: " + currentPlane);
					// detect collision points and place walls
					SimulationResult result;
					while (true) {
						// make copies
						simulatedPlaneStates = new HashMap<Integer, PlaneState>();
						for (int k = 0; k < planes.size(); k++) {
							PlaneState origState = planeStates.get(planes
									.get(k).id);
							if (origState != null) {
								PlaneState newState = new PlaneState();
								if (origState.path != null) {
									logger.trace("copy to simulated state, path exists for plane: "
											+ k);
									newState.path = new ArrayDeque<Waypoint>(
											origState.path);
								}
								newState.target = origState.target;
								simulatedPlaneStates.put(planes.get(k).id,
										newState);
							}
						}
						simulationRound = 0;
						result = startSimulation(planes, round);
						logger.trace("simulation took: " + simulationRound
								+ " rounds");
						if (simulationRound > maxSimulationRounds) {
							wait = true; // abort re-simulation
							break;
						}
						if (result.getReason() == SimulationResult.TOO_CLOSE) {
							logger.trace("collision detected!");
							int origWallNum = walls.size();
							ArrayList<Plane> simulatedPlanes = result
									.getPlanes();
							// locate planes and add walls
							for (int j = 0; j < simulatedPlanes.size(); j++) {
								Plane simulatedPlane = simulatedPlanes.get(j);
								Plane simulatedSelfPlane = simulatedPlanes
										.get(currentPlane);
								if (simulatedPlane.getBearing() == FINISHED
										|| simulatedPlane.getBearing() == WAITING) {
									continue;
								}
								if (currentPlane != j) {
									double distance = simulatedSelfPlane
											.getLocation().distance(
													simulatedPlane
															.getLocation());
									if (distance <= collisionDistance) {
										PlaneState simulatedPlaneState = simulatedPlaneStates
												.get(simulatedPlane.id);
										if (simulatedPlaneState != null) {
											logger.trace("collision!"
													+ " plane "
													+ currentPlane
													+ ": "
													+ simulatedSelfPlane
															.getLocation()
													+ " plane "
													+ j
													+ ": "
													+ simulatedPlane
															.getLocation());
											// create wall here
											Vector alongPath = new Vector(
													simulatedPlane
															.getLocation(),
													simulatedPlaneState.path
															.peekFirst().point);
											alongPath.normalize();
											alongPath.multiply(safetyDistance);
											Vector planeVector = new Vector(
													simulatedPlane
															.getLocation());
											Vector safetyPointVector = Vector
													.addVectors(planeVector,
															alongPath);
											Line2D wall = new Line2D.Double(
													simulatedPlane
															.getLocation(),
													safetyPointVector
															.getPoint());
											// check if we are getting the same
											// collision again
											for (Line2D wall2 : walls) {
												if ((wall.getP1().equals(
														wall2.getP1()) && wall
														.getP2().equals(
																wall2.getP2()))
														|| (wall.getP1()
																.equals(wall2
																		.getP2()) && wall
																.getP2()
																.equals(wall2
																		.getP1()))) {
													logger.trace("detect same collision twice. skip plane.");
													wait = true;
												}
											}
                      wallTrace(wall);
											walls.add(wall);
										}
									}
								}
							}
							if (origWallNum == walls.size()) {
								logger.trace("added no new walls during simulation, abort.");
								// added no new walls, abort
								wait = true;
							}
							if (wait == true)
								break;

						} else if (result.getReason() == SimulationResult.STOPPED
								|| result.getReason() == SimulationResult.NORMAL) {
							logger.trace("simulation end reason: "
									+ result.getReason());
							break;
						} else {
							wait = true;
							logger.trace("simulation end reason: "
									+ result.getReason());
							break;
						}
					}
					if (wait == true) {
						logger.trace("destination unreachable for plane " + i);
						continue;
					}

					// path is available, retrieve from simulated state
					logger.trace("no collisions detected. retrieving path for plane: "
							+ i);
					ArrayList<Plane> simulatedPlanes = result.getPlanes();
					Plane simulatedSelfPlane = simulatedPlanes
							.get(currentPlane);
					PlaneState simulatedPlaneState = simulatedPlaneStates
							.get(simulatedSelfPlane.id);
					if (simulatedPlaneState != null) {
						path = simulatedPlaneState.fullPath;
						state.path = path;
						state.fullPath = new ArrayDeque<Waypoint>(path);
					} else {
						logger.trace("path is null, destination unreachable for plane " + i);
						wait = true;
						continue;
					}
				}

				if (simulating == true) { // run a-star only in simulation mode
											// to make it deterministic
					logger.trace("calculate a-star in simulation, plane " + i);
					AStar astar = new AStar(walls, collisionDistance);
					path = astar.AStarPath(plane.getLocation(),
							plane.getDestination());
					if (path == null) {
						logger.trace("plane: " + i + "can't take off yet");
						if (i == currentPlane) {
							logger.trace("simulated plane can't take off yet. stop simulation.");
							// can't take-off yet. try later
							stopSimulation();
						}
						// destination is unreachable at this time.
						// some other plane is landing/taking-off?
						continue;
					}
					state.path = path;
					state.fullPath = new ArrayDeque<Waypoint>(path); // save
																		// full-path
				}
			}

			if (path != null) {
				if(bearings[i] == WAITING && planeTooClose(plane, planes, bearings)) {
					bearings[i] = WAITING;
				} else {
					Waypoint firstWaypoint = path.peekFirst();
					if (Math.abs(plane.getLocation().distance(firstWaypoint.point)) <= collisionDistance) {
						if (simulating == false) {
							logger.trace("plane: " + i + " reached waypoint: "
									+ firstWaypoint.point);
						}
						if (path.size() > 1) { // keep the last element
							path.removeFirst();
							firstWaypoint = path.peekFirst();
						}
					}
	
					// head to first waypoint
					logger.trace("plane " + i + " heading to: "
							+ firstWaypoint.point + " current location: "
							+ plane.getLocation());
					Vector currVec;
					if (bearings[i] != WAITING) {
						currVec = new Vector(bearings[i]);
					} else {
            if (simulating == false) {
              takenOff.add(i);
              for (Line2D wall: walls) {
                wallTrace(wall);
              }  
            }
						currVec = new Vector(calculateBearing(plane.getLocation(),
								(Point2D.Double) firstWaypoint.point));
					}
					Vector goalVec = new Vector(calculateBearing(
							plane.getLocation(),
							(Point2D.Double) firstWaypoint.point));
	
					bearings[i] = currVec.rotateToward(goalVec, maxBearingDeg)
							.getBearing();
					state.path = path;
					if (simulating) {
						logger.trace("updating simulate state for plane: " + i);
						simulatedPlaneStates.put(plane.id, state);
					} else {
						logger.trace("updating state for plane: " + i);
						planeStates.put(plane.id, state);
					}

          /*if (simulating == false) {
            // wall trace
            Vector alongPath = new Vector(plane.getLocation(), 
                state.path.peekFirst().point);
            alongPath.normalize();
            alongPath.multiply(safetyDistance);
            Vector planeVector = new Vector( plane .getLocation());
            Vector safetyPointVector = Vector.addVectors(planeVector,
                    alongPath);
            Line2D wall = new Line2D.Double(plane.getLocation(), 
                safetyPointVector.getPoint());
            lines.add(wall);

            Vector p1 = new Vector(wall.getP1());
            Vector p2 = new Vector(wall.getP2());
            Vector lineTangent = new Vector(wall.getP2(), wall.getP1());
            lineTangent.normalize();
            lineTangent.multiply(safetyDistance);
            Vector along = lineTangent;
            Vector opposite = lineTangent.rotateOpposite();
            Vector cw = lineTangent.rotate90Clockwise();
            Vector acw = lineTangent.rotate90AntiClockwise();
            Vector p11 = Vector.addVectors(p1, cw);
            Vector p12 = Vector.addVectors(p1, acw);
            Vector p21 = Vector.addVectors(p2, cw);
            Vector p22 = Vector.addVectors(p2, acw);
            Line2D wall1 = new Line2D.Double(p11.getPoint(), p21.getPoint());
            Line2D wall2 = new Line2D.Double(p12.getPoint(), p22.getPoint());
            Line2D wall3 = new Line2D.Double(p11.getPoint(), p12.getPoint());
            Line2D wall4 = new Line2D.Double(p21.getPoint(), p22.getPoint());
            lines.add(wall1);
            lines.add(wall2);
            lines.add(wall3);
            lines.add(wall4);
          }*/
				}
			}
		}
		if (simulating
				&& (allDone || bearings[currentPlane] == FINISHED || simulationRound > maxSimulationRounds)) {
			logger.trace("simulation stopped in round: " + round);
			stopSimulation();
		}
    setPlayerLines(lines);

		return bearings;
	}
	private boolean planeTooClose(Plane p, ArrayList<Plane> planes, double[] bearings) {
		for (int i = 0; i < planes.size(); i++) {
			Plane o = planes.get(i);
			if (p != o && bearings[i] >= 0 && p.getLocation().distance(o.getLocation()) < 5.01) return true;
		}
		return false;
	}

  private void wallTrace (Line2D wall) {
    // wall trace
    lines.add(wall);
    Vector p1 = new Vector(wall.getP1());
    Vector p2 = new Vector(wall.getP2());
    Vector lineTangent = new Vector(wall.getP2(), wall.getP1());
    lineTangent.normalize();
    lineTangent.multiply(collisionDistance);
    Vector along = lineTangent;
    Vector opposite = lineTangent.rotateOpposite();
    Vector cw = lineTangent.rotate90Clockwise();
    Vector acw = lineTangent.rotate90AntiClockwise();
    Vector p11 = Vector.addVectors(p1, cw);
    Vector p12 = Vector.addVectors(p1, acw);
    Vector p21 = Vector.addVectors(p2, cw);
    Vector p22 = Vector.addVectors(p2, acw);
    Line2D wall1 = new Line2D.Double(p11.getPoint(), p21.getPoint());
    Line2D wall2 = new Line2D.Double(p12.getPoint(), p22.getPoint());
    Line2D wall3 = new Line2D.Double(p11.getPoint(), p12.getPoint());
    Line2D wall4 = new Line2D.Double(p21.getPoint(), p22.getPoint());
    lines.add(wall1);
    lines.add(wall2);
    lines.add(wall3);
    lines.add(wall4);

    along.normalize();
    along.multiply(collisionDistance);
    opposite.normalize();
    opposite.multiply(collisionDistance);

    // add waypoints
    // play around with cw and acw
    cw.normalize();
    cw.multiply(1);
    acw.normalize();
    acw.multiply(1);
    
    {
      Point2D point = Vector.addVectors(Vector.addVectors(p11, cw), along).getPoint();
      Line2D wp = new Line2D.Double(point, point);
      lines.add(wp);
    }
    {
      Point2D point = Vector.addVectors(Vector.addVectors(p12, acw), along).getPoint();
      Line2D wp = new Line2D.Double(point, point);
      lines.add(wp);
    }
    {
      Point2D point = Vector.addVectors(Vector.addVectors(p21, cw), opposite).getPoint();
      Line2D wp = new Line2D.Double(point, point);
      lines.add(wp);
    }
    {
      Point2D point = Vector.addVectors(Vector.addVectors(p22, acw), opposite).getPoint();
      Line2D wp = new Line2D.Double(point, point);
      lines.add(wp);
    }
  }

}

class PointTuple {
	Point2D a;
	Point2D b;

	public PointTuple(Point2D a, Point2D b) {
		this.a = a;
		this.b = b;
	}

	@Override
	public String toString() {
		return "(" + a + ", " + b + ")";
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((a == null) ? 0 : a.hashCode());
		result = prime * result + ((b == null) ? 0 : b.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		PointTuple other = (PointTuple) obj;
		if (a == null) {
			if (other.a != null)
				return false;
		} else if (!a.equals(other.a))
			return false;
		if (b == null) {
			if (other.b != null)
				return false;
		} else if (!b.equals(other.b))
			return false;
		return true;
	}

}

class PlaneSorter implements Comparator<Plane> {
	@Override
	public int compare(Plane arg0, Plane arg1) {
		double d0 = dist(arg0.getX(), arg0.getY(),
				arg0.getDestination().getX(), arg0.getDestination().getY());
		double d1 = dist(arg1.getX(), arg1.getY(),
				arg1.getDestination().getX(), arg1.getDestination().getY());
		if (d0 < d1) {
			return 1;
		} else if (d0 > d1) {
			return -1;
		} else {
			return 0;
		}
	}

	private double dist(double x, double y, double x2, double y2) {
		return Math.sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));
	}
}

class IdealIntersectionSorter implements Comparator<Plane> {

	HashMap<Plane, Integer> numIntersections = new HashMap<Plane, Integer>();

	public IdealIntersectionSorter(ArrayList<Plane> planes) {
		HashMap<Plane, Line2D.Double> idealPaths = new HashMap<Plane, Line2D.Double>();

		for (Plane p : planes) {
			idealPaths.put(p,
					new Line2D.Double(p.getLocation(), p.getDestination()));
		}

		for (Plane p : planes) {
			Line2D.Double path = idealPaths.get(p);
			for (Line2D.Double o : idealPaths.values()) {
				if (path == o) {
					continue;
				} else if (path.intersectsLine(o)) {
					numIntersections.put(
							p,
							numIntersections.containsKey(p) ? numIntersections
									.get(p) + 1 : 1);
				}
			}
		}
	}

	@Override
	public int compare(Plane p0, Plane p1) {
		Integer n0 = numIntersections.get(p0);
		Integer n1 = numIntersections.get(p1);
		if (n0 == null || n1 == null)
			return 0;

		return (n0 == n1 ? 0 : (n0 > n1 ? -1 : 1));
	}
}
