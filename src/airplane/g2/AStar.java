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

public class AStar {
	private Set<Line2D> walls;
	private Set<Waypoint> waypointSet;
	private Set<Waypoint> originalWaypointSet;
	private Map<Waypoint, Set<Waypoint> > visibilityMap;
	private Map<Waypoint, Set<Waypoint> > originalVisibilityMap;
	
	private ArrayList<Line2D> lines = new ArrayList<Line2D>();

	static int waypointDist = 1;
	static int starDist = 1;
	static double invalidWaypointDist = 1;
	
	private Logger log = Logger.getLogger(this.getClass()); // for logging

	public AStar(Set<Line2D> inwalls, double safetyDistance) {
    this.walls = new HashSet<Line2D>();
		this.originalWaypointSet = new HashSet<Waypoint>();
		this.waypointSet = new HashSet<Waypoint>();
		this.visibilityMap = new HashMap<Waypoint, Set<Waypoint> >();
		this.originalVisibilityMap = new HashMap<Waypoint, Set<Waypoint> >();
		// init waypoints
		for (Line2D wall : inwalls) {
      this.walls.add(wall);
			log.info("Wall at (" + wall.getX1() + ", " + wall.getY1()
					+ ") to (" + wall.getX2() + ", " + wall.getY2() + ")");

			Vector p1 = new Vector(wall.getP1());
			Vector p2 = new Vector(wall.getP2());
			Vector lineTangent = new Vector(wall.getP2(), wall.getP1());
			lineTangent.normalize();
			lineTangent.multiply(safetyDistance);
			Vector along = lineTangent;
			Vector opposite = lineTangent.rotateOpposite();
			Vector cw = lineTangent.rotate90Clockwise();
			Vector acw = lineTangent.rotate90AntiClockwise();

			/*Vector triQuad1 = lineTangent.rotate(120);
			Vector triQuad2 = lineTangent.rotate(-120);
			Vector triQuadOpp1 = opposite.rotate(120);
			Vector triQuadOpp2 = opposite.rotate(-120);*/

      Vector p11 = Vector.addVectors(p1, cw);
      Vector p12 = Vector.addVectors(p1, acw);
      Vector p21 = Vector.addVectors(p2, cw);
      Vector p22 = Vector.addVectors(p2, acw);

			along.normalize();
			along.multiply(safetyDistance);
			opposite.normalize();
			opposite.multiply(safetyDistance);
      
      addWaypoint(p11, along, lines);
      addWaypoint(p12, along, lines);
      addWaypoint(p21, opposite, lines);
      addWaypoint(p22, opposite, lines);

      // Add 2 more walls
      Line2D wall1 = new Line2D.Double(p11.getPoint(), p21.getPoint());
      Line2D wall2 = new Line2D.Double(p12.getPoint(), p22.getPoint());
      this.walls.add(wall1);
      this.walls.add(wall2);
			log.info("Wall at (" + wall1.getX1() + ", " + wall1.getY1()
					+ ") to (" + wall1.getX2() + ", " + wall1.getY2() + ")");

			log.info("Wall at (" + wall2.getX1() + ", " + wall2.getY1()
					+ ") to (" + wall2.getX2() + ", " + wall2.getY2() + ")");


      /*along.normalize();
			along.multiply(waypointDist);
			opposite.normalize();
			opposite.multiply(waypointDist);

			p1 = Vector.addVectors(p1, along);
			p2 = Vector.addVectors(p2, opposite);

			along.normalize();
			along.multiply(starDist);
			opposite.normalize();
			opposite.multiply(starDist);

			addWaypoint(p1, along, lines);
			addWaypoint(p1, triQuad1, lines);
			addWaypoint(p1, triQuad2, lines);

			addWaypoint(p2, opposite, lines);
			addWaypoint(p2, triQuadOpp1, lines);
			addWaypoint(p2, triQuadOpp2, lines);*/
		}
		originalWaypointSet.addAll(waypointSet);
    originalVisibilityMap.putAll(visibilityMap);
	}

	protected boolean isInLineOfSight(double x, double y, double newX,
			double newY) {
		Line2D.Double pathLine = new Line2D.Double(x, y, newX, newY);
		for (Line2D l : walls) {
			if (l.intersectsLine(pathLine))
				return false;
		}
		return true;
	}
	
	protected boolean isInLineOfSight(Point2D p1, Point2D p2) {
		return isInLineOfSight(p1.getX(), p1.getY(), p2.getX(), p2.getY());
	}

	boolean checkOnWall(Point2D point) {
		for (Line2D wall : walls) {
			if (wall.ptSegDist(point) < 1) {
				return true;
			}
		}
		return false;
	}

	boolean inBoardRange(Point2D point) {
		if (point.getX() > 99 || point.getX() < 0 || point.getY() > 99
				|| point.getY() < 0)
			return false;
		else
			return true;
	}

	public double GetAStarDistance(Point2D source, Point2D target) {
		if (isInLineOfSight(source, target)) {
			return source.distance(target);
		}
		Deque<Waypoint> path = AStarPath(source, target);
		if (path != null) {
			Waypoint waypointLast = path.peekLast();
			/*
			 * Waypoint waypointFirst = path.peekFirst(); return
			 * (waypointLast.point.distance(target) +
			 * waypointFirst.point.distance(source) + GetPathDistance (path));
			 */
			return waypointLast.storedSourceDistance;
		} else {
			return 99;
		}
	}

	Waypoint addWaypoint(Vector p, Vector v, ArrayList<Line2D> lines) {
		Point2D point;
		Line2D line;
		point = Vector.addVectors(p, v).getPoint();
		// do not add if outside board
		if (point.getX() > 100 || point.getX() < 0 || point.getY() > 100
				|| point.getY() < 0)
			return null;
		Waypoint retVal = addToVisibilityMap(point);
		waypointSet.add(retVal);
    log.info("waypoint at: " + retVal.point);
		line = new Line2D.Double(p.getPoint(), point);
		lines.add(line);
		return retVal;
	}

	// A* path finding algo
	public Deque<Waypoint> AStarPath(Point2D source, Point2D target) {
    log.trace ("source: " + source + " target: " + target);
    // if in line-of-sight, simply return path with target as a waypoint.
    if (isInLineOfSight(source, target)) {
      Waypoint targetWP = new Waypoint(target);
      Deque<Waypoint> path = new ArrayDeque<Waypoint>();
      path.addFirst(targetWP);
      return path;
    }
		// revert to original waypointSet and visibilityMap
		waypointSet = new HashSet<Waypoint>();
		visibilityMap = new HashMap<Waypoint, Set<Waypoint>>();
		waypointSet.addAll(originalWaypointSet);
		visibilityMap.putAll(originalVisibilityMap);
		// add target to visibilityMap and waypointSet
		Waypoint waypointTarget = addToVisibilityMap(target);
		waypointSet.add(waypointTarget);
		// add source to visibilityMap and waypointSet
		Waypoint waypointSource = addToVisibilityMap(source);
		waypointSet.add(waypointSource);

		/*
		 * log.trace("visibilityMap: " + visibilityMap);
		 * log.trace("waypointSet: " + waypointSet);
		 */
		// Mark source waypoint as openList and calculate F score
		waypointSource.markAsOpenList();
		double sourceDistance = 0;
		waypointSource.updateFScore(sourceDistance, waypointTarget);

		Waypoint waypointCurrent = waypointSource;

		while (true) {
			// Check if unreachable
			if (waypointCurrent == null) {
				cleanupWaypoints(waypointSet);
        log.trace("Unreachable target!");
				return null;
			}
			// Move towards waypointTarget
			Set<Waypoint> visibleWaypoints = visibilityMap.get(waypointCurrent);
			for (Waypoint waypoint : visibleWaypoints) {
				if (!waypoint.closedList
						&& /* fail safe condition */waypoint != waypointCurrent) {
					double newDistance = waypointCurrent.currentSourceDistance
							+ Waypoint.getDistance(waypointCurrent, waypoint);
					if (newDistance < waypoint.currentSourceDistance
							|| waypoint.currentSourceDistance == -1) {
						waypoint.markAsOpenList();
						waypoint.setParent(waypointCurrent);
						waypoint.updateFScore(newDistance, waypointTarget);
						// Check for exit condition
						if (waypoint == waypointTarget) {
							Deque<Waypoint> path = getPath(waypointSet,
									waypointSource, waypointTarget);
							cleanupWaypoints(waypointSet);
							return path;
						}
					}
				}
			}
			waypointCurrent = leastFScore(waypointSet, waypointCurrent);
			// log.trace("waypointCurrent: " + waypointCurrent +
			// " waypointSource: " + waypointSource + " waypointTarget: " +
			// waypointTarget);
			if (waypointCurrent != null)
				waypointCurrent.markAsClosedList();
		}
	}

	private Waypoint addToVisibilityMap(Point2D target) {
		Waypoint waypoint1 = new Waypoint(target);
		Set<Waypoint> waypointSet1 = new HashSet<Waypoint>();
		visibilityMap.put(waypoint1, waypointSet1);
		for (Waypoint waypoint2 : waypointSet) {
			if (isInLineOfSight(waypoint1.point, waypoint2.point)) {
				Set<Waypoint> waypointSet2 = visibilityMap.get(waypoint2);
				waypointSet1.add(waypoint2);
				waypointSet2.add(waypoint1);
			}
		}
		return waypoint1;
	}

	private Waypoint leastFScore(Set<Waypoint> waypoints,
			Waypoint waypointCurrent) {
		Waypoint leastScoreWaypoint = null;
		double leastScore = -1;
		for (Waypoint waypoint : waypoints) {
			if ((waypoint.fScore < leastScore || leastScoreWaypoint == null)
					&& waypoint != waypointCurrent && waypoint.openList) {
				leastScore = waypoint.fScore;
				leastScoreWaypoint = waypoint;
			}
		}
		return leastScoreWaypoint;
	}

	public boolean checkClosedList(Set<Waypoint> waypoints,
			Waypoint waypointTarget) {
		for (Waypoint waypoint : waypoints) {
			if (waypoint == waypointTarget && waypoint.closedList) {
				return true;
			}
		}
		return false;
	}

	private void cleanupWaypoints(Set<Waypoint> waypoints) {
		for (Waypoint waypoint : waypoints) {
			waypoint.cleanup();
		}
	}

	private Deque<Waypoint> getPath(Set<Waypoint> waypoints,
			Waypoint waypointSource, Waypoint waypointTarget) {
		Waypoint waypointBacktrack = waypointTarget;
		Deque<Waypoint> path = new ArrayDeque<Waypoint>();
		while (true) {
			path.addFirst(waypointBacktrack);
			waypointBacktrack = waypointBacktrack.parent;
			if (waypointBacktrack == waypointSource) {
				cleanupWaypoints(waypointSet);
				return path;
			}
		}
	}

	public Point2D.Float getIntersectionPoint(Line2D line1, Line2D line2) {
		if (!line1.intersectsLine(line2))
			return null;
		double px = line1.getX1(), py = line1.getY1(), rx = line1.getX2() - px, ry = line1
				.getY2() - py;
		double qx = line2.getX1(), qy = line2.getY1(), sx = line2.getX2() - qx, sy = line2
				.getY2() - qy;

		double det = sx * ry - sy * rx;
		if (det == 0) {
			return null;
		} else {
			double z = (sx * (qy - py) + sy * (px - qx)) / det;
			if (z == 0 || z == 1)
				return null; // intersection at end point!
			return new Point2D.Float((float) (px + z * rx), (float) (py + z
					* ry));
		}
	} // end intersection line-line
}
