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
	// private Set<Line2D> orificeSet;
	private Set<Waypoint> waypointSet;
	private Set<Waypoint> originalWaypointSet;
	private Map<Waypoint, Set<Waypoint>> visibilityMap;
	private Map<Waypoint, Set<Waypoint>> originalVisibilityMap;
	// private Set<Waypoint> orificeWaypointSet;
	// private boolean checkOrifice = true;
	private double safetyDistance;

	private double wpDistance = 1;

	private ArrayList<Line2D> lines = new ArrayList<Line2D>();

	private Logger log = Logger.getLogger(this.getClass()); // for logging

	public AStar(Set<Line2D> inwalls, double sDistance) {
		this.walls = new HashSet<Line2D>();
		this.originalWaypointSet = new HashSet<Waypoint>();
		// this.orificeWaypointSet = new HashSet<Waypoint> ();
		this.waypointSet = new HashSet<Waypoint>();
		this.visibilityMap = new HashMap<Waypoint, Set<Waypoint>>();
		this.originalVisibilityMap = new HashMap<Waypoint, Set<Waypoint>>();
		// this.orificeSet = new HashSet<Line2D> ();
		this.safetyDistance = sDistance;

		// init waypoints
		for (Line2D wall : inwalls) {
			this.walls.add(wall);
			log.trace("Wall at (" + wall.getX1() + ", " + wall.getY1()
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

			/*
			 * Vector triQuad1 = lineTangent.rotate(120); Vector triQuad2 =
			 * lineTangent.rotate(-120); Vector triQuadOpp1 =
			 * opposite.rotate(120); Vector triQuadOpp2 = opposite.rotate(-120);
			 */

			Vector p11 = Vector.addVectors(p1, cw);
			Vector p12 = Vector.addVectors(p1, acw);
			Vector p21 = Vector.addVectors(p2, cw);
			Vector p22 = Vector.addVectors(p2, acw);

			along.normalize();
			along.multiply(safetyDistance);
			opposite.normalize();
			opposite.multiply(safetyDistance);

			// add waypoints
			// play around with cw and acw
			cw.normalize();
			cw.multiply(wpDistance);
			acw.normalize();
			acw.multiply(wpDistance);

			Vector nullVector = new Vector(0, 0);

			addWaypoint(Vector.addVectors(p11, cw), along, lines, false);
			addWaypoint(Vector.addVectors(p12, acw), along, lines, false);
			addWaypoint(Vector.addVectors(p21, cw), opposite, lines, false);
			addWaypoint(Vector.addVectors(p22, acw), opposite, lines, false);

			// Add 2 more parallel walls
			Line2D wall1 = new Line2D.Double(p11.getPoint(), p21.getPoint());
			Line2D wall2 = new Line2D.Double(p12.getPoint(), p22.getPoint());
			this.walls.add(wall1);
			this.walls.add(wall2);
			log.trace("Wall at (" + wall1.getX1() + ", " + wall1.getY1()
					+ ") to (" + wall1.getX2() + ", " + wall1.getY2() + ")");

			log.trace("Wall at (" + wall2.getX1() + ", " + wall2.getY1()
					+ ") to (" + wall2.getX2() + ", " + wall2.getY2() + ")");

			// Add 2 perpendicular walls to enclose the no-fly zone
			Line2D wall3 = new Line2D.Double(p11.getPoint(), p12.getPoint());
			Line2D wall4 = new Line2D.Double(p21.getPoint(), p22.getPoint());
			log.trace("Wall at (" + wall3.getX1() + ", " + wall3.getY1()
					+ ") to (" + wall3.getX2() + ", " + wall3.getY2() + ")");

			log.trace("Wall at (" + wall4.getX1() + ", " + wall4.getY1()
					+ ") to (" + wall4.getX2() + ", " + wall4.getY2() + ")");
			this.walls.add(wall3);
			this.walls.add(wall4);

			/*
			 * along.normalize(); along.multiply(waypointDist);
			 * opposite.normalize(); opposite.multiply(waypointDist);
			 * 
			 * p1 = Vector.addVectors(p1, along); p2 = Vector.addVectors(p2,
			 * opposite);
			 * 
			 * along.normalize(); along.multiply(starDist);
			 * opposite.normalize(); opposite.multiply(starDist);
			 * 
			 * addWaypoint(p1, along, lines, false); addWaypoint(p1, triQuad1,
			 * lines, false); addWaypoint(p1, triQuad2, lines, false);
			 * 
			 * addWaypoint(p2, opposite, lines, false); addWaypoint(p2,
			 * triQuadOpp1, lines, false); addWaypoint(p2, triQuadOpp2, lines,
			 * false);
			 */
		}
		originalWaypointSet.addAll(waypointSet);
		// remember orifice lines
		/*
		 * Set<Point2D> consideredPoints = new HashSet<Point2D>(); checkOrifice
		 * = false; for (Line2D line1 : walls) { Point2D point11 =
		 * line1.getP1(); consideredPoints.add(point11); Point2D point12 =
		 * line1.getP2(); consideredPoints.add(point12); for (Line2D line2 :
		 * walls) { if (line2 != line1) { Point2D point21 = line2.getP1();
		 * Point2D point22 = line2.getP2(); if (point11.distance(point21) <=
		 * safetyDistance) { if (!consideredPoints.contains(point21)) { Line2D
		 * opaque = new Line2D.Double(point11, point21); orificeSet.add(opaque);
		 * Point2D point = new Point2D.Double((point11.getX() + point21.getX())
		 * / 2, (point11.getY() + point21.getY()) / 2); Waypoint waypoint =
		 * addToVisibilityMap(point, true); Vector lineTangent = new
		 * Vector(point11, point21); lineTangent.normalize();
		 * lineTangent.multiply(safetyDistance); Vector along = lineTangent;
		 * Vector opposite = lineTangent.rotateOpposite(); Vector cw =
		 * lineTangent.rotate90Clockwise(); Vector acw =
		 * lineTangent.rotate90AntiClockwise(); Vector v = new Vector(point);
		 * addWaypoint(v, cw, lines, true); addWaypoint(v, acw, lines, true); }
		 * } if (point11.distance(point22) <= safetyDistance) { if
		 * (!consideredPoints.contains(point22)) { Line2D opaque = new
		 * Line2D.Double(point11, point22); orificeSet.add(opaque); Point2D
		 * point = new Point2D.Double((point11.getX() + point22.getX()) / 2,
		 * (point11.getY() + point22.getY()) / 2); Waypoint waypoint =
		 * addToVisibilityMap(point, true); Vector lineTangent = new
		 * Vector(point11, point22); lineTangent.normalize();
		 * lineTangent.multiply(safetyDistance); Vector along = lineTangent;
		 * Vector opposite = lineTangent.rotateOpposite(); Vector cw =
		 * lineTangent.rotate90Clockwise(); Vector acw =
		 * lineTangent.rotate90AntiClockwise(); Vector v = new Vector(point);
		 * addWaypoint(v, cw, lines, true); addWaypoint(v, acw, lines, true); }
		 * } if (point12.distance(point21) <= safetyDistance) { if
		 * (!consideredPoints.contains(point21)) { Line2D opaque = new
		 * Line2D.Double(point12, point21); orificeSet.add(opaque); Point2D
		 * point = new Point2D.Double((point12.getX() + point21.getX()) / 2,
		 * (point12.getY() + point21.getY()) / 2); Waypoint waypoint =
		 * addToVisibilityMap(point, true); Vector lineTangent = new
		 * Vector(point12, point21); lineTangent.normalize();
		 * lineTangent.multiply(safetyDistance); Vector along = lineTangent;
		 * Vector opposite = lineTangent.rotateOpposite(); Vector cw =
		 * lineTangent.rotate90Clockwise(); Vector acw =
		 * lineTangent.rotate90AntiClockwise(); Vector v = new Vector(point);
		 * addWaypoint(v, cw, lines, true); addWaypoint(v, acw, lines, true); }
		 * } if (point12.distance(point22) <= safetyDistance) { if
		 * (!consideredPoints.contains(point22)) { Line2D opaque = new
		 * Line2D.Double(point12, point22); orificeSet.add(opaque); Point2D
		 * point = new Point2D.Double((point12.getX() + point22.getX()) / 2,
		 * (point12.getY() + point22.getY()) / 2); Waypoint waypoint =
		 * addToVisibilityMap(point, true); Vector lineTangent = new
		 * Vector(point12, point22); lineTangent.normalize();
		 * lineTangent.multiply(safetyDistance); Vector along = lineTangent;
		 * Vector opposite = lineTangent.rotateOpposite(); Vector cw =
		 * lineTangent.rotate90Clockwise(); Vector acw =
		 * lineTangent.rotate90AntiClockwise(); Vector v = new Vector(point);
		 * addWaypoint(v, cw, lines, true); addWaypoint(v, acw, lines, true); }
		 * } } } } checkOrifice = true; orificeWaypointSet.addAll(waypointSet);
		 */
		originalVisibilityMap.putAll(visibilityMap);
	}

	public boolean isInLineOfSight(double x, double y, double newX,
			double newY) {
		Line2D.Double pathLine = new Line2D.Double(x, y, newX, newY);
		for (Line2D l : walls) {
			if (l.intersectsLine(pathLine))
				return false;
		}
		/*
		 * if (checkOrifice) { // check orifice: make opaque Point2D point1 =
		 * new Point2D.Double(x, y); Point2D point2 = new Point2D.Double(newX,
		 * newY); if (checkSweep(point1, point2) || checkSweep(point2, point1))
		 * { return false; } }
		 */
		return true;
	}

	boolean checkSweep(Point2D point1, Point2D point2) {
		Vector point1Vector = new Vector(point1);
		Vector point2Vector = new Vector(point1);
		Vector lineOfSightVector = new Vector(point1, point2);

		Line2D line = new Line2D.Double(point1, point2);

		double minPositiveDist = safetyDistance;
		double minNegativeDist = -safetyDistance;
		Set<Point2D> positiveWallPoints = new HashSet<Point2D>();
		Set<Point2D> negativeWallPoints = new HashSet<Point2D>();
		for (Line2D l : walls) {
			Point2D wallPoint = l.getP1();
			double dist = line.ptSegDist(wallPoint);
			boolean relativePos = line.relativeCCW(wallPoint) > 0;
			if (relativePos && dist < minPositiveDist)
				positiveWallPoints.add(wallPoint);
			if (!relativePos && dist > minNegativeDist)
				negativeWallPoints.add(wallPoint);
			wallPoint = l.getP2();
			dist = line.ptSegDist(wallPoint);
			relativePos = line.relativeCCW(wallPoint) > 0;
			if (relativePos && dist < minPositiveDist)
				positiveWallPoints.add(wallPoint);
			if (!relativePos && dist > minNegativeDist)
				negativeWallPoints.add(wallPoint);
		}
		for (Point2D p1 : positiveWallPoints) {
			for (Point2D p2 : negativeWallPoints) {
				if (p1.distance(p2) <= safetyDistance) {
					Line2D orificeLine = new Line2D.Double(p1, p2);
					boolean relativePoint1 = orificeLine.relativeCCW(point1) > 0;
					boolean relativePoint2 = orificeLine.relativeCCW(point2) > 0;
					if (relativePoint1 != relativePoint2) {
						// log.trace("orifice!");
						return true;
					}
				}
			}
		}
		return false;
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
		if (point.getX() > 99 || point.getX() < 1 || point.getY() > 99
				|| point.getY() < 1)
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

	Waypoint addWaypoint(Vector p, Vector v, ArrayList<Line2D> lines,
			boolean orificeWaypoint) {
		Point2D point;
		Line2D line;
		point = Vector.addVectors(p, v).getPoint();
		// do not add if outside board
		if (point.getX() > 99 || point.getX() < 1 || point.getY() > 99
				|| point.getY() < 1)
			return null;
		// do not add if near wall
		/*
		 * Point2D wallPoint = getNearWallPoint(point, safetyDistance); if
		 * (wallPoint != null && !orificeWaypoint) return null;
		 */
		Waypoint retVal = addToVisibilityMap(point, orificeWaypoint);
		waypointSet.add(retVal);
		log.trace("waypoint at: " + retVal.point);
		line = new Line2D.Double(p.getPoint(), point);
		lines.add(line);
		return retVal;
	}

	Point2D getNearWallPoint(Point2D point, double minDistance) {
		for (Line2D wall : walls) {
			if (point.distance(wall.getP1()) < minDistance) {
				return wall.getP1();
			} else if (point.distance(wall.getP2()) < minDistance) {
				return wall.getP2();
			}
		}
		return null;
	}

	// A* path finding algo
	public Deque<Waypoint> AStarPath(Point2D source, Point2D target) {
		log.trace("source: " + source + " target: " + target);
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
		// if (checkOrifice)
		waypointSet.addAll(originalWaypointSet);
		/*
		 * else waypointSet.addAll(orificeWaypointSet);
		 */

		visibilityMap.putAll(originalVisibilityMap);
		// add target to visibilityMap and waypointSet
		Waypoint waypointTarget = addToVisibilityMap(target, false);
		waypointSet.add(waypointTarget);
		// add source to visibilityMap and waypointSet
		Waypoint waypointSource = addToVisibilityMap(source, false);
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
				if (!waypoint.closedList /*
										 * && (!waypoint.orificeWaypoint ||
										 * !checkOrifice*
										 */
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

	private Waypoint addToVisibilityMap(Point2D target, boolean orificeWaypoint) {
		Waypoint waypoint1 = new Waypoint(target);
		if (orificeWaypoint)
			waypoint1.orificeWaypoint = true;
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
