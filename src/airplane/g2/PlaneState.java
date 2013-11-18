package airplane.g2;

import java.util.Deque;
import airplane.g2.Waypoint;
import java.awt.geom.Point2D;

public class PlaneState
{
  Deque<Waypoint> path = null;
  Deque<Waypoint> fullPath = null; // full, original path
  Point2D target = null;
}
