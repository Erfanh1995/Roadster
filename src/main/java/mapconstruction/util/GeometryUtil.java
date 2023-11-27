package mapconstruction.util;

import com.google.common.collect.Range;
import com.google.common.collect.Sets;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.distance.TrajectoryDistance;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * Utility Class with convenience methods involving geometry.
 *
 * @author Roel
 * @author Jorrick Sleijster
 */
public final class GeometryUtil {

    /**
     * Determines the length of the given Line2D
     *
     * @param line
     * @return
     */
    public static double lineLength(Line2D line) {
        return line.getP1().distance(line.getP2());
    }

    /**
     * Gets the point for the given parameter in the parametric representation
     * of the line.
     *
     * @param line Line to get the point from
     * @param t    Line parameter. Must be between 0 and 1.
     * @return Point {@code p = line.P2 * t - (t-1) * line.P1}
     */
    public static Point2D getPointOnLine(Line2D line, double t) {
        if (DoubleMath.fuzzyEquals(t, 0.0, 1E-4)) {
            return line.getP1();
        }
        if (DoubleMath.fuzzyEquals(t, 1.0, 1E-4)) {
            return line.getP2();
        }
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException("Parameter out of range: " + t);
        }

        double x = line.getX2() * t + (1 - t) * line.getX1();
        double y = line.getY2() * t + (1 - t) * line.getY1();

        if (Double.isNaN(x) || Double.isNaN(y)) {
            System.out.println("Error in GeometryUtil.GetTrajectoryDecimalPoint. Values are NaN");
        }
        return new Point2D.Double(x, y);
    }

    /**
     * Returns the parameters for the parametric representation of the line
     * segment for which the line segment intersects the circle.
     * <p>
     * This may be 0, 1 or 2 parameters. The parameters are always between 0 and
     * 1.
     * <p>
     * The result list is sorted from low to high.
     *
     * @param line       Line to check
     * @param circCenter center point of the circle
     * @param circRadius radius of the circle.
     * @return
     */
    public static List<Double> segCircIntersectionParams(Line2D line, Point2D circCenter, double circRadius) {
        /*
        We compute the the parameters as follows.

        The parametric representation of the line is given by:
        x = t * x2 + (1 - t) * x1
        y = t * y2 + (1 - y) * y2

        The representation of the circle is

        (x - circX)^2 + (y - circY)^2 = radius^2

        By substituting x and y for the parametric representation, we get
        a quadratic equation for t, which we can solve.
         */
        // For convenience, translate everything such that the circle center is the origin
        double x1 = line.getX1() - circCenter.getX();
        double x2 = line.getX2() - circCenter.getX();
        double y1 = line.getY1() - circCenter.getY();
        double y2 = line.getY2() - circCenter.getY();

        // coeffiecients of quadratic equation
        double a = x2 * x2 + x1 * x1 + y1 * y1 + y2 * y2 - 2 * x1 * x2 - 2 * y1 * y2;
        double b = 2 * x1 * x2 + 2 * y1 * y2 - 2 * x1 * x1 - 2 * y1 * y1;
        double c = x1 * x1 + y1 * y1 - circRadius * circRadius;

        // solutions
        double D = b * b - 4 * a * c;
        int comp = DoubleMath.fuzzyCompare(D, 0, 1E-6);
        if (comp < 0) {
            // no solution
            return Collections.emptyList();
        } else if (comp == 0) {
            double t = (-b) / (2 * a);
            // compensate for rounding errors
            if (DoubleMath.fuzzyEquals(t, 0, 1E-6)) {
                t = 0;
            } else if (DoubleMath.fuzzyEquals(t, 1, 1E-6)) {
                t = 1;
            }
            if (t >= 0 && t <= 1) {
                // valid
                return Collections.singletonList(t);
            } else {
                return Collections.emptyList();
            }
        } else {
            // two solutions;
            double t1 = (-b - Math.sqrt(D)) / (2 * a);
            double t2 = (-b + Math.sqrt(D)) / (2 * a);

            // compensate for rounding errors
            if (DoubleMath.fuzzyEquals(t1, 0, 1E-6)) {
                t1 = 0;
            } else if (DoubleMath.fuzzyEquals(t1, 1, 1E-6)) {
                t1 = 1;
            }

            if (DoubleMath.fuzzyEquals(t2, 0, 1E-6)) {
                t2 = 0;
            } else if (DoubleMath.fuzzyEquals(t2, 1, 1E-6)) {
                t2 = 1;
            }

            List<Double> result = new ArrayList<>();
            if (t1 >= 0 && t1 <= 1) {
                // valid
                result.add(t1);
            }
            if (t2 >= 0 && t2 <= 1) {
                // valid
                result.add(t2);
            }
            result.sort(Comparator.naturalOrder());
            return result;
        }

    }


    public static List<Double> segCircIntersectionParams(Line2D line, Ellipse2D circle) {
        return segCircIntersectionParams(line, new Point2D.Double(circle.getCenterX(), circle.getCenterY()), circle.getWidth() / 2);
    }

    /**
     * Returns the intersection points of the line with the described circle
     *
     * @param line
     * @param circCenter
     * @param circRadius
     * @return
     */
    public static List<Point2D> segCircIntersections(Line2D line, Point2D circCenter, double circRadius) {
        return segCircIntersectionParams(line, circCenter, circRadius)
                .stream()
                .map(t -> getPointOnLine(line, t))
                .collect(Collectors.toList());
    }

    /**
     * Gets the list of subtrajectories of the bundle, sorted at the incoming
     * position (start points), in the direction of the movement
     *
     * @param bundle
     * @return
     */
    public static List<Subtrajectory> getIncoming(Bundle bundle) {
        Line2D edge = getAverageSegment(bundle.getSubtrajectories().stream().map(sub -> sub.getEdge(0)).collect(Collectors.toList()));
        return getSortedTrajectories(bundle, edge, sub -> sub.getFirstPoint());
    }

    /**
     * Gets the list of subtrajectories of the bundle, sorted at the outgoing
     * position (end points), in the direction of the movement
     *
     * @param bundle
     * @return
     */
    public static List<Subtrajectory> getOutgoing(Bundle bundle) {
        Line2D edge = getAverageSegment(bundle.getSubtrajectories().stream().map(sub -> sub.getEdge(sub.numEdges() - 1)).collect(Collectors.toList()));
        return getSortedTrajectories(bundle, edge, sub -> sub.getLastPoint());
    }

    private static List<Subtrajectory> getSortedTrajectories(final Bundle bundle, final Line2D edge, final Function<Subtrajectory, Point2D> pointGetter) {
        ArrayList<Subtrajectory> sorted = new ArrayList<>(bundle.getSubtrajectories());

        // We have to sort in the direction of the edge of the rep
        // We can do this by sorting by the value relativeCCW * ptLineDist
        // Relative CCW indicates the "side" on which the point lies,
        // and ptLineDist the shortest distance to the (infinittly extended) line
        Comparator<Subtrajectory> comp = Comparator.comparing(sub -> {
            final Point2D point = pointGetter.apply(sub);
            return edge.relativeCCW(point) * edge.ptLineDist(point);
        });

        sorted.sort(comp);
        return sorted;
    }

    public static Line2D getAverageSegment(List<Line2D> segments) {
        double sumX1 = 0, sumY1 = 0, sumX2 = 0, sumY2 = 0;
        double count = segments.size();
        for (Line2D segment : segments) {
            sumX1 += segment.getX1();
            sumX2 += segment.getX2();
            sumY1 += segment.getY1();
            sumY2 += segment.getY2();
        }
        return new Line2D.Double(sumX1 / count, sumY1 / count, sumX2 / count, sumY2 / count);
    }

    /**
     * Computes the average pairwise distances of the trajectories in the
     * given set.
     *
     * @param distance     Distance computer to use
     * @param trajectories set of trajectories
     * @return
     */
    public static double getAvgIntraPairwiseDistance(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories) {
        return getAvgInterPairwiseDistance(distance, trajectories, trajectories);
    }

    /**
     * Computes the average pairwise distances of the trajectories between
     * the two given sets.
     *
     * @param distance      Distance computer to use
     * @param trajectories1 first set of trajectories
     * @param trajectories2 second set of trajectories
     * @return
     */
    public static double getAvgInterPairwiseDistance(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories1, final Set<? extends Trajectory> trajectories2) {
        double avg = Sets.cartesianProduct(trajectories1, trajectories2) // Cartesian product as set of lists
                .stream()
                .filter(pair -> pair.get(0) != pair.get(1)) // remove pairs with itself.
                .mapToDouble(pair -> distance.compute(pair.get(0), pair.get(1))) // compute distances between each pair
                .average().orElse(0);
        return avg;
    }

    /**
     * Computes the minimum pairwise distances of the trajectories in the
     * given set.
     * <p>
     * 0 if the set contains just 1 trajectory.
     *
     * @param distance     Distance computer to use
     * @param trajectories set of trajectories
     * @return
     */
    public static double getMinIntraPairwiseDistance(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories) {
        return getMinInterPairwiseDistance(distance, trajectories, trajectories);
    }

    /**
     * Computes the minimum pairwise distances of the trajectories between
     * the two given sets.
     * <p>
     * 0 if the both sets contain just 1 and the same trajectory.
     *
     * @param distance      Distance computer to use
     * @param trajectories1 first set of trajectories
     * @param trajectories2 second set of trajectories
     * @return
     */
    public static double getMinInterPairwiseDistance(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories1, final Set<? extends Trajectory> trajectories2) {
        double avg = Sets.cartesianProduct(trajectories1, trajectories2) // Cartesian product as set of lists
                .stream()
                .mapToDouble(pair -> distance.compute(pair.get(0), pair.get(1))) // compute distances between each pair
                .filter(d -> d > 0)
                .min().orElse(0);
        return avg;
    }

    /**
     * Computes the average distance of the trajectories in the fiven set to
     * the fiven trajectory, using the given distance measure.
     *
     * @param distance     Distance computer to use
     * @param trajectories set of trajectories
     * @param t            trajectory to compute the distance to.
     * @return
     */
    public static double getAvgDistanceToTrajectory(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories, final Trajectory t) {
        double avg = trajectories.stream()
                .mapToDouble(traj -> distance.compute(t, traj))
                .average().orElse(0);
        return avg;
    }

    /**
     * Computes the average distance of the trajectories in the fiven set to
     * the given trajectory, using the given distance measure.
     *
     * @param distance     Distance computer to use
     * @param trajectories set of trajectories
     * @param t            trajectory to compute the distance to.
     * @return
     */
    public static double getMinDistanceToTrajectory(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories, final Trajectory t) {
        double avg = trajectories.stream()
                .mapToDouble(traj -> distance.compute(t, traj))
                .min().orElse(Double.POSITIVE_INFINITY);
        return avg;
    }

    /**
     * Computes the maximum distance of the trajectories in the given set to
     * the given trajectory, using the given distance measure.
     *
     * @param distance     Distance computer to use
     * @param trajectories set of trajectories
     * @param t            trajectory to compute the distance to.
     * @return
     */
    public static double getMaxDistanceToTrajectory(final TrajectoryDistance distance, final Set<? extends Trajectory> trajectories, final Trajectory t) {
        double avg = trajectories.stream()
                .mapToDouble(traj -> distance.compute(t, traj))
                .max().orElse(Double.NEGATIVE_INFINITY);
        return avg;
    }

    /**
     * Computes the diameter of the given set of points.
     * <p>
     * That is, the largest distance between any two points.
     *
     * @param points Points to compute the diameter for
     * @return
     */
    public static double pointsDiameter(Set<Point2D> points) {
        return Sets.cartesianProduct(points, points)
                .stream()
                .mapToDouble(pair -> pair.get(0).distance(pair.get(1)))
                .max()
                .orElse(0);
    }

    // computes the intersection between two *LINES*
    public static Point2D.Double intersectionPoint(Line2D l, Line2D m) {
        Point2D p = l.getP1();
        double ux = l.getX2() - p.getX();
        double uy = l.getY2() - p.getY();

        Point2D q = m.getP1();
        double vx = m.getX2() - q.getX();
        double vy = m.getY2() - q.getY();

        double denom = vy * ux - vx * uy;

        boolean areParallel = 0 == DoubleMath.fuzzyCompare(denom, 0, 1E-6);

        // FIXME this doesn't handle colinear lines well
        if (areParallel) {
//            System.out.print("Colinear: " + l.toString() + " " + m.toString());
            return null;
        } else {
            double alpha = (ux * (p.getY() - q.getY()) + uy * (q.getX() - p.getX())) / denom;
            return new Point2D.Double(q.getX() + alpha * vx
                    , q.getY() + alpha * vy);
        }
    }

    /**
     * Same as getDirection, just in a different context with difference parameters.
     *
     * @param point1, the start point of the line
     * @param point2, the end point of the line
     * @return angle
     */
    public static double getHeadingDirection(Point2D point1, Point2D point2) {
        return getHeadingDirection(new Line2D.Double(point1, point2));
    }

    /**
     * Same as getDirection, just in a different context.
     *
     * @param line, line segment
     * @return angle
     */
    public static double getHeadingDirection(Line2D line) {
        return getDirectionInDegrees(line);
    }

    /**
     * Gets the heading direction of an edge of the a trajectory (specified by the index)
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @return the angle of the edge at the given index for the given trajectory
     */
    public static double getHeadingDirection(Trajectory t, double index) {
        return getHeadingDirection(t.getEdge((int) Math.min(t.numEdges() - 1, Math.floor(index))));
    }

    /**
     * Get's the heading direction of trajectory t closest to the point.
     *
     * @param t,        the trajectory
     * @param location, a point around the trajectory
     * @return the angle of the edge at the given index for the given trajectory
     */
    public static double getHeadingDirection(Trajectory t, Point2D location) {
        double index = GeometryUtil.getIndexOfTrajectoryClosestToPoint(t, location);
        return getHeadingDirection(t.getEdge((int) Math.min(t.numEdges() - 1, Math.floor(index))));
    }

    /**
     * Get's the angle of a line segment
     *
     * @param line, line segment
     * @return angle
     */
    public static double getDirectionInDegrees(Line2D line) {
        double angle = (float) Math.toDegrees(Math.atan2(line.getY2() - line.getY1(), line.getX2() - line.getX1()));

        if (angle < 0) {
            angle += 360;
        }

        return angle;
    }

    /**
     * Get's difference in heading direction between two line segment in degrees.
     *
     * @param line1, the first line
     * @param line2, the second line
     * @return double, the difference between the heading direction of the two in degrees.
     */
    public static double getHeadingDirectionDifference(Line2D line1, Line2D line2) {
        double a = getDirectionInDegrees(line1);
        double b = getDirectionInDegrees(line2);

        // Returning min(min(abs(a-b), abs(b-a))) would give good results if the degrees was a continuous loop without
        // a gap 360 (image one having 359 and the other 1). Hence we also subtract 360 from the results and check if
        // that is less.
        return getAbsoluteAngleDifference(a, b);
    }

    /**
     * Get's the absolute angle difference between two angles.
     *
     * @param a, first angle
     * @param b, second angle
     * @return absolute angle difference, ranging from 0 to 180.
     */
    public static double getAbsoluteAngleDifference(double a, double b) {
        return Math.min(
                Math.min(Math.abs(a - b), Math.abs(b - a)),
                Math.min(Math.abs(a - 360 - b), Math.abs(b - 360 - a))
        );
    }

    /**
     * Get's half angle difference
     *
     * @param a, first angle
     * @param b, second angle
     * @return angles taking into account that a trajectory could be the opposite, ranging from 0 to 90.
     */
    public static double getAngleDifferenceForPossiblyReverseTrajectories(double a, double b) {
        double foundAngle = getAbsoluteAngleDifference(a, b);
        return Math.min(
                Math.abs(180 - foundAngle), foundAngle
        );
    }

    /**
     * Calculates whether a heading angle would be classified as vertical, instead of horizontal
     */
    public static boolean isHeadingVertical(Line2D line) {
        double headingDegrees = GeometryUtil.getDirectionInDegrees(line);
        return (45 < headingDegrees && headingDegrees < 135) || (225 < headingDegrees && headingDegrees < 315);
    }

    /**
     * Get average point location of several points
     *
     * @param points list of points
     * @return average point
     */
    public static Point2D getAverage(List<Point2D> points) {
        double averageX = 0.0;
        double averageY = 0.0;

        for (int i = 0; i < points.size(); i++) {
            Point2D point = points.get(i);
            averageX = ((averageX / (i + 1)) * i) + (point.getX() / (i + 1));
            averageY = ((averageY / (i + 1)) * i) + (point.getY() / (i + 1));
        }
        return new Point2D.Double(averageX, averageY);
    }

    /**
     * Get new average angle
     */
    public static double getNewAverageAngle(double currentAngle, double addedAngle, int oldNoPoints) {
        double newNoPoints = oldNoPoints + 1;
        if (addedAngle > currentAngle + 180) {
            addedAngle = addedAngle - 360;
            return (((currentAngle / newNoPoints) * (oldNoPoints) + addedAngle / newNoPoints) + 360) % 360;
        } else if (currentAngle > addedAngle + 180) {
            addedAngle = addedAngle + 360;
            return ((currentAngle / newNoPoints) * (oldNoPoints) + addedAngle / newNoPoints) % 360;
        } else {
            return (currentAngle / newNoPoints) * (oldNoPoints) + addedAngle / newNoPoints;
        }
    }


    /**
     * Get the point on an edge of the trajectory.
     *
     * @param t,     trajectory
     * @param index, the index which is on an edge of the trajectory
     * @return the point
     */
    public static Point2D getTrajectoryDecimalPoint(Trajectory t, double index) {
        if (index > t.numPoints()) {
            throw new IndexOutOfBoundsException("Index :" + index + " while t.numPoints() is  " + t.numPoints());
        }
        double startingIndexComma = index - Math.floor(index);
        Point2D startingPoint = GeometryUtil.getPointOnLine(
                new Line2D.Double(
                        t.getPoint((int) Math.floor(index)),
                        t.getPoint((int) Math.ceil(index))
                ), startingIndexComma
        );
        if (Double.isNaN(startingPoint.getX()) || Double.isNaN(startingPoint.getY())) {
            System.out.println(t.getPoint((int) Math.floor(index)));
            System.out.println(t.getPoint((int) Math.ceil(index)));
            System.out.println("Error in GeometryUtil.GetTrajectoryDecimalPoint. Values are NaN");
            System.out.println(index);
        }
        return startingPoint;
    }

    /**
     * Get the index of the point on an edge of the trajectory that is closest to a location
     *
     * @param t        the trajectory where we want to find the closest point for
     * @param location the location we are comparing the trajectory to
     * @return the index of the point closest to location.
     */
    public static double getIndexOfTrajectoryClosestToPoint(Trajectory t, Point2D location) {
        return getIndexOfTrajectoryClosestToPointByAngle(t, location, null, null, false);
    }

    /**
     * Get the index of the point on an edge of the trajectory that is closest to a location within some heading
     * direction range.
     *
     * @param t               the trajectory where we want to find the closest point for
     * @param location        the location we are comparing the trajectory to
     * @param preferredAngle  the angle we are looking for
     * @param maxAngleDiff    the maximum difference between the trajectory edge heading angle and our preferredAngle.
     * @param possibleReverse whether the angle difference should be calculated in relation to a possible reverse trajectory
     * @return the index of the point closest to location.
     */
    public static double getIndexOfTrajectoryClosestToPointByAngle(Trajectory t, Point2D location,
                                                                   Double preferredAngle, Double maxAngleDiff,
                                                                   Boolean possibleReverse) {
        double minDistance = Double.MAX_VALUE;
        double minIndex = 0.0;
        for (int i = 0; i < t.numPoints() - 1; i++) {
            double distance = t.getEdge(i).ptSegDist(location);
            // The fuzzyEquals makes sure that if it is equally far from two edges, we take the right point index.
            if (distance < minDistance) {
                if (preferredAngle != null) {
                    double headingAngle = GeometryUtil.getHeadingDirection(t.getEdge(i));
                    double angleDiff;
                    if (possibleReverse) {
                        angleDiff = GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(headingAngle, preferredAngle);
                    } else {
                        angleDiff = GeometryUtil.getAbsoluteAngleDifference(headingAngle, preferredAngle);
                    }

                    if (angleDiff > maxAngleDiff) {
                        continue;
                    }
                }

                minDistance = distance;
                minIndex = i;
            }
        }

        // Is in the segment band
        Line2D minEdge = t.getEdge((int) minIndex);
        double cDistance = t.getPoint((int) minIndex).distance(location);
        if (cDistance > minDistance) {
            double aDistance = Math.sqrt(cDistance * cDistance - minDistance * minDistance);
            double edgeLength = minEdge.getP1().distance(minEdge.getP2());
            double addToEdge = aDistance / edgeLength;
            // We add to the minIndex at least 0 and at most 1. This way we really have the closest place.
            minIndex = minIndex + Math.max(Math.min(1.0, addToEdge), 0.0);
        }

        if (minIndex < 0 || minIndex > t.numPoints() - 1 || Double.isNaN(minIndex)) {
            System.out.println("Error GeometryUtil.getIndexOfTrajectoryClosestToPoint. Incorrect minIndex value");
            System.out.printf("Value minIndex: %a, t.numPoints(): %o", minIndex, t.numPoints());
        }

        return minIndex;
    }

    /**
     * Returns the location of the point on the trajectory which is closest to another point.
     *
     * @param t,        trajectory
     * @param location, the location of the points
     * @return point2D which is the point on the trajectory closest to location.
     */
    public static Point2D getPointOnTrajectoryClosestToOtherPoint(Trajectory t, Point2D location) {
        double index = getIndexOfTrajectoryClosestToPoint(t, location);
        return getTrajectoryDecimalPoint(t, index);
    }

    /**
     * Given a trajectory and an index, we find the new index for a continuous given offset.
     *
     * @param t      the trajectory
     * @param index  the original index
     * @param offset the offset
     * @return index of the trajectory to find the exact bundle
     */
    public static double getTrajectoryIndexAfterOffset(Trajectory t, double index, double offset) {
        double currentIndex;
        Point2D startingPoint = getTrajectoryDecimalPoint(t, index);
        if (offset >= 0) {
            currentIndex = Math.ceil(index);
        } else {
            currentIndex = Math.floor(index);
        }

        double distanceCovered = startingPoint.distance(t.getPoint((int) currentIndex));
        Line2D lastEdge = new Line2D.Double(startingPoint, t.getPoint((int) currentIndex));

        boolean incrementedIndex = false;
        while (distanceCovered < Math.abs(offset)) {
            if (offset >= 0) {
                if (currentIndex + 1 <= t.numPoints() - 1) {
                    distanceCovered += t.getPoint((int) currentIndex).distance(
                            t.getPoint((int) currentIndex + 1));
                    lastEdge = new Line2D.Double(t.getPoint((int) currentIndex),
                            t.getPoint((int) currentIndex + 1));
                    currentIndex += 1;
                    incrementedIndex = true;
                } else {
                    break;
                }
            } else {
                if (currentIndex - 1 >= 0) {
                    distanceCovered += t.getPoint((int) currentIndex).distance(
                            t.getPoint((int) currentIndex - 1));
                    lastEdge = new Line2D.Double(t.getPoint((int) currentIndex),
                            t.getPoint((int) currentIndex - 1));
                    currentIndex -= 1;
                    incrementedIndex = true;
                } else {
                    break;
                }
            }
        }

        double coveredToMuch = Math.max(0, distanceCovered - Math.abs(offset));
        double lastEdgeCovered = lastEdge.getP1().distance(lastEdge.getP2());
        if (!DoubleMath.fuzzyEquals(lastEdgeCovered, 0.0, 1E-8)) {
            if (incrementedIndex) {
                if (offset >= 0) {
                    currentIndex -= coveredToMuch / lastEdgeCovered;
                } else {
                    currentIndex += coveredToMuch / lastEdgeCovered;
                }
            } else {
                if (offset >= 0) {
                    currentIndex = index + (Math.ceil(index) - index) * (1 - (coveredToMuch / lastEdgeCovered));
                } else {
                    currentIndex = index - (index - Math.floor(index)) * (1 - (coveredToMuch / lastEdgeCovered));
                }
            }
        }

        if (Double.isNaN(currentIndex) || currentIndex < 0 || currentIndex > t.numPoints() - 1) {
            System.out.println("Error GeometryUtil.getTrajectoryIndexAfterOffset. Incorrect currentIndex value");
            System.out.printf("Value currentIndex: %f, t.numPoints(): %d%n", currentIndex, t.numPoints());
        }

        if ((currentIndex < index && offset >= 0.0) || (currentIndex > index && offset < 0.0)) {
            System.out.println("Error GeometryUtil.getTrajectoryIndexAfterOffset. Invalid currentIndex value");
            System.out.printf("startingIndex: %f, currentIndex: %f, offset: %f%n", index, currentIndex, offset);
        }

        return currentIndex;
    }

    /**
     * Given a Subtrajectory and it's index, we convert it to the index of it's parent.
     *
     * @param sub,      the subtrajectory
     * @param subIndex, the index on the subtrajectory.
     * @return index of the trajectory, representing the exact same point as the subIndex on sub.
     */
    public static double convertSubIndexToTrajectoryIndex(Subtrajectory sub, double subIndex) {
        if (DoubleMath.fuzzyEquals(subIndex, 0.0, 1E-4)) {
            return sub.getFromIndex();
        }
        if (DoubleMath.fuzzyEquals(subIndex, sub.getToIndex() - sub.getFromIndex(), 1E-4)) {
            return sub.getToIndex();
        }
        double originalSubIndex = subIndex;

        Trajectory parent = sub.getParent();
        Point2D currentPoint = GeometryUtil.getTrajectoryDecimalPoint(sub, subIndex);
        double trajectoryIndex = Math.floor(sub.getFromIndex());
        int flooredFromIndex = (int) Math.floor(sub.getFromIndex());
        if (subIndex >= 1 && subIndex <= sub.numPoints() - 2) {
            trajectoryIndex += subIndex;
        } else {
            if (subIndex >= sub.numPoints() - 1) {
                subIndex -= 1;
            }

            Point2D boundLeft, boundRight;
            boundLeft = parent.getPoint(flooredFromIndex + (int) Math.floor(subIndex));
            if (Math.ceil(subIndex) >= sub.numPoints() - 1) {
                boundRight = parent.getPoint(flooredFromIndex + sub.numPoints() - 1);
            } else {
                boundRight = parent.getPoint(flooredFromIndex + (int) Math.ceil(subIndex + 1E-12));
            }

            double edgeLength = boundLeft.distance(boundRight);
            if (DoubleMath.fuzzyEquals(edgeLength, 0.0, 1E-8)) {
                return trajectoryIndex;
            }
            double cutOffEdgeLength = boundLeft.distance(currentPoint);
            double distanceCovered = cutOffEdgeLength / edgeLength;

            if (Double.isNaN(trajectoryIndex)) {
                throw new IllegalArgumentException();
            }

            trajectoryIndex += (int) Math.floor(subIndex) + distanceCovered;

            if (Double.isNaN(edgeLength)) {
                throw new IllegalArgumentException();
            }

            if (Double.isNaN(cutOffEdgeLength)) {
                throw new IllegalArgumentException();
            }

            if (Double.isNaN(distanceCovered)) {
                throw new IllegalArgumentException();
            }

            if (cutOffEdgeLength > edgeLength) {
                System.out.println("Error GeometryUtil.convertSubIndexToTrajectoryIndex.");
                System.out.printf("EdgeLength: %f, CutOffEdgeLength: %f%n", edgeLength, cutOffEdgeLength);
            }
        }
        if (trajectoryIndex < 0 || trajectoryIndex > sub.getParent().numPoints() - 1 || Double.isNaN(trajectoryIndex)) {
            System.out.println("Error GeometryUtil.convertSubIndexToTrajectoryIndex.");
            System.out.printf("trajectoryIndex: %f, parent.NumPoints: %d%n", trajectoryIndex, sub.getParent().numPoints());
            throw new IllegalArgumentException("Weird indexes");
        }

        return trajectoryIndex;
    }

    /**
     * Converts the index to a non reverse index when the parent is reversed.
     *
     * @param trajectory, the trajectory for which we have an index. May be reversed may be not-reversed.
     * @param index,      the index on the trajectory we might need to convert
     * @return the index on the original non-reverse trajectory.
     */
    public static double convertIndexToNonReverseIndex(Trajectory trajectory, double index) {
        if (trajectory.isReverse()) {
            return trajectory.numPoints() - 1 - index;
        }
        return index;
    }

    /**
     * Returns the index of a point on a trajectory
     *
     * @param t,     trajectory
     * @param point, point on a trajectory
     * @return the index of the point or -1 if it is not part of the trajectory.
     */
    public static double getIndexForPointOnTrajectory(Trajectory t, Point2D point) {
        for (int i = 0; i < t.numEdges(); i++) {
            double foundRelativeIndex = getIndexForPointOnLine(t.getEdge(i), point);
            if (foundRelativeIndex >= 0) {
                return i + foundRelativeIndex;
            }
        }
        return -1.0;
    }

    /**
     * Returns the index of a point on a line segment.
     *
     * @param line,  the line segment which has the point on it somewhere
     * @param point, the point that is on the line segment
     * @return the index of the point or -1 if this point is not on the line segment.
     */
    public static double getIndexForPointOnLine(Line2D line, Point2D point) {
        Point2D edgeStart = line.getP1();
        Point2D edgeEnd = line.getP1();

        if (DoubleMath.fuzzyEquals(edgeStart.getX(), edgeEnd.getX(), 1E-3)) {
            return getProgressInRange(edgeStart.getY(), edgeEnd.getY(), point.getY());

        } else if (DoubleMath.fuzzyEquals(edgeStart.getY(), edgeEnd.getY(), 1E-3)) {
            return getProgressInRange(edgeStart.getX(), edgeEnd.getX(), point.getX());
        } else {
            double indexX = getProgressInRange(edgeStart.getX(), edgeEnd.getX(), point.getX());
            double indexY = getProgressInRange(edgeStart.getY(), edgeEnd.getY(), point.getY());
            if (indexX >= 0 && indexY >= 0 && DoubleMath.fuzzyEquals(indexX, indexY, 1E-5)) {
                return (indexX + indexY) / 2;
            }
        }
        return -1;
    }

    /**
     * For a trajectory finds the points closest to a location, from there we do an offset, and then return the point
     *
     * @param t,        trajectory
     * @param location, the location where we need to find the closest point for on the trajectory
     * @param offset,   the offset we want to check on the trajectory after we found the closest points.
     * @return the new point after finding the closest point to the location and taking into account the offset.
     */
    public static Point2D getTrajectoryPointClosestToLocationAfterOffset(Trajectory t, Point2D location, double offset) {
        double index = getIndexOfTrajectoryClosestToPoint(t, location);
        double newIndex = getTrajectoryIndexAfterOffset(t, index, offset);
        return GeometryUtil.getTrajectoryDecimalPoint(t, newIndex);
    }

    /**
     * Given two values (a range), we calculate the index(progress) between the two points, from start to end.
     * Example: 1, 4, 2 would give us (1/3) as (2/3)* 1 + (1/3) * 4 = 2.
     *
     * @param startPoint,  starting point of range
     * @param endPoint,    ending point of range
     * @param searchPoint, the point that is in between the range.
     * @return the index.
     */
    private static double getProgressInRange(double startPoint, double endPoint, double searchPoint) {
        double min = Math.min(startPoint, endPoint);
        double max = Math.max(startPoint, endPoint);
        if (searchPoint >= min || searchPoint <= max) {
            double diffY = endPoint - startPoint;
            double index = (searchPoint - endPoint) / diffY;
            if (index >= 0 && index <= 1) {
                return index;
            }
        }
        return -1;
    }

    /**
     * Get the euclidean distance between two indexes for a trajectory.
     *
     * @param t,          the trajectory
     * @param startIndex, the starting index
     * @param endIndex,   the ending index
     * @return the distance covered when walking from starting index to ending index, can only be positive.
     */
    public static double getIndexToIndexDistance(Trajectory t, double startIndex, double endIndex) {
        if (endIndex < startIndex) {
            double tempIndex = startIndex;
            startIndex = endIndex;
            endIndex = tempIndex;
        }

        return getDirectionalIndexToIndexDistance(t, startIndex, endIndex);
    }

    /**
     * Get the euclidean distance between two indexes for a list of points.
     *
     * @param pointList  the list of points
     * @param startIndex the starting index
     * @param endIndex   the ending index
     * @return the distance covered when walking from starting index to ending index, can only be positive.
     */
    public static double getIndexToIndexDistance(List<Point2D> pointList, double startIndex, double endIndex) {
        Trajectory t = new FullTrajectory(pointList);
        return getIndexToIndexDistance(t, startIndex, endIndex);
    }

    /**
     * Get the euclidean distance between from the start index to the end index.
     *
     * @param t,          the trajectory
     * @param startIndex, the starting index
     * @param endIndex,   the ending index
     * @return the distance covered when walking from starting index to ending index, can be minus.
     */
    public static double getDirectionalIndexToIndexDistance(Trajectory t, double startIndex, double endIndex) {
        double reverse = 1;
        if (endIndex < startIndex) {
            double tempIndex = startIndex;
            startIndex = endIndex;
            endIndex = tempIndex;
            reverse = -1;
        }

        double distance = 0;
        double currentIndex = startIndex;
        Point2D lastPoint = GeometryUtil.getTrajectoryDecimalPoint(t, currentIndex);
        if (DoubleMath.fuzzyEquals(currentIndex % 1, 0.0, 1E-5)) {
            currentIndex = Math.round(currentIndex) + 1;
        } else {
            currentIndex = Math.ceil(currentIndex);
        }

        while (currentIndex < Math.ceil(endIndex) + 1E-5) {
            if (DoubleMath.fuzzyEquals(currentIndex, Math.ceil(endIndex), 1E-5)) {
                distance += lastPoint.distance(GeometryUtil.getTrajectoryDecimalPoint(t, endIndex));
                currentIndex += 1;
            } else {
                Point2D newPoint = GeometryUtil.getTrajectoryDecimalPoint(t, currentIndex);
                distance += lastPoint.distance(newPoint);
                lastPoint = newPoint;
                currentIndex += 1;
            }
        }

        return reverse * distance;
    }

    /**
     * Get's a range of a specific offset from the point on the trajectory closest to the location
     *
     * @param t,        the trajectory where we should look at
     * @param location, the location were we take the closest point to the trajectory off.
     * @param offset,   the offset we take from the closest point on the trajectory.
     * @param twoSided, whether we should take it in both directions, or only one direction.
     * @return the range
     */
    public static Range<Double> getIndexRangeAroundPointOnTrajectory(Trajectory t, Point2D location, double offset, boolean twoSided) {
        double indexOnTrajectory = GeometryUtil.getIndexOfTrajectoryClosestToPoint(t, location);
        return getIndexRangeByIndexOnTrajectory(t, indexOnTrajectory, offset, twoSided);
    }

    /**
     * Get's a range created by the offset from the index on the trajectory
     *
     * @param t,        the trajectory where we should look at
     * @param index,    the location were we take the closest point to the trajectory off.
     * @param offset,   the offset we take from the closest point on the trajectory.
     * @param twoSided, whether we should take it in both directions, or only one direction.
     * @return the range
     */
    public static Range<Double> getIndexRangeByIndexOnTrajectory(Trajectory t, double index, double offset, boolean twoSided) {
        double positiveOffsetIndex = GeometryUtil.getTrajectoryIndexAfterOffset(t, index, offset);

        if (twoSided) {
            double negativeOffsetIndex = GeometryUtil.getTrajectoryIndexAfterOffset(t, index, -offset);

            if (negativeOffsetIndex > positiveOffsetIndex) {
                return Range.closed(positiveOffsetIndex, negativeOffsetIndex);
            }

            return Range.closed(negativeOffsetIndex, positiveOffsetIndex);
        } else {
            if (index > positiveOffsetIndex) {
                return Range.closed(positiveOffsetIndex, index);
            }

            return Range.closed(index, positiveOffsetIndex);
        }
    }


    /**
     * Get the index of the point on an edge of the trajectory that is closest to a location but within a certain range
     *
     * @param t        the trajectory where we want to find the closest point for
     * @param location the location we are comparing the trajectory to
     * @param range    the allowed range
     * @return the index of the point closest to location.
     */
    public static double getIndexOfTrajectoryClosestToPointWithinRange(Trajectory t, Point2D location, Range<Double> range) {
        if (range == null) {
            return getIndexOfTrajectoryClosestToPoint(t, location);
        } else {
            Subtrajectory sub = new Subtrajectory(t, range.lowerEndpoint(), range.upperEndpoint());
            double subIndex = getIndexOfTrajectoryClosestToPoint(sub, location);
            return GeometryUtil.convertSubIndexToTrajectoryIndex(sub, subIndex);
        }
    }

    public static Subtrajectory cutOffSubtrajectoryByRepresentativeRange(Subtrajectory rep, Subtrajectory sub) {
        double maxAngleDiff = 90;  // This holds for all cases.

        double angleStart = GeometryUtil.getHeadingDirection(rep.getEdge(0));
        double angleEnd = GeometryUtil.getHeadingDirection(rep.getEdge(rep.numEdges() - 1));

        Point2D startPoint = rep.getPoint(0);
        Point2D endPoint = rep.getPoint(rep.numPoints() - 1);

        double startIndex = getIndexOfTrajectoryClosestToPointByAngle(sub, startPoint, angleStart, maxAngleDiff, false);
        startIndex = convertSubIndexToTrajectoryIndex(sub, startIndex);

        double endIndex = getIndexOfTrajectoryClosestToPointByAngle(sub, endPoint, angleEnd, maxAngleDiff, false);
        endIndex = convertSubIndexToTrajectoryIndex(sub, endIndex);

        if (startIndex > endIndex) {
            double tempIndex = endIndex;
            endIndex = startIndex;
            startIndex = tempIndex;
        }

        return new Subtrajectory(sub.getParent(), startIndex, endIndex);
    }

    /**
     * Converts a set of Subtrajectories into set containing all non reverse Subtrajectories
     *
     * @param subList the set of subtrajectories with reverse subtrajectories in there
     * @return the same set but without any reverse subtrajectories.
     */
    public static Set<Subtrajectory> convertSubsIntoNonReverseSubs(Collection<Subtrajectory> subList) {
        Set<Subtrajectory> subs = new HashSet<>();
        for (Subtrajectory sub : subList) {
            if (sub.isReverse()) {
                sub = sub.reverse();
            }
            subs.add(sub);
        }
        return subs;
    }

    /**
     * Returns the maximum line to line discrete segment distance
     *
     * @param line1, the first line
     * @param line2, the second line
     * @return, the distance.
     */
    public static double getDiscreteSegmentDistance(Line2D line1, Line2D line2) {
        double distance = line1.ptSegDist(line2.getP1());
        distance = Math.min(distance, line1.ptSegDist(line2.getP2()));
        distance = Math.min(distance, line2.ptSegDist(line1.getP1()));
        distance = Math.min(distance, line2.ptSegDist(line1.getP2()));
        return distance;
    }

    /**
     * Get's the continuous length of the pointList
     *
     * @param pointList, the pointList
     * @return the continuous length of the pointList.
     */
    public static double getContinuousLength(List<Point2D> pointList) {
        double continuousLength = 0.0;
        for (int i = 0; i < pointList.size() - 1; i++) {
            continuousLength += pointList.get(i).distance(pointList.get(i + 1));
        }
        return continuousLength;
    }

    /**
     * Get's the continuous length of a trajectory
     *
     * @param trajectory, a trajectory
     * @return the continuous length of the pointList.
     */
    public static double getContinuousLength(Trajectory trajectory) {
        double continuousLength = 0.0;
        for (int i = 0; i < trajectory.numPoints() - 1; i++) {
            continuousLength += trajectory.getPoint(i).distance(trajectory.getPoint(i + 1));
        }
        return continuousLength;
    }

    /**
     * Crops a line by a small percentage on both ends.
     *
     * @param line2D the line segment we want to crop
     * @param p      the percentage we want to crop. Between 0.0 and 0.5
     * @return the cropped line segment.
     */
    public static Line2D cropLineByPercentage(Line2D line2D, double p) {
        if (p < 0.0 || p > 0.5) {
            throw new IllegalArgumentException("p is more than 0.5");
        }
        Point2D newStartPoint = getPointOnLine(line2D, p);
        Point2D newEndPoint = getPointOnLine(line2D, 1 - p);

        return new Line2D.Double(newStartPoint, newEndPoint);
    }

    /**
     * Finds the intersection in between two pointLists.
     *
     * @param pointList1 the first trajectory
     * @param pointList2 the second trajectory
     * @return the point where they intersect, if any.
     */
    public static Point2D findIntersectionBetweenPointLists(List<Point2D> pointList1, List<Point2D> pointList2) {
        for (int i = 0; i < pointList1.size() - 1; i++) {
            Line2D firstLine = new Line2D.Double(pointList1.get(i), pointList1.get(i + 1));

            for (int j = 0; j < pointList2.size() - 1; j++) {
                Line2D secondLine = new Line2D.Double(pointList2.get(j), pointList2.get(j + 1));

                if (firstLine.intersectsLine(secondLine)) {
                    Point2D intersection = GeometryUtil.intersectionPoint(firstLine, secondLine);
                    if (intersection != null) {
                        return intersection;
                    }
                }
            }
        }
        return null;
    }

    /**
     * Returns whether a point is NaN
     *
     * @param point2D the point we want to check for
     * @return if it contains a nan in getX or getY.
     */
    public static boolean isPointNan(Point2D point2D) {
        return Double.isNaN(point2D.getX()) || Double.isNaN(point2D.getY());
    }


    /**
     * When we want to compare two trajectories, it is often of key interest that they are heading in the same direction.
     * Therefore, we compare trajectory1 on multiple places to trajectory2.
     * By having at least a precision of 4, we can have a good guess whether they are going in the same direction or
     * opposite.
     * A warning: For this to work efficiently it is of importance that trajectory1 is sufficiently close to trajectory2
     * for the whole of trajectory1.
     *
     * @param trajectory1 the first trajectory, should be completely covered by trajectory2
     * @param trajectory2 the second trajectory, for which we check if it is in the same direction as trajectory1
     * @param precision represents the number of sampling points we compare. 4 should be enough in 99% of the cases.
     * @return whether trajectory1 and trajectory2 have the same direction.
     */
    public static boolean checkIfTrajectoryHasSameDirection(Trajectory trajectory1, Trajectory trajectory2, int precision){
        if (precision <= 1){
            throw new IllegalArgumentException("Precision must be > 1");
        }

        Trajectory trajectory2reversed = trajectory2.reverse();

        double indexCheckAfter = (double) (trajectory1.numPoints() - 1) / (double) (precision - 1);
        double isIncreasingAsWellNormal = 0.0;
        double isIncreasingAsWellReverse = 0.0;

        double lastIndexNormal = 0.0;
        double lastIndexReverse = 0.0;
        for (int i = 0; i < precision; i++){
            Point2D getPoint = GeometryUtil.getTrajectoryDecimalPoint(trajectory1,
                    Math.min(trajectory1.numPoints() - 1, indexCheckAfter * i));

            double closestPointNormal = GeometryUtil.getIndexOfTrajectoryClosestToPoint(trajectory2, getPoint);
            double closestPointReverse = GeometryUtil.getIndexOfTrajectoryClosestToPoint(trajectory2reversed, getPoint);

            if (closestPointNormal > lastIndexNormal){
                isIncreasingAsWellNormal++;
            } else {
                isIncreasingAsWellNormal--;
            }
            lastIndexNormal = closestPointNormal;

            if (closestPointReverse > lastIndexReverse){
                isIncreasingAsWellReverse++;
            } else {
                isIncreasingAsWellReverse--;
            }
            lastIndexReverse = closestPointReverse;
        }

        return isIncreasingAsWellNormal > isIncreasingAsWellReverse;
    }
}
