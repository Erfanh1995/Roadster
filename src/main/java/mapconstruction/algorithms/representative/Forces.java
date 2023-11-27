package mapconstruction.algorithms.representative;

import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.representative.containers.OrthogonalIntersection;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.OrthogonalUtil;
import mapconstruction.util.Pair;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * Class responsible for calculating everything related to the force representative.
 *
 * @author Jorrick Sleijster
 */
public class Forces {

    /**
     * Calculates the force representative for a given bundle. The bundle is defined by the parameters which are:
     *
     * @param trajectories,                  the subtrajectories of the bundle.
     * @param representative,                the representative of the bundle.
     * @param epsilon,                       the birth epsilon of the representative.
     * @param listOfForceSteps,              the list containing the steps the force representative made.
     * @param listOfOrthogonal,              the list of orthogonals used for the calculation.
     * @param listOfIntersectionsWithAngles, containing the lists of intersections we can find, and their angle.
     * @return a list of Points, representing the force representative.
     */
    public static List<Point2D> representativeTrajectory(Set<Subtrajectory> trajectories, Subtrajectory representative,
                                                         double epsilon,
                                                         List<List<Point2D>> listOfForceSteps,
                                                         List<Point2D> listOfOrthogonal,
                                                         List<Point2D> listOfACLines,
                                                         List<List<OrthogonalIntersection>> listOfIntersectionsWithAngles) {

        /* The representative points */
        List<Point2D> representativePoints = new ArrayList<>();

        if (representative.numPoints() == 1) {
            representativePoints.add(representative.getPoint(0));
            return representativePoints;
        }

        /* Creating a list of subtrajectories without representative */
        List<Subtrajectory> subtrajectoriesWithoutRepresentative = new ArrayList<>();
        for (Subtrajectory sub : trajectories) {
            if (!sub.equals(representative)) {
                subtrajectoriesWithoutRepresentative.add(sub);
            }
        }

        /* We filter out points that are close to each other to prevent Z-artifacts */
        List<Point2D> filteredRepPoints = filterOutCloseRepresentativePoints(representative);
        /* We add extra points at places where other subtrajectories have a lot of points of well.
           This is to make sure we can accurately represent the trajectories */
        filteredRepPoints = filterInExtraPointsAtVIPPlaces(
                filteredRepPoints, subtrajectoriesWithoutRepresentative);

        for (int i = 0; i < filteredRepPoints.size(); i++) {
            // SuperSharpCheck: Preventing more than 125 degree turns of making our representative super ugly..
            if (i > 0 && i < filteredRepPoints.size() - 1) {
                Line2D aBLine = new Line2D.Double(filteredRepPoints.get(i - 1), filteredRepPoints.get(i));
                Line2D bCLine = new Line2D.Double(filteredRepPoints.get(i), filteredRepPoints.get(i + 1));
                if (GeometryUtil.getHeadingDirectionDifference(aBLine, bCLine) > 125) {
                    representativePoints.add(filteredRepPoints.get(i));
                    continue;
                }
            }

            Point2D currentPoint = filteredRepPoints.get(i);
            Line2D ABCline = getACLine(filteredRepPoints, i);
            Line2D perpACThroughB = getLongPerpendicularOnACThroughB(ABCline, filteredRepPoints.get(i));

            List<OrthogonalIntersection> intersectionsWithDeltaAngles =
                    findIntersections(trajectories, perpACThroughB, ABCline, currentPoint);

            // Only populate these when their variables are initialized
            if (listOfACLines != null) {
                listOfACLines.add(ABCline.getP1());
                listOfACLines.add(ABCline.getP2());
            }

            if (listOfIntersectionsWithAngles != null) {
                listOfIntersectionsWithAngles.add(intersectionsWithDeltaAngles);
            }

            if (listOfOrthogonal != null) {
                listOfOrthogonal.add(perpACThroughB.getP1());
                listOfOrthogonal.add(perpACThroughB.getP2());
            }

            // List that keeps track whether the last step was an increase of a decrease.
            // increasingDirection[0] is for the x-axis and increasingDirection[1] for the y-axis
            List<Boolean> increasingDirection = new ArrayList<>();

            List<Point2D> places = new ArrayList<>();
            places.add(filteredRepPoints.get(i));

            int stepsDone = 0;
            boolean reversed = false;
            while (places.get(0).distance(places.get(places.size() - 1)) <= epsilon && stepsDone <= 50 && !reversed) {

                places.add(calculateForceStep(intersectionsWithDeltaAngles, perpACThroughB, places, epsilon));

                if (places.size() == 2) {
                    increasingDirection.add(places.get(1).getX() - places.get(0).getX() >= 0.0);
                    increasingDirection.add(places.get(1).getY() - places.get(0).getY() >= 0.0);
                } else {
                    int s = places.size() - 1;
                    if ((places.get(s).getX() - places.get(s - 1).getX() >= 0.0) != increasingDirection.get(0) ||
                            (places.get(s).getY() - places.get(s - 1).getY() >= 0.0) != increasingDirection.get(1)) {
                        reversed = true;
                    }
                }
                if (DoubleMath.fuzzyEquals(places.get(places.size() - 1).getX(), places.get(places.size() - 2).getX(), 1E-3) &&
                        DoubleMath.fuzzyEquals(places.get(places.size() - 1).getY(), places.get(places.size() - 2).getY(), 1E-3)) {
                    /* Officially it is not reversed, it doesn't make sense to add a second variable for this.. */
                    reversed = true;
                }

                stepsDone++;
            }
            if (stepsDone > 35 && STORAGE.getDatasetConfig().isWalkingDataset()) {
                representativePoints.add(places.get(0));
            } else if (!(places.get(0).distance(places.get(places.size() - 1)) <= epsilon)) {
                representativePoints.add(places.get(places.size() - 2));
            } else if (stepsDone > 50) {
                representativePoints.add(places.get(places.size() - 1));
            } else if (reversed) {
                representativePoints.add(places.get(places.size() - 1));
            }

            if (listOfForceSteps != null)
                listOfForceSteps.add(places);
        }

        return representativePoints;
    }

    /**
     * A function to get the best possible row of representative points.
     * This function averages out points which are closer than a certain threshold to each other.
     *
     * @param representative The representative trajectory.
     * @return a point list of which no points are closer than ALGOCONSTANTS.forceMinDistancePoints to each other.
     */
    private static List<Point2D> filterOutCloseRepresentativePoints(Subtrajectory representative) {
        List<Pair<Point2D, Integer>> representativePoints = new ArrayList<>();
        for (int i = 0; i < representative.numPoints(); i++) {
            if (i == 0) {
                representativePoints.add(new Pair<>(representative.getPoint(i), 1));
            } else if (representative.getPoint(i).distance(representative.getPoint(i - 1))
                    < ALGOCONSTANTS.getForceMinDistancePoints()) {
                Pair<Point2D, Integer> pPair = representativePoints.get(representativePoints.size() - 1);
                Point2D newPoint = getAveragePoint(pPair.getFirst(), pPair.getSecond(), representative.getPoint(i));
                representativePoints.set(representativePoints.size() - 1, new Pair<>(newPoint, pPair.getSecond() + 1));
            } else {
                representativePoints.add(new Pair<>(representative.getPoint(i), 1));
            }
        }

        List<Point2D> pointCalculatedAsRepresentative = new ArrayList<>();
        for (Pair<Point2D, Integer> representativePoint : representativePoints) {
            pointCalculatedAsRepresentative.add(representativePoint.getFirst());
        }

        if (pointCalculatedAsRepresentative.size() < 2) {
            pointCalculatedAsRepresentative.clear();
            for (int i = 0; i < representative.numPoints(); i++) {
                pointCalculatedAsRepresentative.add(representative.getPoint(i));
            }
        }

        return pointCalculatedAsRepresentative;
    }

    /**
     * Function which creates extra points if there are a couple of subtrajectory points in the neighbourhood.
     * In order to split an edge up, we need to have more than subtrajectories.size() / 3 intersections from
     * orthogonals on representative edges through points.
     *
     * @param proposedPoints, the current proposed representative points.
     * @return list with the newly proposed representative points.
     */
    private static List<Point2D> filterInExtraPointsAtVIPPlaces(List<Point2D> proposedPoints,
                                                                List<Subtrajectory> subtrajectories) {
        /* First, we find for each subtrajectory the intersection points with the proposedRepresentativePoints
                    and store this in a list
           Second, we look for each edge of the proposed representative if it needs to be split off.
                    which will be a subroutine which returns a lists of points.
         */

        /* Initialize the list with intersections */
        List<List<Point2D>> intersectionsOnEdge = new ArrayList<>();
        for (int i = 0; i < proposedPoints.size() - 1; i++) {
            List<Point2D> emptyIntersections = new ArrayList<>();
            intersectionsOnEdge.add(emptyIntersections);
        }


        /* First action action */
        for (Subtrajectory sub : subtrajectories) {
            int j = 0;
            Line2D lineOnProposedPoints;
            for (int i = 0; i < sub.numPoints(); i++) {
                int k = j;

                Point2D intersectionPoint = null;
                int intersectionK = 0;
                while (true) {
                    /* We get the line off the representative we are looking at */
                    lineOnProposedPoints = new Line2D.Double(proposedPoints.get(k), proposedPoints.get(k + 1));
                    /* Perpendicular line on lineOnProposedPoints through point sub.getPoint(i) */
                    Line2D perpLineThroughPoint = getLongPerpendicularOnACThroughB(lineOnProposedPoints, sub.getPoint(i));

                    /* We check if it intersects, if so, we add the intersectionPoint and update starting search j */
                    double minDistance = Double.MAX_VALUE;
                    if (perpLineThroughPoint.intersectsLine(lineOnProposedPoints)) {
                        Point2D calculatedIntersectionPoint =
                                GeometryUtil.intersectionPoint(perpLineThroughPoint, lineOnProposedPoints);
                        if (calculatedIntersectionPoint == null) {
                            System.out.println("Error Forces.filterInExtraVIPPoints. IntersectionPoint not found");
                        } else if (lineOnProposedPoints.ptSegDist(calculatedIntersectionPoint) < minDistance) {
                            intersectionPoint = calculatedIntersectionPoint;
                            intersectionK = k;
//                            intersectionsOnEdge.get(k).add(intersectionPoint);
//                            j = 0;
//                            j = Math.max(0, k - 5); /* Preventing j = -1; */
//                            break;
                        }
                    }
                    /* Prevent indexOutOfRange */
                    if (k < proposedPoints.size() - 2) {
                        k += 1;
                    } else {
                        break;
                    }
                }
                if (intersectionPoint != null) {
                    intersectionsOnEdge.get(intersectionK).add(intersectionPoint);
                }
            }
        }

        /* Second action */
        List<Point2D> newlyProposedRepresentative = new ArrayList<>();
        for (int i = 0; i < proposedPoints.size() - 1; i++) {
            Line2D lineSegment = new Line2D.Double(proposedPoints.get(i), proposedPoints.get(i + 1));

            double maxLimit = Math.ceil((double) subtrajectories.size() / 3);
            newlyProposedRepresentative.addAll(
                    splitOffRepresentativeEdge(lineSegment, intersectionsOnEdge.get(i), 3, (int) maxLimit));
        }
        newlyProposedRepresentative.add(proposedPoints.get(proposedPoints.size() - 1));
        return newlyProposedRepresentative;
    }

    /**
     * Given a certain lineSegment, we split the lineSegment up into two and run the same again.
     * This functions runs until remainingSplitOffs is 0 or when the length is < 2 * getForceMinDistancePoints().
     * In the end we return a list of Points.
     *
     * @param lineSegment,          the line segment where the intersections are located on and that might be split.
     * @param intersections,        all the intersections on this line
     * @param remainingSplitOffs,   how many times we can still do this operation.
     * @param maximumIntersections, how many intersections are allowed on one edge.
     * @return list of representative edges. Containing all points but the last.
     */
    private static List<Point2D> splitOffRepresentativeEdge(Line2D lineSegment, List<Point2D> intersections,
                                                            int remainingSplitOffs, int maximumIntersections) {
        List<Point2D> newlyProposedRepresentative = new ArrayList<>();

        Point2D lineP1 = lineSegment.getP1();
        Point2D lineP2 = lineSegment.getP2();
        final double percentageOnSidesThatDoesntCount = 0.1;
        int numberOfIntersectionInTheMiddle = 0;
        for (Point2D intersection : intersections) {
            if (lineP1.distance(lineP2) * percentageOnSidesThatDoesntCount < lineP1.distance(intersection) &&
                    lineP1.distance(lineP2) * percentageOnSidesThatDoesntCount < lineP2.distance(intersection)) {
                numberOfIntersectionInTheMiddle = numberOfIntersectionInTheMiddle + 1;
            }
        }


        if (numberOfIntersectionInTheMiddle <= maximumIntersections) {
            newlyProposedRepresentative.add(lineSegment.getP1());
        } else if (remainingSplitOffs > 0 && lineSegment.getP1().distance(lineSegment.getP2()) > ALGOCONSTANTS.getForceMinDistancePoints()) {
            List<Point2D> onFirstEdge = new ArrayList<>();
            List<Point2D> onSecondEdge = new ArrayList<>();

            for (Point2D intersection : intersections) {
                if (lineSegment.getP1().distance(intersection) <= lineSegment.getP2().distance(intersection)) {
                    /* The point is closer to the first point of lineSegment, hence is on the first edge. */
                    onFirstEdge.add(intersection);
                } else {
                    onSecondEdge.add(intersection);
                }
            }

            Point2D middlePoint = new Point2D.Double(
                    (lineSegment.getX1() + lineSegment.getX2()) / 2,
                    (lineSegment.getY1() + lineSegment.getY2()) / 2
            );
            Line2D firstEdge = new Line2D.Double(lineSegment.getP1(), middlePoint);
            Line2D secondEdge = new Line2D.Double(middlePoint, lineSegment.getP2());
            newlyProposedRepresentative.addAll(splitOffRepresentativeEdge(firstEdge, onFirstEdge,
                    remainingSplitOffs - 1, maximumIntersections));
            newlyProposedRepresentative.addAll(splitOffRepresentativeEdge(secondEdge, onSecondEdge,
                    remainingSplitOffs - 1, maximumIntersections));
        } else { /* if remainingSplitOffs <= 0 */
            double averagePointX = 0.0;
            double averagePointY = 0.0;
            for (int i = 0; i < intersections.size(); i++) {
                /* Preventing bufferOverFlow */
                averagePointX += intersections.get(i).getX() / intersections.size();
                averagePointY += intersections.get(i).getY() / intersections.size();
            }

            newlyProposedRepresentative.add(lineSegment.getP1());
            newlyProposedRepresentative.add(new Point.Double(averagePointX, averagePointY));
        }

        return newlyProposedRepresentative;
    }

    /**
     * Given an original point and a weight and another new point, compute their average.
     *
     * @param originalPoint the already averaged out point (not including newPoint).
     * @param weight        weight of the originalPoint = equal to the number of points that were already averaged out
     * @param newPoint      the new point that is also going to be included
     * @return Point2D of the new average.
     */
    private static Point2D getAveragePoint(Point2D originalPoint, int weight, Point2D newPoint) {
        weight = weight + 1;
        double x = originalPoint.getX() / weight * (weight - 1);
        double y = originalPoint.getY() / weight * (weight - 1);
        x += (newPoint.getX() / weight);
        y += (newPoint.getY() / weight);
        return new Point2D.Double(x, y);
    }

    /**
     * For a given edge AC and it's perpendicular line through B, find all edges of the trajectories that
     * intersect with the perpendicular line through B. For every intersection get the intersection point and the angle
     * between the edge and edge AC.
     *
     * @param trajectories      the subtrajectories which points will be on a perpendicular line of OriginalAC.
     * @param perpendicularLine the
     * @param originalAC        the line where we take the perpendicular lines of which we later on check intersection with
     * @return List of pairs from Point2D to Doubles.
     */
    private static List<OrthogonalIntersection> findIntersections(Set<Subtrajectory> trajectories,
                                                                  Line2D perpendicularLine,
                                                                  Line2D originalAC,
                                                                  Point2D originalPoint) {
        List<OrthogonalIntersection> intersectionsWithAngels = new ArrayList<>();

        for (Subtrajectory sub : trajectories) {
            double minDistance = Double.MAX_VALUE;
            double minimumAngleFound = 180;
            Point2D minimumAnglePoint = null;
            int edgeIndex = -1;
            for (int j = 0; j < sub.numPoints() - 1; j++) {
                if (perpendicularLine.intersectsLine(sub.getEdge(j))) {
                    Point2D intersectionPoint = GeometryUtil.intersectionPoint(perpendicularLine, sub.getEdge(j));
                    if (intersectionPoint != null) {
                        double differenceAngle =
                                GeometryUtil.getHeadingDirectionDifference(sub.getEdge(j), originalAC);

                        double distance = sub.getEdge(j).ptSegDist(originalPoint);
                        if (distance < minDistance && differenceAngle < 90) {
                            minimumAnglePoint = intersectionPoint;
                            minimumAngleFound = differenceAngle;
                            edgeIndex = j;
                            minDistance = distance;
                        }
//                        if (differenceAngle < minimumAngleFound) {
//                            minimumAnglePoint = intersectionPoint;
//                            minimumAngleFound = differenceAngle;
//                            edgeIndex = j;
//                        }
                    } else {
//                        This is not really an error. It's quite often possible we can't find this..
//                        System.out.println("Error Forces.calculateForceStep. Not able to find the intersection");
                    }
                }
            }
            if (minimumAngleFound != 180) {
                intersectionsWithAngels.add(
                        new OrthogonalIntersection(minimumAnglePoint, minimumAngleFound, sub, edgeIndex)
                );
            }
        }
        return intersectionsWithAngels;
    }

    /**
     * This calculates a force step for a given trajectory. This force step is always over perpACTroughB.
     *
     * @param intersectionsWithAngles all intersections and their relative angles
     * @param perpACThroughB,         the orthogonal line the force should be executed over
     * @param places                  the points of the representative used for calculation
     * @param epsilon,                the epsilon of the bundle.
     * @return Point2D, a point which calculates the force step
     */
    private static Point2D calculateForceStep(List<OrthogonalIntersection> intersectionsWithAngles,
                                              Line2D perpACThroughB, List<Point2D> places, double epsilon) {
        double force = 0.0;
        for (OrthogonalIntersection intersection : intersectionsWithAngles) {
            if (intersection.getAngle() < ALGOCONSTANTS.getForceHeadingDifference()) {
                force += forceNegate(places.get(places.size() - 1), intersection.getPoint());
            }
        }
        return executeForceStep(places.get(places.size() - 1), force, perpACThroughB);
        /* A force from it's original position */
    }

    /**
     * Calculates a parabola and get's the value according to this.
     * Partial credits: https://www.youtube.com/watch?v=LY3EMo8Gl2Q&ab_channel=ibmathshall
     * Showing of curve: http://www.wolframalpha.com/input/?i=y+%3D+(2%2F2025)(x%2B45)(x-45)+%2B+3
     *
     * @param x,           the x coordinate we will execute the function for
     * @param xDifference, the maximum we are looking at.
     * @param yDifference, the difference in the y axis between the bottom of the parabola and the height of
     *                     the point at xDifference. (Hence for x=xDifference we should get y = yStart+yDifference
     * @param yStart,      the starting point of our y at x=0.
     * @return the y value
     */
    static double calculateParabola(double x, double xDifference, double yDifference, double yStart) {

        /* The function is defined by y=a(x-p)(x-q) as seen in the youtube video. */
        /* Hence we calculate for a for y = -yDifference, x = 0, and p and q are -xDifference and xDifference. */
        double a = -yDifference / (xDifference * -xDifference);
        /* Now we have the complete function, but it is still shifted to much to the bottom. */
        /* Hence we shift it up by yStart+yDifference */
        double shift = yStart + yDifference;

        /* Do you final computation where y = a(x-p)(x-q) + shift */
        return Math.max(yStart, a * (x + xDifference) * (x - xDifference) + shift);
    }


    /**
     * Calculates a step of 1 meter.
     *
     * @param currentPoint   current place of the point
     * @param force          the force applied to the point
     * @param perpACThroughB perpendicular line over which the step should occur.
     * @return the new point.
     */
    private static Point2D executeForceStep(Point2D currentPoint, double force, Line2D perpACThroughB) {
        /* Without this statement there is different behavior for in the middle with slope>99999 && slope <-99999. */
        if (DoubleMath.fuzzyEquals(force, 0.0, 1E-5)) {
            return new Point2D.Double(currentPoint.getX(), currentPoint.getY());
        }

        double diffX = perpACThroughB.getX1() - perpACThroughB.getX2();
        double diffY = perpACThroughB.getY1() - perpACThroughB.getY2();
        /* In the case the difference of X-axis is 0, the line is equal to Line2D(pointB, pointB + some on y-axis) */
        double appendToX = 0.0;
        double appendToY = 0.0;

        double forceDirection = ((force > 0.0) ? 1.0 : -1.0);
        if (DoubleMath.fuzzyEquals(diffX, 0.0, 1E-5)) {
            appendToY = forceDirection * 1.0;
        } else if (DoubleMath.fuzzyEquals(diffY, 0.0, 1E-5)) {
            appendToX = forceDirection * 1.0;
        } else {
            double slope = diffY / diffX;

            // @FixMe Add slope >= -1.0 && make sure it gives the same behavior. It gave behaviour glitches before.
            if (slope <= 1.0) { // to prevent buffer overflow, we split the two cases
                double length = Math.sqrt(1 + (slope * slope));
                appendToX = 1 / length;
                appendToY = slope * appendToX;
            } else {
                double length = Math.sqrt(1 + ((1 / slope) * (1 / slope)));
                appendToY = 1 / length;
                appendToX = (1 / slope) * appendToY;
            }
            appendToX = forceDirection * appendToX;
            appendToY = forceDirection * appendToY;
        }
        return new Point2D.Double(currentPoint.getX() + appendToX, currentPoint.getY() + appendToY);
    }

    /**
     * Given two points, it calculates whether the force of newPlace on originalPlace should be negated.
     * Any forces that has delta(x)>0 is positive, any force that has delta(x)<0 is negative.
     * Then if delta(x) = 0, delta(y)>=0 would return positive, delta(y)<= return negative.
     * If the points are equal, we return 0;
     *
     * @param currentPlace,       place where the force is executed on.
     * @param forceApplyingPoint, place where the force comes from.
     * @return int i {@code = -1} when newPlace is tot he southwest of originalPlace, {@code = 1} otherwise.
     */
    private static int forceNegate(Point2D currentPlace, Point2D forceApplyingPoint) {
        if (DoubleMath.fuzzyEquals(currentPlace.getX(), forceApplyingPoint.getX(), 1E-9) &&
                DoubleMath.fuzzyEquals(currentPlace.getY(), forceApplyingPoint.getY(), 1E-9)) {
            return 0;
        }

        if (forceApplyingPoint.getX() - currentPlace.getX() > 0.0) { // Delta(x) > 0
            return 1;
        } else if (forceApplyingPoint.getX() - currentPlace.getX() < 0.0) { // Delta(x) < 0
            return -1;
        } else if (forceApplyingPoint.getY() - currentPlace.getY() > 0.0) { // At this point we know Delta(x) = 0
            return 1;
        } else if (forceApplyingPoint.getY() - currentPlace.getY() < 0.0) {
            return -1;
        }
        return 0;
    }

    /**
     * Given a representative and an index(of B). Calculate the line AC used for creating the orthogonal line through B.
     * <p>
     * In case there is no neighbour to the left(or right of B), we take B instead of the neighbour.
     *
     * @param representativePoints, the representative line of the bundle.
     * @param i,                    index of the representative.
     * @return Line2D AC, the line from A to C.
     */
    private static Line2D getACLine(List<Point2D> representativePoints, int i) {
        Point2D pointA = representativePoints.get(i);
        Point2D pointB = representativePoints.get(i);
        Point2D pointC = representativePoints.get(i);
//        if (i - 1 < 0) {
//            pointA = representativePoints.get(0);
//        } else {
//            pointA = representativePoints.get(i - 1);
//        }
//
//        if (i + 1 > representativePoints.size() - 1) {
//            pointC = representativePoints.get(representativePoints.size() - 1);
//        } else {
//            pointC = representativePoints.get(i + 1);
//        }

        double dA = 0.0;
        double dC = 0.0;
        double firstAAngle = 0.0;
        double firstCAngle = 0.0;
        int indexA = i;
        int indexC = i;
        double minDistance = ALGOCONSTANTS.getForceMinDistanceAB();
        boolean somethingChanged = true;

        while (somethingChanged) {
            somethingChanged = false;

            if (dA < minDistance) {
                if (indexA > 0) {
                    indexA--;

                    double angleToPoint = GeometryUtil.getHeadingDirection(new Line2D.Double(
                            representativePoints.get(indexA), pointA));
                    if (firstAAngle == 0.0) {
                        firstAAngle = angleToPoint;
                    }
                    double angleDifference = GeometryUtil.getAbsoluteAngleDifference(firstAAngle, angleToPoint);
                    // SuperSharpCheck: Preventing more than 125 degree turns of making our representative super ugly..
                    if (pointB.distance(representativePoints.get(indexA)) < dA || angleDifference > 125) {
                        break;
                    }
                    pointA = representativePoints.get(indexA);
                    dA = pointB.distance(pointA);

                    somethingChanged = true;
                } else {
                    pointA = representativePoints.get(0);
                    dA = pointB.distance(pointA);
                }
            }

            if (dC < minDistance) {
                if (indexC < representativePoints.size() - 1) {
                    indexC++;
                    double angleToPoint = GeometryUtil.getHeadingDirection(new Line2D.Double(
                            pointC, representativePoints.get(indexC)
                    ));

                    if (firstCAngle == 0.0) {
                        firstCAngle = angleToPoint;
                    }

                    double angleDifference = GeometryUtil.getAbsoluteAngleDifference(firstCAngle, angleToPoint);
                    // SuperSharpCheck: Preventing more than 125 degree turns of making our representative super ugly..
                    if (pointB.distance(representativePoints.get(indexC)) < dC || angleDifference > 125) {
                        break;
                    }

                    pointC = representativePoints.get(indexC);
                    dC = pointB.distance(pointC);

                    somethingChanged = true;
                } else {
                    pointC = representativePoints.get(representativePoints.size() - 1);
                    dC = pointB.distance(pointC);
                }
            }

        }
        if (dA >= minDistance && dC > dA) {
            // We are cutting off PointC.
            Point2D pointBeforeC = representativePoints.get(indexC - 1);
            double leftDistance = dA - pointBeforeC.distance(pointB);
            Line2D lastLine = new Line2D.Double(pointBeforeC, pointC);
            double lineLength = pointC.distance(pointB) - pointBeforeC.distance(pointB);

            double cutOff = Math.max(Math.min(1, leftDistance / lineLength), 0);
            if (!DoubleMath.fuzzyEquals(cutOff, leftDistance / lineLength, 1E-5)) {
                System.out.println("Error Forces.GetACLine. leftDistance was less then 0 or?");
            }
            pointC = GeometryUtil.getPointOnLine(lastLine, cutOff);

        } else if (dC >= minDistance && dA > dC) {
            // We are cutting off pointA
            Point2D pointAfterA = representativePoints.get(indexA + 1);
            double leftDistance = dC - pointAfterA.distance(pointB);
            Line2D lastLine = new Line2D.Double(pointAfterA, pointA);
            double lineLength = pointA.distance(pointB) - pointAfterA.distance(pointB);

            double cutOff = Math.max(Math.min(1, leftDistance / lineLength), 0);
            if (!DoubleMath.fuzzyEquals(cutOff, leftDistance / lineLength, 1E-5)) {
                System.out.println("Error Forces.GetACLine. leftDistance was less then 0 or?");
            }
            pointA = GeometryUtil.getPointOnLine(lastLine, cutOff);
        }


        return new Line2D.Double(pointA, pointC);
    }

    /**
     * Calculates a perpendicular line of 2*MAX_EPS length on lineAC which has B exactly in the middle.
     *
     * @param lineAC, calculates the lineAC.
     * @param pointB, calculates point B.
     * @return the perpendicular line
     */
    private static Line2D getLongPerpendicularOnACThroughB(Line2D lineAC, Point2D pointB) {
        double MAX_EPS = ALGOCONSTANTS.getForceMaxEps() / 2;
        // Extracted into the util.
        return OrthogonalUtil.getPerpendicularOnACThroughB(lineAC, pointB, MAX_EPS);
    }
}

