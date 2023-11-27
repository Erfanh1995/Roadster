package mapconstruction.algorithms.representative;

import com.google.common.base.Preconditions;
import mapconstruction.algorithms.representative.containers.Turn;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;

/**
 * Class responsible for merging the representative of the bundle with the identified turns.
 *
 * @author Jorrick Sleijster
 */
public class ForceTurnMerger {

    /**
     * Merges turns and force representatives.
     *
     * @param turns,    all detected turns
     * @param forceRep, force representative of at least 2 points.
     * @return a list of points which merged the representative and the turns.
     */
    public static List<Point2D> MergeTurnsAndForceRepresentative(List<Turn> turns,
                                                                 List<Point2D> forceRep) {

        List<Turn> sortedTurns = sortTurnsBasedOnRep(turns, forceRep);

        List<Point2D> newRep = new ArrayList<>();
        int mergedUpTill = 0;

        for (int i = 0; i < sortedTurns.size(); i++) {
            Turn turn = sortedTurns.get(i);
            Point2D turnLocation = turn.getTurnLocation();
            Line2D beforeLinearRegressionLine = new Line2D.Double(
                    turn.getLinearRegressionLines().get(0),
                    turn.getLinearRegressionLines().get(1)
            );
            Line2D afterLinearRegressionLine = new Line2D.Double(
                    turn.getLinearRegressionLines().get(2),
                    turn.getLinearRegressionLines().get(3)
            );

            double beforeLRHeadingDirection = GeometryUtil.getHeadingDirection(beforeLinearRegressionLine);
            double afterLRHeadingDirection = GeometryUtil.getHeadingDirection(afterLinearRegressionLine);


            double minDistance = Double.MAX_VALUE;
            int minDistanceIndex = 0;

            // We find the edge on the representative with the minimum distance.
            for (int j = 0; j < forceRep.size() - 1; j++) {
                Line2D line = new Line2D.Double(forceRep.get(j), forceRep.get(j + 1));
                double distance = line.ptSegDist(turnLocation);

                if (distance < minDistance) {
                    minDistance = distance;
                    minDistanceIndex = j;
                }
            }

            // Given the closest edge, we first try to improve on the left.
            int bestBeforeMatchIndex = determineBestMergePoint(forceRep, turnLocation, beforeLRHeadingDirection,
                    minDistanceIndex, -1, false);
            int bestAfterMatchIndex = determineBestMergePoint(forceRep, turnLocation, afterLRHeadingDirection,
                    minDistanceIndex + 1, forceRep.size(), true);

            if (bestBeforeMatchIndex >= bestAfterMatchIndex) {
                System.out.println("Error ForceTurnMerger. Unexpected results.");
            }

            double maxAngleInRep = getMaxAngleInPartOfPointList(bestBeforeMatchIndex, bestAfterMatchIndex, forceRep);
            double angleOfTurn = getAngleOfTurn(bestBeforeMatchIndex, bestAfterMatchIndex, turnLocation, forceRep);
            if (angleOfTurn > maxAngleInRep + 60) {
                // This is a turn that shouldn't be added at this place...
//                System.out.println("Filtering out the turn");
                sortedTurns.remove(i);
                turns.remove(turn); // So the bundle list is also updated.
                i--;
                continue;
            }

            for (int j = mergedUpTill; j <= bestBeforeMatchIndex && j < forceRep.size(); j++) {
                newRep.add(forceRep.get(j));
            }
            newRep.add(turnLocation);
            mergedUpTill = bestAfterMatchIndex;
        }

        // Add the remaining points.
        for (int i = mergedUpTill; i < forceRep.size(); i++) {
            newRep.add(forceRep.get(i));
        }

        return newRep;
    }

    /**
     * Determines the best merge point (before or after i) on the representative, given a linear regression line.
     *
     * @param forceRep,     force based representative
     * @param turnLocation, actual turn location
     * @param lrHeading,    linear regression heading direction
     * @param startIndex,   starting index.
     * @param endIndex,     ending index.
     * @param increasing,   whether we are walking backwards on the representative(false), or walking forward(increasing)
     * @return the index of the best possible merge point within reach.
     */
    private static int determineBestMergePoint(List<Point2D> forceRep, Point2D turnLocation, double lrHeading,
                                               int startIndex, int endIndex, boolean increasing) {
        int bestMatchIndex = startIndex;
        double bestMatchScore = Double.MAX_VALUE;
        double score;

        int j = startIndex;

        while ((j < endIndex && increasing) || (j > endIndex && !increasing)) {
            Point2D currentPoint = forceRep.get(j);
//            Line2D line = new Line2D.Double(turnLocation, currentPoint);
            Line2D line;
            if (increasing) {
//                line = new Line2D.Double(currentPoint, turnLocation);
                line = new Line2D.Double(turnLocation, currentPoint);
            } else {
//                line = new Line2D.Double(turnLocation, currentPoint);
                line = new Line2D.Double(currentPoint, turnLocation);
            }
            double lineDirection = GeometryUtil.getHeadingDirection(line);
            double distanceTurnLocation = turnLocation.distance(currentPoint);

            score = GeometryUtil.getAbsoluteAngleDifference(lineDirection, lrHeading);
            score = score * (1 / Math.sqrt(Math.sqrt(distanceTurnLocation)));

            if (score < bestMatchScore) {
                bestMatchIndex = j;
                bestMatchScore = score;
            }

            if (currentPoint.distance(turnLocation) > ALGOCONSTANTS.getTurnLengtenMaxDistance() * (3 / 2)) {
                break;
            }

            if (increasing) {
                j++;
            } else {
                j--;
            }
        }

        return bestMatchIndex;

    }

    /**
     * Based on the representative, we sort the turns based on when they will happen compared to the bundle.
     *
     * @param turns,    the turns to sort
     * @param forceRep, the representative
     * @return the turns but in a sorted order.
     */
    public static List<Turn> sortTurnsBasedOnRep(List<Turn> turns,
                                                 List<Point2D> forceRep) {
        List<Turn> sortedTurns = new ArrayList<>();
        List<Integer> turnIsClosestToIndex = new LinkedList<>();
        for (Turn turn : turns) {
            Point2D turnLocation = turn.getTurnLocation();

            double minDistance = Double.MAX_VALUE;
            int closestIndex = -1;
            for (int j = 0; j < forceRep.size() - 1; j++) {
                Line2D line = new Line2D.Double(forceRep.get(j), forceRep.get(j + 1));
                double distance = line.ptSegDist(turnLocation);

                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = j;
                }
            }
            turnIsClosestToIndex.add(closestIndex);
        }

        List<Integer> sortedTurnIsClosestToIndex = new ArrayList<Integer>(turnIsClosestToIndex);
        Collections.sort(sortedTurnIsClosestToIndex);

        // We merge iteratively
        for (Integer aSortedTurnIsClosestToIndex : sortedTurnIsClosestToIndex) {
            int indexOfCurrentItem = turnIsClosestToIndex.indexOf(aSortedTurnIsClosestToIndex);
            sortedTurns.add(turns.get(indexOfCurrentItem));
        }
        return sortedTurns;
    }

    /**
     * We calculate the maximum heading angle difference between all the edge combination from startIndex to endIndex
     *
     * @param startIndex, the starting index to search for
     * @param endIndex,   the ending index to search for
     * @param forceRep,   the force representative we are getting the points from
     * @return the maximum heading angle difference
     */
    private static double getMaxAngleInPartOfPointList(int startIndex, int endIndex, List<Point2D> forceRep) {
        Preconditions.checkNotNull(forceRep, "forceRep is null");
        if (startIndex >= 1) {
            startIndex = startIndex - 1;
        }
        if (endIndex <= forceRep.size() - 3) {
            // Consider edge at index 0 to be [0, 1] and forceRep.size() to be m
            // [m-2, m-1] would be the last edge, hence we take -3, as [m-3, m-2] is the edge before the last edge.
            endIndex = endIndex + 1;
        }

        double maxAngle = 0.0;
        double edgeMinLength = 10.0;
        for (int i = startIndex; i < endIndex - 1; i++) {
            Line2D firstLine = getLineOfMinLength(i, forceRep, edgeMinLength);

            for (int j = i + 1; j < endIndex; j++) {
                Line2D secondLine = getLineOfMinLength(j, forceRep, edgeMinLength);
                maxAngle = Math.max(maxAngle, GeometryUtil.getHeadingDirectionDifference(firstLine, secondLine));
            }
        }

        return maxAngle;
    }

    /**
     * Get's the heading angle difference of the turn.
     *
     * @param startIndex, the index of the first point
     * @param endIndex,   the index of the ending point
     * @param location,   the location of the turn
     * @param forceRep,   the list of points representing the representative.
     * @return the heading angle difference
     */
    private static double getAngleOfTurn(int startIndex, int endIndex, Point2D location, List<Point2D> forceRep) {
        Line2D firstLine = new Line2D.Double(forceRep.get(startIndex), location);
        Line2D secondLine = new Line2D.Double(location, forceRep.get(endIndex));
        return GeometryUtil.getHeadingDirectionDifference(firstLine, secondLine);
    }

    /**
     * This get's the edge currentIndex. Now if the edge is smaller than length, we extend it with the next point in
     * the list. This is done until the edge is of length.
     *
     * @param index,    index of the edge in the trajectory.
     * @param forceRep, the list of points, representing the index.
     * @param length,   the minimum length of the edge.
     * @return a line object which represents the edge but with a minimum length.
     */
    private static Line2D getLineOfMinLength(int index, List<Point2D> forceRep, double length) {
        if (index > forceRep.size() - 2) {
            throw new IllegalArgumentException("index > forceRep.size() - 2");
        }
        int startingIndex = index;
        int endingIndex = index + 1;
        Point2D startingPoint = forceRep.get(startingIndex);
        Point2D endingPoint = forceRep.get(endingIndex);

        while (startingPoint.distance(endingPoint) < length) {
            if (endingIndex < forceRep.size() - 1) {
                endingIndex += 1;
                endingPoint = forceRep.get(endingIndex);
            } else if (startingIndex > 0) {
                startingIndex -= 1;
                startingPoint = forceRep.get(startingIndex);
            } else {
                break;
            }
        }

        return new Line2D.Double(startingPoint, endingPoint);
    }
}
