package mapconstruction.algorithms.representative.containers;

import com.google.common.collect.Range;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.LinearRegression;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;
import java.util.stream.Collectors;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;

/**
 * Class responsible for keeping track of sharp turns.
 * <p>
 * The class furthermore lengthens the part before and after the turn, it uses linear regression on these parts and
 * finally finds (in most cases) a better actual intersection point.
 *
 * @author Jorrick Sleijster
 */
public class Turn implements Serializable {
    /**
     * Points of the subtrajectories that are indicated as part of this turn.
     */
    private List<List<Point2D>> subtrajectoryTurnParts;
    /**
     * Average turning point (average of all largestAnglePoints).
     */
    private Point2D averagePoint;
    /**
     * All points that have the largest angles of the subtrajectoryTurnParts.
     */
    private List<Point2D> largestAnglePoints;
    /**
     * For the first edge, the average heading direction in degrees.
     */
    private double averageFirstAngle;
    /**
     * For the last edge, the average heading direction in degrees.
     */
    private double averageLastAngle;
    /**
     * Subtrajectories that are part of this turn.
     */
    private List<Subtrajectory> trajectoriesIncluded;
    /**
     * The range of the indexes of the Subtrajectories that indicate which points where part of this turn.
     */
    private List<Range<Integer>> trajectoriesRanges;
    /**
     * Points of the subtrajectories that are indicated as before this turn.
     */
    private List<List<Point2D>> subBeforeTurnParts;
    /**
     * Points of the subtrajectories that are indicated as after this turn.
     */
    private List<List<Point2D>> subAfterTurnParts;
    /**
     * Calculated turn location
     */
    private Point2D turnLocation;
    /**
     * The linear regression lines
     */
    private List<Point2D> linearRegressionLines;
    /**
     * Which bundle the turn is part of.
     */
    private Bundle b;


    /**
     * Initializes a Turn
     *
     * @param turnPart,          all the different points that are part of this turn for this specific sub
     * @param largestAnglePoint, find the largest angle point.
     * @param sub,               subtrajectory which created this turn.
     * @param indexes,           indexes.
     */
    public Turn(List<Point2D> turnPart, Point2D largestAnglePoint, Subtrajectory sub, Range<Integer> indexes, Bundle b) {
        this.b = b;
        averagePoint = new Point2D.Double(0, 0);
        turnLocation = null;

        subtrajectoryTurnParts = new ArrayList<>();
        largestAnglePoints = new ArrayList<>();
        trajectoriesIncluded = new ArrayList<>();
        trajectoriesRanges = new ArrayList<>();
        subBeforeTurnParts = new ArrayList<>();
        subAfterTurnParts = new ArrayList<>();
        linearRegressionLines = new ArrayList<>();

        addSubtrajectoryTurn(turnPart, largestAnglePoint, sub, indexes);
    }

    /**
     * Adding a new turn part of a subtrajectory to the turn together with it's largest angle point.
     *
     * @param turnPart          the points of the edges that are part of the turn.
     * @param largestAnglePoint largest angle of the part of the turn.
     */
    public void addSubtrajectoryTurn(List<Point2D> turnPart, Point2D largestAnglePoint, Subtrajectory sub,
                                     Range<Integer> indexes) {
        addSubtrajectoryTurnPart(turnPart);
        addLargestAnglePoint(largestAnglePoint);
        trajectoriesIncluded.add(sub);
        trajectoriesRanges.add(indexes);
    }

    /**
     * Adding a new Subtrajectories turn part
     *
     * @param turnPart the points of the edges that are part of the turn.
     */
    private void addSubtrajectoryTurnPart(List<Point2D> turnPart) {
        int n = turnPart.size() - 1;
        double firstAngle = GeometryUtil.getDirectionInDegrees(new Line2D.Double(turnPart.get(0), turnPart.get(1)));
        double lastAngle = GeometryUtil.getDirectionInDegrees(new Line2D.Double(turnPart.get(n - 1), turnPart.get(n)));

        averageFirstAngle = getNewAngleAverage(subtrajectoryTurnParts.size(), averageFirstAngle, firstAngle);
        averageLastAngle = getNewAngleAverage(subtrajectoryTurnParts.size(), averageLastAngle, lastAngle);

        subtrajectoryTurnParts.add(turnPart);
    }

    /**
     * We add a new largest angle point and update averagePoint accordingly
     */
    private void addLargestAnglePoint(Point2D largestAnglePoint) {
        int nop = largestAnglePoints.size();
        double averagePointX = getNewAverage(nop, averagePoint.getX(), largestAnglePoint.getX());
        double averagePointY = getNewAverage(nop, averagePoint.getY(), largestAnglePoint.getY());

        averagePoint = new Point2D.Double(averagePointX, averagePointY);
        largestAnglePoints.add(largestAnglePoint);
    }

    /**
     * Get's the difference between the first and last edge average.
     * <p>
     * It returns the difference of the first edge with the average plus and the second edge with the average.
     *
     * @param firstEdge the first edge of another turn
     * @param lastEdge  the last edge of another turn
     * @return
     */
    public double getDifferenceAngle(Line2D firstEdge, Line2D lastEdge) {
        double firstAngle = GeometryUtil.getDirectionInDegrees(firstEdge);
        double lastAngle = GeometryUtil.getDirectionInDegrees(lastEdge);

        double firstDiff = Math.abs(firstAngle - averageFirstAngle);
        double lastDiff = Math.abs(lastAngle - averageLastAngle);

        return Math.min(360 - firstDiff, firstDiff) + Math.min(360 - lastDiff, lastDiff);
    }

    /**
     * Returns whether the subtrajectory is absent and thus not already present in the turn.
     *
     * @param subtrajectory where we want to check for
     * @return boolean whether it already contains a part in this turn.
     */
    public boolean isAbsentInTurn(Subtrajectory subtrajectory) {
        return trajectoriesIncluded.stream().noneMatch(sub -> sub.equals(subtrajectory));
    }

    /**
     * Get the number of subtrajectories that are part of this turn
     *
     * @return size of trajectories
     */
    public int numberOfTrajectoriesRepresented() {
        return trajectoriesIncluded.size();
    }

    /**
     * Calculates the next average based on the current average. Avoids overflow.
     *
     * @param currentlyIncluded, currently included number of doubles.
     * @param currentValue,      current average double value.
     * @param newlyAddedValue,   the new value that is added to the average.
     * @return
     */
    private double getNewAverage(int currentlyIncluded, double currentValue, double newlyAddedValue) {
        int nop = currentlyIncluded + 1;
        return (currentValue / nop) * (nop - 1) + newlyAddedValue / nop;
    }

    /**
     * Does the same thing as new average but now for angles. This requires special attention.
     * Example: getNewAngleAverage(1, 350, 30) = 10.
     *
     * @param currentlyIncluded, currently included number of doubles.
     * @param currentValue,      current average double value.
     * @param newlyAddedValue,   the new value that is added to the average.
     * @return
     */
    private double getNewAngleAverage(int currentlyIncluded, double currentValue, double newlyAddedValue) {
        if (currentlyIncluded == 0) {
            return newlyAddedValue;
        }

        if (Math.abs(currentValue - newlyAddedValue) > 180) {
            // Either increase newlyAddedValue with 360 or decrease it with 360
            if (currentValue < 180 && newlyAddedValue > 180) {
                newlyAddedValue -= 360;
            }
            if (currentValue > 180 && newlyAddedValue < 180) {
                newlyAddedValue += 360;
            }
        }
        double result = getNewAverage(currentlyIncluded, currentValue, newlyAddedValue);
        if (result >= 360) {
            result -= 360;
        }
        if (result < 0) {
            result += 360;
        }
        return result;
        // Check whether output changed.
    }

    /**
     * Get's the turn location.
     */
    public Point2D getTurnLocation() {
        // If already calculated, don't recalculate.
        if (turnLocation != null) {
            return turnLocation;
        }

        List<Point2D> linearRegressionLines = getLinearRegressionLines();

        Line2D firstLine = new Line2D.Double(linearRegressionLines.get(0), linearRegressionLines.get(1));
        Line2D secondLine = new Line2D.Double(linearRegressionLines.get(2), linearRegressionLines.get(3));

        if (firstLine.intersectsLine(secondLine)) {
            turnLocation = GeometryUtil.intersectionPoint(firstLine, secondLine);
        } else {
            turnLocation = averagePoint;
        }
        return turnLocation;
    }

    /**
     * Creates the linear regression lines
     *
     * @return 4 points which contain the linear regression lines
     */
    public List<Point2D> getLinearRegressionLines() {
        enlargePartBeforeTurn();
        enlargePartAfterTurn();

        // If already calculated, ignore.
        if (linearRegressionLines.size() > 0) {
            return linearRegressionLines;
        }
        Map<String, Object> dictBeforeTurn = new HashMap<>();
        Map<String, Object> dictAfterTurn = new HashMap<>();

        List<List<Point2D>> subBeforeTurnPartsFiltered;
        List<List<Point2D>> subAfterTurnPartsFiltered;
//        if (ALGOCONSTANTS.isFrequentDataset()) {
//            subBeforeTurnPartsFiltered = subBeforeTurnParts;
//            subAfterTurnPartsFiltered = subAfterTurnParts;
//        } else {
        subBeforeTurnPartsFiltered = getBestFittingPartForGivenAngle(subBeforeTurnParts, averageLastAngle);
        subAfterTurnPartsFiltered = getBestFittingPartForGivenAngle(subAfterTurnParts, averageFirstAngle);
//        }

        Line2D beforeLRLine = LinearRegression.applyLinearRegressionNextToTurn(
                subBeforeTurnPartsFiltered, dictBeforeTurn, averagePoint, 200);
        Line2D afterLRLine = LinearRegression.applyLinearRegressionNextToTurn(
                subAfterTurnPartsFiltered, dictAfterTurn, averagePoint, 200);
        linearRegressionLines.add(beforeLRLine.getP1());
        linearRegressionLines.add(beforeLRLine.getP2());
        linearRegressionLines.add(afterLRLine.getP1());
        linearRegressionLines.add(afterLRLine.getP2());

        return linearRegressionLines;
    }

    /**
     * This get's the parts that are within a given range compared to the other headingDistance
     * Note: BOA = Before or After
     *
     * @param subBOATurnParts, list of subtrajectories points that are part of before or after the turning part.
     * @param headingDistance, the heading direction of the other side of the turn
     * @return returns only the parts that were not filtered out.
     */
    private List<List<Point2D>> getBestFittingPartForGivenAngle(List<List<Point2D>> subBOATurnParts,
                                                                double headingDistance) {
        int nos = subBOATurnParts.size();
        double filterPercentage = 0.6;

        int partsThatCanStay = Math.max(3, (int) Math.floor(nos * filterPercentage));
        int partsLeftToBeRemoved = nos - partsThatCanStay;

        List<List<Point2D>> filteredSUBBoaTurnParts = new ArrayList<>();

        filteredSUBBoaTurnParts.addAll(subBOATurnParts);
        while (partsLeftToBeRemoved > 0) {
            Map<String, Object> dict = new HashMap<>();
            Line2D lrLine = LinearRegression.applyLinearRegressionNextToTurn(
                    filteredSUBBoaTurnParts, dict, averagePoint, 1000);

            Map<Integer, Double> distanceAverages = new HashMap<>();
            double overallAverage = 0.0;
            for (int i = 0; i < filteredSUBBoaTurnParts.size(); i++) {
                double averageDistance = 0.0;
                List<Point2D> BOATurningPart = filteredSUBBoaTurnParts.get(i);
                for (int j = 0; j < BOATurningPart.size(); j++) {
                    Point2D point = BOATurningPart.get(j);
                    averageDistance = getNewAverage(j, averageDistance, lrLine.ptSegDist(point));
                }
                distanceAverages.put(i, averageDistance);
                overallAverage = getNewAverage(i, overallAverage, averageDistance);
            }

            // Sorting the map based on value
            Map<Integer, Double> sortedMap = distanceAverages.entrySet().stream()
                    .sorted(Map.Entry.comparingByValue())
                    .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
            ArrayList<Integer> keys = new ArrayList<Integer>(sortedMap.keySet());

            int highestValueIndex = keys.get(keys.size() - 1);
            if (distanceAverages.get(highestValueIndex) > overallAverage * 2) {
                filteredSUBBoaTurnParts.remove(highestValueIndex);
            } else {
                break;
            }

            partsLeftToBeRemoved--;
        }

        return filteredSUBBoaTurnParts;
    }

    /**
     * We enlarge the part before the turn.
     * <p>
     * We wish to make it just over 50 meters long for every subtrajectory.
     * If we increase it to much we might include another turn but if we make it to small, the linear regression line
     * could be perpendicular on what it is now.
     * We only include parts that are within a constant angle of the calculated averageFirstAngle.
     * (Note that if we don't include the current point we could still include the next).
     * Finally if there are no points added, we take some part of the actual turn, this is done for less frequently
     * updated datasets.
     * <p>
     * Note: We have to reverse the final list because we are adding points in the reversed way.
     */
    private void enlargePartBeforeTurn() {
        if (subBeforeTurnParts.size() > 0) {
            return;
        }
        for (int i = 0; i < trajectoriesRanges.size(); i++) {
            Range<Integer> range = trajectoriesRanges.get(i);
            Subtrajectory sub = trajectoriesIncluded.get(i);

            Point2D firstPointOfTurn = sub.getPoint(range.lowerEndpoint());
            int j = range.lowerEndpoint() - 1;

            List<Point2D> addedPoints = new ArrayList<>();
            addedPoints.add(firstPointOfTurn);
            while (j >= 0) {
                Line2D edge = new Line2D.Double(sub.getPoint(j), addedPoints.get(addedPoints.size() - 1));
//                Line2D edge = new Line2D.Double(sub.getPoint(j), sub.getPoint(j + 1));
                double headingDirection = GeometryUtil.getHeadingDirection(edge);

                if (sub.getPoint(j).distance(firstPointOfTurn) > ALGOCONSTANTS.getTurnLengtenMaxDistance()) {
                    break;
                }
                if (GeometryUtil.getAbsoluteAngleDifference(headingDirection, averageFirstAngle) <
                        ALGOCONSTANTS.getTurnLengtenMaxAngle()) {
                    addedPoints.add(sub.getPoint(j));
                }

                j--;
            }
            // We have to reverse the list as we want the point in the same direction as the trajectories.
            Collections.reverse(addedPoints);

            if (addedPoints.size() == 1) {
                addedPoints.add(sub.getPoint(range.lowerEndpoint() + 1));
            }

            subBeforeTurnParts.add(addedPoints);
        }
    }

    /**
     * Exactly the same as enlargePartBeforeTurn but not after the turn.
     */
    private void enlargePartAfterTurn() {
        if (subAfterTurnParts.size() > 0) {
            return;
        }
        for (int i = 0; i < trajectoriesRanges.size(); i++) {
            Range<Integer> range = trajectoriesRanges.get(i);
            Subtrajectory sub = trajectoriesIncluded.get(i);

            Point2D lastPointOfTurn = sub.getPoint(range.upperEndpoint());
            int j = range.upperEndpoint() + 1;

            List<Point2D> addedPoints = new ArrayList<>();
            addedPoints.add(lastPointOfTurn);
            while (j < sub.numPoints()) {
                Line2D edge = new Line2D.Double(addedPoints.get(addedPoints.size() - 1), sub.getPoint(j));
//                Line2D edge = new Line2D.Double(sub.getPoint(j - 1), sub.getPoint(j));
                double headingDirection = GeometryUtil.getHeadingDirection(edge);

                if (sub.getPoint(j).distance(lastPointOfTurn) > ALGOCONSTANTS.getTurnLengtenMaxDistance()) {
                    break;
                }
                if (GeometryUtil.getAbsoluteAngleDifference(headingDirection, averageLastAngle) <
                        ALGOCONSTANTS.getTurnLengtenMaxAngle()) {
                    addedPoints.add(sub.getPoint(j));
                }

                j++;
            }

            if (addedPoints.size() == 1) {
                addedPoints.add(0, sub.getPoint(range.upperEndpoint() - 1));
            }

            subAfterTurnParts.add(addedPoints);
        }
    }

    public List<List<Point2D>> getSubtrajectoryTurnParts() {
        return subtrajectoryTurnParts;
    }

    public Point2D getAveragePoint() {
        return averagePoint;
    }

    public List<Point2D> getLargestAnglePoints() {
        return largestAnglePoints;
    }

    public double getAverageFirstAngle() {
        return averageFirstAngle;
    }

    public double getAverageLastAngle() {
        return averageLastAngle;
    }

    public List<List<Point2D>> getSubBeforeTurnParts() {
        enlargePartBeforeTurn();
        return subBeforeTurnParts;
    }

    public List<List<Point2D>> getSubAfterTurnParts() {
        enlargePartAfterTurn();
        return subAfterTurnParts;
    }

    /**
     * Get's the angle of the leaving linear regression line.
     *
     * @return
     */
    public double getTurnIncomingAngle() {
        if (linearRegressionLines.size() < 4) {
            getLinearRegressionLines();
        }

        return GeometryUtil.getHeadingDirection(
                new Line2D.Double(linearRegressionLines.get(0), linearRegressionLines.get(1)));
    }

    /**
     * Get's the angle of the outgoing linear regression line.
     *
     * @return
     */
    public double getTurnOutgoingAngle() {
        if (linearRegressionLines.size() < 4) {
            getLinearRegressionLines();
        }

        return GeometryUtil.getHeadingDirection(
                new Line2D.Double(linearRegressionLines.get(2), linearRegressionLines.get(3)));
    }

    /**
     * Get's the bundle this turn is a part of.
     */
    public Bundle getBundle() {
        return this.b;
    }
}
