package mapconstruction.algorithms.representative;

import com.google.common.collect.Range;
import mapconstruction.algorithms.representative.containers.Turn;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;

import static java.util.Map.Entry.comparingByValue;
import static java.util.stream.Collectors.toList;
import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;

/**
 * Class used for calculating/deciding where the sharp turns are.
 * <p>
 * This is done by using linear regression on the part before and after the turn.
 *
 * @author Jorrick Sleijster
 */
public class TurnDecider {
    public static List<Point2D> getAllSharpTurns(Set<Subtrajectory> trajectories) {
        List<Point2D> sharpTurns = new ArrayList<>();

        for (Subtrajectory sub : trajectories) {
            for (int i = 1; i < sub.numEdges(); i++) {
                Line2D firstEdge = sub.getEdge(i - 1);
                Line2D secondEdge = sub.getEdge(i);

                if (GeometryUtil.getHeadingDirectionDifference(firstEdge, secondEdge) >=
                        ALGOCONSTANTS.getTurnMinAngle()) {
                    if (!firstEdge.getP2().equals(secondEdge.getP1())) {
                        System.out.println("Error in TurnFixer.getAllSharpTurns. Points doesn't make sense..");
                    }
                    sharpTurns.add(firstEdge.getP2());
                }
            }
        }
        return sharpTurns;
    }

    /**
     * Get all angulated parts which have at least two edges and an angle larger than a constantly defined angle.
     * <p>
     * We loop over each subtrajectory and check for each point 50 meters ahead. If we find a large enough difference
     * between the heading direction of the first edge compared to the last edge, we say it is an angulated part.
     * Later on we merge angulated parts of the subtrajectory together.
     *
     * @param trajectories All subtrajectories.
     * @return A list of all angulated parts
     */
    public static List<List<Map<String, Object>>> getMultiEdgesSharpTurnParts(Set<Subtrajectory> trajectories) {
        List<List<Map<String, Object>>> sharpTurns = new ArrayList<>();

        for (Subtrajectory sub : trajectories) {
            List<Map<String, Object>> sharpTurnsSub = new LinkedList<>();

            for (int i = 0; i < sub.numEdges() - 1; i++) {
                Line2D firstEdge = sub.getEdge(i);
                List<Point2D> points = new ArrayList<>();
                points.add(firstEdge.getP1());
                points.add(firstEdge.getP2());

                Map<String, Object> turnInfo = new HashMap<>();

                for (int j = i + 1; j < sub.numEdges(); j++) {
                    Line2D secondEdge = sub.getEdge(j);
                    points.add(secondEdge.getP2());
                    if (GeometryUtil.getHeadingDirectionDifference(firstEdge, secondEdge) >=
                            ALGOCONSTANTS.getTurnMinAngle()) {
                        // We have a successful turn.
                        turnInfo.put("subtrajectory", sub);
                        turnInfo.put("startEdgeIndex", i);
                        turnInfo.put("endEdgeIndex", j);
                        turnInfo.put("angle", GeometryUtil.getHeadingDirectionDifference(firstEdge, secondEdge));
                        turnInfo.put("points", points);
                    }

                    // @ToDo this should be continuous distance instead of flying distance.
                    if (firstEdge.getP1().distance(secondEdge.getP2()) > 50) {
                        break;
                    }
//                    stepsLeft--;
                }

                if (turnInfo.containsKey("startEdgeIndex")) {
                    sharpTurnsSub.add(turnInfo);
                }
            }

            mergeSubTurnParts(sharpTurnsSub);
            sharpTurns.add(sharpTurnsSub);
        }
        return sharpTurns;

    }

    /**
     * Merge angulated parts that have more than edge in common.
     *
     * @param sharpTurnPartsSubs all angulated parts of a specific subtrajectory.
     * @modifies sharpTurnPartsSubs
     */
    private static void mergeSubTurnParts(List<Map<String, Object>> sharpTurnPartsSubs) {
        for (int i = 0; i < sharpTurnPartsSubs.size() - 1; i++) {
            Map<String, Object> firstTurn = sharpTurnPartsSubs.get(i);

            for (int j = i + 1; j < sharpTurnPartsSubs.size(); j++) {
                Map<String, Object> secondTurn = sharpTurnPartsSubs.get(j);

                int firstStartIndex = (int) firstTurn.get("startEdgeIndex");
                int firstEndIndex = (int) firstTurn.get("endEdgeIndex");
                int secondStartIndex = (int) secondTurn.get("startEdgeIndex");
                int secondEndIndex = (int) secondTurn.get("endEdgeIndex");

                Subtrajectory sub = (Subtrajectory) firstTurn.get("subtrajectory");

                // firstStartIndex == secondStartIndex - n
                // This holds for some n >= 1
                if (firstEndIndex > secondStartIndex) {
                    // We have more than just one edge overlap.
                    // Hence we merge the two into one.

                    firstTurn.put("startEdgeIndex", firstStartIndex);
                    firstTurn.put("endEdgeIndex", secondEndIndex);
                    List<Point2D> points = new ArrayList<>();

                    for (int k = firstStartIndex; k <= secondEndIndex + 1; k++) {
                        points.add(sub.getPoint(k));
                    }

                    firstTurn.put("angle", GeometryUtil.getHeadingDirectionDifference(
                            new Line2D.Double(points.get(0), points.get(1)),
                            new Line2D.Double(points.get(points.size() - 2), points.get(points.size() - 1))
                    ));
                    firstTurn.put("points", points);

                    sharpTurnPartsSubs.remove(j);
                    j--;
                }
            }
        }
    }

    /**
     * Given all the detected angles for one subtrajectory, we classify the once close to each other as turns.
     * <p>
     * This function loops over every subtrajectory for every angulated part. If it fits a turn in the proximity, we add
     * this angulated part of the subtrajectory to the best fitting turn. If there was no turn in the proximity that
     * fits, we create a new turn.
     * Fits: having at most a constantly defined difference on the ending edges heading angle difference and at most a
     * constantly defined distance to the average sharpest angle point.
     *
     * @return all turns
     */
    public static List<Turn> clusterIntoTurns(List<List<Map<String, Object>>> sharpTurnsPerSubtrajectory, Bundle b) {
        List<Turn> allTurns = new ArrayList<>();

        for (List<Map<String, Object>> subSharpTurns : sharpTurnsPerSubtrajectory) {

            Subtrajectory sub = null;

            for (Map<String, Object> sharpTurn : subSharpTurns) {
                if (sub == null) {
                    sub = (Subtrajectory) sharpTurn.get("subtrajectory");
                }

                // Calculate points for this sharpTurn compared to allTurns.
                // If points > threshold for all turns in allTurns, we create a new turn.
                List<Point2D> allPoints = (List<Point2D>) sharpTurn.get("points");
                int nop = allPoints.size();

                double maxAngle = 0.0;
                Point2D maxAnglePoint = allPoints.get(0);
                for (int i = 0; i < allPoints.size() - 2; i++) {
                    Line2D edge1 = new Line2D.Double(allPoints.get(i), allPoints.get(i + 1));
                    Line2D edge2 = new Line2D.Double(allPoints.get(i + 1), allPoints.get(i + 2));
                    double calculatedAngle = GeometryUtil.getHeadingDirectionDifference(edge1, edge2);
                    if (maxAngle < calculatedAngle) {
                        maxAngle = calculatedAngle;
                        maxAnglePoint = edge1.getP2();
                    }
                    maxAngle = Math.max(maxAngle, GeometryUtil.getHeadingDirectionDifference(edge1, edge2));
                }

                Line2D firstEdge = new Line2D.Double(allPoints.get(0), allPoints.get(1));
                Line2D lastEdge = new Line2D.Double(allPoints.get(nop - 2), allPoints.get(nop - 1));

//                boolean addedToTurn = false;
//                for (Turn turn : allTurns) {
//                    if (turn.getAveragePoint().distance(maxAnglePoint) < ALGOCONSTANTS.getTurnMaxDistanceSharpestAngle() &&
//                            turn.getDifferenceAngle(firstEdge, lastEdge) < ALGOCONSTANTS.getTurnMaxDifferenceInEndHeadingAngles() &&
//                            turn.isAbsentInTurn(sub)) {
//                        turn.addSubtrajectoryTurn(allPoints, maxAnglePoint, sub);
//                        addedToTurn = true;
//                    }
//                }
//
//                if (!addedToTurn) {
//                    Turn turn = new Turn(allPoints, maxAnglePoint, sub);
//                    allTurns.add(turn);
//                }


                int startPointIndex = (int) sharpTurn.get("startEdgeIndex");
                int endPointIndex = ((int) sharpTurn.get("endEdgeIndex")) + 1;
                Range<Integer> thisRange = Range.closed(startPointIndex, endPointIndex);

                Map<Integer, Double> turnDistance = new HashMap<>();
                Map<Integer, Double> turnAngleDiff = new HashMap<>();
                Map<Integer, Double> turnPointsSystem = new HashMap<>();

                double maxDistance = 0.0;
                double maxAngleDiff = 0.0;

                for (int i = 0; i < allTurns.size(); i++) {
                    Turn turn = allTurns.get(i);

                    double distance = turn.getAveragePoint().distance(maxAnglePoint);
                    double angleDiff = turn.getDifferenceAngle(firstEdge, lastEdge);
                    if (distance < ALGOCONSTANTS.getTurnMaxDistanceSharpestAngle() &&
                            angleDiff < ALGOCONSTANTS.getTurnMaxDifferenceInEndHeadingAngles()) {
                        turnDistance.put(i, distance);
                        turnAngleDiff.put(i, angleDiff);
                        turnPointsSystem.put(i, distance + ALGOCONSTANTS.getTurnScaleHeadingAngleVSDistance() * angleDiff);

                        maxDistance = Math.max(maxDistance, distance);
                        maxAngleDiff = Math.max(maxAngleDiff, angleDiff);
                    }
                }

                for (Integer i : turnDistance.keySet()) {
                    double distance = turnDistance.get(i);
                    double angleDiff = turnAngleDiff.get(i);

                    turnPointsSystem.put(i, distance / maxDistance
                            + ALGOCONSTANTS.getTurnScaleHeadingAngleVSDistance() * (angleDiff / maxAngleDiff));
                }

                if (turnPointsSystem.size() > 0) {
                    List<Map.Entry<Integer, Double>> turnPointsSystemSorted = sort(turnPointsSystem);
                    Map.Entry<Integer, Double> bestTurnPoints = turnPointsSystemSorted.get(0);
                    int bestTurnIndex = bestTurnPoints.getKey();
                    Turn bestTurn = allTurns.get(bestTurnIndex);
                    bestTurn.addSubtrajectoryTurn(allPoints, maxAnglePoint, sub, thisRange);
                } else {
                    Turn turn = new Turn(allPoints, maxAnglePoint, sub, thisRange, b);
                    allTurns.add(turn);
                }
            }
        }

        removeTurnsThatAreBadlyRepresented(allTurns, sharpTurnsPerSubtrajectory.size());
        return allTurns;
    }


    /**
     * We remove turns that are badly represented.
     * <p>
     * All turns that contain less than a constant defined factor or the total number of subtrajectories or less than
     * a specified absolute value, get removed.
     *
     * @param allTurns list of all the turns
     * @param noSubs   number of subtrajectories
     */
    private static void removeTurnsThatAreBadlyRepresented(List<Turn> allTurns, int noSubs) {
        for (int i = 0; i < allTurns.size(); i++) {
            Turn turn = allTurns.get(i);
            if (turn.numberOfTrajectoriesRepresented() < ALGOCONSTANTS.getTurnMinRepresentedFactor() * noSubs ||
                    turn.numberOfTrajectoriesRepresented() < ALGOCONSTANTS.getTurnMinRepresentedAbsolute()) {
                allTurns.remove(i);
                i--;
            }
        }
    }

    private static <K, V extends Comparable<? super V>> List<Map.Entry<K, V>> sort(Map<K, V> map) {
        return map.entrySet().stream().sorted(comparingByValue()).collect(toList());
    }
}
