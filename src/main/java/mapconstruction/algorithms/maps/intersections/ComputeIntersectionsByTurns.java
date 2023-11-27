package mapconstruction.algorithms.maps.intersections;

import mapconstruction.algorithms.maps.intersections.containers.IntersectionPoint;
import mapconstruction.algorithms.maps.intersections.containers.IntersectionPointByTurn;
import mapconstruction.algorithms.representative.containers.Turn;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;
import static mapconstruction.algorithms.maps.intersections.TrajectoryBundleCombiner.TBCombiner;

/**
 * This class is responsible for finding intersections based on turns.
 * This will return multiple objects for the same intersection.
 *
 * PLEASE NOTE!
 * (Note that in the thesis writing, Road Points have been described as either turns or bundle endings, in the code
 * we consider a RoadPoint to be only a bundle ending.
 * Here we create IntersectionPoints, while in the thesis we write we remove RoadPoints we do not consider Intersections.
 * Hence after the computation IntersectionPoints are still called RoadPoints in the thesis writing.)
 *
 * @author Jorrick
 * @since 02/11/2018
 */
public class ComputeIntersectionsByTurns {

    /**
     * Computing the intersections based on Turns.
     *
     * @return the found intersections.
     */
    public static List<IntersectionPoint> computeIntersectionsByTurns() {
        Set<Bundle> shownBundles = STORAGE.getDisplayedBundles();
        Set<Bundle> unfilteredBundles = STORAGE.getAllUnfilteredBundles();

        List<List<Turn>> turnClusters = getTurnClusters(shownBundles);
        TBCombiner.addMultipleDisplayedBundles(shownBundles);
        TBCombiner.addMultipleUndisplayedBundles(unfilteredBundles);

        List<IntersectionPoint> intersections = new ArrayList<>();
        for (List<Turn> cluster : turnClusters) {
            convertClusterTurnIntoIntersection(cluster, intersections);
        }

        return intersections;
    }

    /**
     * This function maps turns together that have similar properties.
     *
     * @param allBundles, all the bundles where we get the turns from.
     * @return all the turns clustered into lists.
     */
    public static List<List<Turn>> getTurnClusters(Set<Bundle> allBundles) {
        HashMap<Turn, List<Turn>> distinguishedTurns = new HashMap<>();

        // Cluster settings. Turns have to remain in this bound for them to be able to be considered the same cluster.
        double maxTurnDistance = 20;
        double maxAngleDiff = 15;

        for (Bundle b : allBundles) {
            List<Turn> bTurns = b.getAllTurns();
            for (Turn t1 : bTurns) {
                boolean merged = false;
                for (Turn t2 : distinguishedTurns.keySet()) {
                    if (t1.getTurnLocation().distance(t2.getTurnLocation()) < maxTurnDistance &&
                            GeometryUtil.getAbsoluteAngleDifference(
                                    t1.getTurnIncomingAngle(), t2.getTurnIncomingAngle()) < maxAngleDiff &&
                            GeometryUtil.getAbsoluteAngleDifference(
                                    t1.getTurnOutgoingAngle(), t2.getTurnOutgoingAngle()) < maxAngleDiff) {
                        // We have a similar turn here.
                        distinguishedTurns.get(t2).add(t1);
                        merged = true;
                    }
                }
                if (!merged) {
                    List<Turn> justThisTurn = new ArrayList<>();
                    justThisTurn.add(t1);
                    distinguishedTurns.put(t1, justThisTurn);
                }
            }
        }

        return new ArrayList<>(distinguishedTurns.values());
    }

    /**
     * This function find out whether the cluster is truly an intersection.
     * If so, we create the object and add it to intersections.
     *
     * @param cluster       cluster of turns
     * @param intersections list of all intersections so far
     */
    private static void convertClusterTurnIntoIntersection(List<Turn> cluster, List<IntersectionPoint> intersections) {

        // Getting the trajectory indexes
        Map<Trajectory, Double> trajectoryIndexes = new HashMap<>();
        Turn firstTurn = cluster.get(0);
        Point2D turnLocation = cluster.get(0).getTurnLocation();
        for (Turn turn : cluster) {
            Map<Trajectory, Double> turnTrajectoryIndexes = turn.getBundle()
                    .getParentTrajectoryIndexes(turnLocation, false);
            trajectoryIndexes.putAll(turnTrajectoryIndexes);
        }

        // Getting the bundles 50 meters before and after the turn.
        // (Found by using trajectories of original turn bundles)
        Set<Bundle> filteredBundlesBothEnds = getBundlesBeforeAndAfterTurn(trajectoryIndexes,
                50.0, firstTurn, true);
        Set<Bundle> unfilteredBundlesBothEnds = getBundlesBeforeAndAfterTurn(trajectoryIndexes,
                50.0, firstTurn, false);


        // We not try to find the maximum overlapping bundle.
        Bundle overlappingBundle = IntersectionUtil.getOverlappingBundle(filteredBundlesBothEnds, firstTurn.getTurnLocation(), 50.0, true);
        if (overlappingBundle == null) {
            // This means there is no bundle detected which is impossible because we have our original bundle.
            return;
        }

        // Getting all the trajectories and their indexes by their bundles.
        // (Found by using the previously found bundles)
        HashMap<Trajectory, Double> newTrajectoryIndexes = getTrajectoryIndexesAtTurn(unfilteredBundlesBothEnds, firstTurn);

        // Getting the bundles 50 meters before and after the turn again.
        Set<Bundle> bundlesBeforeTurn = IntersectionUtil.getBundlesByTurnWithOffset(newTrajectoryIndexes, -50.0, firstTurn.getTurnLocation(), true, true);
        Set<Bundle> bundlesAfterTurn = IntersectionUtil.getBundlesByTurnWithOffset(newTrajectoryIndexes, 50.0, firstTurn.getTurnLocation(), true, true);

        Map<Double, List<Bundle>> bundlePairScores = getBestBundlePairs(cluster, bundlesBeforeTurn, bundlesAfterTurn);

        double bestBundlePairScore = bundlePairScores.keySet().stream().mapToDouble(v -> v).max().orElse(0.0);
        if (bestBundlePairScore == 0.0) {
            return;
        }

        List<Bundle> bestBundlesPair = bundlePairScores.get(bestBundlePairScore);
        Bundle bp1 = bestBundlesPair.get(0);
        Bundle bp2 = bestBundlesPair.get(1);

        IntersectionPointByTurn intersection = new IntersectionPointByTurn(bp1, bp2, overlappingBundle, cluster);
        intersection.setBundlesBeforeAndAfterTurn(bundlesBeforeTurn, bundlesAfterTurn);
        intersections.add(intersection);
    }

    /**
     * This get's the bundles before and after a turn.
     *
     * @param trajectoryIndexes, contains all trajectories and their index at the turn.
     * @param offset,            offset we will be looking before and after a turn
     * @param turn,              the main turn to get the location from.
     * @param filteredBundles,   whether to only return displayed bundles that are not filtered out by the predicate
     * @return set of found bundles.
     */
    private static Set<Bundle> getBundlesBeforeAndAfterTurn(Map<Trajectory, Double> trajectoryIndexes,
                                                            double offset, Turn turn, boolean filteredBundles) {
        Set<Bundle> bundlesBothEnds = new HashSet<>();
        offset = Math.abs(offset);
        Set<Bundle> bundlesBeforeTurn = IntersectionUtil.getBundlesByTurnWithOffset(trajectoryIndexes, -offset, turn.getTurnLocation(), false, filteredBundles);
        Set<Bundle> bundlesAfterTurn = IntersectionUtil.getBundlesByTurnWithOffset(trajectoryIndexes, offset, turn.getTurnLocation(), false, filteredBundles);

        // This is in the case that a bundle can not be part of both sides.. We don't want that at the first point.
        if (false) {
            bundlesBeforeTurn.removeAll(bundlesAfterTurn);
            bundlesAfterTurn.removeAll(bundlesBeforeTurn);
        }
        bundlesBothEnds.addAll(bundlesBeforeTurn);
        bundlesBothEnds.addAll(bundlesAfterTurn);
        return bundlesBothEnds;
    }

    /**
     * Get the trajectory indexes at the turn's location
     *
     * @param bundleList, the list of bundles.
     * @param turn,       the turn for it's location
     * @return all trajectories and their index which is close to the turn location.
     */
    private static HashMap<Trajectory, Double> getTrajectoryIndexesAtTurn(Set<Bundle> bundleList, Turn turn) {
        HashMap<Trajectory, Double> newTrajectoryIndexes = new HashMap<>();
        for (Bundle b : bundleList) {
            Set<Subtrajectory> subs = b.getSubtrajectories();
            for (Subtrajectory sub : subs) {
                if (newTrajectoryIndexes.containsKey(sub.getParent())) {
                    continue;
                }
                double subIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub, turn.getTurnLocation());
                Double currentTIndex = newTrajectoryIndexes.get(sub.getParent());

                if (currentTIndex != null &&
                        GeometryUtil.getTrajectoryDecimalPoint(sub, subIndex).distance(turn.getTurnLocation()) >
                                GeometryUtil.getTrajectoryDecimalPoint(sub.getParent(), currentTIndex).distance(turn.getTurnLocation())) {
                    continue;
                }
                double tIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(sub, subIndex);
                newTrajectoryIndexes.put(sub.getParent(), tIndex);
            }
        }
        return newTrajectoryIndexes;
    }

    /**
     * Get the best bundle pairs.
     *
     * @param cluster           a cluster of turns
     * @param bundlesBeforeTurn all bundles found before the turn
     * @param bundlesBeforeTurn all bundles found after the turn
     */
    private static Map<Double, List<Bundle>> getBestBundlePairs(List<Turn> cluster, Set<Bundle> bundlesBeforeTurn,
                                                                Set<Bundle> bundlesAfterTurn) {

        Map<Double, List<Bundle>> bundlePairScores = new HashMap<>();
        int maxKFound = 0;
        for (Turn t1 : cluster) {
            Bundle b1 = t1.getBundle();

            Representative b1Rep = b1.getRepresentative();
            double dtc = 50;
            double b1IndexAtTurn = GeometryUtil.getIndexOfTrajectoryClosestToPoint(b1Rep, cluster.get(0).getTurnLocation());
            double b1IndexBeforeTurn = GeometryUtil.getTrajectoryIndexAfterOffset(b1Rep, b1IndexAtTurn, -dtc);
            double b1IndexAfterTurn = GeometryUtil.getTrajectoryIndexAfterOffset(b1Rep, b1IndexAtTurn, dtc);

            if (!IntersectionUtil.doesTrajectoryHeadOnForXMeters(b1Rep, b1IndexAtTurn, -dtc) ||
                    !IntersectionUtil.doesTrajectoryHeadOnForXMeters(b1Rep, b1IndexAtTurn, dtc)) {
                continue;
            }

            Point2D repPointBeforeTurn = GeometryUtil.getTrajectoryDecimalPoint(b1Rep, b1IndexBeforeTurn);
            Point2D repPointAfterTurn = GeometryUtil.getTrajectoryDecimalPoint(b1Rep, b1IndexAfterTurn);

            // Get the best scores for the pairs.
            maxKFound = IntersectionUtil.calculateBestPairScores(bundlesBeforeTurn, bundlePairScores, maxKFound, b1,
                    repPointBeforeTurn, repPointAfterTurn, t1.getTurnLocation());
            maxKFound = IntersectionUtil.calculateBestPairScores(bundlesAfterTurn, bundlePairScores, maxKFound, b1,
                    repPointBeforeTurn, repPointAfterTurn, t1.getTurnLocation());

        }
        return bundlePairScores;
    }

}
