package mapconstruction.algorithms.maps.intersections;

import com.google.common.collect.Range;
import com.google.common.math.DoubleMath;
import mapconstruction.attributes.BundleClassAttributes;
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
 * This class is responsible for small function used by both intersection computations.
 *
 * @author Jorrick
 * @since 02/11/2018
 */

public class IntersectionUtil {
    /**
     * For a list of bundles, we check that the rep of the bundle can run 50 offset meters to either side
     * starting from the point on rep closest to location. If it can not, we remove the bundle from the list.
     *
     * @param bundles,  all the bundles
     * @param location, the location where we compare the rep with
     * @param offset,   the distance we should be able to run
     * @param bothWays, whether we want to check in both ways from the location or just the offset way.
     */
    public static void filterOutBundlesWithAToSmallRep(List<Bundle> bundles, Point2D location, double offset,
                                                       boolean bothWays) {
        for (int i = 0; i < bundles.size(); i++) {
            Bundle b = bundles.get(i);
            Representative rep = b.getRepresentative();

            double indexOfLoc = GeometryUtil.getIndexOfTrajectoryClosestToPoint(rep, location);

            if ((bothWays && !doesTrajectoryHeadOnForXMeters(rep, indexOfLoc, -offset)) ||
                    !doesTrajectoryHeadOnForXMeters(rep, indexOfLoc, offset)) {
                bundles.remove(i);
                i--;
            }
        }
    }

    /**
     * Check if the trajectory goes on for at least distance meters.
     *
     * @param t,        the trajectory
     * @param index,    index of the trajectory
     * @param distance, the distance we should be able to cover from the index onwards
     * @return whether the trajectory is at the start or end when we covered (part of) the distance.
     */
    public static boolean doesTrajectoryHeadOnForXMeters(Trajectory t, double index, double distance) {
        double newIndex = GeometryUtil.getTrajectoryIndexAfterOffset(t, index, distance);

        if (DoubleMath.fuzzyEquals(newIndex, 0, 1E-6) ||
                DoubleMath.fuzzyEquals(newIndex, t.numPoints() - 1, 1E-6)) {
            return false;
        }
        return true;
    }

    /**
     * Get's the score for a bundle. Higher the better.
     *
     * @param b, bundle we want the score for
     * @return the score
     */
    public static double getBundleScore(Bundle b) {
        int bundleClass = STORAGE.getClassFromBundle(b);
        if (bundleClass != -1) {
            double bestEps = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);
            double birth = BundleClassAttributes.get("Birth")
                    .applyAsDouble(STORAGE.getEvolutionDiagram(), bundleClass, bestEps);
            double size = b.size();
            return size / (birth + 5);
        } else {
            System.out.println("Error ComputeIntersectionsByRoadPoints.getBundleScore. Bundleclass == -1");
            return 0.0;
        }
    }

    /**
     * Given a point/location, this function get's a naive distance between two trajectories, given a point.
     *
     * @param t1,       the first trajectory
     * @param t2,       the second trajectory
     * @param location, the point
     * @param infIfEnd, return the maximum value of a double if one of the two trajectories is at it's end. (default: false)
     * @return the distance between the closest points to location for our two trajectories.
     */
    public static double getNaiveTrajectoriesDistance(Trajectory t1, Trajectory t2, Point2D location, boolean infIfEnd) {
        double indexT1 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(t1, location);
        double indexT2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(t2, location);
        if (indexT1 == 0 || indexT1 == t1.numPoints() - 1 || indexT2 == 0 || indexT2 == t1.numPoints()){
            return Double.MAX_VALUE;
        }
        Point2D pointT1 = GeometryUtil.getTrajectoryDecimalPoint(t1, indexT1);
        Point2D pointT2 = GeometryUtil.getTrajectoryDecimalPoint(t2, indexT2);
        return pointT1.distance(pointT2);
    }

    /**
     * This function get's a naive distance between two trajectories, given a location and an offset.
     *
     * @param t1,       the first trajectory
     * @param t2,       the second trajectory
     * @param location, the location we want the closest point to from the trajectories.
     * @param offset,   the offset we will apply from the given point.
     * @return the distance between the closest points to location for our two trajectories.
     */
    public static double getNaiveTrajectoriesDistanceWithT1Offset(Trajectory t1, Trajectory t2, Point2D location, double offset) {
        double indexT1 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(t1, location);
        double newIndexT1 = GeometryUtil.getTrajectoryIndexAfterOffset(t1, indexT1, offset);
        Point2D pointOnT1 = GeometryUtil.getTrajectoryDecimalPoint(t1, newIndexT1);

        double indexT2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(t2, location);
        double newIndexT2 = GeometryUtil.getTrajectoryIndexAfterOffset(t2, indexT2, offset);
        Point2D pointOnT2 = GeometryUtil.getTrajectoryDecimalPoint(t2, newIndexT2);

        return Math.max(
                getNaiveTrajectoriesDistance(t1, t2, pointOnT1, false),
                getNaiveTrajectoriesDistance(t1, t2, pointOnT2, false));
    }

    /**
     * Of all the bundles with the largest size, we take the one with the best score.
     *
     * @param bundles          the bundles where we should find our overlapping bundle in
     * @param mustHaveLargeRep
     * @return the best overlapping bundle.
     */
    static Bundle getOverlappingBundle(Set<Bundle> bundles, Point2D location, double offset, boolean mustHaveLargeRep) {
        List<Bundle> bundleList = new ArrayList<>(bundles);

        if (mustHaveLargeRep) {
            filterOutBundlesWithAToSmallRep(bundleList, location, offset, true);
        }

        return getLargestBundle(bundleList);
    }

    /**
     * Of all bundles, we get the bundles with the largest size,
     * and of those bundles we get the one with the best score.
     *
     * @param bundles, all the bundles we need to search in
     * @return the largest best-scoring bundle.
     */
    static Bundle getLargestBundle(Collection<Bundle> bundles) {
        int maxSize = 0;
        double maxScore = 0;
        Bundle bestBundle = null;

        for (Bundle b : bundles) {
            if (b.size() > maxSize) {
                maxSize = b.size();
                maxScore = getBundleScore(b);
                bestBundle = b;
            } else if (b.size() == maxSize) {
                if (getBundleScore(b) > maxScore) {
                    maxScore = getBundleScore(b);
                    bestBundle = b;
                }
            }
        }
        return bestBundle;
    }


    /**
     * Given a set of bundles, a current bundle, a list of bundlepair scores and a maximum found k,
     * we try to find all the possible bundle pairs that cover at least k unique trajectories
     * (not containing any double). Note: We only return the largest bundle pairs!!!
     * <p>
     * The representatives should at least grow 1.5 in distance and be more than 20 meters apart in total.
     *
     * @param bundlesBeforeOrAfterIntersection, bundles that are before or after our intersection
     * @param bundlePairScores,                 bundle pair scores (only of the largest pairs, smaller pairs are removed)!
     * @param maxKFound,                        the number of unique subtrajectories covered by the largest pair
     * @param b1,                               the bundle we are looking for a mate for
     * @param repPointBeforeIntersection,       point on the representative of b1.
     * @param repPointAfterIntersection,        point on the representative of b1.
     * @param possibleIntLoc,                   the possible intersection location.
     * @return
     */
    static int calculateBestPairScores(Set<Bundle> bundlesBeforeOrAfterIntersection,
                                       Map<Double, List<Bundle>> bundlePairScores, int maxKFound, Bundle b1,
                                       Point2D repPointBeforeIntersection, Point2D repPointAfterIntersection,
                                       Point2D possibleIntLoc) {

        // For the car dataset there is 50 meters between repPointBeforeIntersection - possibleIntLoc and
        // 50 emters between repPointAfterIntersection and possibleIntLoc.
        // Therefore,

        double dtc = 50; // Distance to cover
        double rangeLength = dtc * 3; // Length of the range (50 * 2 = 100 for car datasets)
        double halfRange = rangeLength / 2; // Offset we allow our representative in.
        double maxRange = dtc * 3;

        double b1Score = getBundleScore(b1);
        Set<Subtrajectory> b1Subs = b1.getNonReverseSubtrajectories();
        int noB1FoundTrajectories = b1Subs.size();


        /**
         * Getting the data to calculate the distances around, before and after the intersection
         */
        Representative b1Rep = b1.getRepresentative();

        for (Bundle b2 : bundlesBeforeOrAfterIntersection) {
            if (b2.equals(b1)) {
                continue;
            }

            /**
             * Getting the data to calculate the distances around, before and after the intersection
             */
            Representative b2Rep = b2.getRepresentative();
            Subtrajectory b2RepSub = null;

            Range<Double> blockingRange = GeometryUtil.
                    getIndexRangeAroundPointOnTrajectory(b2Rep, possibleIntLoc, halfRange, true);

            Range<Double> defaultRange = GeometryUtil.
                    getIndexRangeAroundPointOnTrajectory(b2Rep, possibleIntLoc, maxRange, true);
            Range<Double> beforeRange = Range.closed(0.0, blockingRange.lowerEndpoint());
            Range<Double> upperRange = Range.closed(blockingRange.upperEndpoint(), (double) (b2Rep.numPoints() - 1));

            ArrayList<Range<Double>> ranges = new ArrayList<>(Arrays.asList(beforeRange, defaultRange, upperRange));

            Set<Subtrajectory> b2Subs = null;

            for (Range<Double> range : ranges){
                if (DoubleMath.fuzzyEquals(range.lowerEndpoint(), range.upperEndpoint(), 1E-5)){
                    continue;
                }

                b2RepSub = new Subtrajectory(b2.getRepresentative(), range.lowerEndpoint(), range.upperEndpoint());

                double p0Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2RepSub, possibleIntLoc, true);
                double p1Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2RepSub, repPointBeforeIntersection, true);
                double p2Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2RepSub, repPointAfterIntersection, true);

                if (p0Distance > 500 || p1Distance > 500 || p2Distance > 500){
                    // In this case we are at the ending of a representative when checking the nearest location.
                    b2RepSub = null;
                    continue;
                }
                if ((p1Distance < p2Distance * 2.0 && p2Distance < p1Distance * 2.0) ||
                         (p1Distance < 20 && p2Distance < 20)) { // (p1Distance + p2Distance < 20) &&
                    // This means we are to close by,
                    b2RepSub = null;
                    continue;
                }

                // This check is to prevent an irregular updating trajectories from being detected as an intersection.
                Point2D checkForCutOffDistanceBefore1 = GeometryUtil
                        .getTrajectoryPointClosestToLocationAfterOffset(b2RepSub, possibleIntLoc, -100);
                Point2D checkForCutOffDistanceBefore2 = GeometryUtil
                        .getTrajectoryPointClosestToLocationAfterOffset(b1Rep, possibleIntLoc, -100);
                Point2D checkForCutOffDistanceAfter1 = GeometryUtil
                        .getTrajectoryPointClosestToLocationAfterOffset(b2RepSub, possibleIntLoc, 100);
                Point2D checkForCutOffDistanceAfter2 = GeometryUtil
                        .getTrajectoryPointClosestToLocationAfterOffset(b1Rep, possibleIntLoc, 100);
                double p11Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2RepSub, checkForCutOffDistanceBefore1, false);
                double p12Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2RepSub, checkForCutOffDistanceBefore2, false);
                double p21Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2Rep, checkForCutOffDistanceAfter1, false);
                double p22Distance = getNaiveTrajectoriesDistance(
                        b1Rep, b2Rep, checkForCutOffDistanceAfter2, false);
                if ((p1Distance > p2Distance * 2.0 && (p11Distance < 15 || p12Distance < 15)) ||
                        (p2Distance > p1Distance * 2.0 && (p21Distance < 15|| p22Distance < 15))) {
                    b2RepSub = null;
                    continue;
                }

                if ((p1Distance > 20 && p2Distance > 20) || (p0Distance > 20)) {
                    // we are not covering the same intersection it seems?
                    b2RepSub = null;
                    continue;
                }

                // In this case we actually have a bundle which makes a u turn and thereby introduces a 'split' effect
                // This is protection against these cases.
                if (checkForCutOffDistanceBefore1.distance(checkForCutOffDistanceAfter1) < 20 ||
                        checkForCutOffDistanceBefore2.distance(checkForCutOffDistanceAfter2) < 20){
                    b2RepSub = null;
                    continue;
                }

                double b1BD = GeometryUtil.getHeadingDirection(b1Rep, repPointBeforeIntersection);
                double b1AD = GeometryUtil.getHeadingDirection(b1Rep, repPointAfterIntersection);

                double b2BD = GeometryUtil.getHeadingDirection(b2RepSub, repPointBeforeIntersection);
                double b2AD = GeometryUtil.getHeadingDirection(b2RepSub, repPointAfterIntersection);

                if (GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(b1BD, b2BD) > 20 &&
                        GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(b1AD, b2AD) > 20) {
                    b2RepSub = null;
                    continue;
                }

                double iRepOnInt = GeometryUtil
                        .getIndexOfTrajectoryClosestToPoint(b2RepSub, possibleIntLoc);
                iRepOnInt = GeometryUtil.convertSubIndexToTrajectoryIndex(b2RepSub, iRepOnInt);

                double iRepAfterInt = GeometryUtil.getTrajectoryIndexAfterOffset(b2Rep, iRepOnInt, dtc * 2);
                double iRepBeforeInt = GeometryUtil.getTrajectoryIndexAfterOffset(b2Rep, iRepOnInt, -dtc * 2);
                Subtrajectory b2RepSubInt = new Subtrajectory(b2Rep, iRepBeforeInt, iRepAfterInt);


                b2Subs = b2.getNonReverseSubTrajectoriesForRange(b2RepSubInt);

                // We want to make sure it contains none of the found
                if (b2Subs.stream().anyMatch(v -> v.overlapsOneItemInList(b1Subs))) {
                    b2RepSub = null;
                    continue;
                }

                Set<Subtrajectory> b1SubsCopy = new HashSet<>();
                for (Subtrajectory sub1: b1Subs){
                    boolean notInB2 = true;
                    for (Subtrajectory sub2: b2Subs){
                        if (sub2.getParent().equals(sub1.getParent())){
                            notInB2 = false;
                            break;
                        }
                    }
                    if (notInB2){
                        b1SubsCopy.add(sub1);
                    }
                }

                Set<Subtrajectory> b2SubsCopy = new HashSet<>(b2Subs);

                boolean foundOverlapper = false;
                for (Bundle b3 : bundlesBeforeOrAfterIntersection) {
                    Set<Subtrajectory> b3subs = b3.getNonReverseSubtrajectories();
                    boolean b3ContainsB1Subs = b3subs.stream().anyMatch(v -> v.overlapsOneItemInList(b1SubsCopy));
                    boolean b3ContainsB2Subs = b3subs.stream().anyMatch(v -> v.completelyOvershadowsOneItemInList(b2SubsCopy));
                    if (b3ContainsB1Subs && b3ContainsB2Subs) {
                        foundOverlapper = true;
                        break;
                    }
                }

                if (foundOverlapper) {
                    b2RepSub = null;
                    continue;
                }

                // In case b2RepSub worked out, we break instead of continuing
                break;
            }

            if (b2RepSub == null){
                continue;
            }

            // We want to count how many trajectory c has of leftTrajectories,
            int noB2FoundTrajectories = b2Subs.size();
            double b2Score = getBundleScore(b2);

            if (maxKFound == noB1FoundTrajectories + noB2FoundTrajectories) {
                bundlePairScores.put(b1Score + b2Score, Arrays.asList(b1, b2));
            } else if (maxKFound < noB1FoundTrajectories + noB2FoundTrajectories) {
                maxKFound = noB1FoundTrajectories + noB2FoundTrajectories;

                bundlePairScores.clear();
                bundlePairScores.put(b1Score + b2Score, Arrays.asList(b1, b2));
            }
        }
        return maxKFound;
    }

    /**
     * Get's the bundles for all trajectories in trajectoryIndexes at a given offset from location.
     *
     * @param trajectoryIndexes,    contains all trajectories and their index at the turn.
     * @param offset,               offset we will be looking before and after a turn
     * @param intLocation,          the possible intersection location
     * @param repMustBeLarge,       whether the rep may be of any size or must be 50 meters on both sides.
     * @param onlyDisplayedBundles, whether we should only return displayed bundles
     * @return the bundles by turn
     */
    static Set<Bundle> getBundlesByTurnWithOffset(Map<Trajectory, Double> trajectoryIndexes,
                                                  double offset, Point2D intLocation, boolean repMustBeLarge,
                                                  boolean onlyDisplayedBundles) {
        Set<Bundle> bundles = new HashSet<>();
        for (Map.Entry<Trajectory, Double> trajectoryIndex : trajectoryIndexes.entrySet()) {
            Trajectory t = trajectoryIndex.getKey();
            double tIndex = trajectoryIndex.getValue();

            if (!doesTrajectoryHeadOnForXMeters(t, tIndex, offset)) {
                continue;
            }
            double foundIndex = GeometryUtil.getTrajectoryIndexAfterOffset(t, tIndex, offset);
            List<Bundle> foundBundles;
            if (onlyDisplayedBundles) {
                foundBundles = TBCombiner.getDisplayedBundlesForTrajectoryAndIndex(t, foundIndex);
            } else {
                foundBundles = TBCombiner.getUndisplayedBundlesForTrajectoryAndIndex(t, foundIndex);
            }

            if (repMustBeLarge) {
                filterOutBundlesWithAToSmallRep(foundBundles, intLocation, offset, true);
            } else {
                filterOutBundlesWithAToSmallRep(foundBundles, intLocation, offset, false);
            }

            bundles.addAll(foundBundles);
        }
        return bundles;
    }
}
