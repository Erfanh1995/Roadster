package mapconstruction.algorithms.maps.intersections;

import mapconstruction.algorithms.maps.intersections.containers.IntersectionPoint;
import mapconstruction.algorithms.maps.intersections.containers.IntersectionPointByRoadPoint;
import mapconstruction.algorithms.maps.intersections.containers.RoadPoint;
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
 * This class is responsible for finding intersections based on Road Points
 *
 * PLEASE NOTE!
 * (Note that in the thesis writing, Road Points have been described as either turns or bundle endings, in the code
 * we consider a RoadPoint to be only a bundle ending.
 * Here we create IntersectionPoints, while in the thesis we write we remove RoadPoints we do not consider Intersections.
 * Hence after the computation IntersectionPoints are still called RoadPoints in the thesis writing.)
 *
 * This will return multiple objects for the same intersection.
 *
 * @author Jorrick
 * @since 30/10/2018
 */
public class ComputeIntersectionsByRoadPoints {

    /**
     * Computing the intersections based on RoadPoints.
     *
     * @return the found intersections.
     */
    public static List<IntersectionPoint> computeIntersectionsByRoadPoints() {
        Set<Bundle> shownBundles = STORAGE.getDisplayedBundles();

        List<RoadPoint> roadPoints = computeRoadPoints(shownBundles);
        TrajectoryBundleCombiner trajectoryBundleCombiner = TBCombiner;
        trajectoryBundleCombiner.addMultipleDisplayedBundles(shownBundles);

        List<IntersectionPoint> intersections = convertRoadPointsIntoIntersections(roadPoints);
        return intersections;
    }


    public static List<RoadPoint> computeRoadPoints(Set<Bundle> bundles) {
        List<RoadPoint> roadPoints = new ArrayList<>();

        for (Bundle b : bundles) {
            int bundleRoadPoints = 0;
            if (!isBundleBeginDueToTrajectoryStart(b)) {
                roadPoints.add(new RoadPoint(b, true));
                bundleRoadPoints += 1;
            }
            if (!isBundleEndDueToTrajectoryEnd(b)) {
                roadPoints.add(new RoadPoint(b, false));
                bundleRoadPoints += 2;
            }
            b.setBundleEndsAreRoadPoints(bundleRoadPoints);
        }
        return roadPoints;
    }

    /**
     * Reports whether the bundle begins due to a trajectory starts there.
     *
     * @param b bundle we are looking at
     * @return true if it was due to a trajectory starting
     */
    private static boolean isBundleBeginDueToTrajectoryStart(Bundle b) {
        for (Subtrajectory sub : b.getSubtrajectories()) {
            if (sub.getFromIndex() == 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Reports whether the bundle ends due to a trajectory ending there.
     *
     * @param b bundle we are looking at
     * @return true if it was due to a trajectory ending
     */
    private static boolean isBundleEndDueToTrajectoryEnd(Bundle b) {
        for (Subtrajectory sub : b.getSubtrajectories()) {
            if (sub.getToIndex() == sub.getParent().numPoints() - 1) {
                return true;
            }
        }
        return false;
    }

    /**
     * Compute all the intersections
     *
     * @param roadPoints, all road points found on the displayed bundles
     * @return all found intersection points
     */
    private static List<IntersectionPoint> convertRoadPointsIntoIntersections(List<RoadPoint> roadPoints) {
        List<IntersectionPoint> intersections = new ArrayList<>();

        // Info about why some intersections were skipped
        Map<String, Integer> skipReasons = new HashMap<>();
        skipReasons.put("largestBundleSizeIs0", 0);
        skipReasons.put("bestBundlePairIsNull", 0);
        skipReasons.put("overlappingBundleContainsPair", 0);
        skipReasons.put("betterOverlappingScore", 0);

        for (RoadPoint roadPoint : roadPoints) {
            convertRoadPointIntoIntersection(roadPoint, intersections, skipReasons);
        }
        return intersections;
    }

    private static void convertRoadPointIntoIntersection(RoadPoint roadPoint, List<IntersectionPoint> intersections,
                                                         Map<String, Integer> skipReasons) {
        // We define the ending bundle.
        Bundle endingBundle = roadPoint.getBundle();
        int endingBundleClass = STORAGE.getClassFromBundle(endingBundle);
        double maxError = STORAGE.getEvolutionDiagram().getBestEpsilon(endingBundleClass);

        // Start out by getting all the trajectory point indexes to know where we should look for bundles
        Map<Trajectory, Double> trajectoryIndexes = roadPoint.getBundle().
                getParentTrajectoryIndexes(roadPoint.getLocation(), false);

        // Do the actual looking for bundles. This is done 50 meters from the closest point to the turning Point.
        // This 50 meters is in the direction of going outwards from the bundle.
        // The bundles we want have to be in both.
        Set<Bundle> bundlesOnRoadPoint = new HashSet<>();

        double distanceForwardLooking = 50.0;
        if (roadPoint.isBundleStart()) {
            distanceForwardLooking = distanceForwardLooking * -1;
        }

        // Find all bundles present in our intersections.
        Set<Bundle> bundlesInsideEnd = IntersectionUtil.getBundlesByTurnWithOffset(trajectoryIndexes, -distanceForwardLooking,
                roadPoint.getLocation(), true, true);
        Set<Bundle> bundlesOutsideEnd = IntersectionUtil.getBundlesByTurnWithOffset(trajectoryIndexes, distanceForwardLooking,
                roadPoint.getLocation(), true, true);

        bundlesOnRoadPoint.addAll(bundlesInsideEnd);
        bundlesOnRoadPoint.addAll(bundlesOutsideEnd);

        // We try to find an overlapping bundles
        Bundle overlappingBundle = IntersectionUtil.getOverlappingBundle(bundlesOnRoadPoint,
                roadPoint.getLocation(), 50.0, false);
        if (overlappingBundle == null) {
            skipReasons.put("largestBundleSizeIs0", skipReasons.get("largestBundleSizeIs0") + 1);
            return;
//            throw new IllegalArgumentException("Could not find an overlappingBundle.. Something weird happened");
        }

        // We try to find the best pair.
        Map<Double, List<Bundle>> bundlePairScores = getBestBundlePairs(roadPoint, bundlesOnRoadPoint);
        double bestBundlePairScore = bundlePairScores.keySet().stream().mapToDouble(v -> v).max().orElse(0.0);
        if (bestBundlePairScore == 0.0) {
            skipReasons.put("bestBundlePairIsNull", skipReasons.get("bestBundlePairIsNull") + 1);
            return;
        }

        List<Bundle> bestBundlesPair = bundlePairScores.get(bestBundlePairScore);
        Bundle bp1 = bestBundlesPair.get(0);
        Bundle bp2 = bestBundlesPair.get(1);

        // Checking if overlapping bundle contains trajectories from both pairs.
//        int bp1Found = (int) bp1.getNonReverseSubtrajectories().stream()
//                .filter(v -> v.overlapsOneItemInList(overlappingBundle.getNonReverseSubtrajectories())).count();
//        int bp2Found = (int) bp2.getNonReverseSubtrajectories().stream()
//                .filter(v -> v.overlapsOneItemInList(overlappingBundle.getNonReverseSubtrajectories())).count();

        // @Future improvement idea.
        // Now we simply check if there is an overlapping bundle that has trajectories from both of them in there.
        // This will prevent parallel ways from being detected. A shame!
        // However, the datasets provided do not contain examples of these.
        // There is simply to much GPS inaccuracy to find the difference between a parallel way and a
        // bad GPS reception area.

        // This is not possible because we already filter them out in the previous step.
//        if (bp1Found > 0 && bp2Found > 0) {
            //This means that our overlapping bundle is able to overlap both sides.
            //Now we should check whether the two pairs are
//            skipReasons.put("overlappingBundleContainsPair", skipReasons.get("overlappingBundleContainsPair") + 1);
//            return;
//        }

        if (IntersectionUtil.getBundleScore(overlappingBundle) > IntersectionUtil.getBundleScore(bp1) &&
                IntersectionUtil.getBundleScore(overlappingBundle) > IntersectionUtil.getBundleScore(bp2)) {
            // In this case, the overlappingBundle has quite a good score!
            // This means that the two bundles are so close to each other that there is no proof of two
            // concurrent streets. Hence we conclude no intersection.
            skipReasons.put("betterOverlappingScore", skipReasons.get("betterOverlappingScore") + 1);
            return;
        }

        IntersectionPoint intersection = new IntersectionPointByRoadPoint(bp1, bp2, overlappingBundle, endingBundle, roadPoint.getLocation());
        intersections.add(intersection);
    }

    /**
     * This get's the best bundle scores for a given Roadpoint and a set of bundles around this roadpoint.
     *
     * @param roadPoint           the roadpoint where we are looking for whether it's an intersection
     * @param bundlesOnRoadPoint, all the bundles on and around the roadpoint
     * @return scores of the largest bundle pairs
     */
    private static Map<Double, List<Bundle>> getBestBundlePairs(RoadPoint roadPoint, Set<Bundle> bundlesOnRoadPoint) {
        Map<Double, List<Bundle>> bundlePairScores = new HashMap<>();
        int maxKFound = 0;
        for (Bundle b1 : bundlesOnRoadPoint) {

            Representative b1Rep = b1.getRepresentative();
            double dtc = 50;  // Distance to cover
            double b1IndexAtRP = GeometryUtil.getIndexOfTrajectoryClosestToPoint(b1Rep, roadPoint.getLocation());
            double b1IndexBeforeRP = GeometryUtil.getTrajectoryIndexAfterOffset(b1Rep, b1IndexAtRP, -dtc);
            double b1IndexAfterRP = GeometryUtil.getTrajectoryIndexAfterOffset(b1Rep, b1IndexAtRP, dtc);

            if (!IntersectionUtil.doesTrajectoryHeadOnForXMeters(b1Rep, b1IndexAtRP, -dtc) ||
                    !IntersectionUtil.doesTrajectoryHeadOnForXMeters(b1Rep, b1IndexAtRP, dtc)) {
                continue;
            }

            Point2D repPointBeforeRP = GeometryUtil.getTrajectoryDecimalPoint(b1Rep, b1IndexBeforeRP);
            Point2D repPointAfterRP = GeometryUtil.getTrajectoryDecimalPoint(b1Rep, b1IndexAfterRP);

            maxKFound = IntersectionUtil.calculateBestPairScores(bundlesOnRoadPoint, bundlePairScores, maxKFound, b1,
                    repPointBeforeRP, repPointAfterRP, roadPoint.getLocation());

        }
        return bundlePairScores;
    }
}
