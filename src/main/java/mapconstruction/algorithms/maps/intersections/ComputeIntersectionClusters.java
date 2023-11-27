package mapconstruction.algorithms.maps.intersections;

import mapconstruction.algorithms.maps.intersections.containers.IntersectionCluster;
import mapconstruction.algorithms.maps.intersections.containers.IntersectionPoint;
import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class ComputeIntersectionClusters {


    /**
     * Merge similar intersections, such that each intersection is only represented by one object
     *
     * @param mergedInts,     the list where we add the final merged intersection objects to
     * @param roadPointsInts, the intersections found with the help of RoadPoints
     * @param turnInts,       the intersections found with the help of turns.
     */
    static void mergeSimilarPairs(List<IntersectionCluster> mergedInts,
                                  List<IntersectionPoint> roadPointsInts,
                                  List<IntersectionPoint> turnInts) {
        // Run some nice algorithm to merge similar ones
        List<IntersectionPoint> allIntersectionsPoints = new ArrayList<>();
        allIntersectionsPoints.addAll(roadPointsInts);
        allIntersectionsPoints.addAll(turnInts);

        for (IntersectionPoint intP1 : allIntersectionsPoints) {
            Set<IntersectionCluster> clustersToMergeWith = new HashSet<>();
            for (IntersectionCluster cluster : mergedInts) {
                for (IntersectionPoint intP2 : cluster.getAllIntersectionPoints()) {
                    if (intP2.getLocation().distance(intP1.getLocation()) > 50) {
                        continue;
                    }
                    if (areIntersectionPointsMergeCompatible(intP1, intP2)) {
                        clustersToMergeWith.add(cluster);
                        break;
                    }
                }

            }
            if (clustersToMergeWith.size() == 0) {
                mergedInts.add(new IntersectionCluster(intP1));
            } else if (clustersToMergeWith.size() == 1) {
                clustersToMergeWith.forEach(v -> v.addNewIntersectionPoint(intP1));
            } else {  // clustersToMergeWith > 1
                Iterator<IntersectionCluster> clusterIterator = clustersToMergeWith.iterator();
                IntersectionCluster mainCluster = clusterIterator.next();
                mainCluster.addNewIntersectionPoint(intP1);

                while (clusterIterator.hasNext()) {
                    IntersectionCluster cluster = clusterIterator.next();
                    mainCluster.mergeWithOtherCluster(cluster);
                    mergedInts.remove(cluster);
                }
            }
        }

    }

    private static boolean areIntersectionPointsMergeCompatible(IntersectionPoint intP1, IntersectionPoint intP2) {
        Set<Subtrajectory> P1B1subs = intP1.getLongBundle1().getNonReverseSubtrajectories();
        Set<Subtrajectory> P1B2subs = intP1.getLongBundle2().getNonReverseSubtrajectories();
        Set<Subtrajectory> P2B1subs = intP2.getLongBundle1().getNonReverseSubtrajectories();
        Set<Subtrajectory> P2B2subs = intP2.getLongBundle2().getNonReverseSubtrajectories();

        if (checkForOverlaps(P1B1subs, P1B2subs, P2B1subs, P2B2subs)) {
            return true;
        }

        if (checkForOverlaps(P1B1subs, P1B2subs, P2B2subs, P2B1subs)) {
            return true;
        }

//        Did not make a difference, so is turned off.
//        if (areTwoIPsPairsCloseToEachOther(intP1, intP2)){
//            return true;
//        }
        return false;
    }

    /**
     * Given a set of nonReverse subtrajecories, we check if the subtrajectories of IntP1 overlap with IntP2
     *
     * @param P1B1subs, first bundle of the par of IntP1
     * @param P1B2subs, second bundle of the par of IntP1
     * @param P2B1subs, first bundle of the par of IntP2
     * @param P2B2subs, second bundle of the par of IntP2
     * @return whether the two overlap
     */
    private static boolean checkForOverlaps(Set<Subtrajectory> P1B1subs, Set<Subtrajectory> P1B2subs,
                                            Set<Subtrajectory> P2B1subs, Set<Subtrajectory> P2B2subs) {
        return P1B1subs.stream().anyMatch(v -> v.overlapsOneItemInList(P2B1subs)) &&
                P1B2subs.stream().anyMatch(v -> v.overlapsOneItemInList(P2B2subs)) &&
                P1B1subs.stream().noneMatch(v -> v.overlapsOneItemInList(P2B2subs)) &&
                P1B2subs.stream().noneMatch(v -> v.overlapsOneItemInList(P2B1subs));

    }

    /**
     * We check for B1 whether the distance between B1 and B2 increases when we increase B1.
     *
     * @param intP1,             intersectionPoint we are checking for
     * @param checkForB2Instead, we check for B2 instead of B1
     * @return we return whether B1 is increasing
     */
    private static boolean isB1IncreasingFromPairedToUnpairPart(IntersectionPoint intP1, boolean checkForB2Instead) {
        Representative rB1 = intP1.getLongBundle1().getRepresentative();
        Representative rB2 = intP1.getLongBundle2().getRepresentative();
        if (checkForB2Instead) {
            rB1 = intP1.getLongBundle2().getRepresentative();
            rB2 = intP1.getLongBundle1().getRepresentative();
        }

        return IntersectionUtil.getNaiveTrajectoriesDistanceWithT1Offset(rB1, rB2, intP1.getLocation(), 50.0) >
                IntersectionUtil.getNaiveTrajectoriesDistanceWithT1Offset(rB1, rB2, intP1.getLocation(), -50.0);
    }

    private static boolean areTwoIPsPairsCloseToEachOther(IntersectionPoint intP1, IntersectionPoint intP2) {
        Representative P1B1rep = intP1.getLongBundle1().getRepresentative();
        Representative P1B2rep = intP1.getLongBundle2().getRepresentative();
        Representative P2B1rep = intP2.getLongBundle1().getRepresentative();
        Representative P2B2rep = intP2.getLongBundle2().getRepresentative();

        double offset = 50.0;
        if (areTwoRepresentativesCloseToEachOther(P1B1rep, P2B1rep, intP1.getLocation(), intP2.getLocation(), offset) &&
                areTwoRepresentativesCloseToEachOther(P1B2rep, P2B2rep, intP1.getLocation(), intP2.getLocation(), offset)){
            return true;
        }
        if (areTwoRepresentativesCloseToEachOther(P1B1rep, P2B2rep, intP1.getLocation(), intP2.getLocation(), offset) &&
                areTwoRepresentativesCloseToEachOther(P1B2rep, P2B1rep, intP1.getLocation(), intP2.getLocation(), offset)){
            return true;
        }

        return false;
    }

    /**
     * We check for a given set of representatives whether starting from an intersectionPoint,
     * they are close after an applied offset in both ways.
     */
    private static boolean areTwoRepresentativesCloseToEachOther(Representative r1, Representative r2,
                                                                 Point2D r1L, Point2D r2L, double offset) {
        // Fine enough distance for now. Could be set lower for Athens and chicago.
        double maxDistance = 15;
        offset = Math.abs(offset);
        double r1r2Positive = getNaiveTrajectoriesDistanceWithT1T2Offset(r1, r2, r1L, r2L, offset);
        double r1r2Negative = getNaiveTrajectoriesDistanceWithT1T2Offset(r1, r2, r1L, r2L, -offset);

        return (r1r2Positive < maxDistance && r1r2Negative < maxDistance);
    }

    /**
     * We check the closest representative whether starting from an intersectionPoint.
     */
    private static double getNaiveTrajectoriesDistanceWithT1T2Offset(Representative r1, Representative r2,
                                                                     Point2D r1l, Point2D r2l, double offset){
        double indexT1 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(r1, r1l);
        double newIndexT1 = GeometryUtil.getTrajectoryIndexAfterOffset(r1, indexT1, offset);
        Point2D pointOnT1 = GeometryUtil.getTrajectoryDecimalPoint(r1, newIndexT1);

        double indexT2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(r2, r2l);
        double newIndexT2 = GeometryUtil.getTrajectoryIndexAfterOffset(r2, indexT2, offset);
        Point2D pointOnT2 = GeometryUtil.getTrajectoryDecimalPoint(r2, newIndexT2);

        return pointOnT1.distance(pointOnT2);
    }

    /**
     * Get all merged intersectionsClusters.
     *
     * @return all merged intersectionsClusters.
     */
    public static List<IntersectionCluster> getIntersectionClusters() {
        List<IntersectionCluster> allIntersections = new ArrayList<>();

        List<IntersectionPoint> roadPointIntersections = ComputeIntersectionsByRoadPoints.computeIntersectionsByRoadPoints();
        List<IntersectionPoint> turnIntersections = new ArrayList<>();
        if (!STORAGE.getDatasetConfig().isWalkingDataset()) {
            turnIntersections = ComputeIntersectionsByTurns.computeIntersectionsByTurns();
        }

        mergeSimilarPairs(allIntersections, roadPointIntersections, turnIntersections);
        return allIntersections;
    }
}
