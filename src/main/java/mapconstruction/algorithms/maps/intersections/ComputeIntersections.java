package mapconstruction.algorithms.maps.intersections;

import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.algorithms.maps.intersections.containers.IntersectionCluster;
import mapconstruction.trajectories.Bundle;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.util.*;

/**
 * This class is responsible for merging the different intersection objects into a single object,
 * if they represent the same intersection.
 *
 * @author Jorrick
 * @since 05/11/2018
 */
public class ComputeIntersections {

    /**
     * Get all intersections
     *
     * @return all intersections
     */
    public static List<Intersection> getIntersections(){
        double mergeDistance = 25;

        List<Intersection> intersections = calculateIntersection(ComputeIntersectionClusters.getIntersectionClusters());
        intersections = mergeBasedOnFinalLocation(intersections, mergeDistance);

        return intersections;
    }

    /**
     * Calculates the merging of intersectionClusters into actual intersections.
     * @param clusters, all found intersectionClusters
     * @return all intersectionsClusters merged into intersections.
     */
    private static List<Intersection> calculateIntersection(List<IntersectionCluster> clusters){
        List<Intersection> allIntersections = new ArrayList<>();

        for (IntersectionCluster cluster1: clusters) {
            Set<Bundle> bundlesAtCluster1 = cluster1.getAllBundlesAroundThisCluster();

            Boolean merged = false;

            Set<Intersection> intersectionsCluster1HasToMergeWith = new HashSet<>();
            for (Intersection intersection : allIntersections){
                for (IntersectionCluster cluster2: intersection.getAllIntersectionClusters()){
                    if (cluster1.equals(cluster2)){
                        continue;
                    }

                    if (cluster1.getLocation().distance(cluster2.getLocation()) > 50){
                        continue;
                    }
                    Set<Bundle> bundlesAtCluster2 = cluster2.getAllBundlesAroundThisCluster();
                    Set<Bundle> bundlesInBothClusters = new HashSet<>(bundlesAtCluster2);
                    bundlesInBothClusters.retainAll(bundlesAtCluster1);

                    List<Bundle> filteredBundles = new ArrayList<>(bundlesInBothClusters);
                    IntersectionUtil.filterOutBundlesWithAToSmallRep(filteredBundles, cluster1.getLocation(), 15.0, true);
                    IntersectionUtil.filterOutBundlesWithAToSmallRep(filteredBundles, cluster2.getLocation(), 15.0, true);

                    bundlesInBothClusters = new HashSet<>(filteredBundles);

                    if (bundlesInBothClusters.size() == 0){
//                        System.out.println("The unexpected happened. There must be a 6-way intersection??");
                        continue;
                    }

                    Bundle largestBundle = IntersectionUtil.getLargestBundle(bundlesInBothClusters);

                    Point2D cluster1OnLargestBundle = GeometryUtil.getPointOnTrajectoryClosestToOtherPoint(
                            largestBundle.getRepresentative(), cluster1.getLocation());
                    Point2D cluster2OnLargestBundle = GeometryUtil.getPointOnTrajectoryClosestToOtherPoint(
                            largestBundle.getRepresentative(), cluster2.getLocation());

                    if (cluster1OnLargestBundle.distance(cluster2OnLargestBundle) < 15){
                        intersectionsCluster1HasToMergeWith.add(intersection);
                        break;
                    }
                }
            }

            // Check where we have to add our cluster1 into
            if (intersectionsCluster1HasToMergeWith.size() == 0){
                allIntersections.add(new Intersection(cluster1));
            } else if (intersectionsCluster1HasToMergeWith.size() == 1){
                intersectionsCluster1HasToMergeWith.forEach(v -> v.addNewIntersectionCluster(cluster1));
            } else {
                Iterator<Intersection> intersectionIterator = intersectionsCluster1HasToMergeWith.iterator();
                Intersection mainIntersection = intersectionIterator.next();
                mainIntersection.addNewIntersectionCluster(cluster1);

                while (intersectionIterator.hasNext()) {
                    Intersection intersection = intersectionIterator.next();
                    mainIntersection.mergeWithAnotherIntersection(intersection, null);
                    allIntersections.remove(intersection);
                }
            }
        }
        return allIntersections;
    }

    private static List<Intersection> mergeBasedOnFinalLocation(List<Intersection> intersections, double mergeDistance){
        List<Intersection> intersectionList = new ArrayList<>(intersections);

        for (int i = 0; i < intersectionList.size(); i++){
            Intersection int1 = intersectionList.get(i);

            for (int j = i + 1; j < intersectionList.size(); j++){
                Intersection int2 = intersectionList.get(j);

                if (int1.getLocation().distance(int2.getLocation()) < mergeDistance){
                    int1.mergeWithAnotherIntersection(int2, int2.getLocation());
                    intersectionList.remove(j);
                    j--;
                }
            }
        }

        return intersectionList;

    }

}