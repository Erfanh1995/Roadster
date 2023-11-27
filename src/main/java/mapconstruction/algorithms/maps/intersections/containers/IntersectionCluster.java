package mapconstruction.algorithms.maps.intersections.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;

import static mapconstruction.algorithms.maps.intersections.TrajectoryBundleCombiner.TBCombiner;

/**
 * DataClass for a combination of IntersectionPoints.
 * In one cluster we can only have intersectionPoints that cover the same streets and parts with their pair.
 * (This is of course relative to the intersectionPoints location).
 *
 * This means that in the case that we are looking at an actual intersection, and there are two different
 * intersectionPoints that have different pair direction, then we never merge them into the same cluster.
 *
 * @author Jorrick
 * @since 06/11/2018
 */
public class IntersectionCluster implements Serializable {

    /**
     * Contains a list of all intersection points.
     */
    private List<IntersectionPoint> allIntersectionPoints;

    /**
     * The average point of this cluster
     */
    private Point2D averageLocation;

    /**
     * The number of points already used for the location
     */
    private int nopUsedAverage;

    /**
     * Bundles around the cluster
     */
    private Set<Bundle> allBundlesAroundThisCluster;

    /**
     * Initializes the cluster with a single IntersectionPoint.
     *
     * @param p the first IntersectionPoint
     */
    public IntersectionCluster(IntersectionPoint p) {
        allIntersectionPoints = new ArrayList<>();
        nopUsedAverage = 0;
        addNewIntersectionPoint(p);
        allBundlesAroundThisCluster = new HashSet<>();
    }


    /**
     * Adds a new intersection point to this cluster and calculates the new average location.
     *
     * @param p, the intersectionPoint
     */
    public void addNewIntersectionPoint(IntersectionPoint p) {
        allIntersectionPoints.add(p);

        int addedPoints = 0;
        Point2D addedAveragePoint;
        if (p instanceof IntersectionPointByTurn){
            addedPoints = ((IntersectionPointByTurn) p).getNumberOfTurnsIncluded();
            addedAveragePoint = p.getLocation();
        } else if (p instanceof IntersectionPointByRoadPoint){
            addedPoints = 1;
            addedAveragePoint = p.getLocation();
        } else {
            throw new InstantiationError("The IntersectionPoint is not of any type");
        }

        double averageX, averageY;
        int all = nopUsedAverage + addedPoints;
        if (nopUsedAverage == 0){
            averageX = addedAveragePoint.getX();
            averageY = addedAveragePoint.getY();
        } else {
            averageX = (averageLocation.getX() * nopUsedAverage / all + addedAveragePoint.getX() * addedPoints / all);
            averageY = (averageLocation.getY() * nopUsedAverage / all + addedAveragePoint.getY() * addedPoints / all);
        }

        nopUsedAverage = all;
        averageLocation = new Point2D.Double(averageX, averageY);
    }


    /**
     * Get's all the intersectionPoints present in this cluster
     *
     * @return the list
     */
    @JsonIgnore
    public List<IntersectionPoint> getAllIntersectionPoints() {
        return allIntersectionPoints;
    }

    /**
     * Get's the location
     */
    @JsonProperty
    public Point2D getLocation() {
        return averageLocation;
    }

    /**
     * We merge another cluster into this object.
     * @param intersectionCluster the other cluster we want to merge into this object.
     */
    public void mergeWithOtherCluster(IntersectionCluster intersectionCluster){
        for (IntersectionPoint intP : intersectionCluster.getAllIntersectionPoints()){
            addNewIntersectionPoint(intP);
        }
    }

    /**
     * Set's all the bundles around this cluster.
     * @return all the bundles around our this cluster
     */
    @JsonIgnore
    public Set<Bundle> getAllBundlesAroundThisCluster(){
        if (allBundlesAroundThisCluster.size() == 0){
            calculateAllBundlesAroundACluster();
        }
        return new HashSet<>(allBundlesAroundThisCluster);
    }

    /**
     * Calculates and sets all bundles around this cluster.
     */
    private void calculateAllBundlesAroundACluster(){
        Set<Bundle> allFoundBundles = new HashSet<>();


        for (IntersectionPoint ip : getAllIntersectionPoints()){
            Set<Bundle> clusterBundles = ip.getAllBundlesConnectedToIntersectionPoint();

            // First we get the first TrajectoryIndexes
            Map<Trajectory, Double> trajectoryIndexes = new HashMap<>();
            for (Bundle b: clusterBundles){
                Map<Trajectory, Double> bundleTrajectoryIndexes = b.getParentTrajectoryIndexes(getLocation(), true);
                for (Map.Entry<Trajectory, Double> trajectoryDoubleEntry : bundleTrajectoryIndexes.entrySet()){
                    trajectoryIndexes.put(trajectoryDoubleEntry.getKey(), trajectoryDoubleEntry.getValue());
                }
            }

            // Now we get all the bundles that can be found, even the unfiltered / undisplayed once.
            Set<Bundle> unfilteredBundles = new HashSet<>();
            for (Map.Entry<Trajectory, Double> trajectoryDoubleEntry : trajectoryIndexes.entrySet()){
                List<Bundle> unfilteredBundlesForEntry = TBCombiner.getUndisplayedRunningBundlesForTrajectoryAndIndex(
                        trajectoryDoubleEntry.getKey(), trajectoryDoubleEntry.getValue(), getLocation());
                unfilteredBundles.addAll(unfilteredBundlesForEntry);
            }

            // Now we want to convert this back to trajectory indexes again.
            trajectoryIndexes = new HashMap<>();
            for (Bundle b: unfilteredBundles){
                Map<Trajectory, Double> bundleTrajectoryIndexes = b.getParentTrajectoryIndexes(getLocation(), true);
                for (Map.Entry<Trajectory, Double> trajectoryDoubleEntry : bundleTrajectoryIndexes.entrySet()){
                    trajectoryIndexes.put(trajectoryDoubleEntry.getKey(), trajectoryDoubleEntry.getValue());
                }
            }

            // Here we filter out trajectory indexes that are two far of our intersectionPoint.
            HashMap<Trajectory, Double> trajectoryIndexesCopy = new HashMap<>(trajectoryIndexes);
            for (Map.Entry<Trajectory, Double> tde: trajectoryIndexesCopy.entrySet()){
                if (GeometryUtil.getTrajectoryDecimalPoint(tde.getKey(), tde.getValue()).distance(getLocation()) > 15){
                    trajectoryIndexes.remove(tde.getKey());
                }
            }
            trajectoryIndexesCopy.clear();

            // Now we want to convert the trajectory indexes back to (filtered) bundles.
            Set<Bundle> finalBundles = new HashSet<>();
            for (Map.Entry<Trajectory, Double> trajectoryDoubleEntry : trajectoryIndexes.entrySet()){
                List<Bundle> unfilteredBundlesForEntry = TBCombiner.getDisplayedRunningBundlesForTrajectoryAndIndex(
                        trajectoryDoubleEntry.getKey(), trajectoryDoubleEntry.getValue(), getLocation());
                finalBundles.addAll(unfilteredBundlesForEntry);
            }

            allFoundBundles.add(ip.getLongBundle1());
            allFoundBundles.add(ip.getLongBundle2());
            allFoundBundles.addAll(finalBundles);
        }
        allBundlesAroundThisCluster.addAll(allFoundBundles);
    }
}
