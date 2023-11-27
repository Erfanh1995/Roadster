package mapconstruction.algorithms.maps.intersections.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.algorithms.maps.containers.IntersectionBundleCombo;
import mapconstruction.algorithms.maps.intersections.ComputeIntersectionLocation;
import mapconstruction.algorithms.maps.intersections.IntersectionUtil;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class Intersection implements Serializable {

    /**
     * The exact location of the intersection.
     */
    private Point2D intersectionLocation;

    /**
     * The score of the calculated location, defines how certain we are.
     */
    private double intersectionLocationScore;

    /**
     * All intersection clusters that were merged.
     */
    private Set<IntersectionCluster> allIntersectionClusters;

    /**
     * All bundles crossing(overlapping) the intersection, found by using pairs.
     */
    private Set<Bundle> allBundlesCrossingTheIntersection;

    /**
     * All parts of the bundle representatives before and after, found by using pairs.
     */
    private List<Subtrajectory> bundleRepsPartsBeforeAfterIntersection;

    /**
     * All bundles that will be connected to this intersection;
     */
    private Set<IntersectionBundleCombo> bundlesConnectedToIntersection;

    /**
     * All parts bundled together that are similar.
     */
    private List<List<Subtrajectory>> bundlePartsBundled;

    /**
     * Shape fitting lines
     */
    private List<Line2D> shapeFittingLines;

    /**
     * Straightest bundle found around intersection
     */
    private Bundle straightestBundle;

    /**
     * Whether we want to output all information.
     */
    private boolean debugJSONInformation;

    /**
     * Initialize our Intersection with the first cluster.
     *
     * @param cluster, the intersection cluster.
     */
    public Intersection(IntersectionCluster cluster) {
        this.allIntersectionClusters = new HashSet<>();
        this.allBundlesCrossingTheIntersection = new HashSet<>();
        this.intersectionLocationScore = 0.0;
        this.shapeFittingLines = new ArrayList<>();
        this.debugJSONInformation = false;

        addNewIntersectionCluster(cluster);
    }

    /**
     * We add a new intersection clusters to this intersection
     *
     * @param cluster, the intersection cluster to add
     */
    public void addNewIntersectionCluster(IntersectionCluster cluster) {
        allIntersectionClusters.add(cluster);
        allBundlesCrossingTheIntersection.addAll(cluster.getAllBundlesAroundThisCluster());
    }

    /**
     * We call this function when we merge another intersection into this one.
     *
     * @param otherIntersection, the other intersection object.
     */
    public void mergeWithAnotherIntersection(Intersection otherIntersection, Point2D otherLocation) {
        if (otherLocation != null) {
            if (otherIntersection.getIntersectionLocationScore() > this.getIntersectionLocationScore()) {
                intersectionLocation = otherLocation;
                this.setIntersectionLocationScore(otherIntersection.getIntersectionLocationScore());
            }
        }


        for (IntersectionCluster cluster : otherIntersection.getAllIntersectionClusters()) {
            addNewIntersectionCluster(cluster);
        }

        List<Line2D> otherShapeFittingLines = otherIntersection.getShapeFittingLines();
        if (otherShapeFittingLines != null) {
            shapeFittingLines.addAll(otherShapeFittingLines);
        }

        List<List<Subtrajectory>> otherBundlePartsBundled = otherIntersection.getBundlePartsBundled();
        if (otherBundlePartsBundled != null) {
            if (bundlePartsBundled == null){
                bundlePartsBundled = new ArrayList<>(otherBundlePartsBundled);
            }
            bundlePartsBundled.addAll(otherBundlePartsBundled);
        }
    }

    /**
     * We get an approximate location for our intersection
     * Used during the forming of this intersection.
     *
     * @return the approximate location
     */
    @JsonProperty
    public Point2D getApproximateLocation() {
        List<Point2D> allIntersectionClusterLocations = new ArrayList<>();
        for (IntersectionCluster cluster : allIntersectionClusters) {
            allIntersectionClusterLocations.add(cluster.getLocation());
        }

        return GeometryUtil.getAverage(allIntersectionClusterLocations);
    }

    /**
     * Get's the location of this intersection as precise as possible
     *
     * @return the location of this intersection
     */
    @JsonProperty
    public Point2D getLocation() {
        // If not walking dataset
        if (intersectionLocation == null && !STORAGE.getDatasetConfig().isWalkingDataset()) {
            intersectionLocation = ComputeIntersectionLocation.computeExactIntersectionLocation(this);
        }
        // If walking dataset
        if (STORAGE.getDatasetConfig().isWalkingDataset() || GeometryUtil.isPointNan(intersectionLocation)) {
            intersectionLocation = getApproximateLocation();
        }
        return intersectionLocation;
    }

    /**
     * Get's all the intersection clusters
     *
     * @return all intersection clusters part of this Intersection.
     */
    @JsonProperty
    public Set<IntersectionCluster> getAllIntersectionClusters() {
        return allIntersectionClusters;
    }

    /**
     * Gets all intersection points that are part of this intersection.
     *
     * @return all intersection points.
     */
    @JsonProperty
    public Set<IntersectionPoint> getAllIntersectionPoints() {
        HashSet<IntersectionPoint> intersectionPoints = new HashSet<>();
        for (IntersectionCluster cluster : allIntersectionClusters) {
            intersectionPoints.addAll(cluster.getAllIntersectionPoints());
        }
        return intersectionPoints;
    }

    /**
     * Get's all bundles crossing(overlapping) the intersection
     *
     * @return a set of bundles crossing(overlapping) the intersection
     */
    @JsonIgnore
    public Set<Bundle> getAllBundlesCrossingTheIntersection() {
        List<Bundle> bundlesFilteredOnRep = new ArrayList<>(allBundlesCrossingTheIntersection);
        IntersectionUtil.filterOutBundlesWithAToSmallRep(bundlesFilteredOnRep, getApproximateLocation(), 50, true);
        return new HashSet<>(bundlesFilteredOnRep);
    }

    /**
     * Get's all classes of the bundles crossing the intersection
     *
     * @return a set of bundleclasses of bundles crossing(overlapping) the intersection
     */
    @JsonProperty
    public Set<Integer> getAllBundlesClassesCrossingTheIntersection() {
        Set<Integer> allBundlesClasses = new HashSet<>();
        for (Bundle b : getAllBundlesCrossingTheIntersection()) {
            allBundlesClasses.add(STORAGE.getClassFromBundle(b));
        }
        return allBundlesClasses;
    }

    /**
     * Get's all bundles around the intersection
     *
     * @return a set of bundles around the intersection
     */
    @JsonIgnore
    public Set<Bundle> getAllBundlesAroundIntersection() {
        return allBundlesCrossingTheIntersection;
    }

    /**
     * Here we override the bundles around the intersection.
     * This is used at a specific point while computing the intersections.
     *
     * @param allBundlesCrossingTheIntersection, the new set of bundles around the intersection
     */
    public void overrideAllBundlesAroundIntersection(Set<Bundle> allBundlesCrossingTheIntersection){
        this.allBundlesCrossingTheIntersection = new HashSet<>(allBundlesCrossingTheIntersection);
    }

    /**
     * Get's all classes of the bundles around the intersection
     *
     * @return a set of bundleclasses of bundles crossing(overlapping) the intersection
     */
    @JsonProperty
    public Set<Integer> getAllBundlesClassesAroundTheIntersection() {
        Set<Integer> allBundlesClasses = new HashSet<>();
        for (Bundle b : getAllBundlesAroundIntersection()) {
            allBundlesClasses.add(STORAGE.getClassFromBundle(b));
        }
        return allBundlesClasses;
    }

    /**
     * Get all bundles reps parts before and after the intersection
     *
     * @return list of the parts.
     */
    public List<Subtrajectory> getBundleRepsPartsBeforeAfterIntersection() {
        return bundleRepsPartsBeforeAfterIntersection;
    }

    /**
     * Set all bundle representative parts before and after an intersection
     *
     * @param bundleRepsPartsBeforeAfterIntersection, the list of items.
     */
    public void setBundleRepsPartsBeforeAfterIntersection(List<Subtrajectory> bundleRepsPartsBeforeAfterIntersection) {
        this.bundleRepsPartsBeforeAfterIntersection = bundleRepsPartsBeforeAfterIntersection;
    }

    @JsonIgnore
    public Bundle getStraightestBundle() {
        return straightestBundle;
    }

    public void setStraightestBundle(Bundle straightestBundle) {
        this.straightestBundle = straightestBundle;
    }

    @JsonProperty
    public int getStraightestBundleClass() {
        return STORAGE.getClassFromBundle(straightestBundle);
    }

    @JsonIgnore
    public List<List<Subtrajectory>> getBundlePartsBundled() {
        return bundlePartsBundled;
    }


    @JsonProperty
    public List<List<Subtrajectory>> getBundlePartsBundledJSON() {
        if (debugJSONInformation){
            return bundlePartsBundled;
        }
        return new ArrayList<>();
    }

    public void setBundlePartsBundled(List<List<Subtrajectory>> bundlePartsBundled) {
        this.bundlePartsBundled = bundlePartsBundled;
    }

    @JsonIgnore
    public List<Line2D> getShapeFittingLines() {
        return shapeFittingLines;
    }

    public void setShapeFittingLines(List<Line2D> shapeFittingLines) {
        if (shapeFittingLines != null)
            this.shapeFittingLines = shapeFittingLines;
    }

    @JsonProperty
    public List<List<Point2D>> getShapeFittingLinesJSON() {
        List<List<Point2D>> shapeFittingPoints = new ArrayList<>();
        if (debugJSONInformation) {
            for (Line2D line2D : shapeFittingLines) {
                shapeFittingPoints.add(new ArrayList<>(Arrays.asList(line2D.getP1(), line2D.getP2())));
            }
        }
        return shapeFittingPoints;
    }

    public double getIntersectionLocationScore() {
        return this.intersectionLocationScore;
    }

    public void setIntersectionLocationScore(double score) {
        this.intersectionLocationScore = score;
    }

    @JsonIgnore
    public Set<IntersectionBundleCombo> getBundlesConnectedToIntersection() {
        return bundlesConnectedToIntersection;
    }

    @JsonProperty
    public Set<Integer> getBundlesClassesConnectedToIntersection() {
        Set<Integer> bundleClasses = new HashSet<>();
        for (IntersectionBundleCombo intersectionBundleCombo : getBundlesConnectedToIntersection()) {
            bundleClasses.add(STORAGE.getClassFromBundle(intersectionBundleCombo.getBundle()));
        }
        return bundleClasses;
    }

    public void setBundlesConnectedToIntersection(Set<IntersectionBundleCombo> bundlesConnectedToIntersection) {
        this.bundlesConnectedToIntersection = new HashSet<>(bundlesConnectedToIntersection);
    }

    @Override
    public boolean equals(Object object){
        if (object == null){
            return false;
        }

        if (!Intersection.class.isAssignableFrom(object.getClass())){
            return false;
        }

        final Intersection other = (Intersection) object;
        if (this.getLocation().distance(other.getLocation()) < 0.1){
            if (this != object){
                System.out.println("This is weird.. Two intersections being the same but still being different " +
                        "objects");
            }
            return true;
        }
        return false;
    }
}
