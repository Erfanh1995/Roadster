package mapconstruction.algorithms.maps.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.algorithms.representative.CutRepToBirthEps;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;
import static mapconstruction.algorithms.maps.IntersectionStorage.INTERSECTION_STORAGE;

public class BundleStreet implements Serializable {

    private Bundle bundle;
    private Set<Subtrajectory> newSubtrajectories;
    private double startIndex;
    private Intersection startIntersection;
    private double endIndex;
    private Intersection endIntersection;
    private int bundleStreetIndex;
    private boolean wasMergeableWithOtherRoad;
    private Set<BundleStreet> twoOfThreebundleStreetsTriedToMergeWith;
    private Set<BundleStreet> onOfThreeBundleStreetsTriedToMergeWith;
    private boolean isDrawnYet;

    public BundleStreet(Bundle bundle, double repStartIndex, Intersection startIntersection,
                        double repEndIndex, Intersection endIntersection, int bundleStreetIndex){
        this.bundle = bundle;
        this.startIndex = repStartIndex;
        this.startIntersection = startIntersection;
        this.endIndex = repEndIndex;
        this.endIntersection = endIntersection;
        this.bundleStreetIndex = bundleStreetIndex;
        this.isDrawnYet = false;

        if (this.startIndex == this.endIndex){
            throw new IllegalStateException("Not allowed!");
        }

        // Here we update the bundles to avoid bundles at bifurcations to continue far to long.
        Subtrajectory currentRep = getRepresentativeSubtrajectory();
        Pair<Subtrajectory, Set<Subtrajectory>> newPair = CutRepToBirthEps.cutOffBundleEndBasedOnEpsilon(bundle, currentRep);
        this.startIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(currentRep, newPair.getFirst().getFromIndex());
        this.endIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(currentRep, newPair.getFirst().getToIndex());
        this.newSubtrajectories = newPair.getSecond();
    }

    /**
     * Returns a subset of this BundleStreet
     * @param newStartPoint the new start point of the representative (may also be switched around)
     * @param newEndPoint the new end point of the representative (may also be switched around)
     * @return bundleStreet
     */
    public BundleStreet getSubBundleStreet(Point2D newStartPoint, Point2D newEndPoint) {
        double startI = this.startIndex;
        double endI = this.endIndex;
        if (this.startIndex > this.endIndex){
            startI = this.endIndex;
            endI = this.startIndex;
        }

        Subtrajectory subRep = new Subtrajectory(this.bundle.getRepresentative(), startI, endI);
        double startIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subRep, newStartPoint);
        startIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subRep, startIndex);

        double endIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subRep, newEndPoint);
        endIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subRep, endIndex);

        if (startIndex > endIndex){
            double tempIndex = startIndex;
            startIndex = endIndex;
            endIndex = tempIndex;
        }

        return getSubBundleStreet(startIndex, endIndex);
    }

    /**
     * Returns a subset of this BundleStreet
     * @param newStartIndex the newStartIndex
     * @param newEndIndex the newEndIndex
     * @return bundleStreet
     */
    public BundleStreet getSubBundleStreet(double newStartIndex, double newEndIndex){
        Intersection newStartIntersection = null;
        Intersection newEndIntersection = null;

        if (Math.abs(newStartIndex - this.startIndex) <= 0.25){
            newStartIntersection = this.startIntersection;
        }

        if (Math.abs(newEndIndex - this.endIndex) <= 0.25){
            newEndIntersection = this.endIntersection;
        }

        // @ Avoid BundleStreets of length 0
        if (DoubleMath.fuzzyEquals(newStartIndex, newEndIndex, 0.1)) {
            newStartIndex = Math.max(0.0, newStartIndex - 0.1);
            newEndIndex = Math.min(this.bundle.getRepresentative().numPoints() - 1, newEndIndex + 0.1);
        }

        return new BundleStreet(this.bundle, newStartIndex, newStartIntersection, newEndIndex, newEndIntersection, this.bundleStreetIndex);
    }

    @JsonIgnore
    public Bundle getBundle() {
        return bundle;
    }

    @JsonProperty
    public int getBundleClass(){
        return STORAGE.getClassFromBundle(getBundle());
    }

    @JsonProperty
    public double getStartIndex() {
        return startIndex;
    }

    @JsonProperty
    public double getEndIndex() {
        return endIndex;
    }

    @JsonProperty
    public int getBundleStreetIndex() {
        return bundleStreetIndex;
    }

    @JsonIgnore
    public Intersection getStartIntersection() {
        return startIntersection;
    }

    @JsonIgnore
    public Intersection getEndIntersection() {
        return endIntersection;
    }

    @JsonProperty
    public int getStartIntersectionClass(){
        return INTERSECTION_STORAGE.getIntersectionClassByIntersection(startIntersection);
    }

    @JsonProperty
    public int getEndIntersectionClass(){
        return INTERSECTION_STORAGE.getIntersectionClassByIntersection(endIntersection);
    }

    /**
     * Sets the status of this BundleStreet to drawn.
     * @return true if it was already drawn, false otherwise.
     */
    @JsonIgnore
    public boolean isAlreadyDrawn(){
        return isDrawnYet;
    }

    /**
     * Sets the status of this BundleStreet to drawn.
     */
    @JsonIgnore
    public void setToDrawn(){
        isDrawnYet = true;
    }

    @JsonProperty
    public Subtrajectory getRepresentativeSubtrajectory(){
        return new Subtrajectory(bundle.getRepresentative(), getStartIndex(), getEndIndex());
    }

    @JsonIgnore
    public Subtrajectory getRepresentativeReverseSubtrajectory(){
        Subtrajectory subtrajectory = new Subtrajectory(bundle.getRepresentative(), getStartIndex(), getEndIndex());
        subtrajectory = subtrajectory.reverse();
        return subtrajectory;
    }

    @JsonIgnore
    public Subtrajectory getRepresentativeSubtrajectoryWithIntersectionAtStart(Intersection intersection){
        if (intersection.equals(startIntersection)){
            return getRepresentativeSubtrajectory();
        } else if (intersection.equals(endIntersection)){
            return getRepresentativeReverseSubtrajectory();
        } else {
            System.out.println("BundleStreet.getRepresentativeSubtrajectory - Serious freaking error!");
            return null;
        }
    }

    @JsonIgnore
    public Set<Subtrajectory> getSubtrajectories(){
        return new HashSet<>(this.newSubtrajectories);
    }

    @JsonProperty
    public double getContinuousLength(){
        return getRepresentativeSubtrajectory().euclideanLength();
    }

    @JsonProperty
    public boolean isWasMergeableWithOtherRoad() {
        return wasMergeableWithOtherRoad;
    }

    public void setWasMergeableWithOtherRoad(boolean wasMergeableWithOtherRoad) {
        this.wasMergeableWithOtherRoad = wasMergeableWithOtherRoad;
    }

    @JsonIgnore
    public Set<BundleStreet> getTwoOfThreebundleStreetsTriedToMergeWith() {
        return twoOfThreebundleStreetsTriedToMergeWith;
    }

    public void setTwoOfThreebundleStreetsTriedToMergeWith(Set<BundleStreet> twoOfThreebundleStreetsTriedToMergeWith) {
        this.twoOfThreebundleStreetsTriedToMergeWith = twoOfThreebundleStreetsTriedToMergeWith;
    }

    @JsonIgnore
    public Set<BundleStreet> getOnOfThreeBundleStreetsTriedToMergeWith() {
        return onOfThreeBundleStreetsTriedToMergeWith;
    }

    public void setOnOfThreeBundleStreetsTriedToMergeWith(Set<BundleStreet> onOfThreeBundleStreetsTriedToMergeWith) {
        this.onOfThreeBundleStreetsTriedToMergeWith = onOfThreeBundleStreetsTriedToMergeWith;
    }
}
