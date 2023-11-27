package mapconstruction.algorithms.maps.containers;

import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is a container class for a combination between an intersection and a bundle.
 *
 * @author Jorrick Sleijster
 */
public class IntersectionBundleCombo implements Serializable {

    /**
     * Contains the Nundle and Intersection match and the index at which the intersection happens relative to the
     * bundle index.
     */
    private Bundle bundle;
    private Intersection intersection;
    private double index;

    /**
     * Contains the Intersection the representative matches before variable 'intersection'. Also contains it's index.
     */
    private BundleStreet bundleStreetBeforeIntersection;

    /**
     * Contains the Intersection the representative matches after variable 'intersection'. Also contains it's index.
     */
    private BundleStreet bundleStreetAfterIntersection;


    /**
     * Initializes our IntersectionBundleCombo
     * @param bundle, the bundle that comes across an intersection
     * @param intersection, the intersection the bundle comes across
     * @param index, the index at which the bundle representatives comes across the intersection.
     */
    public IntersectionBundleCombo(Bundle bundle, Intersection intersection, double index){
        this.bundle = bundle;
        this.intersection = intersection;
        this.index = index;

        bundleStreetBeforeIntersection = null;
        bundleStreetAfterIntersection = null;
    }

    /**
     * Initializes our IntersectionBundleCombo
     * @param bundle, the bundle that comes across an intersection
     * @param intersection, the intersection the bundle comes across
     * @param index, the index at which the bundle representatives comes across the intersection.
     * @param bundleStreetBeforeIntersection, the bundle street that ends at our intersection.
     * @param bundleStreetAfterIntersection, the bundle street that starts at our intersection.
     */
    public IntersectionBundleCombo(Bundle bundle, Intersection intersection, double index, BundleStreet bundleStreetBeforeIntersection, BundleStreet bundleStreetAfterIntersection){
        this(bundle, intersection, index);

        this.bundleStreetBeforeIntersection = bundleStreetBeforeIntersection;
        this.bundleStreetAfterIntersection = bundleStreetAfterIntersection;
    }

    public Bundle getBundle() {
        return bundle;
    }

    public Intersection getIntersection() {
        return intersection;
    }

    public double getIndex() {
        return index;
    }

    public Intersection getIntersectionBefore() {
        return this.bundleStreetBeforeIntersection.getStartIntersection();
    }

    public Double getIndexBefore() {
        return this.bundleStreetBeforeIntersection.getStartIndex();
    }

    public Intersection getIntersectionAfter() {
        return this.bundleStreetAfterIntersection.getEndIntersection();
    }

    public Double getIndexAfter() {
        return this.bundleStreetAfterIntersection.getEndIndex();
    }

    public List<Intersection> getNeighbourIntersections(){
        List<Intersection> intersections = new ArrayList<>();
        intersections.add(getIntersectionBefore());
        intersections.add(getIntersectionAfter());
        return intersections;
    }

    /**
     * Get's the Subtrajectory from the representative of the bundle from the current intersection to the intersection
     * before.
     * @return the subtrajectory.
     */
    public Subtrajectory getSubtrajectoryBefore(){
        Subtrajectory subtrajectory = new Subtrajectory(bundle.getRepresentative(), getIndexBefore(), getIndex());
        subtrajectory = subtrajectory.reverse();
        return subtrajectory;
    }

    /**
     * Get's the Subtrajectory from the representative of the bundle from the current intersection to the intersection
     * after.
     * @return the Subtrajectory.
     */
    public Subtrajectory getSubtrajectoryAfter(){
        return new Subtrajectory(bundle.getRepresentative(), getIndex(), getIndexAfter());
    }

    /**
     * Get's the Subtrajectory from the representative of the bundle to the intersection specified in the parameter
     * @param intersection, the intersection our Subtrajectory should end at.
     * @return the Subtrajectory.
     */
    public Subtrajectory getSubtrajectoryByNeighbourIntersection(Intersection intersection){
        if (intersection.equals(getIntersectionBefore())){
            return getSubtrajectoryBefore();
        } else if (intersection.equals(getIntersectionAfter())){
            return getSubtrajectoryAfter();
        } else {
            return null;
        }
    }



}
