package mapconstruction.algorithms.maps.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.trajectories.Bundle;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;
import static mapconstruction.algorithms.maps.IntersectionStorage.INTERSECTION_STORAGE;

/**
 * This class represents a container class which takes all bundleStreets.
 *
 * @author Jorrick Sleijster
 */

public class IntersectionsConnector implements Serializable {

    private BundleStreet mainBundleStreet;

    private Set<BundleStreet> bundleStreets;

    private Intersection intersection1;

    private Intersection intersection2;

    /**
     * Create a Street object for a part between two intersections.
     *
     * @param intersection1, an intersection.
     * @param intersection2, another intersection. (May also be null)
     * @param bundleStreet,  a part of a bundle between the two intersections
     */
    public IntersectionsConnector(Intersection intersection1, Intersection intersection2, BundleStreet bundleStreet) {
        if (intersection1 == null && intersection2 == null) {
            try {
                throw new Exception("Error in creating street. Both intersections are null");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        this.intersection1 = intersection1;
        this.intersection2 = intersection2;
        this.mainBundleStreet = bundleStreet;
        this.bundleStreets = new HashSet<>();


        addBundleStreet(bundleStreet);
    }

    /**
     * Add a new BundleStreet to the set.
     *
     * @param bundleStreet, the newly added BundleStreet.
     */
    public void addBundleStreet(BundleStreet bundleStreet) {
        this.bundleStreets.add(bundleStreet);
    }

    /**
     * Get's all bundle streets
     *
     * @return all bundleStreets
     */
    @JsonIgnore
    public Set<BundleStreet> getBundleStreets() {
        return bundleStreets;
    }

    /**
     * Get the first intersection
     *
     * @return the first intersection
     */
    @JsonIgnore
    public Intersection getIntersection1() {
        return intersection1;
    }

    /**
     * Get the second intersection
     *
     * @return the second intersection
     */
    @JsonIgnore
    public Intersection getIntersection2() {
        return intersection2;
    }

    /**
     * Get the intersection1 index in the intersections array
     */
    @JsonProperty
    public int getIntersection1Index(){
        return INTERSECTION_STORAGE.getIntersectionClassByIntersection(intersection1);
    }

    /**
     * Get the intersection2 index in the intersections array
     */
    @JsonProperty
    public int getIntersection2Index(){
        return INTERSECTION_STORAGE.getIntersectionClassByIntersection(intersection2);
    }

    /**
     * Get the main Bundle Street that connects the two different intersections.
     *
     * @return the main bundle street.
     */
    @JsonProperty
    public BundleStreet getMainBundleStreet() {
        return mainBundleStreet;
    }

    /**
     * Get the main Bundle Street that connects the two different intersections.
     *
     * @return the main bundle street.
     */
    @JsonProperty
    public Integer getMainBundleStreetClass() {
        return mainBundleStreet.getBundleClass();
    }


    /**
     * Get all Bundle Street classes
     */
    @JsonProperty
    public List<Integer> getBundleStreetClasses(){
        List<Integer> bundleClasses = new ArrayList<>();
        for (BundleStreet bundleStreet : bundleStreets){
            bundleClasses.add(bundleStreet.getBundleClass());
        }
        return bundleClasses;
    }

    /**
     * Get's the number of Bundle streets contained in this intersection connector.
     *
     * @return the number of bundle streets.
     */
    @JsonProperty
    public int getNoBundleStreets() {
        return getBundleStreets().size();
    }

    /**
     * Get's the number of Bundle streets that have both intersections as end points as well.
     * @return the number of bundle streets that have both intersections as end points as well.
     */
    @JsonProperty
    public int getNoBundleStreetsWithSameBothIntersection() {
        int counter = 0;
        for (BundleStreet bundleStreet : bundleStreets) {
            if ((bundleStreet.getStartIntersection() == this.intersection1 &&
                    bundleStreet.getEndIntersection() == this.intersection2) ||
                    (bundleStreet.getStartIntersection() == this.intersection2 &&
                            bundleStreet.getEndIntersection() == this.intersection1)) {
                counter++;
            }
        }
        return counter;
    }

    /**
     * Get's a statistic of this Intersection Connector
     * @return the average bundle epsilon.
     */
    @JsonProperty
    public double getAverageBundleEpsilon(){
        int counter = 0;
        double average = 0.0;
        for (BundleStreet bundleSteet : bundleStreets){
            Bundle bundle = bundleSteet.getBundle();
            int bundleClass = STORAGE.getClassFromBundle(bundle);
            double birthEpsilon = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);
            average = (average * counter + birthEpsilon) / (counter + 1);
            counter++;
        }
        return average;
    }

    /**
     * Get's a statistic of this Intersection Connector
     * @return the average bundle epsilon squared.
     */
    @JsonProperty
    public double getAverageBundleEpsilonSquared(){
        int counter = 0;
        double average = 0.0;
        for (BundleStreet bundleSteet : bundleStreets){
            Bundle bundle = bundleSteet.getBundle();
            int bundleClass = STORAGE.getClassFromBundle(bundle);
            double birthEpsilon = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);
            average = (average * counter + Math.pow(birthEpsilon, 2)) / (counter + 1);
            counter++;
        }
        return average;
    }

    /**
     * Get's a statistic of this Intersection Connector
     * @return the average bundle epsilon squared with threshold.
     */
    @JsonProperty
    public double getAverageBundleEpsilonSquaredWithThreshold(){
        // Only include bundles with >30 epsilon.
        int counter = 0;
        double average = 0.0;
        for (BundleStreet bundleSteet : bundleStreets){
            Bundle bundle = bundleSteet.getBundle();
            int bundleClass = STORAGE.getClassFromBundle(bundle);
            double birthEpsilon = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);

            if (birthEpsilon < 25){
                continue;
            }

            average = (average * counter + Math.pow(birthEpsilon, 2)) / (counter + 1);
            counter++;
        }
        return average;
    }


    /**
     * Get's a statistic of this Intersection Connector
     * @return the number of bundle above a certain threshold.
     */
    @JsonProperty
    public int getNumberOfBundlesAboveThreshold(){
        // Only include bundles with >30 epsilon.
        int counter = 0;
        for (BundleStreet bundleSteet : bundleStreets){
            Bundle bundle = bundleSteet.getBundle();
            int bundleClass = STORAGE.getClassFromBundle(bundle);
            double birthEpsilon = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);

            if (birthEpsilon < 25){
                continue;
            }
            counter++;
        }
        return counter;
    }

    /**
     * Get's a statistic of this Intersection Connector
     * @return the percentage of bundles above a certain threshold.
     */
    @JsonProperty
    public double getPercentageOfBundlesAboveThreshold(){
        // Only include bundles with >30 epsilon.
        return (double) (getNumberOfBundlesAboveThreshold()) / getNoBundleStreets();
    }

    /**
     * Get the main BundleStreets continuous length
     * @return the continuous length
     */
    @JsonIgnore
    public double getMainBundleStreetCL(){
        return getMainBundleStreet().getContinuousLength();
    }
}
