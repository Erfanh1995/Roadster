package mapconstruction.algorithms.maps.intersections.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.trajectories.Bundle;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.Set;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * DataClass superclass for an IntersectionPoint.
 *
 * An IntersectionPoint can be created due to a RoadPoint or a Turn.
 *
 * @author Jorrick
 * @since 06/11/2018
 */
public abstract class IntersectionPoint implements Serializable {

    /**
     * The bundles that go through the intersection and split up the trajectories.
     */
    private Bundle longBundle1;
    private int longBundle1Class;
    private Bundle longBundle2;
    private int longBundle2Class;
    private Bundle overlappingBundle;
    private int overlappingBundleClass;

    /**
     * Location of the actual intersection
     */
    private Point2D location;

    /**
     * Initializes the IntersectionPoint. Should only be called by superclass
     *
     * @param longBundle1, the first bundle of the pair
     * @param longBundle2, the second bundle of the pair
     * @param overlappingBundle, an overlapping bundle of the pair (or at least a try to get one..)
     */
    IntersectionPoint(Bundle longBundle1, Bundle longBundle2, Bundle overlappingBundle) {
        this.longBundle1 = longBundle1;
        this.longBundle1Class = STORAGE.getClassFromBundle(longBundle1);
        this.longBundle2 = longBundle2;
        this.longBundle2Class = STORAGE.getClassFromBundle(longBundle2);
        this.overlappingBundle = overlappingBundle;
        this.overlappingBundleClass = STORAGE.getClassFromBundle(overlappingBundle);
        this.location = null;
    }

    @JsonProperty
    public Point2D getLocation() {
        if (location == null) {
            this.location = calculateLocation();
        }
        return location;
    }

    /**
     * This function calculates the location of the intersection. For turns this is obvious, however, this is much
     * less obvious for intersections by RoadPoints.
     *
     * @return the location.
     */
    @JsonIgnore
    abstract Point2D calculateLocation();

    /**
     * Get's all bundles connected to the IntersectionPoint
     * @return all bundles around the IntersectionPoint
     */
    @JsonIgnore
    public abstract Set<Bundle> getAllBundlesConnectedToIntersectionPoint();

    /**
     * Standard getters and setters collection. Nothing much interesting.
     *
     * @return the set element in all cases.
     */
    @JsonIgnore
    public Bundle getLongBundle1() {
        return longBundle1;
    }

    @JsonIgnore
    public Bundle getLongBundle2() {
        return longBundle2;
    }

    @JsonIgnore
    public Bundle getOverlappingBundle() {
        return overlappingBundle;
    }

    @JsonProperty
    public int getLongBundle1Class() {
        return longBundle1Class;
    }

    @JsonProperty
    public int getLongBundle2Class() {
        return longBundle2Class;
    }

    @JsonProperty
    public int getOverlappingBundleClass() {
        return overlappingBundleClass;
    }
}
