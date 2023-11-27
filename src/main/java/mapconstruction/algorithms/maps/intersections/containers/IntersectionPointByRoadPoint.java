package mapconstruction.algorithms.maps.intersections.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import mapconstruction.trajectories.Bundle;

import java.awt.geom.Point2D;
import java.util.HashSet;
import java.util.Set;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * DataClass for an IntersectionPoint created by a RoadPoint(an ending bundle).
 *
 * @author Jorrick
 * @since 06/11/2018
 */
public class IntersectionPointByRoadPoint extends IntersectionPoint {
    /**
     * Bundle that ended at the intersection
     */
    private Bundle endBundle;
    private int endBundleClass;
    private Point2D roadPointLocation;

    /**
     * Initializes IntersectionPoint by a RoadPoint.
     *
     * @param longBundle1, the first bundle of the pair
     * @param longBundle2, the second bundle of the pair
     * @param overlappingBundle, an overlapping bundle of the pair (or at least a try to get one..)
     * @param endBundle, the ending bundle that introduces this IntersectionPoint.
     * @param roadPointLocation, the location of the roadPoint.
     */
    public IntersectionPointByRoadPoint(Bundle longBundle1, Bundle longBundle2, Bundle overlappingBundle,
                                        Bundle endBundle, Point2D roadPointLocation) {
        super(longBundle1, longBundle2, overlappingBundle);

        this.endBundle = endBundle;
        this.endBundleClass = STORAGE.getClassFromBundle(endBundle);
        this.roadPointLocation = roadPointLocation;
    }

    /**
     * We calculate the location of this intersectionPoint
     *
     * @return the location
     */
    @JsonIgnore
    Point2D calculateLocation() {
        return roadPointLocation;
    }


    /**
     * Get's all bundles connected to the IntersectionPoint
     * @return all bundles around the IntersectionPoint
     */
    @JsonIgnore
    public Set<Bundle> getAllBundlesConnectedToIntersectionPoint(){
        Set<Bundle> bundles = new HashSet<>();
        bundles.add(endBundle);
        bundles.add(getLongBundle1());
        bundles.add(getLongBundle2());
        bundles.add(getOverlappingBundle());
        return bundles;
    }

    /**
     * Get's the ending of the bundle
     * @return the ending bundle
     */
    @JsonIgnore
    public Bundle getEndBundle() {
        return endBundle;
    }

    /**
     * Get's the ending bundle class.
     * @return the ending bundle class
     */
    public int getEndBundleClass() {
        return endBundleClass;
    }

}
