package mapconstruction.algorithms.maps.intersections.containers;

import mapconstruction.trajectories.Bundle;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.List;

public class RoadPoint implements Serializable {

    /**
     * The bundle that ended which caused this road point.
     */
    private Bundle endedBundle;

    /**
     * The location of the roadPoint
     */
    private Point2D location;

    /**
     * Whether it was the start of the end of the bundle. True for start, false for end.
     */
    private boolean bundleStart;

    /**
     * Initializes the roadPoint
     *
     * @param b,          The bundle that ended and thus created this roadpoint
     * @param startOrEnd, Whether it was the start or the end of the bundle. True for start, false for end.
     */
    public RoadPoint(Bundle b, Boolean startOrEnd) {
        this.endedBundle = b;
        this.bundleStart = startOrEnd;

        List<Point2D> representativePoly = b.getRepresentativePolyline();
        if (startOrEnd) {
            this.location = representativePoly.get(0);
        } else {
            this.location = representativePoly.get(representativePoly.size() - 1);
        }
    }

    public Bundle getBundle() {
        return endedBundle;
    }

    public Point2D getLocation() {
        return location;
    }


    /**
     * Whether it was the start of the end of the bundle. True for start, false for end.
     *
     * @return bundleStart, true if it was the start.
     */
    public boolean isBundleStart() {
        return bundleStart;
    }
}
