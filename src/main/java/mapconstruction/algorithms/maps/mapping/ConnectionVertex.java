package mapconstruction.algorithms.maps.mapping;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;

import java.awt.geom.Point2D;
import java.io.Serializable;

/**
 * A ConnectionVertex is an object which is at least connected to one RoadSection.
 *
 * It contains the logic similar to an intersection in a RoadMap, however, it can also contain just one RoadSection.
 */

public class ConnectionVertex implements Serializable {

    private Point2D location;
    private boolean isIntersection;
    private Intersection intersection;

    ConnectionVertex(Point2D location, Intersection intersection){
        this.location = location;
        this.intersection = intersection;
        isIntersection = (intersection != null);
    }

    /**
     * Get's the location of the ConnectionVertex
     * @return the location
     */
    @JsonProperty
    public Point2D getLocation() {
        return location;
    }

    /**
     * Returns whether it is created due to an Intersection object.
     * @return true if it is an intersection
     */
    @JsonProperty
    public boolean isIntersection() {
        return isIntersection;
    }

    /**
     * Returns the Intersection object if it is created due to an Intersection object, otherwise returns null.
     * @return the Intersection object or {@code null}.
     */
    @JsonIgnore
    public Intersection getIntersection() {
        return intersection;
    }
}
