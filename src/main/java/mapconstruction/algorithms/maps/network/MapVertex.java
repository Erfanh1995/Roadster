package mapconstruction.algorithms.maps.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

/**
 * Class representing a vertex on a map.
 *
 * @author Roel
 */
public class MapVertex implements Serializable {

    static int UNIQUE_ID = 0;
    static Map<Double, Map<Double, Integer>> indexMap = new HashMap<Double, Map<Double, Integer>>();
    /**
     * X-coordinate of the vertex
     */
    private final double x;
    /**
     * Y-coordinate of the vertex
     */
    private final double y;
    /**
     * Containing whether is it was introduced by an Intersection class or by a merge or split or continue;
     */
    private final Intersection intersection;
    /**
     * Intersection boolean, illustrating whether it is an intersection.
     */
    private final boolean isIntersection;

    int uid;

    /**
     * Creates a vertex with he given coordinates.
     *  @param x
     * @param y
     * @param isIntersection
     */
    public MapVertex(double x, double y, Intersection intersection, boolean isIntersection) {
        this.x = x;
        this.y = y;
        this.uid = generateId(x, y);
        this.intersection = intersection;
        this.isIntersection = isIntersection;
    }

    /**
     * Creates a vertex at the given location.
     *
     * @param location location of the vertex.
     * @param isIntersection
     */
    public MapVertex(Point2D location, Intersection intersection, boolean isIntersection) {
        this.x = location.getX();
        this.y = location.getY();
        this.uid = generateId(this.x, this.y);
        this.intersection = intersection;
        this.isIntersection = isIntersection;
    }

    private static int generateId(double x, double y) {
        // See if (x,y) already exists
        if (indexMap.get(x) == null) {
            indexMap.put(x, new HashMap<Double, Integer>());
        }
        Integer existing = indexMap.get(x).get(y);
        if (existing == null) {
            existing = new Integer(++UNIQUE_ID);
            indexMap.get(x).put(y, existing);
        }
        return existing.intValue();
    }

    /**
     * Returns a "friendly" unique id needed to export vertex to text file
     *
     * @return
     */
    @JsonProperty
    public int getId() {
        return uid;
    }

    /**
     * Gets the x-coordinate of the location of this vertex.
     *
     * @return
     */
    @JsonProperty
    public double getX() {
        return x;
    }

    /**
     * Gets the y-coordinate of the location of this vertex.
     *
     * @return
     */
    @JsonProperty
    public double getY() {
        return y;
    }

    /**
     * Gets whether the vertex is an intersection.
     */
    @JsonProperty
    public boolean isIntersection(){
        return isIntersection;
    }

    /**
     * Gets a point representing the location of this vertex.
     *
     * @return
     */
    public Point2D getLocation() {
        return new Point2D.Double(x, y);
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 59 * hash + (int) (Double.doubleToLongBits(this.x) ^ (Double.doubleToLongBits(this.x) >>> 32));
        hash = 59 * hash + (int) (Double.doubleToLongBits(this.y) ^ (Double.doubleToLongBits(this.y) >>> 32));
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final MapVertex other = (MapVertex) obj;
        if (Double.doubleToLongBits(this.x) != Double.doubleToLongBits(other.x)) {
            return false;
        }
        if (Double.doubleToLongBits(this.y) != Double.doubleToLongBits(other.y)) {
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return String.format("(%s, %s)", Double.toString(x), Double.toString(y));
    }


}
