package mapconstruction.algorithms.maps.network;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * Class representing an edge of the map.
 * Along with the two vertices of the edge, there
 * is a geometric representation with the edge.
 * <p>
 * The edge has a weight attribute that can be used to add additional information.
 *
 * @author Roel
 */
public class MapEdge implements Serializable {

    int id;
    /**
     * Bundlestreet that is drawn here
     */
    final BundleStreet bundleStreet;
    /**
     * first vertex.
     */
    private final MapVertex v1;
    /**
     * second vertex.
     */
    private final MapVertex v2;
    /**
     * Representation of the edge as a sequence of points.
     */
    private final ImmutableList<Point2D> representation;
    /**
     * Weight for the edge.
     */
    private final double weight;
    private Subtrajectory median;

    /**
     * Creates an edge with the given two vertices and the given representation and weight.
     * <p>
     * None of the parameters may be null.
     * Also, the first point in the representation must correspond to the location
     * of v1, and the last point must correspond to the location of v2.
     *
     * @param v1
     * @param v2
     * @param representation
     * @param weight
     * @throws NullPointerException     if any argument is {@code null}
     * @throws IllegalArgumentException if the location of v1 does not correspond to the first point in the representation,
     *                                  or the location of v2 does not correspond to the last point in the representation.
     */
    public MapEdge(MapVertex v1, MapVertex v2, List<Point2D> representation, double weight, BundleStreet b) {
        Preconditions.checkNotNull(v1, "v1 = null");
        Preconditions.checkNotNull(v2, "v2 == null");
        Preconditions.checkNotNull(representation, "representation == null");
        Preconditions.checkArgument(v1.getLocation().equals(representation.get(0)), "location of v1 does not match the first point");
        Preconditions.checkArgument(v2.getLocation().equals(representation.get(representation.size() - 1)), "location of v2 does not match the last point");
        this.v1 = v1;
        this.v2 = v2;

        this.representation = ImmutableList.copyOf(representation);

        this.weight = weight;
        this.bundleStreet = b;
    }

    /**
     * Creates an edge with the given two vertices and the given representation.
     * <p>
     * Sets the weight to 0;
     * <p>
     * None of the parameters may be null.
     * Also, the first point in the representation must correspond to the location
     * of v1, and the last point must correspond to the location of v2.
     *
     * @param v1
     * @param v2
     * @param representation
     * @throws NullPointerException     if any argument is {@code null}
     * @throws IllegalArgumentException if the location of v1 does not correspond to the first point in the representation,
     *                                  or the location of v2 does not correspond to the last point in the representation.
     */
    public MapEdge(MapVertex v1, MapVertex v2, List<Point2D> representation, BundleStreet b) {
        this(v1, v2, representation, 0, b);
    }

    /**
     * Creates an edge with the given two vertices.
     * <p>
     * Representation is simply a straight line between the two vertices.
     * <p>
     * Sets the weight to 0;
     * <p>
     * None of the parameters may be null.
     * Also, the first point in the representation must correspond to the location
     * of v1, and the last point must correspond to the location of v2.
     *
     * @param v1
     * @param v2
     * @throws NullPointerException if any argument is {@code null}
     */
    public MapEdge(MapVertex v1, MapVertex v2, BundleStreet b) {
        this(v1, v2, Arrays.asList(v1.getLocation(), v2.getLocation()), 0, b);
    }

    public MapEdge(Integer id, MapVertex v1, MapVertex v2) {
        this(v1, v2, null);
        this.id = id;
    }

    /**
     * Creates an edge with the given two vertices, and the given weight.
     * <p>
     * Representation is simply a straight line between the two vertices.
     * <p>
     * None of the parameters may be null.
     * Also, the first point in the representation must correspond to the location
     * of v1, and the last point must correspond to the location of v2.
     *
     * @param v1
     * @param v2
     * @param weight
     * @throws NullPointerException if any argument is {@code null}
     */
    public MapEdge(MapVertex v1, MapVertex v2, double weight, BundleStreet b) {
        this(v1, v2, Arrays.asList(v1.getLocation(), v2.getLocation()), weight, b);
    }

    @JsonProperty
    public int getId() {
        return id;
    }

    @JsonProperty
    public BundleStreet getBundleStreet() {
        return bundleStreet;
    }

    @JsonIgnore
    public Bundle getBundle() {
        if (bundleStreet == null){
            return null;
        }
        return bundleStreet.getBundle();
    }

    @JsonProperty
    public int getBundleClass() {
        if (getBundle() != null) {
            return STORAGE.getClassFromBundle(getBundle());
        } else {
            return -1;
        }
    }

    /**
     * Gets the first vertex.
     *
     * @return
     */
    @JsonProperty
    public MapVertex getV1() {
        return v1;
    }

    /**
     * Gets the second vertex.
     *
     * @return
     */
    @JsonProperty
    public MapVertex getV2() {
        return v2;
    }

    /**
     * Gets the representation of the edge.
     *
     * @return
     */
    @JsonProperty
    public ImmutableList<Point2D> getRepresentation() {
        return representation;
    }

    @JsonProperty
    public double getWeight() {
        return weight;
    }


    /**
     * Creates the reverse of this edge, with the same weight.
     *
     * @return
     */
    public MapEdge reverse() {
        return new MapEdge(v2, v1, Lists.reverse(representation), weight, bundleStreet);
    }

    /**
     * Creates the reverse of this edge, with the given weight.
     *
     * @param weight
     * @return
     */
    public MapEdge reverse(double weight) {
        return new MapEdge(v2, v1, Lists.reverse(representation), weight, bundleStreet);
    }

    @Override
    public int hashCode() {
        int hash = 7;
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
        final MapEdge other = (MapEdge) obj;
        if (Double.doubleToLongBits(this.weight) != Double.doubleToLongBits(other.weight)) {
            return false;
        }
        if (!Objects.equals(this.v1, other.v1)) {
            return false;
        }
        if (!Objects.equals(this.v2, other.v2)) {
            return false;
        }
        if (!Objects.equals(this.representation, other.representation)) {
            return false;
        }
        return true;
    }


    @Override
    public String toString() {
        return "<" + v1 + "; " + v2 + ">(" + weight + ")";
    }


}
