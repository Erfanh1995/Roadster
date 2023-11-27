package mapconstruction.trajectories;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.distance.RTree;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

/**
 * Class representing a trajectory.
 * <p>
 * A trajectory is a polygonal curve, consisting of a sequence of points with
 * straight lines in between.
 * <p>
 * After creation, trajectories are immutable.
 *
 * <p>
 * Implementation of equality depends on subclasses. Use {@code pointEquals} to
 * check equality based on the order of points.
 *
 * @author Roel Jacobs
 */
public abstract class Trajectory implements Serializable {

    private static final long serialVersionUID = 2L;
    /**
     * Memorization for efficient length computation
     */
    private volatile double length;

    /**
     * Memorization for efficient discrete length computation
     */
    private double discreteLength;

    /**
     * Array containing all edges. Created on demand
     */
    protected transient Line2D[] edges;

    /**
     * Needed such that subclasses can define their own constructors.
     */
    protected Trajectory() {
        this.length = Double.NEGATIVE_INFINITY;
        this.discreteLength = Double.NEGATIVE_INFINITY;
        this.edges = null;
    }

    /**
     * Returns the number of vertices in the trajectory.
     *
     * @return the number of vertices in the trajectory.
     */
    public abstract int numPoints();

    /**
     * Returns the number of edges in the trajectory.
     *
     * @return the number of edges in the trajectory. 0 if there are no
     * vertices.
     */
    public int numEdges() {
        return Math.max(0, numPoints() - 1);
    }

    /**
     * Returns the Euclidean length of this trajectory.
     *
     * @return Euclidean length of this trajectory.
     */
    public double euclideanLength() {
        if (length == Double.NEGATIVE_INFINITY) {
            double len = 0;
            for (Line2D l : edges()) {
                len += GeometryUtil.lineLength(l);
            }
            length = len;
        }
        return length;
    }

    /**
     * Returns the discrete length of this trajectory.
     *
     * @return Discrete length of this trajectory.
     */
    public double discreteLength(){
        if (discreteLength == Double.NEGATIVE_INFINITY) {
            discreteLength = getPoint(0).distance(getPoint(numPoints() - 1));
        }
        return discreteLength;
    }

    /**
     * Returns an immutable view on the points list in this trajectory.
     *
     * @return the trajectory as a list of points, in the order that they appear
     * on the trajectory.
     */
    //public abstract List<Point2D> getPointsList();

    /**
     * Returns the point at the given position in the trajectory.
     *
     * @param pos position in the trajectory of the point to get. Must be
     *            between 0 (inclusive) and {@code numPoints()} (exclusive)
     * @return point at the given position in the trajectory.
     * @throws IndexOutOfBoundsException if
     *                                   {@code pos < 0 || pos >= numPoints()}
     */
    public abstract Point2D getPoint(int pos);

    /**
     * Returns the edge at the i-th position in the trajectory.
     *
     * @param pos position in the trajectory of the point to get. Must be
     *            between 0 (inclusive) and {@code numEdges()} (exclusive)
     * @return point at the given position in the trajectory.
     * @throws IndexOutOfBoundsException if {@code pos < 0 || pos >= numEdges()}
     */
    public Line2D getEdge(int pos) {
        Preconditions.checkPositionIndex(pos, numEdges(), "pos");
        if (edges == null) {
            edges = new Line2D[numEdges()];
        }

        if (edges.length == 0){
            throw new ArrayIndexOutOfBoundsException("Edges.length = 0");
        }

        if (edges[pos] == null) {
            edges[pos] = new Line2D.Double(getPoint(pos), getPoint(pos + 1));
        }

        return edges[pos];
    }

    /**
     * Creates the reverse of this trajectory.
     *
     * @return reverse of this trajectory.
     */
    public abstract Trajectory reverse();

    /**
     * Whether this trajectory is the reversed version of an input trajectory.
     *
     * @return Whether this trajectory is the reversed version of an input
     * trajectory.
     */
    public abstract boolean isReverse();

    /**
     * Returns List of the points in proper order.
     * <p>
     * The returned list is immutable, and a view on the trajectory.
     *
     * @return List of the points in proper order.
     */
    public List<Point2D> points() {
        return new PointsList();
    }

    /**
     * @return List over edges in the trajectory, in proper order.
     */
    public List<Line2D> edges() {
        return new EdgesList();
    }

    @Override
    public String toString() {
        return getLabel();
    }

    public String toPointsString() {
        String pointList = StreamSupport.stream(points().spliterator(), true)
                .map(p -> "(" + p.getX() + ", " + p.getY() + ")") // point to string
                .collect(Collectors.joining(","));

        return "Trajectory{" + pointList + '}';
    }

    /**
     * Determines whether this trajectory has the same sequence of points at the
     * other trajectory.
     *
     * @param other trajectory to compare.
     * @return {@code true} if trajectory has the same sequence of points at the
     * other trajectory. {@code false} otherwise.
     */
    public boolean pointEquals(Trajectory other) {
        if (this == other) {
            return true;
        }

        return Iterables.elementsEqual(this.points(), other.points());
    }

    /**
     * Label for the given trajectory.
     *
     * @returnthe label of the given trajectory.
     */
    public abstract String getLabel();

    // caching of this label to prevent continuous string manipulation
    private String undirectedLabel;

    public String getUndirectionalLabel() {
        if (undirectedLabel == null) {
            undirectedLabel = getLabel().replaceFirst("_\\(r\\)", "");
        }
        return undirectedLabel;
    }

    public boolean hasAsLambdaEndpoints(Trajectory trajectory, double lambda) {
        double distance = Math.max(
                Math.min(
                        this.getPoint(0).distance(trajectory.getPoint(0)),
                        this.getPoint(0).distance(trajectory.getPoint(trajectory.numEdges()))
                ), Math.min(
                        this.getPoint(this.numEdges()).distance(trajectory.getPoint(trajectory.numEdges())),
                        this.getPoint(this.numEdges()).distance(trajectory.getPoint(0))
                )
        );

        return DoubleMath.fuzzyCompare(distance, lambda, 1E-6) <= 0;
    }

    /**
     * Private list class over the points in this trajectory.
     */
    private class PointsList extends AbstractList<Point2D> {

        @Override
        public Point2D get(int index) {
            return getPoint(index);
        }

        @Override
        public int size() {
            return numPoints();
        }

    }

    /**
     * Private list class over the points in this trajectory.
     */
    private class EdgesList extends AbstractList<Line2D> {

        @Override
        public Line2D get(int index) {
            return getEdge(index);
        }

        @Override
        public int size() {
            return numEdges();
        }


    }

}
