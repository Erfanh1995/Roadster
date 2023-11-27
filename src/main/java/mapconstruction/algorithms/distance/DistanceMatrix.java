package mapconstruction.algorithms.distance;

import com.google.common.base.Preconditions;
import com.google.common.primitives.Doubles;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

/**
 * Distance matrix between two trajectories.
 * <p>
 * Does not explicitly store all the distances, as that costs too much memory,
 * but it is calculated every query.
 *
 * @author Roel
 */
public class DistanceMatrix {

    /**
     * First trajectory, populating the first index of the matrix.
     */
    private final Trajectory t1;

    /**
     * First trajectory, populating the second index of the matrix.
     */
    private final Trajectory t2;

    /**
     * Computes the discrete distance matrix for the given the two trajectories.
     * <p>
     * Trajectories may not be {@code null}
     *
     * @param t1 first trajectory
     * @param t2 second trajectory
     * @throws NullPointerException if one of the trajectories is {@code null}.
     */
    public DistanceMatrix(Trajectory t1, Trajectory t2) {
        Preconditions.checkNotNull(t1, "t1 == null");
        Preconditions.checkNotNull(t2, "t2 == null");
        this.t1 = t1;
        this.t2 = t2;
    }

    /**
     * Gets the first trajectory. This trajectory corresponds to the first index
     * of the diagram.
     *
     * @return the first trajectory.
     */
    public Trajectory getT1() {
        return t1;
    }

    /**
     * Gets the second trajectory. This trajectory corresponds to the second
     * index of the diagram.
     *
     * @return the second trajectory.
     */
    public Trajectory getT2() {
        return t2;
    }

    /**
     * Returns the distance between i-th point of the first trajectory, and the
     * j-th point of the second trajectory
     * <p>
     * We must have {@code 0 <= i < getT1.numPoints && 0 <= j < getT2.numPoints}
     *
     * @param i point index of first trajectory
     * @param j point index of second trajectory
     * @return distance between i-th point of the first trajectory, and the j-th
     * point of the second trajectory
     * @throws IndexOutOfBoundsException if NOT {@code 0 <= i < getT1.numPoints && 0 <= j < getT2.numPoints}
     */
    public double getPointDistance(int i, int j) {
        Preconditions.checkPositionIndex(i, t1.numPoints(), "i");
        Preconditions.checkPositionIndex(j, t2.numPoints(), "j");
        final Point2D p1 = t1.getPoint(i);
        final Point2D p2 = t2.getPoint(j);
        return p1.distance(p2);
    }

    /**
     * Returns the distance between i-th edge of the first trajectory, and the
     * j-th edge of the second trajectory, ignoring intersections
     * <p>
     * The distance between two edges is the minimum distance from any endpoint
     * of one edge to the other edge. Note that this means that the distance is
     * NOT 0 if the interiors of the edges intersect.
     * <p>
     * We must have
     * {@code 0 <= i < getT1.numPoints - 1 && 0 <= j < getT2.numPoints - 1}
     *
     * @param i edge index of first trajectory
     * @param j edge index of second trajectory
     * @return distance between i-th edge of the first trajectory, and the j-th
     * edge of the second trajectory, ignoring intersections.
     * @throws IndexOutOfBoundsException if NOT {@code 0 <= i < getT1.numPoints - 1 && 0 <= j < getT2.numPoints - 1}
     */
    public double getEdgeDistanceNoIntersect(int i, int j) {
        Preconditions.checkPositionIndex(i, t1.numEdges(), "i");
        Preconditions.checkPositionIndex(j, t2.numEdges(), "j");
        Line2D edge1 = t1.getEdge(i);
        Line2D edge2 = t2.getEdge(j);

        double d11 = edge2.ptSegDist(edge1.getP1());
        double d12 = edge2.ptSegDist(edge1.getP2());

        double d21 = edge1.ptSegDist(edge2.getP1());
        double d22 = edge1.ptSegDist(edge2.getP2());

        return Doubles.min(d11, d12, d21, d22);
    }

    /**
     * Returns the distance between i-th edge of the first trajectory, and the
     * j-th edge of the second trajectory, taking intersections into account.
     * <p>
     * The distance between two edges is the minimum distance from any endpoint
     * of one edge to the other edge, or 0 if the edges intersect.
     * <p>
     * We must have {@code 0 <= i < getT1.numEdges && 0 <= j < getT2.numEdges}
     *
     * @param i edge index of first trajectory
     * @param j edge index of second trajectory
     * @return distance between i-th edge of the first trajectory, and the j-th
     * edge of the second trajectory
     * @throws IndexOutOfBoundsException if NOT {@code 0 <= i < getT1.numEdges && 0 <= j < getT2.numEdges}
     */
    public double getEdgeDistance(int i, int j) {
        Preconditions.checkPositionIndex(i, t1.numEdges(), "i");
        Preconditions.checkPositionIndex(j, t2.numEdges(), "j");
        Line2D edge1 = t1.getEdge(i);
        Line2D edge2 = t2.getEdge(j);

        if (edge1.intersectsLine(edge2)) {
            return 0;
        } else {
            return getEdgeDistanceNoIntersect(i, j);
        }
    }

    /**
     * Returns the distance between i-th point of the first trajectory, and the
     * j-th edge of the second trajectory
     * <p>
     * We must have {@code 0 <= pi < getT1.numPoints  && 0 <= ej < getT2.numEdges}
     *
     * @param pi point index of first trajectory
     * @param ej edge index of second trajectory
     * @return distance between i-th point of the first trajectory, and the j-th
     * edge of the second trajectory
     * @throws IndexOutOfBoundsException if NOT {@code 0 <= pi < getT1.numPoints  && 0 <= ej < getT2.numEdges}
     */
    public double getPointEdgeDistance(int pi, int ej) {
        Preconditions.checkPositionIndex(pi, t1.numPoints(), "pi");
        Preconditions.checkPositionIndex(ej, t2.numEdges(), "ej");
        final Line2D edge = t2.getEdge(ej);
        final Point2D point = t1.getPoint(pi);
        return edge.ptSegDist(point);
    }

    /**
     * Returns the distance between i-th edge of the first trajectory, and the
     * j-th point of the second trajectory
     * <p>
     * We must have
     * {@code 0 <= ei < getT1.numEdges  && 0 <= pj < getT2.numPoints}
     *
     * @param ei edge index of first trajectory
     * @param pj point index of second trajectory
     * @return distance between i-th edge of the first trajectory, and the j-th
     * point of the second trajectory
     * @throws IndexOutOfBoundsException if NOT {@code 0 <= ei < getT1.numEdges  && 0 <= pj < getT2.numPoints}
     */
    public double getEdgePointDistance(int ei, int pj) {
        Preconditions.checkPositionIndex(ei, t1.numEdges(), "ei");
        Preconditions.checkPositionIndex(pj, t2.numPoints(), "pj");
        final Line2D edge = t1.getEdge(ei);
        final Point2D point = t2.getPoint(pj);
        return edge.ptSegDist(point);
    }

}
