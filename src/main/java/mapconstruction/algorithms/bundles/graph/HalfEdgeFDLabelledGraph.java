/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.algorithms.bundles.graph;

import com.google.common.collect.TreeRangeSet;
import com.google.common.math.Stats;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

/**
 * @author Roel
 */
public class HalfEdgeFDLabelledGraph extends DiscreteFDLabelledGraph {

    public HalfEdgeFDLabelledGraph(Trajectory concatenated, TreeRangeSet<Integer> existing, double epsilon) {
        super(concatenated, existing, epsilon);
    }

    /**
     * Returns whether the space between the i-th point of the first trajectory,
     * and the j-th point of the second trajectory is part of free space, based
     * on the given distance.
     * <p>
     * The space is free if the distance from one point to to the half
     * edges extending from the other point is at most the given distance.
     * <p>
     * We must have {@code <= i < getT1.numPoints && 0 <= j < getT2.numPoints}
     *
     * @param i point index of first trajectory
     * @param j point index of second trajectory
     * @return {@code true} if {@code d(T1[i], T2[j]) <= d}
     * @throws IndexOutOfBoundsException
     */
    @Override
    protected boolean isFree(int i, int j) {
        return getPointHalfEdgeDistance(i, j) <= epsilon || getPointHalfEdgeDistance(j, i) <= epsilon;
    }


    /**
     * Gets the distance between the ith point and the half edges extending from
     * the jth point.
     *
     * @param i
     * @param j
     * @return
     */
    public double getPointHalfEdgeDistance(int i, int j) {
        Point2D start;
        if (j == existing.rangeContaining(j).lowerEndpoint()) {
            start = dm.getT2().getPoint(j);
        } else {
            Line2D edge = dm.getT2().getEdge(j - 1);
            start = new Point2D.Double(Stats.meanOf(edge.getX1(), edge.getX2()),
                    Stats.meanOf(edge.getY1(), edge.getY2()));
        }

        Point2D end;
        if (j == existing.rangeContaining(j).upperEndpoint()) {
            end = dm.getT2().getPoint(j);
        } else {
            Line2D edge = dm.getT2().getEdge(j);
            end = new Point2D.Double(Stats.meanOf(edge.getX1(), edge.getX2()),
                    Stats.meanOf(edge.getY1(), edge.getY2()));
        }

        return new Line2D.Double(start, end).ptSegDist(dm.getT1().getPoint(i));
    }


}
