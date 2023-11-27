package mapconstruction.algorithms.bundles.graph;

import com.google.common.collect.Range;
import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.trajectories.Trajectory;

import java.util.Collection;
import java.util.OptionalInt;

/**
 * Abstract class representing a labelled graph on the free space
 * of a trajectory to itself.
 * <p>
 * The main purpose is finding the y-coodinate of the start point in the FD
 * of a cluster curve, given its x-coordinate and the x/y coordinate of the end
 * point
 *
 * @author Roel
 */
public abstract class LabelledGraph {

    /**
     * Distance matrix to use, containing the concatenated trajectories
     */
    protected DistanceMatrix dm;


    /**
     * Frechet distance.
     */
    protected double epsilon;

    protected LabelledGraph(Trajectory concatenated, double epsilon) {
        this.dm = new DistanceMatrix(concatenated, concatenated);
        this.epsilon = epsilon;
    }


    /**
     * Tries to find the y-coordinate of the point on s of cluster curve between
     * s and t, ending at (t,yt).
     * <p>
     * Makes the curve as long as possible.
     * <p>
     * The returned optional does not contain a value if no curve exists, and
     * the value of the y-coordinate if it does exist.
     *
     * @param s         x-coordinate of the start point
     * @param t         x-coordinate of the end point
     * @param yt        y-coordinate of the end point
     * @param forbidden Index ranges indicating ranges that the cluster curves is not allowed to intersect,
     *                  in addition to th boundaries. The algorithm should try its best to make the curves not cross the
     *                  forbidden ranges, but is not required to do so.
     * @return
     */
    public abstract OptionalInt findStart(int s, int t, int yt, Collection<Range<Integer>> forbidden);

    /**
     * Adds a column at the end of the graph, if possible.
     */
    public abstract void addColumn();


    /**
     * Removes the leftmost column from the graph, if possible.
     */
    public abstract void removeColumn();

    public int getTotalNumPoints() {
        return dm.getT1().numPoints();
    }

    public DistanceMatrix getDistanceMatrix() {
        return dm;
    }

    public double getEpsilon() {
        return epsilon;
    }


}
