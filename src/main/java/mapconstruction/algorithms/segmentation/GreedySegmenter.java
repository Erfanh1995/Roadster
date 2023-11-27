package mapconstruction.algorithms.segmentation;

import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Segmentation using a greedy approach.
 * <p>
 * Based on a monotone decreasing criterion.
 * That is, if the criterion holds on a segment, it holds on any subsegment.
 * <p>
 * It works at follows.
 * <p>
 * Say we look at the segment [i,j]. We keep increasing j and check if
 * the criterion still holds.
 * <p>
 * As soon as it does not hold, we split off the the previous subtrajectory T[i,j - 1], and
 * continue with looking at [j-1, j-1].
 *
 * @author Roel
 */
public abstract class GreedySegmenter implements TrajectorySegmenter {

    public GreedySegmenter() {

    }


    @Override
    public List<Subtrajectory> segment(Trajectory original) {
        List<Subtrajectory> segments = new ArrayList<>();
        int i = 0;
        int j = 0;

        while (true) {

            if (check(original, i, j)) {
                // criterion still ok
                if (j == original.numPoints() - 1) {
                    // reached the end, add last segment and break
                    segments.add(new Subtrajectory(original, i, j));
                    break;
                } else {
                    j++;
                }


            } else {
                // split off segment [i, j - 1]
                segments.add(new Subtrajectory(original, i, j - 1));
                i = j - 1;
                j = j - 1;
            }
        }
        return segments;
    }


    /**
     * Checks whether the criterion holds on the given trajectory between the ith and jth point,
     * both inclusive.
     *
     * @param t trajectory to check
     * @param i index of start point
     * @param j index of end point.
     * @return
     */
    protected abstract boolean check(Trajectory t, int i, int j);
}
