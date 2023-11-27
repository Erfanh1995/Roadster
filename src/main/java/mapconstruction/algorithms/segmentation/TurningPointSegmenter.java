/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.algorithms.segmentation;

import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Splits the trajectory at turning points. These turning points are detected by
 * inspecting the free space diagram.
 *
 * @author Roel
 */
public class TurningPointSegmenter implements TrajectorySegmenter {

    /**
     * Maximum distance to use to determine free space.
     */
    private final double distance;

    private final double factor;

    public TurningPointSegmenter(double distance, double factor) {
        this.distance = distance;
        this.factor = factor;
    }

    @Override
    public List<Subtrajectory> segment(Trajectory original) {
        List<Subtrajectory> result = new ArrayList<>();
        int start = 0;
        for (int i : findTurningPoints(original)) {
            result.add(new Subtrajectory(original, start, i));
            start = i;
        }
        result.add(new Subtrajectory(original, start, original.numPoints() - 1));
        return result;
    }

    /**
     * Finds a list of indices indicating the turning points in the given
     * trajectory
     *
     * @return
     */
    private List<Integer> findTurningPoints(Trajectory t) {
        List<Integer> result = new ArrayList<>();

        // create a distance matric between the given trajectory and its reverse
        DistanceMatrix dm = new DistanceMatrix(t, t.reverse());

        // We move along the "main" (topleft to bottom right) diagonal, trying to
        // find maximall intervals intersecting the diagonal.
        // The intervals are indiced with respect to the original trajectory
        // Since the free space is symmertric along the main diagonal,
        // we compute the intervals with respect to the diagonal.
        int N = t.numPoints();
        int[] previousInterval = new int[]{0, 0};
        for (int i = 0; i < N; i++) {
            // coordinates on diagonal
            int x = i;
            int y = N - i - 1;

            int[] interval = new int[2];
            interval[0] = x;
            interval[1] = x;

            // NB, diagonal is free by definition
            // so no need to check if it is free.
            for (int j = 0; j <= Math.min(i, N - i - 1); j++) {
                int x2 = x + j;
                int y2 = y + j;
                if (isFree(dm, x2, y2)) {
                    interval[1] = x2;
                } else {
                    break;
                }
            }

            // Check if the previously found interval was maximal,
            // if so, see if we should report the turning point.
            double l1 = intervalTrajectoryLength(t, interval);
            double l2 = intervalTrajectoryLength(t, previousInterval);
            if (l1 < l2 && l2 >= Math.sqrt(2) * distance * factor) {
                result.add(previousInterval[0]);
                i = previousInterval[1] - 1;
                previousInterval = new int[]{0, 0};
            } else {
                previousInterval = interval;
            }

        }

        return result;
    }

    private boolean isFree(DistanceMatrix dm, int x, int y) {
        return dm.getPointDistance(x, y) <= distance;
    }

    private double intervalTrajectoryLength(Trajectory t, int[] interval) {
        return new Subtrajectory(t, interval[0], interval[1]).euclideanLength();
    }

}
