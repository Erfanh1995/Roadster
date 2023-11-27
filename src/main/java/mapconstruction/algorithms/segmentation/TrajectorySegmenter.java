package mapconstruction.algorithms.segmentation;

import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.util.List;

/**
 * Interface for algorithms segmenting a trajectory.
 *
 * @author Roel
 */
public interface TrajectorySegmenter {

    /**
     * Segments the given trajectory.
     * <p>
     * Returns a list of subtrajectories representing the found segments.
     *
     * @param original
     * @return
     */
    List<Subtrajectory> segment(Trajectory original);

}
