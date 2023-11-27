package mapconstruction.algorithms;

import mapconstruction.trajectories.Trajectory;

import java.util.List;

/**
 * Interface for algorithms that take a collection of
 * trajectories as input, producing some output.
 *
 * @param <R> type of the output.
 * @author Roel
 */
public interface TrajectoryAlgorithm<R> {

    /**
     * Runs the algorithm on the given collection of trajectories.
     *
     * @param trajectories
     * @return
     */
    R run(List<Trajectory> trajectories);

}
