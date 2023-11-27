package mapconstruction.algorithms.simplification;

import mapconstruction.trajectories.SimplifiedTrajectory;
import mapconstruction.trajectories.Trajectory;

/**
 * Interface for algorithms that simplify trajectories.
 *
 * @author Roel
 */
public interface TrajectorySimplifier {

    /**
     * Simplifies the given trajectory using the given error.
     * <p>
     * Returns a simplified view on the original trajectory.
     *
     * @param original
     * @param error
     * @return
     */
    SimplifiedTrajectory simplify(Trajectory original, double error);

}
