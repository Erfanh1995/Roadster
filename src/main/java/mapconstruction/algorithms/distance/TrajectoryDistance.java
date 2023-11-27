package mapconstruction.algorithms.distance;

import mapconstruction.trajectories.Trajectory;

/**
 * Interface for ways to compute the distance between two trajectories
 */
public interface TrajectoryDistance {

    /**
     * Computes the distance between the two trajectories.
     *
     * @param t1 first trajectory
     * @param t2 second trajectory
     * @return distance between the trajectory. Which distance depends in the implementing class.
     */
    double compute(Trajectory t1, Trajectory t2);

}
