package mapconstruction.GUI.listeners;

/**
 * Interface for listeners to changes in trajectories.
 *
 * @author Roel
 */
public interface TrajectoryChangeListener {

    /**
     * Signals listener that trajectories have changed.
     *
     * @param evt event containing information about the change.
     */
    void trajectoriesChanged(TrajectoryChangeEvent evt);
}
