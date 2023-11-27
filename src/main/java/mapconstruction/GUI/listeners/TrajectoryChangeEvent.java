package mapconstruction.GUI.listeners;

import mapconstruction.trajectories.Trajectory;

import java.util.List;

/**
 * Event with information about changed trajectories.
 *
 * @author Roel
 */
public class TrajectoryChangeEvent {

    /**
     * Source of the event.
     */
    private final Object source;

    /**
     * Current list of trajectories.
     */
    private final List<Trajectory> trajectories;

    /**
     * List of added trajectories.
     */
    private final List<Trajectory> added;


    /**
     * List of removed trajectories..
     */
    private final List<Trajectory> removed;

    public TrajectoryChangeEvent(Object source, List<Trajectory> trajectories, List<Trajectory> added, List<Trajectory> removed) {
        this.source = source;
        this.trajectories = trajectories;
        this.added = added;
        this.removed = removed;
    }

    public Object getSource() {
        return source;
    }

    public List<Trajectory> getTrajectories() {
        return trajectories;
    }

    public List<Trajectory> getAdded() {
        return added;
    }

    public List<Trajectory> getRemoved() {
        return removed;
    }


}
