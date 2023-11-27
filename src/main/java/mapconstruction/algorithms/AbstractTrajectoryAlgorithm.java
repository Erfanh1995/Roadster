package mapconstruction.algorithms;

import mapconstruction.algorithms.listeners.AlgorithmProgressListener;
import mapconstruction.exceptions.AlgorithmAbortedException;
import mapconstruction.trajectories.Trajectory;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Convenience class for algorithms that transform trajectory by implementing
 * all additional features. This includes aborting the algorithm and setting
 * progress.
 *
 * @author Roel
 */
public abstract class AbstractTrajectoryAlgorithm<R> implements TrajectoryAlgorithm<R>, AbortableAlgorithm, ProgressAlgorithm {

    private final Set<AlgorithmProgressListener> listeners;
    protected volatile boolean aborted;
    protected int progress;

    public AbstractTrajectoryAlgorithm() {
        this.aborted = false;
        listeners = new HashSet<>();

    }

    @Override
    public int getProgress() {
        return progress;
    }

    protected void setProgress(int progress) {
        this.progress = Math.min(99, progress);
        notifyListeners();
    }

    /**
     * {@inheritDoc }
     * <p>
     * The method is implemented as a template method,
     * to properly handle aborting the algorithm.
     * <p>
     * Subclasses must overrride the protected
     * runAlgorithm to add functionality.
     *
     * @param trajectories
     * @return
     */
    @Override
    public final R run(List<Trajectory> trajectories) {
        aborted = false;
        return runAlgorithm(trajectories);
    }

    protected abstract R runAlgorithm(List<Trajectory> trajectories);


    /**
     * Attempts to abort the algorithm
     */
    @Override
    public void abort() {
        aborted = true;
    }

    /**
     * Determines whether the algorithm is aborted.
     *
     * @return whether the algorithm is aborted.
     */
    @Override
    public boolean isAborted() {
        return aborted;
    }

    /**
     * Checks whether the algorithm should abort, and throws an exception if
     * that is the case
     */
    protected void checkAbort() throws AlgorithmAbortedException {
        if (isAborted()) {
            throw new AlgorithmAbortedException("Algorithm aborted");
        }
    }

    @Override
    public void notifyListeners() {
        listeners.forEach(AlgorithmProgressListener::progressChanged);
    }

    @Override
    public void addListener(AlgorithmProgressListener listener) {
        listeners.add(listener);
    }

}
