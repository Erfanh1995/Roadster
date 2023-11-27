package mapconstruction.algorithms;

import mapconstruction.algorithms.listeners.AlgorithmProgressListener;

/**
 * Interface for algorithms that can report their progress to observers.
 *
 * @author Roel
 */
public interface ProgressAlgorithm {

    /**
     * Returns (an estimate of) the current progress of the algorithm
     *
     * @return The progress of the algorithm as a value from 0 to 100.
     */
    int getProgress();

    /**
     * Notify listeners about a change in progress.
     */
    void notifyListeners();

    void addListener(AlgorithmProgressListener listener);
}
