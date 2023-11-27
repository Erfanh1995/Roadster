package mapconstruction.algorithms;

/**
 * Interface for algorithms that can be aborted.
 *
 * @author Roel
 */
public interface AbortableAlgorithm {

    /**
     * Attempts to abort the algorithm
     */
    void abort();

    /**
     * @return whether the algorithm is aborted.
     */
    boolean isAborted();
}
