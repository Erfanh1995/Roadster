package mapconstruction.algorithms.separation;

import mapconstruction.trajectories.Bundle;

/**
 * @author Roel
 */
public interface SeparabilityComputer {

    /**
     * Computes the separability score of the given bundle.
     *
     * @param bundle
     * @return Seperabilirty score. The meaning of the value depends on the
     * implementing class.
     */
    double computeSeparability(Bundle bundle);

}
