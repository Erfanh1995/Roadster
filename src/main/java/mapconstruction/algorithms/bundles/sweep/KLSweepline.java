package mapconstruction.algorithms.bundles.sweep;

import mapconstruction.algorithms.bundles.graph.GeneratingSemiWeakFDLabelledGraph;
import mapconstruction.algorithms.bundles.graph.representation.Event;
import mapconstruction.trajectories.Bundle;

import java.util.Set;

public abstract class KLSweepline {

    protected GeneratingSemiWeakFDLabelledGraph freeSpace;

    KLSweepline(GeneratingSemiWeakFDLabelledGraph freeSpace) {
        this.freeSpace = freeSpace;
    }

    /**
     * Initialize the sweepline algorithm. Generally, this includes generating a {@link java.util.Queue} of
     * {@link Event}s on the given free space.
     */
    public abstract void initialize();

    /**
     * Sweep over the previously computed events and process these in order to compute the desired result, which is a
     * set of Maximal (k,l)-Subbundles for an epsilon as specified in the free space.
     *
     * @return A set of Maximal (k,l)-Subbundles.
     */
    public abstract Set<Bundle> sweep();

}
