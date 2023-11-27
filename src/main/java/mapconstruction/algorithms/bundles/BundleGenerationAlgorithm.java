package mapconstruction.algorithms.bundles;

import mapconstruction.algorithms.AbstractTrajectoryAlgorithm;
import mapconstruction.trajectories.Bundle;

import java.util.Map;
import java.util.Set;

/**
 * Abstract class for algorithms generating bundles.
 *
 * @author Roel
 */
public abstract class BundleGenerationAlgorithm extends AbstractTrajectoryAlgorithm<Set<Bundle>> {


    /**
     * Whether the direction of the input trajectories should ignores when
     * generating the bundles.
     */
    protected final boolean ignoreDirection;

    protected BundleGenerationAlgorithm(boolean ignoreDirection) {
        super();
        this.ignoreDirection = ignoreDirection;
    }


    public boolean isIgnoreDirection() {
        return ignoreDirection;
    }

    public abstract Map<Bundle, Bundle> getMerges();


}
