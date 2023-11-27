package mapconstruction.algorithms.diagram;

import com.google.common.collect.BiMap;
import mapconstruction.trajectories.Bundle;

import java.io.Serializable;
import java.util.Map;
import java.util.Set;

/**
 * State of the diagram for a given epsilon. Keeps track of: - The set of bundle
 * "classes" that are alive - Bundles that have merged with another bundle -
 * Moments on which the alive classes were born.
 *
 * @author Roel
 */
class DiagramState implements Serializable {
    private static final long serialVersionUID = 1L;
    /**
     * Bundle with associated class number. Each present bundle represents one
     * class.
     */
    private final BiMap<Bundle, Integer> bundleClasses;

    /**
     * The classes born this state
     */
    private final Set<Integer> births;

    /**
     * Mor each merged old class, the class it has merged into.
     */
    private final Map<Integer, Integer> merges;


    /**
     * Creates a new state with the given data
     *
     * @param bundleClasses
     * @param births
     * @param merges
     */
    public DiagramState(BiMap<Bundle, Integer> bundleClasses, Set<Integer> births, Map<Integer, Integer> merges) {
        this.bundleClasses = bundleClasses;
        this.births = births;
        this.merges = merges;
    }


    @Override
    public String toString() {
        return "DiagramState{" + "bundleClasses=" + bundleClasses + ", births=" + births + '}';
    }

    public BiMap<Bundle, Integer> getBundleClasses() {
        return bundleClasses;
    }

    public Set<Integer> getBirths() {
        return births;
    }

    public Map<Integer, Integer> getMerges() {
        return merges;
    }

}
