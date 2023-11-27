package mapconstruction.GUI.listeners;

import com.google.common.collect.BiMap;
import mapconstruction.trajectories.Bundle;

import java.util.Collections;
import java.util.Set;

/**
 * Information about changed bundles
 *
 * @author Roel
 */
public class BundleChangeEvent {
    /**
     * Source of the event.
     */
    private final Object source;

    /**
     * Bimap mapping bundles to their classes.
     */
    private final BiMap<Bundle, Integer> bundlesWithClasses;

    public BundleChangeEvent(Object source, BiMap<Bundle, Integer> bundlesWithClasses) {
        this.source = source;
        this.bundlesWithClasses = bundlesWithClasses;
    }

    public Object getSource() {
        return source;
    }

    public BiMap<Bundle, Integer> getBundlesWithClasses() {
        return bundlesWithClasses;
    }

    public Set<Bundle> getBundles() {
        return Collections.unmodifiableSet(bundlesWithClasses.keySet());
    }

    public Set<Integer> getClasses() {
        return Collections.unmodifiableSet(bundlesWithClasses.inverse().keySet());
    }

}
