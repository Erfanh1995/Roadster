package mapconstruction.GUI.listeners;

/**
 * Interface for listeners to changes in trajectories of the data storage.
 *
 * @author Roel
 */
public interface BundleChangeListener {

    /**
     * Notification that the bundles that should be displayed in a view have changed.
     *
     * @param evt event with information about the added and removed displayed bundles.
     */
    default void displayedBundlesChanged(BundleChangeEvent evt) {

    }

    /**
     * Notification that the complete list of bundles has changed.
     *
     * @param evt event with information about the added and removed bundles.
     */
    default void bundlesChanged(BundleChangeEvent evt) {

    }


}
