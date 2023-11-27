package mapconstruction.GUI.listeners;

/**
 * Interface for listeners to changes in the evolution diagram of the data containing information the RoadNetwork.
 *
 * @author Jorrick Sleijster
 */
public interface NetworkChangeListener {

    /**
     * Signals listener that the roadnetwork has changed.
     *
     * @param evt, containing the new roadnetwork.
     */
    void networkChanged(NetworkChangeEvent evt);
}
