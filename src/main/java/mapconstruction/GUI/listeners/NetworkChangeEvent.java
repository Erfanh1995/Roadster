package mapconstruction.GUI.listeners;

import mapconstruction.algorithms.maps.mapping.RoadMap;

/**
 * Event containing information about changed RoadNetworks.
 *
 * @author Jorrick Sleijster
 */
public class NetworkChangeEvent {

    /**
     * Source of the event.
     */
    private final Object source;

    /**
     * Old evolution diagram
     */
    private final RoadMap oldNetwork;

    /**
     * New evolution diagram.
     */
    private final RoadMap newNetwork;


    public NetworkChangeEvent(Object source, RoadMap oldNetwork, RoadMap newNetwork) {
        this.source = source;
        this.oldNetwork = oldNetwork;
        this.newNetwork = newNetwork;
    }

    public Object getSource() {
        return source;
    }

    public RoadMap getOldNetwork() {
        return oldNetwork;
    }

    public RoadMap getNewNetwork() {
        return newNetwork;
    }
}
