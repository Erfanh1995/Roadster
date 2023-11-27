package mapconstruction.algorithms.maps.network;

import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.HashSet;
import java.util.Set;

public class FilteredRoadNetwork extends RoadNetwork {

    private final Set<Integer> validEdges;

    /**
     * Creates an empty road network.
     *
     * @param isDirected whether the network is directed.
     */
    public FilteredRoadNetwork(boolean isDirected) {
        super(isDirected);
        validEdges = new HashSet<>();
    }

    /**
     * Adds the given edge to the graph and add it to the set of 'valid' edges
     * <p>
     * If the network is undirected, also the reverse of the edge is added.
     *
     * @param edge
     * @param valid
     */
    public void addEdge(MapEdge edge, boolean valid) {
        if (valid) {
            validEdges.add(edge.id);
        }
        super.addEdge(edge);
    }

    /**
     * Removes the given edge to the graph. Also removes 'valid' marking if present
     * <p>
     * If the network is undirected, also the reverse of the edge is removed.
     *
     * @param edge
     * @param valid
     */
    public void removeEdge(MapEdge edge, boolean valid) {
        if (valid) {
            validEdges.remove(edge.id);
        }
        super.removeEdge(edge);
    }

    /**
     * Get all valid edges on this road network.
     * @return The set of indices corresponding with valid edges.
     */
    @JsonProperty
    public Set<Integer> getValidEdges() {
        return validEdges;
    }

}
