package mapconstruction.algorithms.maps.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.SetMultimap;

import java.io.Serializable;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * A road network is a geometric graph, consisting of vertices and edges,
 * and each edge as an embedding.
 * <p>
 * The network may be directed or undirected.
 * If it is undirected, addition and removal of edges will
 * also be applied to the reverse of the edges.
 *
 * @author Roel
 */
public class RoadNetwork implements Serializable {

    /**
     * Map mapping a vertex to the adjacent edges, and hence indirectly to the
     * adjacent vertices.
     */
    private final SetMultimap<MapVertex, MapEdge> adjacencyList;

    /**
     * Whether the given graph is directed.
     * This has effect on the addition and removal operation.s
     */
    private final boolean directed;

    /**
     * Creates an empty road network.
     *
     * @param isDirected whether the network is directed.
     */
    public RoadNetwork(boolean isDirected) {
        adjacencyList = HashMultimap.create();
        this.directed = isDirected;
    }

    /**
     * Adds the given edge to the graph.
     * <p>
     * If the network is undirected, also the reverse of the edge is added.
     *
     * @param edge
     */
    public void addEdge(MapEdge edge) {
        adjacencyList.put(edge.getV1(), edge);

        if (!directed) {
            MapEdge reverse = edge.reverse();
            adjacencyList.put(reverse.getV1(), reverse);
        }
    }

    /**
     * Removes the given edge to the graph.
     * <p>
     * If the network is undirected, also the reverse of the edge is removed.
     *
     * @param edge
     */
    public void removeEdge(MapEdge edge) {
        adjacencyList.remove(edge.getV1(), edge);

        if (!directed) {
            MapEdge reverse = edge.reverse();
            adjacencyList.remove(reverse.getV1(), reverse);
        }
    }

    /**
     * Returns an unmodifiable view on the vertices in the network.
     *
     * @return
     */

    @JsonProperty
    public Set<MapVertex> vertices() {
        return Collections.unmodifiableSet(adjacencyList.keySet());
    }

    /**
     * Returns an the edges in the network.
     *
     * @return
     */
    @JsonProperty
    public Set<MapEdge> edges() {
        return new HashSet<>(adjacencyList.values());
    }

    /**
     * Whether the network is directed.
     *
     * @return
     */
    public boolean isDirected() {
        return directed;
    }

    /**
     * Gets the outdegree of the given vertex.
     *
     * @param v
     * @return
     */
    public int getOutDegree(MapVertex v) {
        return adjacencyList.get(v).size();
    }

    /**
     * Gets the outgoing edges of the given vertex.
     *
     * @param v
     * @return
     */
    public Set<MapEdge> getOutEdges(MapVertex v) {
        return Collections.unmodifiableSet(adjacencyList.get(v));
    }

    /**
     * Gets the outgoing neighbours of the given vertex.
     *
     * @param v
     * @return
     */
    public Set<MapVertex> getOutNeighbours(MapVertex v) {
        return getOutEdges(v)
                .stream()
                .map(edge -> edge.getV2())
                .collect(Collectors.toSet());
    }

    @Override
    public String toString() {
        return "RoadNetwork{" + adjacencyList + '}';
    }

    /**
     * Gets the maximum weight in the graph.
     * <p>
     * Negative infinity if there are not edges.
     *
     * @return
     */
    @JsonProperty
    public double getMaxWeight() {
        return edges()
                .stream()
                .mapToDouble(MapEdge::getWeight)
                .filter(w -> !Double.isNaN(w))
                .max()
                .orElse(Double.NEGATIVE_INFINITY);
    }

    /**
     * Gets the maximum weight in the graph.
     * <p>
     * Positive infinity if there are not edges.
     *
     * @return
     */
    @JsonProperty
    public double getMinWeight() {
        return edges()
                .stream()
                .mapToDouble(MapEdge::getWeight)
                .filter(w -> !Double.isNaN(w))
                .min()
                .orElse(Double.POSITIVE_INFINITY);
    }

    /**
     * Gets the avegarge weight in the graph.
     * <p>
     * 0 if there are not edges.
     *
     * @return
     */
    @JsonProperty
    public double getAvgWeight() {
        return edges()
                .stream()
                .mapToDouble(MapEdge::getWeight)
                .filter(w -> !Double.isNaN(w))
                .average()
                .orElse(0);
    }

    /**
     * Gets the standard deviation of the  weights in the graph.
     * <p>
     * 0 if there are not edges.
     *
     * @return
     */
    @JsonProperty
    public double getStdDevWeight() {
        double avg = getAvgWeight();
        double var = edges()
                .stream()
                .mapToDouble(MapEdge::getWeight)
                .filter(w -> !Double.isNaN(w))
                .map(w -> w - avg)
                .map(x -> x * x)
                .average()
                .orElse(0);
        return Math.sqrt(var);
    }


    public String toVertexString() {
        String thisVertex, output = "";

        for (MapVertex v : this.vertices()) {
            thisVertex = String.format("%s,%s,%s\n", Integer.toString(v.getId()), Double.toString(v.getX()), Double.toString(v.getY()));
            output += thisVertex;
        }

        return output;
    }

    public String toEdgeString() {
        String thisEdge, output = "";

        for (MapEdge e : this.edges()) {
            // Format: 0, id vertex 1, id vertex 2, 99999999 // the first and last parameter don't seem to be used, so I put dummy values
            thisEdge = String.format("0,%s,%s,99999999\n", Integer.toString(e.getV1().getId()), Integer.toString(e.getV2().getId()));
            output += thisEdge;
        }

        return output;
    }

}
