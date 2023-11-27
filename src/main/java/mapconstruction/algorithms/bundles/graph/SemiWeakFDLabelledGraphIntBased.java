package mapconstruction.algorithms.bundles.graph;

import com.google.common.collect.HashBasedTable;
import com.google.common.collect.Lists;
import com.google.common.collect.Range;
import com.google.common.collect.Table;
import mapconstruction.trajectories.Trajectory;

import java.util.*;

/**
 * Labelled graph based on the Semi-weak frechet distance.
 * <p>
 * Instead of taking grid points of the free-space diagram as vertices of the
 * graph, we take the grid edges as vertices.
 * <p>
 * An edge at horizontal interval [i, i+1] at height j is part of the free space
 * if the distance between the edge (i, i + 1) and point j is at most epsilon.
 * <p>
 * Similarly for a vertical interval [j, j + 1] and horizontal distance i.
 *
 * @author Roel
 */
public class SemiWeakFDLabelledGraphIntBased extends LabelledGraph {

    /**
     * 2D table representing the labelled graph.
     * <p>
     * Vertices of the graph correspond to grid-edges of the free space diagram.
     * The vertices are indexed using integers as follows
     * <p>
     * Coordinates indicating vertices are indexed using even numbers,
     * coordinates indicating edges are indexed using odd numbers.
     * <p>
     * A horizontal grid edge between points i and i + 1 at height j is indexed
     * as (i * 2 + 1, j * 2)
     * <p>
     * A Vertical grid edge between points j and j + 1 at horizontal distance i
     * is indexed as (i * 2 , j * 2 + )
     * <p>
     * The label indicated the lowest reachable x-coordinate of the
     * leftmost/bottommost point of an edge. Outgoing edges are ordered from top
     * to bottom
     */
    private final Table<Integer, Integer, List<LabelledEdge>> labelledGraph;

    /**
     * Range set containing the start og ranges where the trajectories in the
     * concatenated do not exist, that is, they represent the connecting edges
     * (borders)
     */
    //protected Set<Integer> borders;

    /**
     * Array storing for each potential start coordinate of an edge,
     * whether that edge is a border.
     */
    private final boolean[] borderCheck;

    int graphMin;
    int graphMax;

    public SemiWeakFDLabelledGraphIntBased(Trajectory concatenated, Set<Range<Integer>> borders, double epsilon) {
        super(concatenated, epsilon);
        labelledGraph = HashBasedTable.create();
        graphMin = 0;
        graphMax = -1;
        borderCheck = new boolean[concatenated.numPoints()];
        borders.stream().mapToInt(r -> r.lowerEndpoint())
                .filter(i -> i >= 0)
                .forEach(i -> borderCheck[i] = true);
        //this.borders = borders.stream().map(r -> r.lowerEndpoint()).collect(Collectors.toCollection(() -> Sets.newHashSetWithExpectedSize(borders.size())));
    }

    /**
     * {@inheritDoc}
     * <p>
     * To find the curve between s and t, we try to find a path from the
     * vertical edge ending at (t, yt) to a vertical edge on s.
     * <p>
     * This is to make sure the end points of the representativeSubtrajectory are matched.
     */
    @Override
    public OptionalInt findStart(int s, int t, int yt, Collection<Range<Integer>> forbidden) {
        // Current vertex of the graph, the topmost edge ending at (t, yt)
        Vertex current = verticalGridEdge(t, yt - 1);

        if (!isFree(current)) {
            // ending edge is not free
            return OptionalInt.empty();
        }

        while (low(current.x()) >= 0) {
            List<LabelledEdge> edges = labelledGraph.get(current.x(), current.y());
            Vertex target = null;

            // Instead of picking the topmost, we pick the bottom most edge as long as possible.
            // This ensures we make the cluster curves as
            // long as possible
            Iterator<LabelledEdge> it;

            int upy = up(current.y());
            if (forbidden.stream().anyMatch(r -> r.lowerEndpoint() <= upy && upy <= r.upperEndpoint() + 1)) {
                it = edges.iterator();
            } else {
                it = Lists.reverse(edges).iterator();
            }
            while (it.hasNext()) {
                LabelledEdge e = it.next();
                if (e.getLabel() <= s) {
                    target = e.getTarget();
                    break;
                }
            }
            if (target == null) {
                // dead end.
                return OptionalInt.empty();
            }

            if (isVerticalGridEdge(target) && low(target.x()) == s) {
                int ys = low(target.y());
                return OptionalInt.of(ys);
            } else {
                current = target;
            }
        }
        return OptionalInt.of(0);
    }

    /**
     * Checks whether the given number is even, bu checking whether the last bit
     * if the number is 0.
     *
     * @param num
     * @return
     */
    private boolean isEven(int num) {
        return (num & 1) == 0;
    }

    /**
     * Gets the coord-th edge coordinate.
     *
     * @param coord
     * @return
     */
    private int edgeGraphCoord(int coord) {
        return 2 * coord + 1;
    }

    /**
     * Gets the coord-th grid-point coordinate.
     *
     * @param coord
     * @return
     */
    private int vertexGraphCoord(int coord) {
        return 2 * coord;
    }

    /**
     * Gets the lower position coordinate of the given graph coordinate
     *
     * @param graphCoord
     * @return
     */
    private int low(int graphCoord) {
        if (graphCoord < 0) {
            return (graphCoord - 1) / 2;
        } else {
            return graphCoord / 2;
        }

    }

    /**
     * Gets the upper position coordinate of the given graph coordinate
     *
     * @param graphCoord
     * @return
     */
    private int up(int graphCoord) {
        if (graphCoord < 0) {
            return graphCoord / 2;
        } else {
            return (graphCoord + 1) / 2;
        }

    }

    /**
     * Gets the vertex representing the horizontal grid-edge starting at (x, y)
     *
     * @param x start x-coordinate
     * @param y start y-coordinate
     * @return
     */
    private Vertex horizontalGridEdge(int x, int y) {
        return new Vertex(edgeGraphCoord(x), vertexGraphCoord(y));
    }

    /**
     * Gets the vertex representing the vertical grid-edge starting at (x, y)
     *
     * @param x start x-coordinate
     * @param y start y-coordinate
     * @return
     */
    private Vertex verticalGridEdge(int x, int y) {
        return new Vertex(vertexGraphCoord(x), edgeGraphCoord(y));
    }

    /**
     * Returns whether the given vertex represents a horizontal grid edge.
     *
     * @param v
     * @return
     */
    private boolean isHorizontalGridEdge(Vertex v) {
        return isEven(v.y());
    }

    /**
     * Returns whether the given vertex represents a Vertical grid edge.
     *
     * @param v
     * @return
     */
    private boolean isVerticalGridEdge(Vertex v) {
        return isEven(v.x());
    }

    @Override
    public void addColumn() {
        /*
        When adding a column (of the free space), we have to do two things:
        - add entries for vertical edges in the column.
        - add entries for horizontal edges ending in this column. (except first column)
         */
        int i = ++graphMax;

        for (int j = 0; j < getTotalNumPoints(); j++) {
            // Handle horizontal edges
            int x = edgeGraphCoord(i - 1);
            int y = vertexGraphCoord(j);
            if (isFree(x, y)) {

                // curent cell is in free space.
                ArrayList<LabelledEdge> edges = new ArrayList<>();
                tryHorAddLeftEdge(x, y, edges);
                tryHorAddBottomEdge(x, y, edges);

                labelledGraph.put(x, y, edges);
            }

            if (j < getTotalNumPoints() - 1) {
                x = vertexGraphCoord(i);
                y = edgeGraphCoord(j);
                if (isFree(x, y)) {
                    // curent cell isin free space.
                    ArrayList<LabelledEdge> edges = new ArrayList<>();
                    tryVertAddLeftEdge(x, y, edges);
                    tryVertAddBottomEdge(x, y, edges);

                    labelledGraph.put(x, y, edges);
                }

            }

        }

    }

    private void tryVertAddLeftEdge(int x, int y, List<LabelledEdge> edges) {

        // Target
        Vertex target = verticalGridEdge(low(x) - 1, low(y));
        // Make sure we do not cross boundary, and target must be free
        if (!crossesBorder(edgeGraphCoord(low(x) - 1)) && isFree(target)) {
            edges.add(createEdgeToVertical(new Vertex(x, y), target));
        }
    }

    private void tryVertAddBottomEdge(int x, int y, List<LabelledEdge> edges) {
        // Target
        Vertex target = horizontalGridEdge(low(x) - 1, low(y));

        // Make sure we do not cross boundary, and target must be free
        if (isFree(target)) {
            Optional<LabelledEdge> opt = createEdgeToHorizontal(new Vertex(x, y), target);

            if (opt.isPresent()) {
                edges.add(opt.get());
            }
        }
    }

    private void tryHorAddLeftEdge(int x, int y, List<LabelledEdge> edges) {
        // Target
        Vertex target = verticalGridEdge(low(x), low(y) - 1);

        // Make sure we do not cross boundary, and target must be free
        if (isFree(target)) {
            edges.add(createEdgeToVertical(new Vertex(x, y), target));
        }
    }

    private void tryHorAddBottomEdge(int x, int y, List<LabelledEdge> edges) {
        // Target

        Vertex target = horizontalGridEdge(low(x), low(y) - 1);

        // Make sure we do not cross boundary, and target must be free
        if (!crossesBorder(edgeGraphCoord(low(y) - 1)) && isFree(target)) {
            Optional<LabelledEdge> opt = createEdgeToHorizontal(new Vertex(x, y), target);

            if (opt.isPresent()) {
                edges.add(opt.get());
            }
        }
    }

    /**
     * Creates a labelled edge from source to target, where target is a vertex
     * representing a vertical grid edge.
     */
    private LabelledEdge createEdgeToVertical(Vertex source, Vertex target) {
        // get edges of target
        List<LabelledEdge> tedges = labelledGraph.get(target.x(), target.y());
        // find smallest label
        int label = tedges.stream()
                .mapToInt(LabelledEdge::getLabel) // get all labels of outgoin edges
                .min() // find maximum
                .orElse(low(target.x())); // x coord of target if it has no outgoing edges

        return new LabelledEdge(source, target, label);
    }

    /**
     * Creates a labelled edge from source to target, where target is a vertex
     * representing a horizontal grid edge.
     */
    private Optional<LabelledEdge> createEdgeToHorizontal(Vertex source, Vertex target) {
        // get edges of target
        List<LabelledEdge> tedges = labelledGraph.get(target.x(), target.y());
        // find smallest label
        OptionalInt label = tedges.stream()
                .mapToInt(LabelledEdge::getLabel) // get all labels of outgoin edges
                .min(); // find minimum

        if (!label.isPresent()) {
            // target has no outgoing edges. Do not add edge to prevent getting stuck
            return Optional.empty();
        }

        return Optional.of(new LabelledEdge(source, target, label.getAsInt()));
    }

    @Override
    public void removeColumn() {
        /*
        When we remove a column, we need to remove the vertical edges and
        horizontal edges. ending in the column
         */

        int min = graphMin++;
        // Remove vertical
        labelledGraph.row(vertexGraphCoord(min)).clear();
        // Remove horizontal
        labelledGraph.row(edgeGraphCoord(min - 1)).clear();

    }

    /**
     * Whether the given vertex in the labeled graph (edge in free space) is
     * free.
     *
     * @param v
     * @return
     */
    private boolean isFree(Vertex v) {
        return isFree(v.x(), v.y());
    }

    /**
     * Whether the given vertex given by two coordinates is free.
     */
    private boolean isFree(int x, int y) {

        // x range must not cross forbidden boundaries.
        if (crossesBorder(x)) {
            return false;
        }

        // y range must not cross forbidden boundaries.
        if (crossesBorder(y)) {
            return false;
        }

        if (isEven(x)) {

            // x is singleton [i,i]
            // This means y is an interval [j, j+1].
            // We compare the ith point to the jth edge
            return dm.getPointEdgeDistance(low(x), low(y)) <= epsilon;
        } else {
            // y is singleton [j,j]
            // This means x is an interval [i, i + 1].
            // We compare the ith edge to the jth point
            return dm.getEdgePointDistance(low(x), low(y)) <= epsilon;
        }

    }

    /**
     * Whether the given gridpoint in the free space diagram is free.
     *
     * @param x
     * @param y
     * @return
     */
    private boolean isPointFree(int x, int y) {
        return dm.getPointDistance(x, y) <= epsilon;
    }

    /**
     * Checks whether the given interval crosses an invalid border.
     *
     * @param coord coordinate
     * @return
     */
    private boolean crossesBorder(int coord) {
        if (isEven(coord)) {
            return false; // not an edge coord
        } else {
            int low = low(coord);
            return low == -1 || borderCheck[low];
        }
    }

    /**
     * Helper class representing a vertex in the labelled graph. Position is
     * indicated by two indices.
     */
    private static class Vertex {

        private final int x;
        private final int y;

        public Vertex(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public int x() {
            return x;
        }

        public int y() {
            return y;
        }

        @Override
        public String toString() {
            return "Vertex{" + "x=" + x + ", y=" + y + '}';
        }

    }

    /**
     * Helper class representing a n edge in the labelled graph.
     */
    private static class LabelledEdge {

        private final Vertex source;
        private final Vertex target;
        private final int label;

        public LabelledEdge(Vertex source, Vertex target, int label) {
            this.source = source;
            this.target = target;
            this.label = label;
        }

        public LabelledEdge(int si, int sj, int ti, int tj, int label) {
            this.source = new Vertex(si, sj);
            this.target = new Vertex(ti, tj);
            this.label = label;
        }

        public Vertex getSource() {
            return source;
        }

        public Vertex getTarget() {
            return target;
        }

        public int getLabel() {
            return label;
        }

        @Override
        public String toString() {
            return "LabelledEdge{" + "source=" + source + ", target=" + target + ", label=" + label + '}';
        }

    }

}
