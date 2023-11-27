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
public class SemiWeakFDLabelledGraph extends LabelledGraph {

    /**
     * Range set containing the ranges where the trajectories in the
     * concatenated do not exist, that is, they represent the connecting edges
     * (borders)
     */
    protected Set<Range<Integer>> borders;
    int graphMin;
    int graphMax;
    /**
     * 2D table representing the labelled graph.
     * <p>
     * Vertices of the graph correspond to grid-edges of the free space diagram.
     * The vertices are indexed using intervals as follows.
     * <p>
     * A horizontal grid edge between points i and i + 1 at height j is indexed
     * as ([i, i + 1] , [j, j])
     * <p>
     * A Vertical grid edge between points j and j + 1 at horizontal distance i
     * is indexed as ([i, i] , [j, j + 1])
     * <p>
     * The label indicated the lowest reachable x-coordinate of the
     * leftmost/bottommost point of an edge. Outgoing edges are ordered from top
     * to bottom
     */
    private Table<Range<Integer>, Range<Integer>, List<LabelledEdge>> labelledGraph;

    public SemiWeakFDLabelledGraph(Trajectory concatenated, Set<Range<Integer>> borders, double epsilon) {
        super(concatenated, epsilon);
        labelledGraph = HashBasedTable.create();
        graphMin = 0;
        graphMax = -1;
        this.borders = borders;
    }

    /**
     * {@inheritDoc}
     * <p>
     * To find the cuvre between s and t, we try to find a path from the
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

        while (current.x().lowerEndpoint() >= 0) {
            List<LabelledEdge> edges = labelledGraph.get(current.x(), current.y());
            Vertex target = null;

            // Instead of picking the topmost, we pick the bottom most edge as long as possible.
            // This ensures we make the cluster curves as
            // long as possible
            Iterator<LabelledEdge> it;

            int upy = current.y().upperEndpoint();
            if (forbidden.stream().anyMatch(r -> r.isConnected(Range.closed(upy - 1, upy)))) {
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

            if (isVerticalGridEdge(target) && target.x().lowerEndpoint() == s) {
                int ys = target.y().lowerEndpoint();
                return OptionalInt.of(ys);
            } else {
                current = target;
            }
        }
        return OptionalInt.of(0);
    }

    /**
     * Gets the vertex representing the horizontal grid-edge starting at (x, y)
     *
     * @param x start x-coordinate
     * @param y start y-coordinate
     * @return
     */
    private Vertex horizontalGridEdge(int x, int y) {
        return new Vertex(Range.closed(x, x + 1), Range.singleton(y));
    }

    /**
     * Gets the vertex representing the vertical grid-edge starting at (x, y)
     *
     * @param x start x-coordinate
     * @param y start y-coordinate
     * @return
     */
    private Vertex verticalGridEdge(int x, int y) {
        return new Vertex(Range.singleton(x), Range.closed(y, y + 1));
    }

    /**
     * Returns whether the given vertex represents a horizontal grid edge.
     *
     * @param v
     * @return
     */
    private boolean isHorizontalGridEdge(Vertex v) {
        return isSingleton(v.y());
    }

    /**
     * Returns whether the given vertex represents a Vertical grid edge.
     *
     * @param v
     * @return
     */
    private boolean isVerticalGridEdge(Vertex v) {
        return isSingleton(v.x());
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
            Range<Integer> x = Range.closed(i - 1, i);
            Range<Integer> y = Range.singleton(j);

            if (isFree(x, y)) {
                // curent cell is in free space.
                ArrayList<LabelledEdge> edges = new ArrayList<>();
                tryHorAddLeftEdge(x, y, edges);
                tryHorAddBottomEdge(x, y, edges);

                labelledGraph.put(x, y, edges);
            }

            if (j < getTotalNumPoints() - 1) {
                x = Range.singleton(i);
                y = Range.closed(j, j + 1);
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

    private void tryVertAddLeftEdge(Range<Integer> x, Range<Integer> y, List<LabelledEdge> edges) {

        // Target
        Vertex target = verticalGridEdge(x.lowerEndpoint() - 1, y.lowerEndpoint());

        // Make sure we do not cross boundary, and target must be free
        if (!crossesBorder(Range.closed(x.lowerEndpoint() - 1, x.lowerEndpoint())) && isFree(target)) {
            edges.add(createEdgeToVertical(new Vertex(x, y), target));
        }
    }

    private void tryVertAddBottomEdge(Range<Integer> x, Range<Integer> y, List<LabelledEdge> edges) {
        // Target
        Vertex target = horizontalGridEdge(x.lowerEndpoint() - 1, y.lowerEndpoint());

        // Make sure we do not cross boundary, and target must be free
        if (isFree(target)) {
            Optional<LabelledEdge> opt = createEdgeToHorizontal(new Vertex(x, y), target);

            if (opt.isPresent()) {
                edges.add(opt.get());
            }
        }
    }

    private void tryHorAddLeftEdge(Range<Integer> x, Range<Integer> y, List<LabelledEdge> edges) {
        // Target
        Vertex target = verticalGridEdge(x.lowerEndpoint(), y.lowerEndpoint() - 1);

        // Make sure we do not cross boundary, and target must be free
        if (isFree(target)) {
            edges.add(createEdgeToVertical(new Vertex(x, y), target));
        }
    }

    private void tryHorAddBottomEdge(Range<Integer> x, Range<Integer> y, List<LabelledEdge> edges) {
        // Target

        Vertex target = horizontalGridEdge(x.lowerEndpoint(), y.lowerEndpoint() - 1);

        // Make sure we do not cross boundary, and target must be free
        if (!crossesBorder(Range.closed(y.lowerEndpoint() - 1, y.lowerEndpoint())) && isFree(target)) {
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
                .orElse(target.x().lowerEndpoint()); // x coord of target if it has no outgoing edges

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
        labelledGraph.row(Range.singleton(min)).clear();
        // Remove horizontal
        labelledGraph.row(Range.closed(min - 1, min)).clear();

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
    private boolean isFree(Range<Integer> x, Range<Integer> y) {
        // x range must not cross forbidden boundaries.
        if (crossesBorder(x)) {
            return false;
        }

        // y range must not cross forbidden boundaries.
        if (crossesBorder(y)) {
            return false;
        }

        if (isSingleton(x)) {
            // x is singleton [i,i]
            // This means y is an interval [j, j+1].
            // We compare the ith point to the jth edge
            return dm.getPointEdgeDistance(x.lowerEndpoint(), y.lowerEndpoint()) <= epsilon;
        } else {
            // y is singleton [j,j]
            // This means x is an interval [i, i + 1].
            // We compare the ith edge to the jth point
            return dm.getEdgePointDistance(x.lowerEndpoint(), y.lowerEndpoint()) <= epsilon;
        }

    }

    private boolean isSingleton(Range<Integer> r) {
        return Objects.equals(r.lowerEndpoint(), r.upperEndpoint());
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
     * @param r
     * @return
     */
    private boolean crossesBorder(Range<Integer> r) {
        return this.borders.contains(r);
    }

    /**
     * Helper class representing a vertex in the labelled graph. Position is
     * indicated by two indices.
     */
    private static class Vertex {

        private final Range<Integer> x;
        private final Range<Integer> y;

        public Vertex(Range<Integer> x, Range<Integer> y) {
            this.x = x;
            this.y = y;
        }

        public Range<Integer> x() {
            return x;
        }

        public Range<Integer> y() {
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

        public LabelledEdge(Range<Integer> si, Range<Integer> sj, Range<Integer> ti, Range<Integer> tj, int label) {
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
