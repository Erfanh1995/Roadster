package mapconstruction.algorithms.bundles.graph;

import com.google.common.collect.Lists;
import com.google.common.collect.Range;
import com.google.common.collect.TreeRangeSet;
import mapconstruction.trajectories.Trajectory;

import java.util.*;

/**
 * Labelled graph based on discrete fréchet distance.
 *
 * @author Roel
 */
public class DiscreteFDLabelledGraph extends LabelledGraph {

    /**
     * Range set containing the ranges where the trajecgtories in the
     * concatenated trajectories really exist.
     */
    protected final TreeRangeSet<Integer> existing;
    /**
     * 2D map representing the labelled graph. A cell is not in the map if the
     * vertex at that position does not exist. If a cell exists, the list
     * contains all outgoing edges. These are at most 3, and only go to the left
     * or down.
     * <p>
     * The label indicated the lowest reachable x-coordinate from that cell.
     * Outgoing edges are ordered from top to bottom
     */
    private Map<Integer, Map<Integer, List<LabelledEdge>>> labelledGraph;
    private int graphMin;
    private int graphMax;

    private Range<Integer> iRange;

    public DiscreteFDLabelledGraph(Trajectory concatenated, TreeRangeSet<Integer> existing, double epsilon) {
        super(concatenated, epsilon);
        this.labelledGraph = new HashMap<>();
        this.existing = existing;
        graphMin = 0;
        graphMax = -1;
    }

    @Override
    public OptionalInt findStart(int s, int t, int yt, Collection<Range<Integer>> forbidden) {
        if (!isFree(t, yt)) {
            return OptionalInt.empty();
        }

        int cx = t; // current i-coord (horizontal)
        int cy = yt; // current j coord (vertical)

        while (cx > 0) {
            List<LabelledEdge> edges = labelledGraph.get(cx).get(cy);
            Vertex target = null;

            // Instead of picking the topmost, we pick the bottom most edge as long as possible.
            // This ensures we make the cluster curves as
            // long as possible
            Iterator<LabelledEdge> it;
            int fcy = cy;
            if (forbidden.stream().anyMatch(r -> r.isConnected(Range.closed(fcy - 1, fcy)))) {
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
                return OptionalInt.empty();
            }

            if (target.x() == s) {
                return OptionalInt.of(target.y());
            } else {
                cx = target.x();
                cy = target.y();
            }
        }
        return OptionalInt.of(0);
    }

    @Override
    public void addColumn() {
        int i = ++graphMax;
        HashMap<Integer, List<LabelledEdge>> map = new HashMap<>();
        labelledGraph.put(i, map);
        if (iRange == null || !iRange.contains(i)) {
            iRange = this.existing.rangeContaining(i);
        }

        int starti = Math.max(iRange.lowerEndpoint(), graphMin);
        Range<Integer> jRange = null;
        for (int j = 0; j < getTotalNumPoints(); j++) {
            if (jRange == null || !jRange.contains(j)) {
                jRange = this.existing.rangeContaining(j);
            }

            int startj = jRange.lowerEndpoint();

            if (!isFree(i, j)) {
                // curent cell is not in free space.
                continue;
            }

            ArrayList<LabelledEdge> edges = new ArrayList<>();
            tryAddLeftEdge(i, j, starti, edges);
            tryAddBottomLeftEdge(i, j, starti, startj, edges);
            tryAddBottomEdge(i, j, startj, edges);

            map.put(j, edges);
        }
    }

    /**
     * Creates a labelled edge from (si, sj) to (ti, tj). Tries to find the
     * correct label from the current content of the graph.
     */
    private LabelledEdge createEdge(int si, int sj, int ti, int tj) {
        // get edges of target
        List<LabelledEdge> tedges = labelledGraph.get(ti).get(tj);
        // find smallest label
        int label = tedges.stream()
                .mapToInt(LabelledEdge::getLabel) // get all labels of outgoin edges
                .min() // find maximum
                .orElse(ti); // x coord of target if it has no outgoing edges

        return new LabelledEdge(si, sj, ti, tj, label);
    }

    /**
     * Tries to create the edge to the left of (i, j) and add it to the list of
     * edges.
     *
     * @param i
     * @param j
     * @param boundary
     * @param edges
     */
    private void tryAddLeftEdge(int i, int j, int boundary, List<LabelledEdge> edges) {
        if (i > boundary && isFree(i - 1, j)) {
            edges.add(createEdge(i, j, i - 1, j));
        }
    }

    /**
     * Tries to create the edge to the bottom-left of (i, j) and add it to the
     * list of edges.
     *
     * @param i
     * @param j
     * @param jBoundary
     * @param edges
     */
    private void tryAddBottomLeftEdge(int i, int j, int iBoundary, int jBoundary, List<LabelledEdge> edges) {
        if (i > iBoundary && j > jBoundary && isFree(i - 1, j - 1)) {
            edges.add(createEdge(i, j, i - 1, j - 1));
        }
    }

    /**
     * Tries to create the edge to the bottom of (i, j) and add it to the list
     * of edges.
     *
     * @param i
     * @param j
     * @param boundary
     * @param edges
     */
    private void tryAddBottomEdge(int i, int j, int boundary, List<LabelledEdge> edges) {
        if (j > boundary && isFree(i, j - 1)) {
            edges.add(createEdge(i, j, i, j - 1));
        }
    }

    /**
     * Returns whether the space between the i-th point of the first trajectory,
     * and the j-th point of the second trajectory is part of free space, based
     * on the given distance
     * <p>
     * We must have {@code <= i < getT1.numPoints && 0 <= j < getT2.numPoints}
     *
     * @param i point index of first trajectory
     * @param j point index of second trajectory
     * @return {@code true} if {@code d(T1[i], T2[j]) <= d}
     * @throws IndexOutOfBoundsException
     */
    protected boolean isFree(int i, int j) {
        return dm.getPointDistance(i, j) <= epsilon;
    }

    @Override
    public void removeColumn() {
        int min = graphMin++;
        labelledGraph.remove(min);

        // Remove all outgoing edges of next
        for (List<LabelledEdge> list : labelledGraph.getOrDefault(min + 1, new HashMap<>()).values()) {
            if (list != null) {
                list.clear();
            }
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
