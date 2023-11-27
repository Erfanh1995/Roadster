package mapconstruction.algorithms.bundles.graph;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashBasedTable;
import com.google.common.collect.Range;
import com.google.common.collect.Table;
import mapconstruction.algorithms.bundles.graph.representation.Event;
import mapconstruction.algorithms.bundles.graph.representation.LabelledEdge;
import mapconstruction.algorithms.bundles.graph.representation.Vertex;
import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.benchmark.Benchmark;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.security.InvalidParameterException;
import java.util.*;
import java.util.List;

/**
 * Labelled graph representation for the Semi-Weak (Vertex Monotone) Fréchet distance.
 * <p>
 * Heavily inspired by the {@link SemiWeakFDLabelledGraphIntBased} implementation, however due to the different approach
 * where we compare only a single representative trajectory against multiple concatenated trajectories this implementation
 * doesn't strictly adhere to the previously used interface.
 * <p>
 * Additionally this representation provides means to collect a queue of sweepline events after having discovered the
 * (partial) labelled graph.
 *
 * @author Jorren
 */
public class GeneratingSemiWeakFDLabelledGraph /*extends LabelledGraph*/ implements EventGenerator<Event> {

    double epsilon;
    Trajectory representative;
//    private List<Trajectory> concatenated;
TreeMap<Integer, Trajectory> concatenated;

    final Table<Integer, Integer, List<LabelledEdge>> labelledGraph;
    private DistanceMatrix dm;

    private Set<Vertex> endpoints;

    public GeneratingSemiWeakFDLabelledGraph(double epsilon, Trajectory representative, List<Trajectory> concatenated) {
        this.epsilon = epsilon;
        this.representative = representative;
        // Build a concatenated trajectory and a mapping from index to original trajectory
        this.concatenated = new TreeMap<>();
        List<Point2D> points = new ArrayList<>();
        for (Trajectory t : concatenated) {
            this.concatenated.put(points.size(), t);
            points.addAll(t.points());
        }

        dm = new DistanceMatrix(representative, new FullTrajectory(points));
        labelledGraph = HashBasedTable.create();

        endpoints = new HashSet<>();
    }

    public OptionalInt findStart(int s, int t, int yt, Collection<Range<Integer>> forbidden) {
        return null;
    }

    public void addColumn() {
        Benchmark.push("addColumn");
        for (int i = 1; i < representative.numPoints(); i++) {
            for (int J : concatenated.keySet()) {
                Trajectory t = concatenated.get(J);
                for (int j = J + 1; j < J + t.numPoints(); j++) {
                    // try to add edges from a vertical segment
                    int x = vertexGraphCoord(i);
                    int y = edgeGraphCoord(j - 1);
                    if (isFree(x, y)) {
                        ArrayList<LabelledEdge> edges = new ArrayList<>();
                        tryVertAddLeftEdge(x, y, edges);
                        tryVertAddBottomEdge(x, y, edges);
                        labelledGraph.put(x, y, edges);
                    }

                    // don't try to add edges to the 'top' of the diagram
                    if (j == J + t.numEdges()) continue;

                    // try to add edges from a horizontal segment
                    x = edgeGraphCoord(i - 1);
                    y = vertexGraphCoord(j);
                    if (isFree(x, y)) {
                        ArrayList<LabelledEdge> edges = new ArrayList<>();
                        tryHorAddLeftEdge(x, y, edges);
                        tryHorAddBottomEdge(x, y, edges);
                        labelledGraph.put(x, y, edges);
                    }
                }
            }
        }
        Benchmark.pop();
    }

    public void removeColumn() {

    }

    @Override
    public SortedSet<Event> collectEvents() {
        Map<Vertex, Set<Vertex>> events = new HashMap<>();
        SortedSet<Event> result = new TreeSet<>();

        for (Vertex t : endpoints) {
            t = findVerticalEdge(t); // find first vertical edge in path
            if (t == null) continue;

            List<LabelledEdge> edges = labelledGraph.get(t.x(), t.y());
            // only add longest
            if (edges != null) {
                Vertex b = t.toSubtrajectoryEnd();
                edges.stream().min(Comparator.comparingInt(LabelledEdge::getLabel)).ifPresent(edge -> {
                    Vertex a = edge.getOrigin().toSubtrajectoryStart();
                    result.add(new Event(a, b, true));
                    result.add(new Event(a, b, false));
                });
            }

//            if (edges != null && !edges.isEmpty()) {
//                Vertex b = t.toSubtrajectoryEnd();
//                for (LabelledEdge edge : edges) {
//                    Vertex a = edge.getOrigin().toSubtrajectoryStart();
//                    result.add(new Event(a, b, true));
//                    result.add(new Event(a, b, false));
//                }
//            }
        }
        return result;
    }

    public String getGraph() {
        StringBuilder out = new StringBuilder("[\n");
        for (Table.Cell<Integer,Integer,List<LabelledEdge>> cell : this.labelledGraph.cellSet()) {
            if (!cell.getValue().isEmpty()) {
                out.append("(").append(cell.getColumnKey()).append(",").append(cell.getRowKey()).append("): ");;
                for (LabelledEdge edge : cell.getValue()) {
                    out.append(edge.getTarget()).append(" -> ").append(edge.getSource()).append(", ");
                }
                out.append("\n");
            }

        }
        out.append("]");

        return out.toString();
    }

    private Vertex findVerticalEdge(Vertex t) {
        while (!isVerticalGridEdge(t)) {
            List<LabelledEdge> edges = labelledGraph.get(t.x(), t.y());
            if (!edges.isEmpty()) {
                t = edges.get(0).getTarget();
            } else {
                return null;
            }
        }
        return t;
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
    int edgeGraphCoord(int coord) {
        return 2 * coord + 1;
    }

    /**
     * Gets the coord-th grid-point coordinate.
     *
     * @param coord
     * @return
     */
    int vertexGraphCoord(int coord) {
        return 2 * coord;
    }

    /**
     * Gets the lower position coordinate of the given graph coordinate
     *
     * @param graphCoord
     * @return
     */
    int low(int graphCoord) {
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
    Vertex horizontalGridEdge(int x, int y) {
        return new Vertex(edgeGraphCoord(x), vertexGraphCoord(y));
    }

    /**
     * Gets the vertex representing the vertical grid-edge starting at (x, y)
     *
     * @param x start x-coordinate
     * @param y start y-coordinate
     * @return
     */
    Vertex verticalGridEdge(int x, int y) {
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

    /**
     * Whether the given vertex in the labeled graph (edge in free space) is
     * free.
     *
     * @param v
     * @return
     */
    boolean isFree(Vertex v) {
        return isFree(v.x(), v.y());
    }

    /**
     * Whether the given vertex given by two coordinates is free.
     */
    boolean isFree(int x, int y) {
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

    void tryVertAddLeftEdge(int x, int y, List<LabelledEdge> edges) {
        Vertex target = verticalGridEdge(low(x) - 1, low(y));
        // Check whether there is a free path from (x,y) to target.
        if (isFree(target)) {
            edges.add(createEdgeToVertical(new Vertex(x, y), target));
        }
    }

    void tryVertAddBottomEdge(int x, int y, List<LabelledEdge> edges) {
        Vertex target = horizontalGridEdge(low(x) - 1, low(y));
        // Check whether there is a free path from (x,y) to target.
        if (isFree(target)) {
            Optional<LabelledEdge> opt = createEdgeToHorizontal(new Vertex(x, y), target);
            opt.ifPresent(edges::add);
        }
    }

    void tryHorAddLeftEdge(int x, int y, List<LabelledEdge> edges) {
        Vertex target = verticalGridEdge(low(x), low(y) - 1);
        // Check whether there is a free path from (x,y) to target.
        if (isFree(target)) {
            edges.add(createEdgeToVertical(new Vertex(x, y), target));
        }
    }

    void tryHorAddBottomEdge(int x, int y, List<LabelledEdge> edges) {
        Vertex target = horizontalGridEdge(low(x), low(y) - 1);
        // Check whether there is a free path from (x,y) to target.
        if (isFree(target)) {
            Optional<LabelledEdge> opt = createEdgeToHorizontal(new Vertex(x, y), target);
            opt.ifPresent(edges::add);
        }
    }

    /**
     * Creates a labelled edge from source to target, where target is a vertex
     * representing a vertical grid edge.
     */
    private LabelledEdge createEdgeToVertical(Vertex source, Vertex target) {
        // get edges of target
        List<LabelledEdge> tedges = labelledGraph.get(target.x(), target.y());
        if (tedges == null) tedges = new ArrayList<>();

        // Find the labelled edge with the 'maximum' (smallest x) label.
        Optional<LabelledEdge> min = tedges.stream().min(Comparator.comparingInt(LabelledEdge::getLabel));
        // if no such edge exists, make target the new origin.
        int label = min.map(LabelledEdge::getLabel).orElse(low(target.x()));
        Vertex origin = min.map(LabelledEdge::getOrigin).orElse(target);

        // update endpoints
        endpoints.remove(target);
        endpoints.add(source);

        return new LabelledEdge(source, target, label, origin);
    }

    /**
     * Creates a labelled edge from source to target, where target is a vertex
     * representing a horizontal grid edge.
     */
    private Optional<LabelledEdge> createEdgeToHorizontal(Vertex source, Vertex target) {
        // get edges of target
        List<LabelledEdge> tedges = labelledGraph.get(target.x(), target.y());
        if (tedges == null) tedges = new ArrayList<>();

        Optional<LabelledEdge> min = tedges.stream().min(Comparator.comparingInt(LabelledEdge::getLabel));
        if (!min.isPresent()) {
            // target has no outgoing edges. Do not add edge to prevent dead ends.
            return Optional.empty();
        }

        // update endpoints
        endpoints.remove(target);
        endpoints.add(source);

        return Optional.of(new LabelledEdge(source, target, min.get().getLabel(), min.get().getOrigin()));
    }

    public Trajectory getRepresentative() {
        return representative;
    }

    public Trajectory getTrajectory(Event e) {
        Map.Entry<Integer, Trajectory> entry = concatenated.floorEntry(e.getHeight());
        return entry == null ? null : entry.getValue();
    }

    public Subtrajectory getSubTrajectory(int from, int to) {
        Map.Entry<Integer, Trajectory> entry = concatenated.floorEntry(from);
        if (entry != null) {
            int low = entry.getKey();
            Trajectory trajectory = entry.getValue();
            if (trajectory.numEdges() >= to - low) {
                return new Subtrajectory(trajectory, from - low, to - low);
            }
        }
        throw new InvalidParameterException("The subtrajectory interval must match a valid Trajectory.");
    }

    public Subtrajectory getTrimmedSubTrajectory(int from, int to, Vertex t) {
        if (t.x() < to) return null;
        from = vertexGraphCoord(from);
        to = vertexGraphCoord(to);
        Vertex start = backpass(verticalGridEdge(t.x(), t.y()-1), from, to);
        if (start == null) return null;
        Vertex end = backpass(start, from, from);
        if (end == null) return null;

        Map.Entry<Integer,Trajectory> p = concatenated.floorEntry(t.y());
        double[] bounds = computeSubtrajectoryBounds(low(end.x()), low(start.x()), low(end.y()), up(start.y()));
        return new Subtrajectory(p.getValue(), bounds[0] - p.getKey(), bounds[1] - p.getKey());
    }

    private Vertex backpass(Vertex start, int lower, int higher) {
        while (start.x() > higher) {
            Vertex target = null;
            List<LabelledEdge> edges = labelledGraph.get(start.x(), start.y());
            if (edges == null) {
                return null;
            }
            for (LabelledEdge edge : edges) {
                if (edge.getOrigin().x() <= lower) {
                    target = edge.getTarget();
                    break;
                }
            }

            if (target == null) {
                return null;
            }
            start = target;
        }
        return start;
    }

    /**
     * Computes the subtrajectory bounds of the found cluster curve.
     * <p>
     * [s,t] indicates the representativeSubtrajectory [ys, yt] indicates the found cluster
     * curve
     * <p>
     * These input and returned indices are relative to the concatenated
     * trajectory.
     * <p>
     * If allowPartialBounds is false, simply ys and yt are returned. Otherwise,
     * partial bounds are computed, based on the first and last edges of the
     * curves and eps.
     *
     * @param s
     * @param t
     * @param ys
     * @param yt
     * @return
     */
    private double[] computeSubtrajectoryBounds(int s, int t, int ys, int yt) {
        // first edge
        double from;
        if (dm.getPointDistance(s, ys) <= epsilon) {
            from = ys;
        } else {
            // need to create a partial edge
            List<Double> startparams = GeometryUtil.segCircIntersectionParams(getEdge(ys), getPoint(s), epsilon);
            from = ys + startparams.get(0); // + Collections.min(startparams);
        }

        // last edge
        double to;
        if (dm.getPointDistance(t, yt) <= epsilon) {
            to = yt;
        } else {
            List<Double> endparams = GeometryUtil.segCircIntersectionParams(getEdge(yt - 1), getPoint(t), epsilon);
            to = yt - 1 + endparams.get(endparams.size() - 1); // + Collections.max(endparams);
        }
        return new double[]{from, to};
    }

    private Line2D getEdge(int y) {
        Map.Entry<Integer, Trajectory> trajectory = concatenated.floorEntry(y);
        return trajectory.getValue().getEdge(y - trajectory.getKey());
    }

    private Point2D getPoint(int x) {
        return representative.getPoint(x);
    }
}
