package mapconstruction.algorithms.bundles.graph;

import mapconstruction.algorithms.bundles.graph.representation.LabelledEdge;
import mapconstruction.algorithms.distance.QuadTree;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.stream.Collectors;

public class GeneratingQTSemiWeakFDLabelledGraph extends GeneratingSemiWeakFDLabelledGraph {

    private QuadTree<Integer> quadTree;

    public GeneratingQTSemiWeakFDLabelledGraph(double epsilon, Trajectory representative, List<Trajectory> concatenated, QuadTree<Integer> quadTree) {
        super(epsilon, representative, concatenated);
        this.quadTree = quadTree;
    }

    public void addColumn() {
        int threshold = (int) Math.pow(quadTree.size(), 0.77);
        List<Integer> previous = new ArrayList<>(); // query(0, Integer.MAX_VALUE);
        for (int i = 1; i < representative.numPoints(); i++) {
            if (previous.size() > threshold) {
                previous = scanColumn(i);
            } else {
                List<Integer> query = query(i - 1, threshold - previous.size());
                if (query == null) {
                    previous = scanColumn(i);
                } else {
                    // linear merge both lists
                    List<Integer> indices = linearMerge(previous, query);
                    ListIterator<Integer> iter = indices.listIterator();
                    previous = new ArrayList<>();
                    while (iter.hasNext()) {
                        int j = iter.next();
                        Map.Entry<Integer,Trajectory> traj = concatenated.floorEntry(j);
                        int jMax = traj.getKey() + traj.getValue().numEdges() - 1;
                        if (j > jMax) continue;

                        // try to add edges from a vertical segment
                        int x = vertexGraphCoord(i);
                        int y = edgeGraphCoord(j);
                        if (isFree(x, y)) {
                            ArrayList<LabelledEdge> edges = new ArrayList<>();
                            tryVertAddLeftEdge(x, y, edges);
                            tryVertAddBottomEdge(x, y, edges);
                            if (edges.size() > 0) {
                                labelledGraph.put(x, y, edges);
                                previous.add(j);
                            }
                        }
                        if (j >= jMax) continue;

                        // try to add edges from a horizontal segment
                        x = edgeGraphCoord(i - 1);
                        y = vertexGraphCoord(j + 1);
                        if (isFree(x, y)) {
                            ArrayList<LabelledEdge> edges = new ArrayList<>();
                            tryHorAddLeftEdge(x, y, edges);
                            tryHorAddBottomEdge(x, y, edges);
                            if (edges.size() > 0) {
                                labelledGraph.put(x, y, edges);
                                // insert new j (if needed) at the correct location
                                if (iter.hasNext()) {
                                    if (iter.next() > j + 1) {
                                        iter.previous();
                                        iter.add(j + 1);
                                        iter.previous();
                                    }
                                    iter.previous();
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private List<Integer> linearMerge(List<Integer> first, List<Integer> second) {
        List<Integer> indices = new LinkedList<>();
        for (int u = 0, v = 0; u < first.size() || v < second.size();) {
            if (u >= first.size() || (v < second.size() && first.get(u) > second.get(v))) {
                indices.add(second.get(v)); // when first[u] > second[v]
                v ++;
            } else {
                indices.add(first.get(u)); // when first[u] <= second[v]
                if (!(v < second.size() && first.get(u).equals(second.get(v)))) {
                    v ++; // when first[u] == second[v], increment both
                }
                u ++;
            }
        }
        return indices;
    }

    private List<Integer> query(int i, int threshold) {
        Point2D p = representative.getPoint(i);
        Set<Integer> result = quadTree.getInRange(p.getX(), p.getY(), epsilon);
        if (result.size() < threshold) {
            return result.stream().sorted().collect(Collectors.toList());
        }
        return null;
    }

    private List<Integer> scanColumn(int i) {
        List<Integer> free = new ArrayList<>();

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
                    if (edges.size() > 0) {
                        labelledGraph.put(x, y, edges);
                        free.add(j - 1);
                    }
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
                    if (edges.size() > 0) {
                        labelledGraph.put(x, y, edges);
                    }
                }
            }
        }

        return free;
    }

//    public void addColumn() {
//        for (int i = 1; i < representative.numPoints(); i++) {
//            Point2D p = representative.getPoint(i);
//            Set<Integer> indices = quadTree.getInRange(p.getX(), p.getY(), epsilon);
//            for (int j : indices) {
//                Map.Entry<Integer, Trajectory> it = concatenated.floorEntry(j);
//                Trajectory t = it.getValue();
//
//                // try to add edges from a vertical segment
//                int x = vertexGraphCoord(i);
//                int y = edgeGraphCoord(j - 1);
//                if (isFree(x, y)) {
//                    ArrayList<LabelledEdge> edges = new ArrayList<>();
//                    tryVertAddLeftEdge(x, y, edges);
//                    tryVertAddBottomEdge(x, y, edges);
//                    labelledGraph.put(x, y, edges);
//                }
//
//                // don't try to add edges to the 'top' of the diagram
//                Integer jMax = concatenated.ceilingKey(j);
//                if (jMax == null) jMax = concatenated.floorKey(j) + t.numPoints();
//                if (j == jMax - 1) continue;
//
//                // try to add edges from a horizontal segment
//                x = edgeGraphCoord(i - 1);
//                y = vertexGraphCoord(j);
//                if (isFree(x, y)) {
//                    ArrayList<LabelledEdge> edges = new ArrayList<>();
//                    tryHorAddLeftEdge(x, y, edges);
//                    tryHorAddBottomEdge(x, y, edges);
//                    labelledGraph.put(x, y, edges);
//                }
//            }
//        }
//    }
}
