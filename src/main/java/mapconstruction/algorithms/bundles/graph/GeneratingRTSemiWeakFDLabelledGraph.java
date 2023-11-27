package mapconstruction.algorithms.bundles.graph;

import mapconstruction.algorithms.bundles.graph.representation.LabelledEdge;
import mapconstruction.algorithms.bundles.graph.representation.Vertex;
import mapconstruction.algorithms.distance.RTree;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.stream.Collectors;

public class GeneratingRTSemiWeakFDLabelledGraph extends GeneratingSemiWeakFDLabelledGraph {

    private RTree<Line2D, Integer> rTree;

    public GeneratingRTSemiWeakFDLabelledGraph(double epsilon, Trajectory representative, List<Trajectory> concatenated, RTree<Line2D, Integer> rTree) {
        super(epsilon, representative, concatenated);
        this.rTree = rTree;
    }

    public void addColumn() {
        addLayer();
        if (true) return;

        int threshold = (int) Math.pow(rTree.size(), 0.77);
        int previous = 0; // query(0, Integer.MAX_VALUE);
        for (int i = 1; i < representative.numPoints(); i++) {
            if (previous > threshold) {
                previous = scanColumn(i);
            } else {
                List<Integer> query = query(i - 1, threshold);
                if (query == null) {
                    previous = scanColumn(i);
                } else {
                    // linear merge both lists
//                    List<Integer> indices = linearMerge(previous, query);
//                    System.out.println(query.size() >= previous.size());
                    ListIterator<Integer> iter = query.listIterator();
                    previous = 0;
                    while (iter.hasNext()) {
                        int j = iter.next();
                        Map.Entry<Integer, Trajectory> traj = concatenated.floorEntry(j);
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
                            }
                            previous++;
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
                                // insert new j (if needed) at the correct location, move pointer back to current index.
                                if (!iter.hasNext() || query.get(iter.nextIndex()) > j + 1) {
                                    iter.add(j + 1);
                                    iter.previous();
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    public void addLayer() {
        int x, y;

        for (int i = 0; i < representative.numEdges(); i++) {
            Set<Integer> query = queryS(i);
            Map<Integer, List<Integer>> candidates = new HashMap<>();

            for (int j : query) {
                x = vertexGraphCoord(i + 1);
                y = edgeGraphCoord(j);
                if (isFree(x, y)) {
                    List<LabelledEdge> edges = getEdges(x, y);
                    tryVertAddLeftEdge(x, y, edges);
                }

                x = edgeGraphCoord(i);
                y = vertexGraphCoord(j + 1);
                if (isFree(x, y)) {
                    Map.Entry<Integer, Trajectory> index = concatenated.floorEntry(j);
                    // if j+1 strictly fits the trajectory at j, add it to the candidates
                    if (j + 1 < index.getKey() + index.getValue().numEdges()) {
                        List<LabelledEdge> edges = getEdges(x, y);
                        tryHorAddLeftEdge(x, y, edges);
                        if (edges.size() > 0) {
                            // add (x,y) as candidate
                            List<Integer> candidate = getCandidate(candidates, index.getKey());
                            candidate.add(j + 1);
                        }
                    }
                }
            }

            for (List<Integer> candidate : candidates.values()) {
                candidate.sort(Integer::compareTo);
//                List<Integer> candidate = new ArrayList<>(c);
                ListIterator<Integer> iter = candidate.listIterator();
                while (iter.hasNext()) {
                    // don't check whether it fits the corresponding trajectory, do that on insertion only
                    int j = iter.next();

                    x = vertexGraphCoord(i + 1);
                    y = edgeGraphCoord(j);
                    if (isFree(x, y)) {
                        List<LabelledEdge> edges = getEdges(x, y);
                        tryVertAddBottomEdge(x, y, edges);
                    }
                    x = edgeGraphCoord(i);
                    y = vertexGraphCoord(j + 1);
                    if (isFree(x, y)) {
                        List<LabelledEdge> edges = getEdges(x, y);
                        tryHorAddBottomEdge(x, y, edges);
                        if (edges.size() > 0) {
                            Map.Entry<Integer,Trajectory> index = concatenated.floorEntry(j);
                            if (j + 1 < index.getKey() + index.getValue().numEdges()) {
                                // insert new j (if needed) at the correct location, move pointer back to current index.
                                if (!iter.hasNext() || candidate.get(iter.nextIndex()) > j + 1) {
                                    iter.add(j + 1);
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
        for (int u = 0, v = 0; u < first.size() || v < second.size(); ) {
            if (u >= first.size() || (v < second.size() && first.get(u) > second.get(v))) {
                indices.add(second.get(v)); // when first[u] > second[v]
                v++;
            } else {
                indices.add(first.get(u)); // when first[u] <= second[v]
                if (!(v < second.size() && first.get(u).equals(second.get(v)))) {
                    v++; // when first[u] == second[v], increment both
                }
                u++;
            }
        }
        return indices;
    }

    private List<Integer> query(int i, int threshold) {
        Point2D p = representative.getPoint(i);
        Set<Integer> result = rTree.windowQuery(p.getX() - epsilon, p.getY() - epsilon, p.getX() + epsilon, p.getY() + epsilon);
        if (result.size() < threshold) {
            return result.stream().sorted().collect(Collectors.toList());
        }
        return null;
    }

    private Set<Integer> queryS(int i) {
        Point2D p = representative.getPoint(i);
        return rTree.windowQuery(p.getX() - epsilon, p.getY() - epsilon, p.getX() + epsilon, p.getY() + epsilon);
    }

    private List<LabelledEdge> getEdges(int x, int y) {
        if (!labelledGraph.contains(x, y)) {
            labelledGraph.put(x, y, new ArrayList<>(2));
        }
        return labelledGraph.get(x, y);
    }

    private List<Integer> getCandidate(Map<Integer, List<Integer>> candidates, int index) {
        if (!candidates.containsKey(index)) {
            candidates.put(index, new ArrayList<>());
        }
        return candidates.get(index);
    }

    private List<Integer> getFree(int i) {
        List<Integer> free = new ArrayList<>();

        for (int J : concatenated.keySet()) {
            Trajectory t = concatenated.get(J);
            for (int j = J; j < J + t.numEdges(); j++) {
                int x = vertexGraphCoord(i);
                int y = edgeGraphCoord(j);
                if (isFree(x, y)) {
                    if (isFree(edgeGraphCoord(i), vertexGraphCoord(j + 1))) {
                        free.add(j);
                    }
                }
            }
        }

        return free;
    }

    private List<Integer> getFreeHorizontal(int i) {
        List<Integer> free = new ArrayList<>();

        for (int J : concatenated.keySet()) {
            Trajectory t = concatenated.get(J);
            for (int j = J + 1; j < J + t.numEdges(); j++) {
                int x = edgeGraphCoord(i);
                int y = vertexGraphCoord(j);
                if (isFree(x, y)) {
                    Vertex t1 = super.horizontalGridEdge(i, j - 1);
                    if (isFree(t1) && j - 1 > J) {
                        free.add(j);
                    }
                }
            }
        }

        return free;
    }

    private int scanColumn(int i) {
        int free = 0;

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
                    }
                    free ++;
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

    public void addColumn2() {
        for (int i = 1; i < representative.numPoints(); i++) {
            addEdge(i); // construct graph for representative edge [i-1,i]
            addVertex(i); // construct graph for representative vertex i
//            Point2D p = representative.getPoint(i);
//            Set<Integer> indices = rTree.windowQuery(p.getX() - epsilon, p.getY() - epsilon, p.getX() + epsilon, p.getY() + epsilon, new TreeSet<>());
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
        }
    }

    private void addVertex(int i) {
        Point2D p = representative.getPoint(i);
        Set<Integer> indices = rTree.windowQuery(p.getX() - epsilon, p.getY() - epsilon, p.getX() + epsilon, p.getY() + epsilon, new TreeSet<>());
        for (int j : indices) {
            // try to add edges from a vertical segment
            int x = vertexGraphCoord(i);
            int y = edgeGraphCoord(j);
            if (isFree(x, y)) {
                ArrayList<LabelledEdge> edges = new ArrayList<>();
                tryVertAddLeftEdge(x, y, edges);
                tryVertAddBottomEdge(x, y, edges);
                labelledGraph.put(x, y, edges);
            }
        }
    }

    private void addEdge(int i) {
        Line2D s = representative.getEdge(i - 1);
        Rectangle2D b = s.getBounds();
        Set<Integer> indices = rTree.windowQuery(b.getMinX() - epsilon, b.getMinY() - epsilon, b.getMaxX() + epsilon, b.getMaxY() + epsilon, new TreeSet<>());
        for (int j : indices) {
            // don't try to add edges to the 'top' of the diagram
            Integer jMin = concatenated.floorKey(j);
            if (j == jMin) continue;

            // try to add edges from a horizontal segment
            int x = edgeGraphCoord(i - 1);
            int y = vertexGraphCoord(j);
            if (isFree(x, y)) {
                ArrayList<LabelledEdge> edges = new ArrayList<>();
                tryHorAddLeftEdge(x, y, edges);
                tryHorAddBottomEdge(x, y, edges);
                labelledGraph.put(x, y, edges);
            }
        }
    }
}
