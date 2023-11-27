package mapconstruction.algorithms.bundles;

import com.google.common.collect.*;
import mapconstruction.algorithms.bundles.graph.GeneratingQTSemiWeakFDLabelledGraph;
import mapconstruction.algorithms.bundles.graph.GeneratingRTSemiWeakFDLabelledGraph;
import mapconstruction.algorithms.bundles.graph.GeneratingSemiWeakFDLabelledGraph;
import mapconstruction.algorithms.bundles.graph.representation.Event;
import mapconstruction.algorithms.bundles.sweep.FurthestEndpointSweep;
import mapconstruction.algorithms.bundles.sweep.KLSweepline;
import mapconstruction.algorithms.distance.KdTree;
import mapconstruction.algorithms.distance.QuadTree;
import mapconstruction.algorithms.distance.RTree;
import mapconstruction.benchmark.Benchmark;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class KLSubbundleAlgorithm extends BundleGenerationAlgorithm {

    private static final String LOGTAG = "KL-Subbundle";

    private double epsilon;
    private double lambda;

    private Map<Bundle, Bundle> merges;

    public KLSubbundleAlgorithm(double epsilon, double lambda, boolean ignoreDirection) {
        super(ignoreDirection);
        this.epsilon = epsilon;
        this.lambda = lambda;
    }

    @Override
    protected Set<Bundle> runAlgorithm(List<Trajectory> trajectories) {
        Set<Bundle> bundles;
//        try (Timing _t = Benchmark.parallel("epsilon %.0f", epsilon)) {
            Log.log(LogLevel.STATUS, LOGTAG, "Generating all bundles");
            Log.log(LogLevel.INFO, LOGTAG, "Parameters for Generating bundles: eps=%.2f, ignoreDir=%b", epsilon, ignoreDirection);

            merges = new HashMap<>();

            Benchmark.push("generate");
            bundles = generateAllBundlesRT(trajectories);
            System.out.println("EPS " + ((int) epsilon));
            Benchmark.pop();
            int old_number_of_bundles = bundles.size();
            Log.log(LogLevel.INFO, LOGTAG, "Result: %d bundles before removal.", old_number_of_bundles);
            Log.log(LogLevel.STATUS, LOGTAG, "Removing lambda-subbundles");
            Log.log(LogLevel.INFO, LOGTAG, "Parameters for removing subbundles: lambda=%.2f, ignoreDir=%b", lambda, ignoreDirection);

            Benchmark.push("lambda");
            removeLambdaSubbundles(bundles, lambda);
            Benchmark.pop();

            old_number_of_bundles = old_number_of_bundles - bundles.size();
            Log.log(LogLevel.STATUS, LOGTAG, "Total number of subbundles removed: %d", old_number_of_bundles);
            Log.log(LogLevel.INFO, LOGTAG, "Result: %d bundles left after removal.", bundles.size());

            setProgress(100);
//        }
        return bundles;
    }

    @Override
    public Map<Bundle, Bundle> getMerges() {
        return merges;
    }

    private Set<Bundle> generateAllBundles(List<Trajectory> trajectories) {
        Set<Bundle> results = new LinkedHashSet<>();

        List<Trajectory> representatives = new ArrayList<>(trajectories);
        if (ignoreDirection) {
            trajectories = Stream.concat(trajectories.stream(), trajectories.stream().map(Trajectory::reverse)).collect(Collectors.toList());
        }

        for (Trajectory representative : representatives) {
            GeneratingSemiWeakFDLabelledGraph freeSpace = new GeneratingSemiWeakFDLabelledGraph(epsilon, representative, trajectories);
            KLSweepline klSweepline = new FurthestEndpointSweep(freeSpace, lambda);
            klSweepline.initialize(); // compute free space and generate events
            Set<Bundle> representativeBundles = klSweepline.sweep(); // sweep over the events

            // TODO (possibly start removing lambda-equal bundles here since it is not a linear operation), seems to work, but is not 100% accurate
            // removeLambdaSubbundles(representativeBundles, lambda);
//            Benchmark.split("lambda");
//            mergeLambdaSubbundles(results, representativeBundles, lambda);
//            Benchmark.split("generate");

            results.addAll(representativeBundles);
        }

        return results;
    }

    private Set<Bundle> generateAllBundlesRT(List<Trajectory> trajectories) {
        Set<Bundle> results = new LinkedHashSet<>();

        List<Trajectory> representatives = new ArrayList<>(trajectories);
        if (ignoreDirection) {
            trajectories = Stream.concat(trajectories.stream(), trajectories.stream().map(Trajectory::reverse)).collect(Collectors.toList());
        }


        Map<Line2D, Integer> values = new HashMap<>();
        int i = 0;
        for (Trajectory t : trajectories) {
            for (Line2D segment : t.edges()) {
                values.put(segment, i);
                i++;
            }
            i++;
        }
        RTree<Line2D, Integer> rTree = new RTree<>(10, values);

        for (Trajectory representative : representatives) {
            GeneratingRTSemiWeakFDLabelledGraph freeSpace = new GeneratingRTSemiWeakFDLabelledGraph(epsilon, representative, trajectories, rTree);
            KLSweepline klSweepline = new FurthestEndpointSweep(freeSpace, lambda);
            Benchmark.push("init");
            klSweepline.initialize();
            Benchmark.split("sweep");
            Set<Bundle> representativeBundles = klSweepline.sweep();
            Benchmark.pop();

            results.addAll(representativeBundles);
        }
        return results;
    }

    private Set<Bundle> generateAllBundlesQT(List<Trajectory> trajectories) {
        Set<Bundle> results = new LinkedHashSet<>();
        // find bounds of the problem
        double x1 = Double.MAX_VALUE, y1 = Double.MAX_VALUE, x2 = Double.MIN_VALUE, y2 = Double.MIN_VALUE;
        for (Trajectory t : trajectories) {
            for (Point2D p : t.points()) {
                x1 = Math.min(x1, p.getX());
                y1 = Math.min(y1, p.getY());
                x2 = Math.max(x2, p.getX());
                y2 = Math.max(y2, p.getY());
            }
        }
        List<Trajectory> representatives = new ArrayList<>(trajectories);
        if (ignoreDirection) {
            trajectories = Stream.concat(trajectories.stream(), trajectories.stream().map(Trajectory::reverse)).collect(Collectors.toList());
        }

        QuadTree<Integer> quadTree = indexVertices(trajectories, x1, y1, x2, y2);

        for (Trajectory representative : representatives) {
            GeneratingQTSemiWeakFDLabelledGraph freeSpace = new GeneratingQTSemiWeakFDLabelledGraph(epsilon, representative, trajectories, quadTree);
            KLSweepline klSweepline = new FurthestEndpointSweep(freeSpace, lambda);
            Benchmark.push("init");
            klSweepline.initialize();
            Benchmark.split("sweep");
            Set<Bundle> representativeBundles = klSweepline.sweep();
            Benchmark.pop();

            results.addAll(representativeBundles);
        }
        return results;
    }

    private QuadTree<Integer> indexVertices(List<Trajectory> trajectories, double x1, double y1, double x2, double y2) {
        QuadTree<Integer> quadTree = new QuadTree<>();
        quadTree.initialize(x1, y1, x2, y2);

        for (int i = 0, s = 0; i < trajectories.size(); i++) {
            Trajectory c = trajectories.get(i);
            List<Point2D> p = c.points();
            // index all points (except first)
            for (int j = 0; j < p.size() - 1; j++) {
                Point2D point = p.get(j);
                quadTree.insert(point.getX(), point.getY(), s + j);
            }
            s += c.numPoints();
        }

        return quadTree;
    }

    public Map<Bundle, Bundle> removeLambdaSubbundlesKD(Set<Bundle> bundles, double lambda) {
        // Of the removed bundles, it keeps track into which bundle it merged.
        Map<Bundle, Bundle> merge = new HashMap<>();

        // Comparator to sort bundles, first by decreasing size, then by decreasing length.
        Comparator<Bundle> compSizeDec = (b1, b2) -> Integer.compare(b2.size(), b1.size());
        Comparator<Bundle> compSizeLengthLex = compSizeDec.thenComparing(Comparator.comparingDouble(Bundle::continuousLength).reversed());

        // Sort all the bundles.
        List<Bundle> bundleList = bundles.stream()
                .sorted(compSizeLengthLex)
                .collect(Collectors.toCollection(ArrayList::new));

        // First and last pair in the coordinate are representing endpoints such that
        // the distance between two 4d coordinates is the maximum of the eucledian distance
        // of both pairs.
        KdTree<Bundle> bundleQuery = new KdTree<>(4, (c1, c2) -> Math.max(
                Math.sqrt(Math.pow(c1[0] - c2[0], 2) + Math.pow(c1[1] - c2[1], 2)),
                Math.sqrt(Math.pow(c1[2] - c2[2], 2) + Math.pow(c1[3] - c2[3], 2))
        ));

        for (Bundle b1 : bundleList) {
            Subtrajectory representative = b1.getOriginalRepresentative();
            Point2D s = representative.getFirstPoint();
            Point2D t = representative.getLastPoint();
            Set<Bundle> closeBundles = bundleQuery.rangeQuery(2*epsilon + lambda, s.getX(), s.getY(), t.getX(), t.getY());

            Bundle b2 = null;
            for (Bundle c : closeBundles) {
                if (c.hasAsLambdaSubBundle(b1, lambda)) {
                    b2 = c;
                    break;
                }
            }
            if (b2 == null) {
                bundleQuery.insert(b1, s.getX(), s.getY(), t.getX(), t.getY());
            } else {
                merge.put(b1, b2);
                bundles.remove(b1);
            }
        }

        return merge;
    }

    public void mergeLambdaSubbundles(Set<Bundle> resultSet, Set<Bundle> bundles, double lambda) {
        // Comparator to sort bundles, first by decreasing size, then by decreasing length.
        Comparator<Bundle> compSizeDec = (b1, b2) -> Integer.compare(b2.size(), b1.size());
        Comparator<Bundle> compSizeLengthLex = compSizeDec.thenComparing(Comparator.comparingDouble(Bundle::continuousLength).reversed());

        // Sort all the bundles.
        List<Bundle> bundleList = bundles.stream()
                .sorted(compSizeLengthLex)
                .collect(Collectors.toCollection(ArrayList::new));

        for (Bundle b1 : bundleList) {
            if (merges.containsKey(b1)) continue;

            for (Bundle b2 : bundleList) {
                if (b1 == b2 || merges.containsKey(b2)) continue;

                if (!b1.getOriginalRepresentative().hasAsLambdaSimilar(b2.getOriginalRepresentative(), 2*epsilon)) continue;

                if (b1.hasAsLambdaSubBundle(b2, lambda)) {
                    // b2 is a lambda subbundle of b1
                    merges.put(b2, b1);
                    bundles.remove(b2);
                }
            }
        }

        resultSet.addAll(bundles);
    }

    public void removeLambdaSubbundles(Set<Bundle> bundles, double lambda) {
        // Comparator to sort bundles lexicograpgically,
        // first by decreasing size, then by decreasing length.
        Comparator<Bundle> compSizeDec = (b1, b2) -> Integer.compare(b2.size(), b1.size());
        Comparator<Bundle> compSizeLengthLex = compSizeDec.thenComparing(Comparator.comparingDouble(Bundle::continuousLength).reversed());

        // Sort all the bundles.
        List<Bundle> bundleList = bundles.stream()
                .sorted(compSizeLengthLex)
                .collect(Collectors.toCollection(ArrayList::new));

        for (Bundle b1 : bundleList) {
            if (merges.containsKey(b1)) continue;

            // Find all bundles that are a subbundle of the current bundle
            for (Bundle b2 : bundleList) {
                if (b1 == b2 || b2.size() > b1.size() || merges.containsKey(b2)) continue;

                // FurthestEndpointSweep already filters same-rep bundles.
                if (b1.getOriginalRepresentative().getParent().getUndirectionalLabel().equals(b2.getOriginalRepresentative().getParent().getUndirectionalLabel())) continue;
//                if (b1.getOriginalRepresentative().getUndirectionalLabel().equals(b2.getOriginalRepresentative().getUndirectionalLabel())) continue;

                // if the endpoints are not close enough
                // if (!b1.getOriginalRepresentative().hasAsLambdaEndpoints(b2.getOriginalRepresentative(), 2*epsilon + lambda)) continue;
                // boolean wouldSkip = !b1.getOriginalRepresentative().hasAsLambdaEndpoints(b2.getOriginalRepresentative(), 2*epsilon + lambda);
                if (!b1.getOriginalRepresentative().hasAsLambdaSimilar(b2.getOriginalRepresentative(), 2*epsilon)) continue;

                if (b1.hasAsLambdaSubBundle(b2, lambda, false)) {
                    // b2 is a lambda subbundle of b1
                    merges.put(b2, b1);
                    bundles.remove(b2);
                }
            }
        }
    }

    public void removeLambdaSubbundlesRT(Set<Bundle> bundles, double lambda) {
        // Comparator to sort bundles lexicograpgically,
        // first by decreasing size, then by decreasing length.
        Comparator<Bundle> compSizeDec = (b1, b2) -> Integer.compare(b2.size(), b1.size());
        Comparator<Bundle> compSizeLengthLex = compSizeDec.thenComparing(Comparator.comparingDouble(Bundle::continuousLength).reversed());
        Comparator<Bundle> compSizeLengthUnique = compSizeLengthLex.thenComparing(Bundle::hashCode);

        // Sort all the bundles.
        List<Bundle> bundleList = bundles.stream()
                .sorted(compSizeLengthLex)
                .collect(Collectors.toCollection(ArrayList::new));

//        RTree<Path2D,Bundle> rTree = new RTree<>(30);
//        bundleList.forEach(b -> {
//            Path2D path = null;
//            double length = 0d;
//            for (Point2D p : b.getOriginalRepresentative().points()) {
//                if (path == null) {
//                    path = new Path2D.Double();
//                    path.moveTo(p.getX(), p.getY());
//                } else {
//                    Point2D last = path.getCurrentPoint();
//                    path.lineTo(p.getX(), p.getY());
//                    length += last.distance(p);
//                    if (length > b.continuousLength() / 5d) {
//                        rTree.insert(path, b);
//                        path = new Path2D.Double();
//                        path.moveTo(p.getX(), p.getY());
//                        length = 0d;
//                    }
//                }
//            }
//            if (length > 0d) {
//                rTree.insert(path, b);
//            }
//        });
        RTree<Line2D,Bundle> rTree = new RTree<>(30);
        bundleList.forEach(b -> {
            b.getOriginalRepresentative().edges().forEach(e -> {
                rTree.insert(e, b);
            });
        });

        for (Bundle b1 : bundleList) {
            if (merges.containsKey(b1)) continue;

            Point2D p = b1.getOriginalRepresentative().getPoint(0);
            double r = 2*epsilon;
            Benchmark.push("window");
            Set<Bundle> candidates = rTree.windowQuery(p.getX() - r, p.getY() - r, p.getX() + r, p.getY() + r, new TreeSet<>(compSizeLengthUnique));
            Benchmark.pop();
            for (Bundle b2 : candidates) {
                if (b1 == b2 || merges.containsKey(b2)) continue;

                if (!b1.getOriginalRepresentative().hasAsLambdaSimilar(b2.getOriginalRepresentative(), 2*epsilon)) continue;

                if (b1.hasAsLambdaSubBundle(b2, lambda)) {
                    // b2 is a lambda subbundle of b1
                    merges.put(b2, b1);
                    bundles.remove(b2);
                }
            }
        }
    }

}
