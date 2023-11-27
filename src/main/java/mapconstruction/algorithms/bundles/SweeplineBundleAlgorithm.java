package mapconstruction.algorithms.bundles;

import com.google.common.collect.*;
import mapconstruction.algorithms.bundles.graph.LabelledGraph;
import mapconstruction.algorithms.bundles.graph.SemiWeakFDLabelledGraphIntBased;
import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.*;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.util.*;
import java.util.Map.Entry;

/**
 * Algorithm for finding maximal length bundles in a set of trajectories.
 * <p>
 * We fix the size and the distance, and the algorithm will maximize the length.
 * <p>
 * We use the discrete frechet distance, and we allow that subtrajectories in
 * the resulting bundle come from the same trajectory as long as they do not
 * overlap.
 * <p>
 * Based on the paper "Detecting Commuting Patterns by Clustering
 * Subtrajectories"
 *
 * @author Roel
 */
public class SweeplineBundleAlgorithm extends BundleGenerationAlgorithm {

    private static final String LOGTAG = "SweepLine";
    private final boolean allowPartialBounds;
    private final boolean allowMultipleSubOfSameTraj;
    /**
     * Distance matrix used to determine the free space.
     */
    private DistanceMatrix dm;
    /**
     * Sorted map mapping a particular range of indices in the concatenated
     * trajectory to a trajectory in the input.
     */
    private TreeRangeMap<Integer, Trajectory> trajectories;
    /**
     * Trajectory formed by concatenating all trajectories.
     */
    private Trajectory concatenated;
    /**
     * distance to use
     */
    private double epsilon;
    /**
     * Size of the bundles to find.
     */
    private int k;
    private LabelledGraph labelledGraph;

    /**
     * Constructs the algorithm to work on the given trajectories.
     * <p>
     * Allows subtrajectories to have partial bounds, and disallows multiple
     * subtrajectories from the same trajectory.
     *
     * @param dist
     * @param size
     * @param ignoreDirection
     */
    public SweeplineBundleAlgorithm(double dist, int size, boolean ignoreDirection) {
        this(dist, size, ignoreDirection, true, false);
    }

    /**
     * Constructs the algorithm to work on the given trajectories.
     *
     * @param dist
     * @param size
     * @param ignoreDirection
     * @param allowPartialBounds
     * @param allowMultipleSubOfSameTraj
     */
    public SweeplineBundleAlgorithm(double dist, int size, boolean ignoreDirection, boolean allowPartialBounds, boolean allowMultipleSubOfSameTraj) {
        super(ignoreDirection);
        this.epsilon = dist;
        this.k = size;
        this.allowPartialBounds = allowPartialBounds;
        this.allowMultipleSubOfSameTraj = allowMultipleSubOfSameTraj;
    }

    /**
     * Finds the set of maximal bundles with respect to length.
     * <p>
     * Maximal length here means that no bundle is a subbundle of another.
     *
     * @param trajectories
     * @return
     */
    @Override
    public Set<Bundle> runAlgorithm(List<Trajectory> trajectories) {
        Log.log(LogLevel.STATUS, LOGTAG, "Starting sweep line bundle algorithm");
        Log.log(LogLevel.INFO, LOGTAG, "Parameters sweep line algorithm: k=%d, eps=%.2f, ignoreDir=%b", k, epsilon, ignoreDirection);
        Set<Bundle> result;
        if (epsilon == 0.0d) {
            Log.log(LogLevel.STATUS, LOGTAG, "eps == 0, one bundle for every trajectory");
            // shortcut. For eps = 0, essentially every trajectory should get its own bundle
            result = new LinkedHashSet<>();
            if (k <= 1) {
                for (Trajectory t : trajectories) {
                    Subtrajectory s = new Subtrajectory(t, 0, t.numPoints() - 1);
                    Bundle b;
                    if (this.ignoreDirection) {
                        result.add(UndirectionalBundle.create(Collections.singleton(s), s));
                    } else {
                        result.add(Bundle.create(Collections.singleton(s), s));
                    }
                }
            }

        } else {
            Log.log(LogLevel.STATUS, LOGTAG, "Initialization");
            initialize(trajectories);

            Log.log(LogLevel.STATUS, LOGTAG, "Sweep");
            if (this.ignoreDirection) {
                result = sweep(0, (concatenated.numPoints() - 1) / 2);
            } else {
                result = sweep(0, concatenated.numPoints() - 1);
            }
        }
        Log.log(LogLevel.STATUS, LOGTAG, "Sweep line algorithm finished");

        Log.log(LogLevel.INFO, LOGTAG, "Result: %d bundles", result.size());
        return result;
    }

    /**
     * Initializes the algorithm. Computes the distance matrix and the labelled
     * graph.
     * <p>
     * Must be called be fore running the algorithm.
     *
     * @param trajectories
     */
    public void initialize(List<Trajectory> trajectories) {
        this.trajectories = TreeRangeMap.create();

        // Ranges where trajectories do not exist
        Set<Range<Integer>> borders = new HashSet<>();
        borders.add(Range.closed(-1, 0));

        // Create long trajectory
        // Collect points
        ArrayList<Point2D> points = new ArrayList<>();

        int rangeStart = 0;
        for (Trajectory t : trajectories) {

            for (Point2D p : t.points()) {
                points.add(p);
            }

            // Compute range for t
            int rangeEnd = rangeStart + t.numPoints() - 1;
            this.trajectories.put(Range.closed(rangeStart, rangeEnd), t);
            borders.add(Range.closed(rangeEnd, rangeEnd + 1));
            rangeStart = rangeEnd + 1;

        }

        if (this.ignoreDirection) {
            // Add all trajectories in opposite direction.
            for (Trajectory t : Lists.reverse(trajectories)) {
                Trajectory reverse = t.reverse();
                for (Point2D p : reverse.points()) {
                    points.add(p);
                }

                // Compute range for t
                int rangeEnd = rangeStart + reverse.numPoints() - 1;
                this.trajectories.put(Range.closed(rangeStart, rangeEnd), reverse);
                borders.add(Range.closed(rangeEnd, rangeEnd + 1));
                rangeStart = rangeEnd + 1;
            }
        }

        concatenated = new FullTrajectory(points);
        // Compute distance matrix
        dm = new DistanceMatrix(concatenated, concatenated);

        // create labelled graph
        labelledGraph = new SemiWeakFDLabelledGraphIntBased(concatenated, borders, epsilon);
    }

    /**
     * Runs the algorithm by sweeping both lines from min to max Will initialize
     * the algorithm if it has not yet been done.
     *
     * @param min
     * @param max
     * @return generated bundle
     */
    private Set<Bundle> sweep(int min, int max) {
        // sweep lines
        int ls = min;
        int lt = min;

        // Set of maximal length bundles.
        Set<Bundle> maxLengthBundles = new LinkedHashSet<>();

        labelledGraph.addColumn();
        /*
         * cluster curves and representativeSubtrajectory found in
         * the previous iteration, if any.
         *
         * Whenever we have to increase ls,
         * the bundle found in the previous iteration was a maximal-length one,
         * as the length now decreases again. Hence we have to report it.
         */
        Bundle prevBundle = null;

        while (ls < max) {
            checkAbort();
            setProgress((int) (50.0 * ls / max + 50.0 * lt / max));
            if (ls == lt) {
                lt++;
                labelledGraph.addColumn();
                continue;
            }

            // Check if a representativeSubtrajectory can be created.
            // If the range [ls, lt] refers to a non-existing trajectory
            // It will become null.
            Subtrajectory representative = createSubtrajectory(ls, lt);

            if (representative != null) {
                // Find the cluster curves between ls and lt, not
                // taking the span [s,t] into account
                // The set includes the representativeSubtrajectory itself
                Set<Subtrajectory> curves = findClusterCurves(ls, lt, representative);
                if (curves.size() >= k) {
                    // Found enough cluster curves
                    Bundle b;

                    if (this.ignoreDirection) {
                        b = UndirectionalBundle.create(curves, representative);
                    } else {
                        b = Bundle.create(curves, representative);
                    }
                    prevBundle = b;

                    lt++;
                    if (lt <= max) {
                        labelledGraph.addColumn();
                    }
                    continue;

                }
            }

            // ls has to be increased, hence we have to report the previous
            // found longest bundle
            if (prevBundle != null) {
                maxLengthBundles.add(prevBundle);
                prevBundle = null;
            }

            ls++;
            labelledGraph.removeColumn();
        }
        setProgress(100);
        return maxLengthBundles;

    }

    /**
     * Determines the cluster curves between s and t.
     * <p>
     * Includes the representativeSubtrajectory itself
     *
     * @param s
     * @param t
     * @return
     */
    private Set<Subtrajectory> findClusterCurves(int s, int t, Subtrajectory representative) {
        HashSet<Subtrajectory> clusterCurves = new HashSet<>();
        clusterCurves.add(representative);

        /*
         * Ranges that a new cluster curve may not overlap.
         */
        TreeMap<Integer, Range<Integer>> forbiddenRanges = new TreeMap<>();

        /*
         * Map relting an input trajectory to a one or more cluster curves.
         * The cluster curve may be in either direction.
         *
         * If allowMultipleSubOfSameTraj is false, then
         * a trajectory will only be mapped to its longest clustercurve
         *
         * If true, all found curves that are found are added, as long as they do
         * not overlap.
         */
        HashMultimap<Trajectory, Subtrajectory> curvesPerTrajectory = HashMultimap.create();

        // Forbidden range of representativeSubtrajectory
        Range<Integer> forbidden = findForbiddenRange(s, t);
        forbiddenRanges.put(forbidden.lowerEndpoint(), forbidden);
        // Forbidden range for reversed trajectories, if any
        if (this.ignoreDirection) {
            Range<Integer> rev = reversedRange(forbidden.lowerEndpoint(), forbidden.upperEndpoint());
            forbiddenRanges.put(rev.lowerEndpoint(), rev);
        }

        /**
         * Start with topmost vertex on lt, which has an outgoing transition
         * labelled at most s, and try to find a cluster curve.
         */
        int yt = concatenated.numPoints() - 1;

        /**
         * Current trajectory we are checking against representativeSubtrajectory.
         */
        Trajectory currentTrajectory = null;

        /**
         * Span of the current trajectory we are checking against
         * representativeSubtrajectory.
         */
        Range<Integer> currentTrajectoryRange = null;
        while (yt >= 0) {
            if (currentTrajectoryRange == null || !currentTrajectoryRange.contains(yt)) {
                // update trajectory we are checking.
                Entry<Range<Integer>, Trajectory> entry = trajectories.getEntry(yt);
                currentTrajectory = entry.getValue();

                currentTrajectoryRange = entry.getKey();
            }

            // value for yt is valid if it is not in any of the forbidden ranges.
            // @ReferenceJorrick: 1
            Entry<Integer, Range<Integer>> forbidYT = forbiddenRanges.floorEntry(yt);
            if (forbidYT != null && forbidYT.getValue().contains(yt)) {
                yt--;
                continue;
            }

            // Find a path in the labelled graph.
            OptionalInt optYS = labelledGraph.findStart(s, t, yt, forbiddenRanges.values());
            if (!optYS.isPresent()) {
                // No curve exists.
                yt--;
                continue;
            }

            int ys = optYS.getAsInt();

            Entry<Integer, Range<Integer>> forbidfloorR = forbiddenRanges.floorEntry(ys);
            Entry<Integer, Range<Integer>> forbidceilR = forbiddenRanges.ceilingEntry(ys);
            if (forbidfloorR != null && forbidfloorR.getValue().contains(ys)) {
                // @NoteJorrick: This never occurs, Because::
                // FloorEntry will return the range containing ys or the range just below it.
                // However, we check in this if that ys contains it. That is not possible by
                // @ReferenceJorrick: 1.
                yt = forbidfloorR.getValue().lowerEndpoint() - 1;
            } else if (forbidceilR != null && yt > forbidceilR.getValue().lowerEndpoint()) {
                // @NoteJorrick: This never occurs, Because::
                // CeilingEntry will return the range containing ys or the range just above it.
                // However, we check in this if ys is larger than the lower end point. Hence it should be in the range.
                // That is not possible by @ReferenceJorrick: 1.
                yt = forbidceilR.getValue().lowerEndpoint() - 1;
            } else {

                // determine partial bounds if needed
                double[] bounds = computeSubtrajectoryBounds(s, t, ys, yt);
                double from = bounds[0] - currentTrajectoryRange.lowerEndpoint();
                double to = bounds[1] - currentTrajectoryRange.lowerEndpoint();

                if (from <= to) {
                    // correct order
                    // make sure the found range is part of a single tajectory
                    Subtrajectory tr = new Subtrajectory(currentTrajectory, from, to);

                    /*
                    check if the found curve is longer than one we found already
                     */
                    Trajectory currNonReversed;
                    if (this.ignoreDirection && currentTrajectory.isReverse()) {
                        // Make sure we have the trajectory in the "normal" direction.
                        currNonReversed = currentTrajectory.reverse();
                    } else {
                        currNonReversed = currentTrajectory;
                    }

                    if (allowMultipleSubOfSameTraj) {
                        Set<Subtrajectory> others = curvesPerTrajectory.get(currNonReversed);
                        // add trajectory if it does not overlap any of the existing ones
                        boolean overlaps = others.stream().anyMatch((Subtrajectory sub) -> {
                            Subtrajectory tr2 = tr;
                            if (tr2.isReverse()) {
                                tr2 = tr2.reverse();
                            }
                            if (sub.isReverse()) {
                                sub = sub.reverse();
                            }
                            return sub.getFromIndex() <= tr2.getToIndex() && tr2.getFromIndex() <= sub.getToIndex();
                        });

                        if (!overlaps) {
                            curvesPerTrajectory.put(currNonReversed, tr);
                        }
                    } else {
                        // make sure we store only one cluster curve: the longest.
                        Subtrajectory other = Iterables.getFirst(curvesPerTrajectory.get(currNonReversed), null);
                        // Other trajectory, check longest.
                        if (other == null || other.euclideanLength() < tr.euclideanLength()) {
                            curvesPerTrajectory.replaceValues(currNonReversed, Collections.singleton(tr));//.put(currNonReversed, tr);
                        }
                    }
                }

                if (ys == yt) {
                    yt = ys - 1;
                } else {
                    yt = ys;
                }
            }
        }
        clusterCurves.addAll(curvesPerTrajectory.values());
        return clusterCurves;
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
        if (allowPartialBounds) {
            // first edge
            double from;
            if (dm.getPointDistance(s, ys) <= epsilon) {
                from = ys;
            } else {
                // need to create a partial edge 
                List<Double> startparams = GeometryUtil.segCircIntersectionParams(concatenated.getEdge(ys), concatenated.getPoint(s), epsilon);
                from = ys + startparams.get(0);
            }

            // last edge
            double to;
            if (dm.getPointDistance(t, yt) <= epsilon) {
                to = yt;
            } else {
                List<Double> endparams = GeometryUtil.segCircIntersectionParams(concatenated.getEdge(yt - 1), concatenated.getPoint(t), epsilon);
                to = yt - 1 + endparams.get(endparams.size() - 1);
            }
            return new double[]{from, to};
        } else {
            return new double[]{ys, yt};
        }

    }

    /**
     * Returns the range in the concatenated trajectory representing the
     * subtrajectory between s and t in reverse.
     *
     * @param s
     * @param t
     * @return
     */
    private Range<Integer> reversedRange(int s, int t) {
        int start = concatenated.numPoints() - t - 1;
        int end = concatenated.numPoints() - s - 1;
        if (allowMultipleSubOfSameTraj) {
            return Range.open(start, end);
        } else {
            return Range.closed(start, end);
        }

    }

    /**
     * Creates a subtrajectory from the concatenated trajectory between s and t.
     * Makes sure the proper parent is found and referred to.
     * <p>
     * Returns null if the range is part of a non-existing trajectory (artefact
     * from concatenating all trajectories).
     *
     * @param s
     * @param t
     * @return
     */
    private Subtrajectory createSubtrajectory(int s, int t) {
        // find <range, trajectory> pair for the range containing s
        Entry<Range<Integer>, Trajectory> entry = trajectories.getEntry(s);

        if (entry != null && entry.getKey().contains(t)) {
            // valid
            int start = s - entry.getKey().lowerEndpoint();
            int end = start + t - s;
            return new Subtrajectory(entry.getValue(), start, end);
        }
        return null;
    }

    /**
     * Finds the forbidden range for the trajectory between s and t.
     * <p>
     * The forbidden range is the range which no cluster curves may overlap.
     * This includes the range of the trajectory containing s, t
     *
     * @param s
     * @param t
     * @return
     */
    private Range<Integer> findForbiddenRange(int s, int t) {
        // Find the trajectory range
        if (!allowMultipleSubOfSameTraj) {
            return trajectories.getEntry(s).getKey();
        } else {
            return Range.open(s, t);
        }
    }

    private boolean checkStartPointsCurves(Set<Subtrajectory> curves, Subtrajectory representative) {
        return curves.stream().allMatch(curve -> {
            int n1 = curve.numPoints();
            int n2 = representative.numPoints();
            return curve.getPoint(0).distance(representative.getPoint(0)) <= epsilon
                    && curve.getPoint(n1 - 1).distance(representative.getPoint(n2 - 1)) <= epsilon;
        });
    }

    @Override
    public Map<Bundle, Bundle> getMerges() {
        return new HashMap<>();
    }
}
