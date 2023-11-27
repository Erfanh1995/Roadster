package mapconstruction.algorithms.bundles;

import mapconstruction.benchmark.Benchmark;
import mapconstruction.benchmark.Timing;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;

import java.util.*;
import java.util.function.IntUnaryOperator;
import java.util.stream.Collectors;

/**
 * For a given epsilon and lambda, this algorithm finds all maximal length
 * bundles for all sizes.
 * <p>
 * The bundles that are returned are maximal with respect to the
 * lambda-subbundle relation.
 *
 * @author Roel
 */
public class MaximalSubbundleAlgorithm extends BundleGenerationAlgorithm {

    private static final String LOGTAG = "MaxBundle";
    public static int minK = 3;
    /**
     * Distance
     */
    private final double epsilon;
    /**
     * Length ratio error
     */
    private final double lambda;
    /**
     * Step to increase k.
     */
    private final IntUnaryOperator kStep;
    private SweeplineBundleAlgorithm bundleAlgo;
    /**
     * Latest mapping of merges found by this algorithm.
     */
    private Map<Bundle, Bundle> merges;

    /**
     * Constructs the algorithm with the given parameters.
     *
     * @param epsilon         distance
     * @param lambda          Length ratio error
     * @param ignoreDirection
     * @param kStep
     */
    public MaximalSubbundleAlgorithm(double epsilon, double lambda, boolean ignoreDirection, IntUnaryOperator kStep) {
        super(ignoreDirection);
        this.epsilon = epsilon;
        this.lambda = lambda;
        this.kStep = kStep;
    }

    /**
     * Removes all lambda sub-bundles from the given set of bundles.
     * The input bundle set is modified.
     * <p>
     * It returns a mapping indicating which bundle merged with which other bundle.
     *
     * @param bundles
     * @param lambda
     * @return
     */
    public static Map<Bundle, Bundle> removeLambdaSubbundles(Set<Bundle> bundles, double lambda) {
        // Of the removed bundles, it keeps track into which bundle it merged.
        Map<Bundle, Bundle> merge = new HashMap<>();

        // Comparator to sort bundles lexicograpgically,
        // first by decreasing size, then by decreasing length.
        Comparator<Bundle> compSizeDec = (b1, b2) -> Integer.compare(b2.size(), b1.size());
        Comparator<Bundle> compSizeLengthLex = compSizeDec.thenComparing((b1, b2) -> Double.compare(b2.continuousLength(), b1.continuousLength()));

        // Sort all the bundles.
        List<Bundle> bundleList = bundles.stream()
                .sorted(compSizeLengthLex)
                .collect(Collectors.toCollection(ArrayList::new));

        for (Bundle b1 : bundleList) {
            if (merge.containsKey(b1)) {
                continue;
            }

            // Find all bundles that are a subbundle of the current bundle
            for (Bundle b2 : bundleList) {
                if (b1 == b2) {
                    continue;
                }

                if (merge.containsKey(b2)) {
                } else if (b1.hasAsLambdaSubBundle(b2, lambda)) {
                    // b2 is a lambda subbundle of b1
                    merge.put(b2, b1);
                    bundles.remove(b2);
                }
            }

        }
        return merge;
    }

    @Override
    public Set<Bundle> runAlgorithm(List<Trajectory> trajectories) {
        Set<Bundle> bundles;
//        try (Timing _t = Benchmark.parallel("epsilon %.0f", epsilon)) {
            Log.log(LogLevel.STATUS, LOGTAG, "Finding all maximal lambda-subbundles");

            Log.log(LogLevel.STATUS, LOGTAG, "Generating all bundles");
            Log.log(LogLevel.INFO, LOGTAG, "Parameters for Generating bundles: eps=%.2f, ignoreDir=%b", epsilon, ignoreDirection);
            Benchmark.push("generate");
            bundles = generateAllBundles(trajectories);
            System.out.println("EPS " + ((int) epsilon) + ": " + bundles);
            Benchmark.pop();

            int old_number_of_bundles = bundles.size();
            Log.log(LogLevel.INFO, LOGTAG, "Result: %d bundles before removal.", old_number_of_bundles);
            Log.log(LogLevel.STATUS, LOGTAG, "Removing lambda-subbundles");
            Log.log(LogLevel.INFO, LOGTAG, "Parameters for removing subbundles: lambda=%.2f, ignoreDir=%b", lambda, ignoreDirection);

            Benchmark.push("lambda");
            merges = removeLambdaSubbundles(bundles);
            Benchmark.pop();

            old_number_of_bundles = old_number_of_bundles - bundles.size();

            Log.log(LogLevel.STATUS, LOGTAG, "Total number of subbundles removed: %d", old_number_of_bundles);
            Log.log(LogLevel.INFO, LOGTAG, "Result: %d bundles left after removal.", bundles.size());

            setProgress(100);
//        }
        return bundles;
    }

    /**
     * Generates all maxlength bundles for all sizes.
     *
     * @param trajectories trajectories to find bundles for.
     * @return
     */
    private Set<Bundle> generateAllBundles(List<Trajectory> trajectories) {
        int k = minK;
        int N = trajectories.size();
        Set<Bundle> result = new LinkedHashSet<>();
        Set<Bundle> M;
        do {
            checkAbort();
            bundleAlgo = new SweeplineBundleAlgorithm(epsilon, k, this.ignoreDirection);
            M = bundleAlgo.run(trajectories);
            result.addAll(M);
            setProgress((int) (100.0 * k / N));

            // set k to the size of the smallest bundle in the result set
            k = Math.max(M.stream()
                    .mapToInt(b -> b.size())
                    .min()
                    .orElse(Integer.MAX_VALUE), kStep.applyAsInt(k));
        } while (!M.isEmpty() && k <= N);

        return result;
    }

    /**
     * Removes all lambda-subbundles from the given set of bundles.
     * <p>
     * Modifies the given set of bundles. Returns a map indicating the merges
     * that happened.
     *
     * @param bundles
     */
    private Map<Bundle, Bundle> removeLambdaSubbundles(Set<Bundle> bundles) {
        return removeLambdaSubbundles(bundles, lambda);
    }

    @Override
    public void abort() {
        super.abort();
        bundleAlgo.abort();
    }

    /**
     * Returns the merges that were found the last time the algorithm was run.
     *
     * @return
     */
    public Map<Bundle, Bundle> getMerges() {
        return merges;
    }

}
