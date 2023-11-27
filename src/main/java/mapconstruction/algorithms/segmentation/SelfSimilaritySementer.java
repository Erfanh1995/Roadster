package mapconstruction.algorithms.segmentation;

import com.google.common.math.DoubleMath;
import com.google.common.math.Stats;
import mapconstruction.algorithms.bundles.MaximalSubbundleAlgorithm;
import mapconstruction.algorithms.bundles.SweeplineBundleAlgorithm;
import mapconstruction.attributes.BundleAttribute;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.math.RoundingMode;
import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;

/**
 * Segmenter segmenting trajectories based on self similarity.
 * <p>
 * Say we have a trajectory T, and some subtrajectory s of T is similar to
 * another subtrajectory r of T, then we have to split such that s and r are on
 * different segments.
 * <p>
 * We will use the bundle algorithm to do this. We try to create a bundle on a
 * single trajectory. This means it has (and we will allow it) to find bundles
 * consiting of subtrajectories of itself. For a found bundle, we check if it is
 * sufficiently large, and make sure the subtrajectories end up on different
 * segments. We will try to "cut" in the middle (discrete points) of the gap
 * between two consecutive subtrajectories.
 *
 * @author Roel
 */
public class SelfSimilaritySementer implements TrajectorySegmenter {

    private static final int MIN_DIST_FACTOR = 3;

    /**
     * Distance for which we consider subtrajectories similar.
     */
    private final double distance;

    /**
     * Allowed error at the end of subtrajectories.
     */
    private final double lambda;

    /**
     * Whether direction of the subtrajectories should be ignored.
     */
    private final boolean ignoreDirection;

    public SelfSimilaritySementer(double distance, double lambda, boolean ignoreDirection) {
        this.distance = distance;
        this.ignoreDirection = ignoreDirection;
        this.lambda = lambda;
    }

    @Override
    public List<Subtrajectory> segment(Trajectory original) {
        // create sweep line algorithm
        // We explicitly look for bundles of at least size 2, as that may indicate a self-similarity.
        // Also we allow partial bounds, but we will round them later.
        // Finally we have to make sure we allow multiple subtrajectories of the same trajectory
        SweeplineBundleAlgorithm algo = new SweeplineBundleAlgorithm(distance, 2, ignoreDirection, true, true);

        // find bundles
        Set<Bundle> bundles = algo.run(Collections.singletonList(original));
        MaximalSubbundleAlgorithm.removeLambdaSubbundles(bundles, lambda);
        bundles = bundles.stream()
                .filter(bun -> BundleAttribute.MinContinuousLength.applyAsDouble(bun) > MIN_DIST_FACTOR * distance) // remove bundles that are too short
                .sorted(Comparator.comparing((Bundle bun) -> bun.continuousLength()).reversed()) // start with the longest bundles.
                .collect(Collectors.toSet());

        /*
        We treat each bundle separately.
        Starting witht he first one, we split the original trajectory between
        subtrajectories in the bundle.
        
        For each next bundle, we check for each consecutive pair of subtrajectories
        whether we still have to split it (that is, The end of the first subttajectory
        and the start of the second subtrajectory are on the smae segment.)
        
         */
        // tree set representing the segmentation.
        // Sorted by lower bound
        TreeMap<Integer, Subtrajectory> segmentBounds = new TreeMap<>();
        segmentBounds.put(0, new Subtrajectory(original, 0, original.numPoints() - 1));

        for (Bundle bundle : bundles) {
            List<Subtrajectory> subtrajectories = bundle.getSubtrajectories()
                    .stream()
                    .map(sub -> sub.isReverse() ? sub.reverse() : sub)
                    .sorted(Comparator.comparing(sub -> sub.getFromIndex()))
                    .collect(Collectors.toList());
            for (int i = 0; i < subtrajectories.size() - 1; i++) {
                Subtrajectory s1 = subtrajectories.get(i);
                Subtrajectory s2 = subtrajectories.get(i + 1);

                // check if it should be split.
                Entry<Integer, Subtrajectory> s1EndEntry = segmentBounds.floorEntry((int) s1.getToIndex());
                if (s1EndEntry.getKey().equals(segmentBounds.floorKey((int) s2.getFromIndex()))) {
                    // s1 ends and s2 starts on the same segment. Split in between.
                    Subtrajectory toSplit = segmentBounds.remove(s1EndEntry.getKey());

                    // compute split index
                    int index = DoubleMath.roundToInt(Stats.meanOf(s2.getFromIndex(), s1.getToIndex()), RoundingMode.HALF_UP);

                    // split
                    segmentBounds.put((int) toSplit.getFromIndex(), new Subtrajectory(original, toSplit.getFromIndex(), index));
                    segmentBounds.put(index, new Subtrajectory(original, index, toSplit.getToIndex()));

                }

            }

        }
        return new ArrayList<>(segmentBounds.values());

//        // collect all subtrajectories into one list, sorted by starting index
//        List<Subtrajectory> subtrajectories = bundles.stream()
//                .flatMap(bundle -> bundle.getSubtrajectories().stream())
//                .map(sub -> sub.isReverse() ? sub.reverse() : sub)
//                .sorted(Comparator.comparing(sub -> sub.getFromIndex()))
//                .collect(Collectors.toList());
//        
//        // indices at which we will split the original trajectory.
//        List<Integer> splitIndices = new ArrayList<>();
//        subtrajectories.forEach(sub -> System.out.println(sub.getLabel()));
//        // scan the list of subtrajectories.
//        // For each pair of consecutive subtrajectories, we pick a split index
//        // in the middle of the end of the first trajectory and the start of the second
//        // If the start and end match, we pick this matching point
//        // If the trajectorie soverlap, we do not split.
//        
//        for (int i = 0; i < subtrajectories.size() - 1; i++) {
//            Subtrajectory s1 = subtrajectories.get(i);
//            Subtrajectory s2 = subtrajectories.get(i + 1);
//            
//            if (s2.getFromIndex() >= s1.getToIndex()) {
//                int index  = DoubleMath.roundToInt(Stats.meanOf(s2.getFromIndex(), s1.getToIndex()), RoundingMode.HALF_UP);
//                splitIndices.add(index);
//            }
//            
//        }
//        
//        // perfom the split
//        List<Subtrajectory> result = new ArrayList<>();
//        
//        
//        int lastIndex = 0;
//        for (int i = 0; i < splitIndices.size(); i++) {
//            int index = splitIndices.get(i);
//            result.add(new Subtrajectory(original, lastIndex, index));
//            lastIndex = index;
//        }
//        // add last edge
//        result.add(new Subtrajectory(original, lastIndex, original.numPoints() - 1));
//        
//        return result;
    }

}
