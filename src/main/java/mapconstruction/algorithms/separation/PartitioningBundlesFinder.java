package mapconstruction.algorithms.separation;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Multimaps;
import com.google.common.collect.SetMultimap;
import com.google.common.collect.Sets;
import mapconstruction.algorithms.distance.frechet.SemiWeakFrechetDistance;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.util.*;
import java.util.Map.Entry;

/**
 * Algorithm that finds a pair of bundles that can be used to partition a given
 * bundle.
 * <p>
 * Computes a partition of a given bundle in the context of a set of available
 * bundles. It tries to find a pair of bundles non-empty bundles that pass
 * through the given bundle, and that each have a subbundle that together
 * partition the given bundle.
 * <p>
 * The partition must be such that the partitions do not interact. That is, if
 * we look at the order of the bundles in the incoming direction of the bundle,
 * then the partition should split the trajectories in two parts of consecutive
 * trajectories. Similarly for the outgoing direction.
 * <p>
 * <p>
 * We compute it as follows:
 * <ul>
 * <li> Fist, given a context and a bundle B, we generate a lookup table, giving
 * for a Bundle B', the subtrajectories in B that share a parent with a
 * subtrajectory in B', (which forms a subbundle of B)
 * <li> Then bundle B, we use the table to find two bundles for which the two
 * corresponding sets are disjoint.
 * <li> for these two disjoint sets, we make sure the split is significant
 * enough.
 * <li> We report the pair for which the smallest bundle is as large as
 * possible.
 * </ul>
 *
 * @author Roel
 */
public class PartitioningBundlesFinder {

    /**
     * Computes the pair of bundles in the given context that can be used to
     * partition the given bundle, using the given diatance factor.
     * <p>
     * Returns null if no such partition can be found.
     *
     * @param bundle         the bundle to partition.
     * @param context        Context (of bundles) in which the separation should be
     *                       found
     * @param distanceFactor Factor by which the inter-split distances must
     *                       differ from the intra-split distances
     * @param minInterDist   Minimum distance between the sets of the split to
     *                       consider the split significant.
     * @return pair of bundles in the given context that can be used to
     * partition the given bundle. {@code null} if no such pair exists. The
     * first element of the pair will always be the smaller bundle.
     * @throws NullPointerException if either argument is {@code null}.
     */
    public Pair<Bundle, Bundle> computePartioningBundlePair(Bundle bundle, Set<Bundle> context, double distanceFactor, double minInterDist) {
        Preconditions.checkNotNull(bundle, "bundle == null");
        Preconditions.checkNotNull(context, "context == null");

        Pair<Bundle, Bundle> best = null;
        // try to decompose
        // get bundle map
        Map<Bundle, Set<Subtrajectory>> row = findSubbundleTrajectories(bundle, context);

        for (Entry<Bundle, Set<Subtrajectory>> e1 : row.entrySet()) {
            Bundle b1 = e1.getKey();
            Set<Subtrajectory> set1 = e1.getValue();
            if (set1 == null || b1.size() != set1.size()) {
                continue;
            }
            for (Entry<Bundle, Set<Subtrajectory>> e2 : row.entrySet()) {
                Bundle b2 = e2.getKey();

                Set<Subtrajectory> set2 = e2.getValue();
                if (set2 == null || b2.size() != set2.size()) {
                    continue;
                }

                // Check each ordered pair, where the first is the smallest.
                if (b1 != b2 && b1.size() <= b2.size()) {
                    // check if disjoint, and a valid partitioning.
                    if (Sets.intersection(set1, set2).isEmpty() && set1.size() + set2.size() >= 0.75 * bundle.size()
                            && Math.max(set1.size(), set2.size()) / Math.min(set1.size(), set2.size()) <= 2) {
                        if (isSignificantSplit(set1, set2, distanceFactor, minInterDist)) {
                            if (best == null || b1.size() > best.getFirst().size()) {
                                best = new Pair<>(b1, b2);
                            }
                        }

                    }

                }
            }
        }

        return best;
    }

    private Map<Bundle, Set<Subtrajectory>> findSubbundleTrajectories(Bundle b, Set<Bundle> context) {
        // iterate over all pairs of bundles (b1, b2), ignoring pairs with itself.
        // and pairs for which b1.size <= b2.size.
        SetMultimap<Bundle, Subtrajectory> result = HashMultimap.create();

        for (Bundle b2 : context) {
            if (b != b2 && b.size() > b2.size()) {
                Set<Trajectory> full2 = b2.getParentTrajectories();

                Set<Subtrajectory> subs1 = b.getSubtrajectories();

                for (Subtrajectory s1 : subs1) {
                    if (full2.contains(s1.getParent()) || full2.contains(s1.reverse().getParent())) {
                        result.put(b2, s1);
                    }
                }

            }
        }
        return Multimaps.asMap(result);
    }

    /**
     * Checks the interaction of the partitioning of the given bundle in set1
     * and set2.
     *
     * @param b
     * @param set1
     * @param set2
     * @return
     */
    private boolean checkInteraction(Bundle b, Set<Subtrajectory> set1, Set<Subtrajectory> set2) {
        // Check if the the partition is such that the permutation of
        // trajectories incoming and outgoing niceply partitions in the same
        // set of consecutive trajectories.
        List<Subtrajectory> incoming = GeometryUtil.getIncoming(b);
        List<Subtrajectory> outgoing = GeometryUtil.getOutgoing(b);

        // check placing set1 first
        Set<Subtrajectory> in1 = new HashSet<>(incoming.subList(0, set1.size()));
        Set<Subtrajectory> in2 = new HashSet<>(incoming.subList(0, set2.size()));
        Set<Subtrajectory> out1 = new HashSet<>(outgoing.subList(0, set1.size()));
        Set<Subtrajectory> out2 = new HashSet<>(outgoing.subList(0, set2.size()));

        // check split
        return !((in1.equals(set1) || in2.equals(set2)) && (out1.equals(set1) || out2.equals(set2)));
        //return ! (in1.equals(set1) && out1.equals(set1)) && !(in2.equals(set2) && out2.equals(set2));
    }

    /**
     * Checks whether the split in set1 and set2 is significant enough, meaning
     * that the intra-set similarity is higher than the inter-set similarity.
     * <p>
     * Or put differently, the intra set distances are small and the inter-set
     * distances are larger.
     *
     * @param set1
     * @param set2
     * @return
     */
    private boolean isSignificantSplit(Set<Subtrajectory> set1, Set<Subtrajectory> set2, double distanceFactor, double minInterDist) {
        if (set1.size() == 1 || set2.size() == 1) {
            return false;
        }
        final SemiWeakFrechetDistance distance = new SemiWeakFrechetDistance();
        final double intra1 = GeometryUtil.getMinIntraPairwiseDistance(distance, set1);
        final double intra2 = GeometryUtil.getMinIntraPairwiseDistance(distance, set2);
        final double inter = GeometryUtil.getMinInterPairwiseDistance(distance, set1, set2);

        return inter >= minInterDist && distanceFactor * Math.min(intra1, intra2) < inter;
    }

    /**
     * Checks whether the split of bundle main into set1 and set2 is
     * significant, based on distances to the representativeSubtrajectory of the main bundle.
     * <p>
     * If set1 and set2 interact with each other, we expect the avg distance
     * from set1 or set2 to the representativeSubtrajectory to be more or less equal.
     * However, if they do not interact, then the representativeSubtrajectory lies in one of
     * the set, and all trajectories in the other set have a large distance to
     * it.
     *
     * @param set1
     * @param set2
     * @return
     */
    private boolean isSignificantSplitDistToRep(Bundle main, Set<Subtrajectory> set1, Set<Subtrajectory> set2, double distanceFactor) {
        if (set1.size() == 1 || set2.size() == 1) {
            return false;
        }
        Set<Subtrajectory> set1b = Sets.difference(set1, Collections.singleton(main.getOriginalRepresentative()));
        Set<Subtrajectory> set2b = Sets.difference(set2, Collections.singleton(main.getOriginalRepresentative()));

        final SemiWeakFrechetDistance distance = new SemiWeakFrechetDistance();
        final double dist1 = GeometryUtil.getMinDistanceToTrajectory(distance, set1b, main.getOriginalRepresentative());
        final double dist2 = GeometryUtil.getMinDistanceToTrajectory(distance, set2b, main.getOriginalRepresentative());

        return Math.max(dist1, dist2) / Math.min(dist1, dist2) >= distanceFactor;
    }

}
