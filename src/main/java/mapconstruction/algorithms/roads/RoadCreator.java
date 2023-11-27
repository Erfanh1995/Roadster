package mapconstruction.algorithms.roads;

import com.google.common.collect.Range;
import com.google.common.collect.RangeSet;
import com.google.common.collect.TreeRangeSet;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.util.*;
import java.util.stream.Collectors;

/**
 * creates a set of trajectories that are supposed to represent roads based on a set of bundles.
 * The "Roads" are sections of representatives of the bundles.
 * <p>
 * It will work as follows:
 * - We handle the bundles in descending size.
 * <p>
 * - We pick the representativeSubtrajectory as a piece of "road".
 * <p>
 * - We remove the points in the bundle from for each trajectory going through the bundle,
 * essentially splitting the other trajectories up.
 * <p>
 * - We collect athh the small sections and return the set.
 *
 * @author Roel
 */
public class RoadCreator {


    public List<Subtrajectory> createRoads(Collection<Bundle> bundles) {

        // sort bundles in decreasing size.
        List<Bundle> bundleList = bundles.stream()
                .sorted(Comparator.comparing(Bundle::size).reversed())
                .collect(Collectors.toList());

        // Maping from all full trajectories represented by the bundles
        // to a set of ranges where we have not yet removed any sections yet
        Map<Trajectory, RangeSet<Double>> notYetRemoved = new HashMap<>();

        // Add all trajectories with corresponding intervals.
        for (Bundle b : bundleList) {
            for (Subtrajectory sub : b.createNonInversed()) {
                notYetRemoved.putIfAbsent(sub.getParent(), TreeRangeSet.create());
                notYetRemoved.get(sub.getParent()).add(Range.closed(sub.getFromIndex(), sub.getToIndex()));
            }
        }

        // Process everything
        List<Subtrajectory> result = new ArrayList<>();

        for (Bundle b : bundleList) {
            Subtrajectory rep = b.getOriginalRepresentative();
            rep = rep.isReverse() ? rep.reverse() : rep;

            // get intervals of the representativeSubtrajectory still present.
            Trajectory parent = rep.getParent();
            RangeSet<Double> repRanges = notYetRemoved.get(parent).subRangeSet(Range.closed(rep.getFromIndex(), rep.getToIndex()));

            // create subtrajectories
            repRanges.asRanges().forEach(range -> result.add(new Subtrajectory(parent, range.lowerEndpoint(), range.upperEndpoint())));

            // remove ranges from all subtrajectories in this bundle
            for (Subtrajectory sub : b.createNonInversed()) {
                notYetRemoved.get(sub.getParent()).remove(Range.closed(sub.getFromIndex(), sub.getToIndex()));
            }

        }

        return result;

    }

}
