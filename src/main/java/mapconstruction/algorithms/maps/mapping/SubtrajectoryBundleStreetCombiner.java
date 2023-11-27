package mapconstruction.algorithms.maps.mapping;

import com.google.common.collect.Range;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.geom.Point2D;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public enum SubtrajectoryBundleStreetCombiner {
    SubBSCombiner;

    /**
     * Array containing for each trajectory in which DISPLAYED bundle they are found and for which range.
     */
    private HashMap<Trajectory, List<Pair<Range<Double>, BundleStreet>>> subtrajectoriesInBundleStreets;


    SubtrajectoryBundleStreetCombiner() {
        // Required to have a constructor.
    }

    public void initialize(){
        subtrajectoriesInBundleStreets = new HashMap<>();
    }

    /**
     * Adding a list of displayed bundles for which all subtrajectories will be indexed.
     *
     * @param listOfBundleStreets, the list of bundles to be added
     */
    public void addMultipleBundleStreets(List<BundleStreet> listOfBundleStreets) {
        for (BundleStreet bs : listOfBundleStreets) {
            if (bs != null) {
                addNewBundleStreet(bs);
            }
        }
    }

    /**
     * Adding a new BundleStreet
     *
     * @param bs the new BundleStreet to be added
     */
    private void addNewBundleStreet(BundleStreet bs) {
        for (Subtrajectory sub : bs.getSubtrajectories()) {
            Trajectory t = sub.getParent();
            double fromIndex = sub.getFromIndex();
            double toIndex = sub.getToIndex();

            // Check whether the trajectory is actually a reverse, if so we reverse it.
            Range<Double> range;
            if (!t.isReverse()) {
                range = Range.closed(fromIndex, toIndex);
            } else {
                int p = t.numPoints() - 1;

                range = Range.closed(p - toIndex, p - fromIndex);
                t = t.reverse();
            }
            Pair<Range<Double>, BundleStreet> combination = new Pair<>(range, bs);

            // Add the bundle with it's range to the list
            if (subtrajectoriesInBundleStreets.containsKey(t)) {
                subtrajectoriesInBundleStreets.get(t).add(combination);
            } else {
                List<Pair<Range<Double>, BundleStreet>> list = new ArrayList<>();
                list.add(combination);
                subtrajectoriesInBundleStreets.put(t, list);
            }
        }
    }


    /**
     * Given a trajectory, it returns which BundleStreets contain the (non-reversed) trajectory at the specifically
     * given index.
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @return a list of bundles for the specifically given index
     */
    public Set<BundleStreet> getBundleStreetsForTrajectoryAndIndex(Trajectory t, double index, double errorMargin) {
        return privateGetBundleStreetsForTrajectoryAndIndex(t, index, errorMargin);
    }


    /**
     * Given a trajectory, it returns which BundleStreets contain the (non-reversed) trajectory at the specifically
     * given indexes.
     *
     * @param t,            the trajectory
     * @param indices,      the index of the trajectory
     * @param intersection, true for Intersection between sets and false for Union between sets.
     * @return a list of bundles for the specifically given index
     */
    public Set<BundleStreet> getBundleStreetsForTrajectoryAndIndex(Trajectory t, List<Double> indices, double errorMargin, boolean intersection) {
        Set<BundleStreet> bundleStreetList = new HashSet<>();
        for (Double index : indices) {
            if (intersection) {
                bundleStreetList.retainAll(privateGetBundleStreetsForTrajectoryAndIndex(t, index, errorMargin));
            } else {
                bundleStreetList.addAll(privateGetBundleStreetsForTrajectoryAndIndex(t, index, errorMargin));
            }
        }
        return bundleStreetList;
    }


    /**
     * Given a trajectory, it returns which BundleStreets contain the (non-reversed) trajectory at the
     * specifically given index.
     *
     * @param t,      the trajectory
     * @param index,  the index of the trajectory
     * @param margin, the margin allowed for our trajectory to move up or down. Standard is 0
     * @return BundleStreet set containing the given index.
     */
    private Set<BundleStreet> privateGetBundleStreetsForTrajectoryAndIndex(Trajectory t, double index, double margin) {
        if (t.isReverse()) {
            t = t.reverse();
            index = t.numPoints() - 1 - index;
        }

        Set<BundleStreet> bundleStreetList = new HashSet<>();
        List<Pair<Range<Double>, BundleStreet>> bundlesForAllRanges = subtrajectoriesInBundleStreets.get(t);

        if (bundlesForAllRanges == null) {
            return bundleStreetList;
        }
        for (Pair<Range<Double>, BundleStreet> combination : bundlesForAllRanges) {
            if (combination.getFirst().contains(index)) {
                bundleStreetList.add(combination.getSecond());
            } else if (margin > 1) {
                double indexBeforeOffset = GeometryUtil.getTrajectoryIndexAfterOffset(t, index, -margin);
                double indexAfterOffset = GeometryUtil.getTrajectoryIndexAfterOffset(t, index, margin);

                if (combination.getFirst().contains(indexBeforeOffset) ||
                        combination.getFirst().contains(indexAfterOffset)) {
                    bundleStreetList.add(combination.getSecond());
                }
            }
        }
        return bundleStreetList;
    }

    /**
     * Destroy all objects for space purposes
     * After this function we can consider this class useless
     */
    public void destroy() {
        subtrajectoriesInBundleStreets = null;
    }

}
