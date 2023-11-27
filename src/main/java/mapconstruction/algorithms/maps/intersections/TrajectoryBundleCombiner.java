package mapconstruction.algorithms.maps.intersections;

import com.google.common.collect.Range;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.Pair;

import java.awt.geom.Point2D;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public enum TrajectoryBundleCombiner {
    TBCombiner;

    /**
     * Array containing for each trajectory in which DISPLAYED bundle they are found and for which range.
     */
    private HashMap<Trajectory, List<Pair<Range<Double>, Bundle>>> trajectoriesContainedInDisplayedBundles;

    /**
     * Array containing for each trajectory in which possibly undisplayed bundle they are found and for which range.
     */
    private HashMap<Trajectory, List<Pair<Range<Double>, Bundle>>> trajectoriesContainedInUndisplayedBundles;


    TrajectoryBundleCombiner() {
        // Required to have a constructor.
    }

    public void initialize(){
        trajectoriesContainedInDisplayedBundles = new HashMap<>();
        trajectoriesContainedInUndisplayedBundles = new HashMap<>();
    }

    /**
     * Adding a list of displayed bundles for which all subtrajectories will be indexed.
     *
     * @param listOfBundles, the list of bundles to be added
     */
    public void addMultipleDisplayedBundles(Set<Bundle> listOfBundles) {
        for (Bundle b : listOfBundles) {
            addNewBundle(b, trajectoriesContainedInDisplayedBundles);
        }
    }

    /**
     * Adding a list of (UN)displayed bundles for which all subtrajectories will be indexed.
     *
     * @param listOfBundles, the list of bundles to be added
     */
    public void addMultipleUndisplayedBundles(Set<Bundle> listOfBundles) {
        for (Bundle b : listOfBundles) {
            addNewBundle(b, trajectoriesContainedInUndisplayedBundles);
        }
    }

    private void addNewBundle(Bundle b, HashMap<Trajectory, List<Pair<Range<Double>, Bundle>>> tcib) {
        int bundleClass = STORAGE.getClassFromBundle(b);

        for (Subtrajectory sub : b.getSubtrajectories()) {
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
            Pair<Range<Double>, Bundle> combination = new Pair<>(range, b);

            // Add the bundle with it's range to the list
            if (tcib.containsKey(t)) {
                tcib.get(t).add(combination);
            } else {
                List<Pair<Range<Double>, Bundle>> list = new ArrayList<>();
                list.add(combination);
                tcib.put(t, list);
            }
        }
    }

    /**
     * Given a trajectory, it returns which displayed bundles contain the (non-reversed) trajectory at the specifically
     * given index.
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @return a list of bundles for the specifically given index
     */
    public List<Bundle> getDisplayedBundlesForTrajectoryAndIndex(Trajectory t, double index) {
        return getBundlesForTrajectoryAndIndex(t, index, trajectoriesContainedInDisplayedBundles);
    }

    /**
     * Given a trajectory, it returns which displayed bundles contain the (non-reversed) trajectory at the specifically
     * given index. It applies extra filtering to make sure the representative is still present at this place.
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @return a list of bundles for the specifically given index
     */
    public List<Bundle> getDisplayedRunningBundlesForTrajectoryAndIndex(Trajectory t, double index, Point2D location) {
        List<Bundle> bundles = getBundlesForTrajectoryAndIndex(t, index, trajectoriesContainedInDisplayedBundles);
        IntersectionUtil.filterOutBundlesWithAToSmallRep(bundles, location, 15.0, true);
        return bundles;
    }

    /**
     * Given a trajectory, it returns which possibly undisplayed (but might also be displayed) bundles contain the
     * (non-reversed) trajectory at the specifically given index.
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @return a list of bundles for the specifically given index
     */
    public List<Bundle> getUndisplayedBundlesForTrajectoryAndIndex(Trajectory t, double index) {
        return getBundlesForTrajectoryAndIndex(t, index, trajectoriesContainedInUndisplayedBundles);

    }


    /**
     * Given a trajectory, it returns which possibly undisplayed (but might also be displayed) bundles contain the
     * (non-reversed) trajectory at the specifically given index.
     * It applies extra filtering to make sure the representative is still present at this place.
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @return a list of bundles for the specifically given index
     */
    public List<Bundle> getUndisplayedRunningBundlesForTrajectoryAndIndex(Trajectory t, double index, Point2D location) {
        List<Bundle> bundles = getBundlesForTrajectoryAndIndex(t, index, trajectoriesContainedInUndisplayedBundles);
        IntersectionUtil.filterOutBundlesWithAToSmallRep(bundles, location, 15.0, true);
        return bundles;

    }

    /**
     * Given a trajectory, it returns which possibly bundles contain the (non-reversed) trajectory at the
     * specifically given index.
     *
     * @param t,     the trajectory
     * @param index, the index of the trajectory
     * @param tcib,  the trajectory contained in bundle combinations for either all displayed or undisplayed bundles.
     * @return
     */
    private List<Bundle> getBundlesForTrajectoryAndIndex(Trajectory t, double index,
                                                         HashMap<Trajectory, List<Pair<Range<Double>, Bundle>>> tcib) {
        if (t.isReverse()) {
            t = t.reverse();
            index = t.numPoints() - 1 - index;
        }

        List<Bundle> bundles = new LinkedList<>();
        List<Pair<Range<Double>, Bundle>> bundlesForAllRanges = tcib.get(t);

        if (bundlesForAllRanges == null) {
            return bundles;
        }
        for (Pair<Range<Double>, Bundle> combination : bundlesForAllRanges) {
            if (combination.getFirst().contains(index)) {
                bundles.add(combination.getSecond());
            }
        }
        return bundles;
    }

    /**
     * Destroy all objects for space purposes
     * After this function we can consider this class useless
     */
    public void destroy(){
        trajectoriesContainedInDisplayedBundles = null;
        trajectoriesContainedInUndisplayedBundles = null;
    }

}
