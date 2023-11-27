package mapconstruction.algorithms.maps.roads;

import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.algorithms.maps.containers.IntersectionBundleCombo;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.trajectories.Bundle;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.geom.Point2D;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public enum BundleIntersectionMapper {
    BundleIntersectionMapper;

    /**
     * A HashMap containing for every bundle all the intersections it came across and at which index(in ascending order)
     */
    private HashMap<Bundle, List<Pair<Double, Intersection>>> bundleListHashMap;

    /**
     * A HashMap containing for all bundles the intersections, indexes and which ones come before and after
     */
    private HashMap<Intersection, Set<IntersectionBundleCombo>> intBundleComboHashMap;

    /**
     * A HashMap containing for every bundle the Bundle Streets it contains.
     */
    private HashMap<Bundle, List<BundleStreet>> bundleBundleStreetsHashMap;

    /**
     * Initializes the bundleListHashMap.
     */
    BundleIntersectionMapper() {
        bundleListHashMap = new HashMap<>();
    }

    /**
     * Gets the intersection indexes for a bundle.
     *
     * @param b, the bundle for which we want to get the intersection indexes.
     * @return all the indexes.
     */
    public List<Pair<Double, Intersection>> getIntersectionIndexesForBundle(Bundle b) {
        return bundleListHashMap.get(b);
    }

    /**
     * Sets te intersection indexes for a specific bundle.
     *
     * @param bundle, the bundle for which we set it
     */
    public void initIntersectionIndexesForBundle(Bundle bundle) {
        List<Pair<Double, Intersection>> indexesForBundle = getIntersectionIndexesForBundle(bundle);
        if (indexesForBundle == null) {
            indexesForBundle = new ArrayList<>();
        }
    }

    /**
     * This function appends an intersectionIndexForBundle to the bundle specific list.
     *
     * @param bundle,                     the bundle for which we have the intersectionIndexForBundle
     * @param intersectionIndexForBundle, contains the intersection and the index of the representative at that int.
     */
    public void appendIntersectionIndexToBundle(Bundle bundle, Pair<Double, Intersection> intersectionIndexForBundle) {
        int bundleClass = STORAGE.getClassFromBundle(bundle);
        Point2D point2D = GeometryUtil.getTrajectoryDecimalPoint(bundle.getRepresentative(), intersectionIndexForBundle.getFirst());

        List<Pair<Double, Intersection>> indexesForBundle = getIntersectionIndexesForBundle(bundle);
        if (indexesForBundle == null) {
            indexesForBundle = new ArrayList<>();
        }

        double indexToAdd = intersectionIndexForBundle.getFirst();
        Intersection intersectionToAdd = intersectionIndexForBundle.getSecond();
        boolean added = false;
        for (int i = 0; i < indexesForBundle.size(); i++) {
            Pair<Double, Intersection> intersectionPair = indexesForBundle.get(i);
            if (intersectionPair.getFirst() > indexToAdd) {

                indexesForBundle.add(i, intersectionIndexForBundle);
                added = true;
                break;
            }
        }
        // Last one on the list.
        if (!added) {
            indexesForBundle.add(intersectionIndexForBundle);
        }
        bundleListHashMap.put(bundle, indexesForBundle);
    }

    /**
     * Get the neighbouring intersections with indexes for a specific bundle and a specific intersection.
     */
    public List<Pair<Double, Intersection>> getNeighbourIntersectionWithIndexes(Bundle bundle, Intersection intersection) {
        List<Pair<Double, Intersection>> indexesForBundle = getIntersectionIndexesForBundle(bundle);

        List<Pair<Double, Intersection>> neighbours = new ArrayList<>();
        Pair<Double, Intersection> previousPair = null;
        for (int i = 0; i < indexesForBundle.size(); i++) {
            Pair<Double, Intersection> intersectionPair = indexesForBundle.get(i);
            if (intersection.equals(intersectionPair.getSecond())) {
                if (previousPair != null) {
                    neighbours.add(previousPair);
                }
                if (i + 1 < indexesForBundle.size()) {
                    neighbours.add(indexesForBundle.get(i + 1));
                }
                break;
            }
            previousPair = intersectionPair;
        }
        return neighbours;
    }

    /**
     * Get the neighbouring intersections for a specific bundle and a specific intersection.
     */
    public List<Intersection> getNeighbourIntersection(Bundle bundle, Intersection intersection) {
        List<Pair<Double, Intersection>> neighbourIndexes = getNeighbourIntersectionWithIndexes(bundle, intersection);
        List<Intersection> neighbourIntersections = new ArrayList<>();

        for (Pair<Double, Intersection> intersectionPair : neighbourIndexes) {
            neighbourIntersections.add(intersectionPair.getSecond());
        }

        return neighbourIntersections;
    }

    /**
     * For all intersections, we check which bundles are around an intersection and create IntersectionBundleCombo objects for it.
     */
    public void setAllIntersectionBundleCombosAndComputeBundleStreets() {
        if (intBundleComboHashMap != null) {
            return;
        }
        intBundleComboHashMap = new HashMap<>();
        bundleBundleStreetsHashMap = new HashMap<>();
        for (Bundle bundle : STORAGE.getDisplayedBundles()) {

            List<BundleStreet> bundleStreetList = new ArrayList<>();
            List<Pair<Double, Intersection>> intersectionIndexesList = bundleListHashMap.get(bundle);

            if (intersectionIndexesList == null || intersectionIndexesList.size() == 0) {
                BundleStreet longBundleStreet = new BundleStreet(bundle, 0, null,
                        bundle.getRepresentative().numPoints() - 1, null,
                        bundleStreetList.size());
                bundleStreetList.add(longBundleStreet);
            } else {

                BundleStreet previousBundleStreet = null;

                for (int i = 0; i < intersectionIndexesList.size(); i++) {
                    Pair<Double, Intersection> iip = intersectionIndexesList.get(i);
                    Double intIndex = iip.getFirst();

//                    For debugging purposes
//                    int bundleClass = STORAGE.getClassFromBundle(bundle);
//                    Point2D pointAtIndex = GeometryUtil.getTrajectoryDecimalPoint(bundle.getRepresentative(), intIndex);

                    Intersection intersection = iip.getSecond();

                    // Get/create the bundle street before the intersection.
                    BundleStreet beforeIntersectionBundleStreet = null;
                    if (previousBundleStreet == null) {
                        // Avoid BundleStreet of 0 length
                        if (intIndex >= 0.01){
                            beforeIntersectionBundleStreet = new BundleStreet(bundle, 0, null,
                                    intIndex, intersection, bundleStreetList.size());
                            bundleStreetList.add(beforeIntersectionBundleStreet);
                        }
                    } else {
                        beforeIntersectionBundleStreet = previousBundleStreet;
                    }

                    // Create the bundle street after the intersection.
                    BundleStreet afterIntersectionBundleStreet = null;
                    if (i + 1 < intersectionIndexesList.size()) {
                        Pair<Double, Intersection> nextIIP = intersectionIndexesList.get(i + 1);

                        // Avoid BundleStreet of 0 length
                        if (Math.abs(nextIIP.getFirst() - intIndex) > 0.1){
                            afterIntersectionBundleStreet = new BundleStreet(bundle, intIndex, intersection,
                                    nextIIP.getFirst(), nextIIP.getSecond(), bundleStreetList.size());
                        }
                    } else {
                        // Avoid BundleStreet of 0 length
                        if (Math.abs(bundle.getRepresentative().numPoints() - 1 - intIndex) > 0.1){
                            afterIntersectionBundleStreet = new BundleStreet(bundle, intIndex, intersection,
                                    bundle.getRepresentative().numPoints() - 1, null,
                                    bundleStreetList.size());
                        }
                    }
                    if (afterIntersectionBundleStreet != null) {
                        bundleStreetList.add(afterIntersectionBundleStreet);
                    }

                    // Create the object
                    IntersectionBundleCombo intersectionBundleCombo = new IntersectionBundleCombo(bundle, intersection,
                            intIndex, beforeIntersectionBundleStreet, afterIntersectionBundleStreet);


                    // Add it to the HashMap
                    Set<IntersectionBundleCombo> intersectionBundleComboSet;
                    if (intBundleComboHashMap.containsKey(intersection)) {
                        intersectionBundleComboSet = intBundleComboHashMap.get(intersection);
                    } else {
                        intersectionBundleComboSet = new HashSet<>();
                    }
                    intersectionBundleComboSet.add(intersectionBundleCombo);
                    intBundleComboHashMap.put(intersection, intersectionBundleComboSet);

                    previousBundleStreet = afterIntersectionBundleStreet;
                }
            }

            bundleBundleStreetsHashMap.put(bundle, bundleStreetList);

        }

        for (Map.Entry<Intersection, Set<IntersectionBundleCombo>> intersectionSetEntry : intBundleComboHashMap.entrySet()) {
            Intersection intersection = intersectionSetEntry.getKey();
            intersection.setBundlesConnectedToIntersection(intersectionSetEntry.getValue());
        }
    }

    /**
     * Returns all bundleStreets for a specific bundle.
     *
     * @param bundle, the bundle.
     * @return all bundleStreets
     */
    public List<BundleStreet> getBundleStreetsForBundle(Bundle bundle) {
        List<BundleStreet> bundleStreets = bundleBundleStreetsHashMap.get(bundle);
        if (bundleStreets == null) {
            System.out.println("Error in BundleIntersectionMapper. A bundle request is not in the bundleStreetHashmap");
            return new ArrayList<>();
        }
        return bundleStreets;
    }
}
