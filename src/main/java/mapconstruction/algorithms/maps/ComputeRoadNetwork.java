package mapconstruction.algorithms.maps;

import com.google.common.collect.Range;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.algorithms.maps.containers.IntersectionsConnector;
import mapconstruction.algorithms.maps.containers.MergeType;
import mapconstruction.algorithms.maps.intersections.ComputeIntersections;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.algorithms.maps.mapping.RoadMap;
import mapconstruction.algorithms.maps.merge.Merger;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;
import static mapconstruction.algorithms.maps.IntersectionStorage.INTERSECTION_STORAGE;
import static mapconstruction.algorithms.maps.intersections.TrajectoryBundleCombiner.TBCombiner;
import static mapconstruction.algorithms.maps.mapping.SubtrajectoryBundleStreetCombiner.SubBSCombiner;
import static mapconstruction.algorithms.maps.roads.BundleIntersectionMapper.BundleIntersectionMapper;


/**
 * Class responsible for computing the final Road Network.
 *
 * @author Jorrick Sleijster
 */
public class ComputeRoadNetwork implements Serializable {
    /**
     * The list of all intersectionsConnectors. Note that the distinct actual roads can be represented by the same
     */
    private List<IntersectionsConnector> intersectionsConnectors;

    /**
     * The list of all intersectionsConnectors. Note that the distinct actual roads can be represented by the same
     */
    private List<BundleStreet> bundleStreetsNotPartOfAnIntersectionConnector;

    /**
     * The RoadMap which contains all the streets and intersections in the end.
     */
    private RoadMap roadMap;

    /**
     * The class that adds RoadSections to the RoadMap.
     */
    private DrawOnRoadMap drawOnRoadMap;

    /**
     * Initiates the computation of the RoadNetwork
     */
    public ComputeRoadNetwork() {
        System.out.println("[RoadNetwork] Using " + STORAGE.getDisplayedBundles().size() + " bundles");

        // Run algorithms
        // First, we start by getting the intersections
        long start = System.currentTimeMillis();

        // We need a trajectory bundle dictionary look up for computing the intersections.
        // Hence we start by initialising it, then once all intersections are computed, we destroy it.
        TBCombiner.initialize();
        List<Intersection> intersections = new ArrayList<>(ComputeIntersections.getIntersections());
        intersections.forEach(Intersection::getLocation);
        // Destroy the hashmaps to get RAM Memory back.
        TBCombiner.destroy();

        long end = System.currentTimeMillis();
        Log.log(LogLevel.INFO, "Intersection", "Intersection computation time: %d ms", end - start);

        // The list of all calculated intersections
        INTERSECTION_STORAGE.setIntersections(intersections);

        start = System.currentTimeMillis();
        // Second we should get all the representative index at which point we cross the intersection, for every bundle
        // that crosses the intersection.
        //      2A. We remove bundles that are closer to other intersections than to this intersection.
        //          This is to prevent annoying cases that occur due to intersection clusters merging.
        removeAllIntersectionBundlesThatAreDominantInAnotherIntersection();
        //      2B. We compute all intersection by bundles around intersection
        computeAllIntersectionIndexesByBundlesAroundIntersections();
        //      2C. At this point we assume we caught all bundles that actually 'hit' the intersection.
        //      Therefore we now only have to check whether the endings of the bundles also hit a specific intersection.
        computeAllIntersectionIndexesByBundlesEndings();

        // Third, we set for each intersection which bundles cross this intersection, at which index and what the
        //      next and previous intersection are.
        BundleIntersectionMapper.setAllIntersectionBundleCombosAndComputeBundleStreets();

        // Fourth, we check which sections between intersections contain a lot of noise / undetected intersections.
        // Generally this means we check for each connection in between intersections, what other bundle parts cover
        // this part (partly) as well. From there we decide what the merging distance would be.
        intersectionsConnectors = new ArrayList<>();
        bundleStreetsNotPartOfAnIntersectionConnector = new ArrayList<>();
        SubBSCombiner.initialize();

        computeIntersectionConnectors();

        end = System.currentTimeMillis();
        Log.log(LogLevel.INFO, "Connectors", "Computing connectors time: %d ms", end - start);

        // Initializing the Roadmap
        roadMap = new RoadMap();
        drawOnRoadMap = new DrawOnRoadMap(roadMap);
//        drawBetweenIntsOnRoadMap = new DrawBetweenIntsOnRoadMap(roadMap);
//        drawImpreciseOnRoadMap = new DrawImpreciseOnRoadMap(roadMap);


        // Fifth, we actually draw the different streets
        //      5A, we draw bundle parts that are connected in-between two intersections.
        start = System.currentTimeMillis();
        System.out.println("ComputeRoadNetwork - IntersectionConnectors with intersections at both ends");
        drawIntersectionConnectorsWithIntersectionsAtBothEnds();
        //      5B, we draw bundle parts that has just one endpoint to an intersection.
        System.out.println("ComputeRoadNetwork - IntersectionConnectors with intersections at one end");
        drawIntersectionConnectorsWithIntersectionsAtOneEnd();
        //      5C, we draw bundle parts that are not connected to an intersection at all.
        System.out.println("ComputeRoadNetwork - IntersectionConnectors with no intersections at it's end");
        drawAllBundleStreetsWithNoIntersectionsConnected();

        // Sixth, we do some clean up.
        //      6A, remove roadSections with same intersection connections that can be merged together.
        MapFiltering.removeRoadEdgesBetweenIntersectionsThatCanBeMerged(roadMap);
        //      6B, remove really small ugly edges sticking out.
        MapFiltering.removeSmallOneSidedEdges(roadMap);
        //      6C, connect single endings together.
        if (STORAGE.getDatasetConfig() != null && STORAGE.getDatasetConfig().isWalkingDataset()) {
//            @ToDo Does not work well!!!!!! Maybe just remove it completely?
//            DrawGeneralFunctions.connectSingleEndings(roadMap, 50.0);
        }

        SubBSCombiner.destroy();

        end = System.currentTimeMillis();
        Log.log(LogLevel.INFO, "Road Map", "Constructing road map time: %d ms", end - start);
        Log.log(LogLevel.INFO, "Road Map", "Bundles present in road map: %d", roadMap.numberOfBundlesPresentInRoadMap());
    }


    /**
     * Get all intersections
     *
     * @return all intersection objects
     */
    public List<Intersection> getIntersection() {
        return INTERSECTION_STORAGE.getIntersections();
    }

    /**
     * Get all intersection connectors
     *
     * @return all intersection connectors
     */
    public List<IntersectionsConnector> getIntersectionsConnectors() {
        return intersectionsConnectors;
    }

    /**
     * Returns an ordered bundle score list from highest score to lowest score.
     *
     * @param bundlesList what collection of bundles we have to order.
     * @return the bundle with the highest score at the first index, and then in decreasing order.
     */
    private List<Bundle> getOrderedBundleScoreList(Collection<Bundle> bundlesList) {
        List<Bundle> bundles = new ArrayList<>(bundlesList);
        bundles.sort((b1, b2) -> {
            double b1Score = getBundleScore(b1);
            double b2Score = getBundleScore(b2);
            return Double.compare(b1Score, b2Score);
        });
        Collections.reverse(bundles);
        return bundles;
    }

    /**
     * Returns an ordered BundleStreet score list from highest score to lowest score.
     *
     * @param bundlesList what collection of bundles we have to order.
     * @return the bundle with the highest score at the first index, and then in decreasing order.
     */
    private List<BundleStreet> getOrderedBundleStreetScoreList(Collection<BundleStreet> bundlesList) {
        List<BundleStreet> bundleStreets = new ArrayList<>(bundlesList);
        bundleStreets.sort((b1, b2) -> {
            double b1Score = getBundleScore(b1.getBundle());
            double b2Score = getBundleScore(b2.getBundle());
            return Double.compare(b1Score, b2Score);
        });
        Collections.reverse(bundleStreets);
        return bundleStreets;
    }

    /**
     * Get's the score of a bundle
     *
     * @param bundle a bundle
     * @return score of the bundle
     */
    private double getBundleScore(Bundle bundle) {
        int bundleClass = STORAGE.getClassFromBundle(bundle);
        double k = bundle.getSubtrajectories().size();
        double e = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);

        // k/((e/4)^2 + 20)
        return k / (Math.pow(e / 4, 2) + 20);
    }

    /**
     * We remove all intersection bundles that are dominant in another intersection and get merged incorrectly into
     * this intersection.
     */
    private void removeAllIntersectionBundlesThatAreDominantInAnotherIntersection() {
        for (Intersection intersection1 : INTERSECTION_STORAGE.getIntersections()) {
            // First we get the nearbyIntersections.
            List<Intersection> nearbyIntersections = new ArrayList<>();
            for (Intersection intersection2 : INTERSECTION_STORAGE.getIntersections()) {
                if (intersection1 != intersection2 &&
                        intersection1.getLocation().distance(intersection2.getLocation()) < 200) {
                    nearbyIntersections.add(intersection2);
                }
            }

            // Second, we calculate the distance to the representative for each bundle and compare it to the other
            // intersection.
            Set<Bundle> allBundlesAroundIntersection1 = new HashSet<>(intersection1.getAllBundlesAroundIntersection());
            for (Bundle bundle : intersection1.getAllBundlesAroundIntersection()) {
                Representative bundleRep = bundle.getRepresentative();
                double index1 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(bundleRep, intersection1.getLocation());
                Point2D location = GeometryUtil.getTrajectoryDecimalPoint(bundleRep, index1);

                double distanceFromInt1 = intersection1.getLocation().distance(location);
                // If we are at most 25 meters away from the bundle, we skip.
                if (distanceFromInt1 < 25) {
                    continue;
                }

                for (Intersection intersection2 : nearbyIntersections) {
                    double staticDistanceFromInt2 = intersection2.getLocation().distance(location);
                    // If the distance to intersection2 from the closest point to intersection1, is too large we skip.
                    if (staticDistanceFromInt2 > 2.0 * distanceFromInt1) {
                        continue;
                    }

                    double index2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(bundleRep, intersection2.getLocation());
                    Point2D location2 = GeometryUtil.getTrajectoryDecimalPoint(bundleRep, index2);
                    // If the distance from the rep to the intersection2 is also larger, than we skip.
                    if (location2.distance(intersection2.getLocation()) > 25) {
                        continue;
                    }

                    // In any case we can say that the bundle is far more closer to intersection2, than to intersection1.
                    // Hence we cut this from the list.
                    allBundlesAroundIntersection1.remove(bundle);
                }
            }
            intersection1.overrideAllBundlesAroundIntersection(allBundlesAroundIntersection1);
        }
    }

    /**
     * Computing all the indexes where a representative meets an intersection.
     * Stored in the BundleIntersectionMapper.
     */
    private void computeAllIntersectionIndexesByBundlesAroundIntersections() {
        for (Intersection intersection : INTERSECTION_STORAGE.getIntersections()) {
            Set<Bundle> bundlesAroundIntersection = intersection.getAllBundlesAroundIntersection();
            for (Bundle bundle : bundlesAroundIntersection) {
                computeIntersectionIndexForCrossingBundle(intersection, bundle, null);
            }
        }
    }

    /**
     * Computes the indexes for which the representative meets an intersection.
     *
     * @param intersection, the intersection we get the index for
     * @param bundle,       the bundle we are looking for.
     * @param range,        the range for which we allow the closest index to be in.
     */
    private void computeIntersectionIndexForCrossingBundle(Intersection intersection, Bundle bundle, Range<Double> range) {
        double maxDistanceFromIntersection = 25.0;
        double noNewIntDetectionWithin = 50.0;
        if (STORAGE.getDatasetConfig().isWalkingDataset()){
            maxDistanceFromIntersection = 15.0;
            noNewIntDetectionWithin = 30.0;
        }

        // First we get the point closest by.
        Point2D location = intersection.getLocation();
        Representative bundleRep = bundle.getRepresentative();
        double repIndex;
        if (range == null) {
            repIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(bundleRep, location);
        } else {
            // Prevent infinite recursion.
            if (DoubleMath.fuzzyEquals(range.lowerEndpoint(), range.upperEndpoint(), 1E-5)) {
                return;
            }
            Subtrajectory subRep = new Subtrajectory(bundleRep, range.lowerEndpoint(), range.upperEndpoint());
            repIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subRep, location);
            repIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subRep, repIndex);
        }

        // Second, we check if this is the first time checking for this combo.
        // If so, we always add the indexPair.
        // If not, we have to check that the distance is within a specific distance.
        // @ToDo this doesn't seem to make sense.. We will always add the repIndex at least once??
        if (range != null) {
            Point2D pointOnRep = GeometryUtil.getTrajectoryDecimalPoint(bundleRep, repIndex);
            if (pointOnRep.distance(location) > maxDistanceFromIntersection) {
                return;
            }
        } else {
            range = Range.closed(0.0, (double) (bundleRep.numPoints() - 1));
        }
        BundleIntersectionMapper.appendIntersectionIndexToBundle(bundle, new Pair<>(repIndex, intersection));

        // Third, we now compute it for the parts before and after this index.
        double newLow = GeometryUtil.getTrajectoryIndexAfterOffset(bundleRep, repIndex, -noNewIntDetectionWithin);
        double newHigh = GeometryUtil.getTrajectoryIndexAfterOffset(bundleRep, repIndex, noNewIntDetectionWithin);

        Range<Double> firstRange = Range.closed(range.lowerEndpoint(), Math.max(range.lowerEndpoint(), newLow));
        Range<Double> secondRange = Range.closed(Math.min(range.upperEndpoint(), newHigh), range.upperEndpoint());

        computeIntersectionIndexForCrossingBundle(intersection, bundle, firstRange);
        computeIntersectionIndexForCrossingBundle(intersection, bundle, secondRange);
    }

    /**
     * We check for each bundle if it's bundle ending is very close to an intersection or whether it can be extended
     * and would then approximately hit an intersection.
     * <p>
     * It is possible that a bundle quits just before the intersection because of irregular trajectory updates,
     * to avoid missing out on these, we also check those because often these are the largest (and best) bundles for
     * predicting the actual road.
     */
    private void computeAllIntersectionIndexesByBundlesEndings() {
        double maxLookAhead = 75;
        if (STORAGE.getDatasetConfig().isWalkingDataset()){
            maxLookAhead = 25;
        }

        for (Bundle bundle : STORAGE.getDisplayedBundles()) {
            Representative bundleRep = bundle.getRepresentative();

            double afterStartingIndex = GeometryUtil.getTrajectoryIndexAfterOffset(bundleRep, 0, maxLookAhead);
            double beforeEndingIndex = GeometryUtil.getTrajectoryIndexAfterOffset(bundleRep, bundleRep.numPoints() - 1, -maxLookAhead);

            // Here we check whether the ending of the bundle actually contained something nice.
            Subtrajectory subRep1 = new Subtrajectory(bundleRep, 0, afterStartingIndex);
            subRep1 = subRep1.reverse();

            Subtrajectory subRep2 = new Subtrajectory(bundleRep, beforeEndingIndex, bundleRep.numPoints() - 1);

            checkForSubrepIfEndingIsNear(subRep1);
            checkForSubrepIfEndingIsNear(subRep2);

            if (afterStartingIndex >= beforeEndingIndex) {
                continue;
            }
            Subtrajectory subRep3 = new Subtrajectory(bundleRep, afterStartingIndex, beforeEndingIndex);
            computeForWholeRepExceptEndingsIfNearIntersection(subRep3);
        }
    }

    /**
     * Checks for a Subtrajectory of the representative whether there is an ending near the intersection.
     * For each iteration of this run, we add at most one bundle.
     *
     * @param subRep The maxLookAhead(75m) long Subtrajectory. The ending(to) of the Subtrajectory is always
     *               an ending of the Representative(either from or to).
     */
    private void checkForSubrepIfEndingIsNear(Subtrajectory subRep) {
        Representative bundleRep = (Representative) subRep.getParent();
        Bundle bundle = bundleRep.getParentBundle();

        double trajectoryLookAheadForDeterminingAngle = 25.0;
        double maxDistanceFromIntersection = 25.0;
        double maxHeadingDirectionChange = 25.0;
        double maxTrajectoryExtension = 75.0; // Can be at most maxLookAhead.

        double closestIntersectionThatFits = Double.MAX_VALUE;
        Intersection closestExtensionIntersection = null;
        double closestIntersectionIndexOnSubRep = 0.0;

        for (Intersection intersection : INTERSECTION_STORAGE.getIntersections()) {
            Point2D location = intersection.getLocation();

            // Just to prevent doing a load of work for nothing..
            if (subRep.getPoint(subRep.numPoints() - 1).distance(location) > maxTrajectoryExtension &&
                    subRep.getPoint(0).distance(location) > maxTrajectoryExtension) {
                continue;
            }

            double subRepIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subRep, location);

            // If the representative is very close to the intersection, add it.
            if (GeometryUtil.getTrajectoryDecimalPoint(subRep, subRepIndex).distance(location) < maxDistanceFromIntersection) {
                double repIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subRep, subRepIndex);
                repIndex = GeometryUtil.convertIndexToNonReverseIndex(bundleRep, repIndex);
                BundleIntersectionMapper.appendIntersectionIndexToBundle(bundle, new Pair<>(repIndex, intersection));
                return;
            }

            // We check if the intersection would be hit if we extended our representative 100 meters further.
            if (!DoubleMath.fuzzyEquals(subRepIndex, subRep.numPoints() - 1, 1E-5)) {
                continue;
            }


            double highestIndex = -1;
            double distanceToIntersection = 0;
            for (int i = 0; i < subRep.numPoints() - 2; i++) {


                double almostEndingIndex = GeometryUtil.getTrajectoryIndexAfterOffset(subRep, i,
                        trajectoryLookAheadForDeterminingAngle);
                Line2D lineSegment = new Line2D.Double(subRep.getPoint(i),
                        GeometryUtil.getTrajectoryDecimalPoint(subRep, almostEndingIndex));
                double angleSegment = GeometryUtil.getHeadingDirection(lineSegment);


                Line2D lineToIntersection = new Line2D.Double(subRep.getPoint(subRep.numPoints() - 1), location);
                double angleToIntersection = GeometryUtil.getHeadingDirection(lineToIntersection);


                if (GeometryUtil.getAbsoluteAngleDifference(angleSegment, angleToIntersection) < maxHeadingDirectionChange) {
                    double distance = GeometryUtil.getTrajectoryDecimalPoint(subRep, almostEndingIndex).distance(location);
                    if (distance < maxTrajectoryExtension && distance < closestIntersectionThatFits) {
                        highestIndex = almostEndingIndex;
                        distanceToIntersection = distance;
                    }
                }
            }

            if (highestIndex > 0) {
                closestIntersectionThatFits = distanceToIntersection;
                closestExtensionIntersection = intersection;
                closestIntersectionIndexOnSubRep = highestIndex;
            }
        }

        if (closestExtensionIntersection != null) {
            double repIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subRep, closestIntersectionIndexOnSubRep);
            repIndex = GeometryUtil.convertIndexToNonReverseIndex(bundleRep, repIndex);
            BundleIntersectionMapper.appendIntersectionIndexToBundle(bundle, new Pair<>(repIndex, closestExtensionIntersection));
        }
    }

    /**
     * Computes for the whole representative except the ending parts(as that is done by another function), whether there
     * is an intersection within reach.
     */
    private void computeForWholeRepExceptEndingsIfNearIntersection(Subtrajectory subRep) {
        for (Intersection intersection : INTERSECTION_STORAGE.getIntersections()) {
            computeForASubRepIfNearIntersection(subRep, intersection);
        }
    }

    /**
     * Computes for a given intersection, and a given subtrajectory whether it is closer by than a given threshold,
     * if so, it adds it to the bundleIntersectionMapper.
     *
     * @param subRep       a rep that does not contain the ending, for the rest can be any part of the trajectory
     * @param intersection the intersection we are currently looking for
     */
    private void computeForASubRepIfNearIntersection(Subtrajectory subRep, Intersection intersection) {
        double maxDistanceFromIntersection = 20.0;

        Representative bundleRep = (Representative) subRep.getParent();
        Bundle bundle = bundleRep.getParentBundle();
        if (subRep.euclideanLength() < 10){
            return;
        }

        double bestIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subRep, intersection.getLocation());

        if (GeometryUtil.getTrajectoryDecimalPoint(subRep, bestIndex).distance(intersection.getLocation())
                < maxDistanceFromIntersection) {
            bestIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subRep, bestIndex);
            BundleIntersectionMapper.appendIntersectionIndexToBundle(bundle, new Pair<>(bestIndex, intersection));

            double indexBeforeBestIndex = GeometryUtil.getTrajectoryIndexAfterOffset(bundleRep, bestIndex, -250);
            double indexAfterBestIndex = GeometryUtil.getTrajectoryIndexAfterOffset(bundleRep, bestIndex, 250);

            if (subRep.getFromIndex() < indexBeforeBestIndex) {
                Subtrajectory beforeBestIndex = new Subtrajectory(bundleRep, subRep.getFromIndex(), indexBeforeBestIndex);
                computeForASubRepIfNearIntersection(beforeBestIndex, intersection);
            }
            if (indexAfterBestIndex < subRep.getToIndex()){
                Subtrajectory afterBestIndex = new Subtrajectory(bundleRep, indexAfterBestIndex, subRep.getToIndex());
                computeForASubRepIfNearIntersection(afterBestIndex, intersection);
            }
        }
    }

    /**
     * Computes all IntersectionsConnector.
     * <p>
     * This means that we look for each BundleStreet which intersections it connects.
     * If it is in between two intersections, we either add it to a intersectionConnector, or we create a new one.
     */
    private void computeIntersectionConnectors() {
        List<Bundle> orderedBundles = getOrderedBundleScoreList(STORAGE.getDisplayedBundles());

        for (Bundle bundle : orderedBundles) {
            List<BundleStreet> bundleStreetList = BundleIntersectionMapper.getBundleStreetsForBundle(bundle);
            // Adding the bundleStreets to our Subtrajectory vs BundleStreet hashMap.
            SubBSCombiner.addMultipleBundleStreets(bundleStreetList);

            for (BundleStreet bundleStreet : bundleStreetList) {

                Intersection sInt = bundleStreet.getStartIntersection();
                Intersection eInt = bundleStreet.getEndIntersection();

                // We want to make sure that our street is not to small.
                if (bundleStreet.getContinuousLength() < ALGOCONSTANTS.getMinBundleStreetLength()) {
                    continue;
                }

                if (sInt != null && eInt != null) {
                    // Both ends are at an intersection.
                    IntersectionsConnector bestIntersectionConnector = null;

                    for (IntersectionsConnector intersectionsConnector : intersectionsConnectors) {
                        Intersection icsInt = intersectionsConnector.getIntersection1();
                        Intersection iceInt = intersectionsConnector.getIntersection2();
                        if ((icsInt == sInt && iceInt == eInt) || (icsInt == eInt && iceInt == sInt)) {
                            if (DoubleMath.fuzzyEquals(
                                    intersectionsConnector.getMainBundleStreet().getContinuousLength(),
                                    bundleStreet.getContinuousLength(),
                                    Math.max(bundleStreet.getContinuousLength() * 0.25, 100))) {
                                bestIntersectionConnector = intersectionsConnector;
                            }
                        }
                    }

                    if (bestIntersectionConnector != null) {
                        bestIntersectionConnector.addBundleStreet(bundleStreet);
                    } else {
                        bestIntersectionConnector = new IntersectionsConnector(sInt, eInt, bundleStreet);
                        intersectionsConnectors.add(bestIntersectionConnector);
                    }
                } else if (sInt != null || eInt != null) {
                    // We want to make sure that our street is not to small.
                    // If we would allow 50 we would allow a whole Subtrajectory to match any point starting at the same intersection..
                    if (bundleStreet.getContinuousLength() < ALGOCONSTANTS.getMinBundleStreetLength()) {
                        continue;
                    }

                    // Starting end or ending end is at an intersection.
                    Intersection theInt = null;
                    Subtrajectory theSub = null;
                    if (sInt != null) {
                        theInt = sInt;
                    } else {
                        theInt = eInt;
                    }
                    theSub = bundleStreet.getRepresentativeSubtrajectoryWithIntersectionAtStart(theInt);


                    IntersectionsConnector bestIntersectionConnector = null;
                    for (IntersectionsConnector intersectionsConnector : intersectionsConnectors) {
                        Intersection icsInt = intersectionsConnector.getIntersection1();
                        Intersection iceInt = intersectionsConnector.getIntersection2();
                        if (icsInt == theInt && iceInt == null || iceInt == theInt && icsInt == null) {
                            Subtrajectory secondSub = intersectionsConnector.getMainBundleStreet()
                                    .getRepresentativeSubtrajectoryWithIntersectionAtStart(theInt);

                            if (Merger.wouldBundleStreetsBeMerged(secondSub, theSub, 50, 30, 25) ||
                                    Merger.wouldBundleStreetsBeMerged(theSub, secondSub, 50, 30, 25)) {
                                bestIntersectionConnector = intersectionsConnector;
                            }
                        }
                    }

                    if (bestIntersectionConnector != null) {
                        bestIntersectionConnector.addBundleStreet(bundleStreet);
                    } else {
                        bestIntersectionConnector = new IntersectionsConnector(sInt, eInt, bundleStreet);
                        intersectionsConnectors.add(bestIntersectionConnector);
                    }
                } else {
                    bundleStreetsNotPartOfAnIntersectionConnector.add(bundleStreet);
                }
            }
        }
    }

    /**
     * Get the intersection connectors in the order of the number of bundles.
     *
     * @return sorted list
     */
    private List<IntersectionsConnector> getSortedIntersectionConnectors() {
        List<IntersectionsConnector> sortedIntersectionsConnectors = new ArrayList<>(this.intersectionsConnectors);

        sortedIntersectionsConnectors
                .sort(Comparator.comparing(IntersectionsConnector::getNoBundleStreets)
                        .thenComparing(IntersectionsConnector::getMainBundleStreetCL));

        Collections.reverse(sortedIntersectionsConnectors);
        return sortedIntersectionsConnectors;
    }

    /**
     * We draw all BundleStreets that are part of an IntersectionConnector which has intersection at both ends.
     * By drawing, we mean we add them to the RoadMap.
     *
     * We first draw all bundleStreets that have a decent match with the intersection location. If we can't find such a
     * match, we save them for last and add them at the end.
     */
    private void drawIntersectionConnectorsWithIntersectionsAtBothEnds() {
        MergeType noIntersections = MergeType.DoubleIntersection;

        List<IntersectionsConnector> remainingDrawsICs = new ArrayList<>();
        List<List<Point2D>> remainingDrawsPointList = new ArrayList<>();
        List<BundleStreet> remainingDrawsBundleStreet = new ArrayList<>();

        for (IntersectionsConnector intersectionsConnector : getSortedIntersectionConnectors()) {
            if (intersectionsConnector.getIntersection1() != null && intersectionsConnector.getIntersection2() != null) {

                // Avoid noisy cases.
                if (intersectionsConnector.getIntersection1() == intersectionsConnector.getIntersection2() &&
                intersectionsConnector.getMainBundleStreet().getContinuousLength() < 150){
                    continue;
                }

                List<List<Point2D>> icRemainingDraws = new ArrayList<>();
                List<BundleStreet> icRemainingDrawsBundle = new ArrayList<>();

                boolean managedToDrawAtLeastOne = false;

                for (BundleStreet bundleStreet : getOrderedBundleStreetScoreList(intersectionsConnector.getBundleStreets())) {
                    Subtrajectory subtrajectory = bundleStreet.getRepresentativeSubtrajectoryWithIntersectionAtStart(
                            intersectionsConnector.getIntersection1());
                    List<Point2D> pointList = new ArrayList<>();

                    for (int i = 0; i < subtrajectory.numPoints(); i++) {
                        pointList.add(subtrajectory.getPoint(i));
                    }


                    // We start by merging the intersection into the pointList
                    Intersection startIntersection = intersectionsConnector.getIntersection1();
                    Intersection endIntersection = intersectionsConnector.getIntersection2();
                    boolean goodMerge = DrawGeneralFunctions.mergeStreetBeginningWithIntersection(pointList, startIntersection.getLocation(), false);

                    Collections.reverse(pointList);
                    boolean secondGoodMerge = DrawGeneralFunctions.mergeStreetBeginningWithIntersection(pointList, endIntersection.getLocation(), false);
                    goodMerge = goodMerge && secondGoodMerge;
                    Collections.reverse(pointList);

                    // Fix for possible reverse
                    if (bundleStreet.getStartIntersection() != intersectionsConnector.getIntersection1()){
                        Collections.reverse(pointList);
                    }

                    if(goodMerge) {
                        managedToDrawAtLeastOne = true;
                        drawOnRoadMap.drawNewBundleStreet(pointList, bundleStreet, noIntersections);
                    } else {
                        icRemainingDraws.add(pointList);
                        icRemainingDrawsBundle.add(bundleStreet);
                    }
                }

                if (!managedToDrawAtLeastOne){
                    drawOnRoadMap.drawNewBundleStreet(
                            icRemainingDraws.get(0), icRemainingDrawsBundle.get(0), noIntersections);
                    icRemainingDraws.remove(0);
                    icRemainingDrawsBundle.remove(0);
                }

                remainingDrawsPointList.addAll(icRemainingDraws);
                remainingDrawsBundleStreet.addAll(icRemainingDrawsBundle);

                for (int i = 0; i < icRemainingDrawsBundle.size(); i++){
                    remainingDrawsICs.add(intersectionsConnector);
                }
            }
        }

        for (int i = 0; i < remainingDrawsBundleStreet.size(); i++){
            drawOnRoadMap.drawNewBundleStreet(remainingDrawsPointList.get(i),
                    remainingDrawsBundleStreet.get(i), noIntersections);
        }
    }

    /**
     * We draw all BundleStreets that are part of an IntersectionConnector which has an intersection at exactly one end.
     * <p>
     * By drawing, we mean we add them to the RoadMap
     */
    private void drawIntersectionConnectorsWithIntersectionsAtOneEnd() {
        MergeType noIntersections = MergeType.SingleIntersection;

        for (IntersectionsConnector intersectionsConnector : getSortedIntersectionConnectors()) {
            if ((intersectionsConnector.getIntersection1() != null &&
                    intersectionsConnector.getIntersection2() == null) ||
                    (intersectionsConnector.getIntersection1() == null &&
                            intersectionsConnector.getIntersection2() != null)) {
                for (BundleStreet bundleStreet : getOrderedBundleStreetScoreList(intersectionsConnector.getBundleStreets())) {
                    Intersection intersection = null;
                    if (intersectionsConnector.getIntersection1() != null) {
                        intersection = intersectionsConnector.getIntersection1();
                    } else {
                        intersection = intersectionsConnector.getIntersection2();
                    }

//
//
//                    Subtrajectory subtrajectory = bundleStreet.getRepresentativeSubtrajectoryWithIntersectionAtStart(intersection);
//                    List<Point2D> pointList = new ArrayList<>();
//                    for (int i = 0; i < subtrajectory.numPoints(); i++) {
//                        pointList.add(subtrajectory.getPoint(i));
//                    }
//                    boolean goodMerge = DrawGeneralFunctions.mergeStreetBeginningWithIntersection(pointList, intersection.getLocation(), false);

                    Subtrajectory subtrajectory = bundleStreet.getRepresentativeSubtrajectory();
                    List<Point2D> pointList = new ArrayList<>();
                    for (int i = 0; i < subtrajectory.numPoints(); i++) {
                        pointList.add(subtrajectory.getPoint(i));
                    }

                    boolean goodMerge;
                    if (bundleStreet.getEndIntersection() != null && bundleStreet.getEndIntersection() == intersection){
                        Collections.reverse(pointList);
                        goodMerge = DrawGeneralFunctions.mergeStreetBeginningWithIntersection(
                                pointList, intersection.getLocation(), false);
                        Collections.reverse(pointList);
                    } else {
                        goodMerge = DrawGeneralFunctions.mergeStreetBeginningWithIntersection(
                                pointList, intersection.getLocation(), false);
                    }

                    drawOnRoadMap.drawNewBundleStreet(pointList, bundleStreet, noIntersections);
                }
            }
        }
    }

    /**
     * We draw all BundleStreets that are part of an IntersectionConnector which has no intersections at it's endings.
     */
    private void drawAllBundleStreetsWithNoIntersectionsConnected() {
        MergeType noIntersections = MergeType.Loner;

        for (BundleStreet bundleStreet : bundleStreetsNotPartOfAnIntersectionConnector) {
            Subtrajectory subtrajectory = bundleStreet.getRepresentativeSubtrajectory();
            List<Point2D> pointList = new ArrayList<>();
            for (int i = 0; i < subtrajectory.numPoints(); i++) {
                pointList.add(subtrajectory.getPoint(i));
            }


            drawOnRoadMap.drawNewBundleStreet(pointList, bundleStreet, noIntersections);
        }
    }

    /**
     * Get's the roadMap
     * @return the roadMap
     */
    public RoadMap getRoadMap() {
        return roadMap;
    }

}
