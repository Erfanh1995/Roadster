package mapconstruction.algorithms.maps;

import com.google.common.collect.Range;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.algorithms.maps.containers.MergeType;
import mapconstruction.algorithms.maps.containers.MergedBundleStreetPart;
import mapconstruction.algorithms.maps.mapping.ConnectionVertex;
import mapconstruction.algorithms.maps.mapping.RoadMap;
import mapconstruction.algorithms.maps.mapping.RoadSection;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.Collectors;

import static java.util.Map.Entry.comparingByValue;
import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * This class is responsible for drawing on the map.
 *
 * @author Jorricks
 */
public class DrawOnRoadMap {
    private RoadMap roadMap;


    /**
     * Initializes the class.
     *
     * @param roadMap, the roadMap we are looking for. {@Code not null!}
     */
    public DrawOnRoadMap(RoadMap roadMap) {
        if (roadMap == null) {
            System.out.println("Error DrawBetweenIntsOnRoadMap.DrawBetweenIntsOnRoadMap()! roadMap is null.");
        }

        this.roadMap = roadMap;
    }

    /**
     * Draw a new bundleStreet that is connect to at least one intersection
     *
     * @param pointList,       the pointList that we want to merge with the intersection.
     * @param bundleStreet,    this is used for future reference, things as epsilon etc.
     * @param noIntersections, enum type
     */
    public void drawNewBundleStreet(List<Point2D> pointList, BundleStreet bundleStreet, MergeType noIntersections) {

        // Remove points that appear twice in sequence in the pointList
        DrawGeneralFunctions.removeDuplicatePoints(pointList);

        // The normal direction.
        List<MergedBundleStreetPart> mergeRanges = new ArrayList<>();
        List<Double> bestPointDistance = new ArrayList<>();
        findBestMergeRanges(pointList, mergeRanges, bestPointDistance, bundleStreet, noIntersections);

        if (pointList.size() == 0) {
            throw new IllegalArgumentException("Empty pointList");
        }

        if (noIntersections == MergeType.DoubleIntersection) {
            if (!STORAGE.getDatasetConfig().isWalkingDataset() && GeometryUtil.getContinuousLength(pointList) < 150 ||
                    STORAGE.getDatasetConfig().isWalkingDataset() && GeometryUtil.getContinuousLength(pointList) < 50) {
                addSmallRoadEdge(pointList, bundleStreet);
                return;
            }
        }

        createAdditionalStreets(pointList, bestPointDistance, mergeRanges, bundleStreet, noIntersections);

    }

    /**
     * Adding a small roadEdge that could be missed because it is already considered to be merged.
     * Used when MergeType -> MergeType.DoubleIntersections.
     *
     * @param pointList    the point we want to add
     * @param bundleStreet the bundlestreet
     */
    private void addSmallRoadEdge(List<Point2D> pointList, BundleStreet bundleStreet) {
        // First we check whether there is currently a nice small roadEdge between the two intersection
        boolean foundAMergableRoadEdge = false;

        ConnectionVertex startCV = roadMap.getConnectionVertex(pointList.get(0));
        ConnectionVertex endCV = roadMap.getConnectionVertex(pointList.get(pointList.size() - 1));

        for (RoadSection roadSection : roadMap.getRoadSectionsForConnectionVertex(startCV)) {
            if (roadSection.getConnectionVertices().contains(endCV)) {
                if (GeometryUtil.getContinuousLength(roadSection.getPointList()) < 1.5 * GeometryUtil.getContinuousLength(pointList)) {
                    foundAMergableRoadEdge = true;
                }
            }
        }

        if (!foundAMergableRoadEdge) {
            // @BugFix
            roadMap.forceAddRoadSection(new ArrayList<>(Arrays.asList(startCV.getLocation(), endCV.getLocation())),
                    startCV, endCV, bundleStreet);
        }
    }


    /**
     * Finds the best possible merging ranges for a proposed pointList in combination with a chain of roadSections
     * starting from one of the intersections.
     *
     * @param pointList         the points that should be added (that might be in merging distance of the map already)
     * @param mergeRanges       the ranges we are computing. This list is modified (and must be kept).
     * @param bestPointDistance the point distance. This list is modified (and must be kept).
     * @param bundleStreet      the bundleStreet we are trying to add to the road map.
     * @param noIntersections   the number of intersections at the bundleStreet ends.
     */
    private void findBestMergeRanges(List<Point2D> pointList, List<MergedBundleStreetPart> mergeRanges,
                                     List<Double> bestPointDistance, BundleStreet bundleStreet,
                                     MergeType noIntersections) {
        for (int i = 0; i < pointList.size(); i++) {
            bestPointDistance.add(Double.MAX_VALUE);
        }

        int currentIndex = 0;
        while (true) {
            MergedBundleStreetPart mergedBundleStreetPart;

            Set<RoadSection> triedRoadSections = new HashSet<>();
            while (true) {
                int i = triedRoadSections.size();
                mergedBundleStreetPart = checkEarliestIndexesCoveredByRoadEdge(bundleStreet,
                        pointList, currentIndex, noIntersections, triedRoadSections);
                if (i == triedRoadSections.size()) {
                    break;
                }
            }

            DrawGeneralFunctions.processMergedBundleStreetIntoBestPointDistance(
                    mergedBundleStreetPart, bestPointDistance);


            if (mergedBundleStreetPart == null || mergedBundleStreetPart.getLastIndexOfCoveredParts() == null) {
                break;
            }

            mergeRanges.add(mergedBundleStreetPart);

            // We add to the roadSection that we found this part.
            Pair<Integer, Integer> mergeIndexes = mergedBundleStreetPart.getCoveredParts().get(0);
            Pair<Double, Double> ri = mergedBundleStreetPart.getRoadEdgeIndexesByParts().get(0);

            Range<Double> range;
            if (ri.getFirst() > ri.getSecond()) {
                range = Range.closed(ri.getSecond(), ri.getFirst());
            } else {
                range = Range.closed(ri.getFirst(), ri.getSecond());
            }

            // Indexes on the bundleStreetRepresentative
            double startIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
                    bundleStreet.getBundle().getRepresentative(), pointList.get(mergeIndexes.getFirst()));
            double endIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
                    bundleStreet.getBundle().getRepresentative(), pointList.get(mergeIndexes.getSecond()));
            if (startIndex > endIndex) {
                double tempIndex = startIndex;
                startIndex = endIndex;
                endIndex = tempIndex;
            }

            // @ Avoid BundleStreets of length 0
            if (DoubleMath.fuzzyEquals(startIndex, endIndex, 0.1)) {
                startIndex = Math.max(0.0, startIndex - 0.1);
                endIndex = Math.min(bundleStreet.getBundle().getRepresentative().numPoints() - 1, endIndex + 0.1);
            }

            mergedBundleStreetPart.getRoadSection().addNewBundleStreet(
                    bundleStreet.getSubBundleStreet(startIndex, endIndex), range);

            List<Pair<Integer, Integer>> coveredParts = mergedBundleStreetPart.getCoveredParts();
            if (coveredParts.size() > 1) {
                System.out.println("Error DrawImpreciseOnRoadMap.findBestMergeRanges(). coveredParts.size() > 1..");
            }
            currentIndex = coveredParts.get(0).getSecond() + 1;

            // @Fix! Avoided my last o
            if (currentIndex > pointList.size() - 1) {
                break;
            }
        }
    }

    /**
     * Given the index, we try to find the RoadSection in the RoadMap that merges with our pointList at the given index,
     * or the earliest possible index after that, with the extra requirement that of all RoadEdges that can merge at
     * the specific index, we return the one that has the longest merging area.
     *
     * @param bundleStreet      the bundleStreet we are currently trying to add to our road map
     * @param pointList         the pointList of the bundleStreet we want to add to our map (might include new points)
     * @param startIndex        the current index we are looking at.
     * @param noIntersections   the number of intersections at the bundleStreet ends.
     * @param triedRoadSections what the tried road sections were.
     * @return MergedBundleStreetPart which contains all the info required for creating the best possible merge.
     */
    private MergedBundleStreetPart checkEarliestIndexesCoveredByRoadEdge(
            BundleStreet bundleStreet, List<Point2D> pointList, int startIndex, MergeType noIntersections, Set<RoadSection> triedRoadSections) {

        double maxSinglePointDistance;
        double maxMergeDistance;
        double maxMergingHeadingDegreeDifference = ALGOCONSTANTS.getMaxMergingHeadingDirectionDifference();

        switch (noIntersections) {
            case DoubleIntersection:
                maxMergeDistance = ALGOCONSTANTS.getSmallMergeDistance();
                maxSinglePointDistance = ALGOCONSTANTS.getMidMaxSinglePointDistance();
                break;

            case SingleIntersection:
                maxMergeDistance = ALGOCONSTANTS.getMidMergeDistance();
                maxSinglePointDistance = ALGOCONSTANTS.getMidMaxSinglePointDistance();
                break;

            default:
                maxMergeDistance = ALGOCONSTANTS.getLargeMergeDistance();
                maxSinglePointDistance = ALGOCONSTANTS.getLargeMaxSinglePointDistance();
                break;
        }

        RoadSection bestRoadSection = null;
        // Ranges of the pointList which are considered to be part of another roadEdge and hence are merged later on.
        Pair<Integer, Integer> bestMergedPart = null;
        // For the start and end point of the bestMergedPart, we want the indexes on roadSection.
        Pair<Double, Double> bestRoadEdgePart = null;
        Double bestAverageDistance = Double.MAX_VALUE;

        HashMap<RoadSection, Double> startingIndexRoadSection = new HashMap<>();
        HashMap<RoadSection, Double> lastIndexRoadSection = new HashMap<>();
        HashMap<RoadSection, List<Double>> distanceToRoadSection = new HashMap<>();
        HashMap<RoadSection, Double> averageDistanceToRoadSection = new HashMap<>();

        Set<RoadSection> remainingRoadSections = new HashSet<>(roadMap.getPresentRoadSections());
        remainingRoadSections.removeAll(triedRoadSections);

        for (int i = startIndex; i < pointList.size(); i++) {
            int nopInMerge = 1 + (i - startIndex); // number of points in merge
            for (RoadSection roadSection : new HashSet<>(remainingRoadSections)) {

                Pair<Double, Double> pairBestMerge = DrawGeneralFunctions.findBestMerge(roadSection, pointList, i,
                        maxMergingHeadingDegreeDifference);
                double foundMergeDistance = pairBestMerge.getFirst();
                double foundRoadEdgeIndex = pairBestMerge.getSecond();

                if (foundMergeDistance >= maxSinglePointDistance) {
                    remainingRoadSections.remove(roadSection);

                    startingIndexRoadSection.remove(roadSection);
                    lastIndexRoadSection.remove(roadSection);
                    averageDistanceToRoadSection.remove(roadSection);

                } else {
                    List<Double> distances = distanceToRoadSection.get(roadSection);

                    if (foundMergeDistance > maxMergeDistance &&
                            (distances != null && distances.get(i - 1) > maxMergeDistance)) {

                        remainingRoadSections.remove(roadSection);

                        startingIndexRoadSection.remove(roadSection);
                        lastIndexRoadSection.remove(roadSection);
                        averageDistanceToRoadSection.remove(roadSection);

                    } else if (i == startIndex) {

                        startingIndexRoadSection.put(roadSection, foundRoadEdgeIndex);
                        lastIndexRoadSection.put(roadSection, foundRoadEdgeIndex);
                        List<Double> distanceArray = new ArrayList<>();
                        for (int k = 0; k < pointList.size(); k++) {
                            distanceArray.add(Double.MAX_VALUE);
                        }
                        distanceArray.set(i, foundMergeDistance);

                        distanceToRoadSection.put(roadSection, distanceArray);
                        averageDistanceToRoadSection.put(roadSection, foundMergeDistance);

                    } else {

                        lastIndexRoadSection.put(roadSection, foundRoadEdgeIndex);
                        distanceToRoadSection.get(roadSection).set(i, foundMergeDistance);

                        double average = averageDistanceToRoadSection.get(roadSection);
                        average = average * (nopInMerge - 1) / nopInMerge;
                        average += foundMergeDistance / average;
                        averageDistanceToRoadSection.put(roadSection, average);

                    }
                }
            }

            if (remainingRoadSections.size() == 0) {
                if (i == startIndex) {

                    startIndex++;
                    remainingRoadSections = new HashSet<>(roadMap.getPresentRoadSections());
                    remainingRoadSections.removeAll(triedRoadSections);
                    continue;

                } else {
                    // Here we first checked whether we actually found a decent merge part. If not, we continue
                    // searching. If we did found a decent merge part, we simply break;
                    if (bestRoadSection != null) {
                        boolean found = false;
                        for (int j = 0; j < distanceToRoadSection.get(bestRoadSection).size(); j++) {
                            if (distanceToRoadSection.get(bestRoadSection).get(j) <= maxMergeDistance) {
                                found = true;
                            }
                        }
                        if (!found) {
                            startIndex = i + 1;
                            remainingRoadSections = new HashSet<>(roadMap.getPresentRoadSections());
                            remainingRoadSections.removeAll(triedRoadSections);
                            continue;
                        } else {
                            break;
                        }
                    }

                    System.out.println("\n \n SPECIAL CASEEEEEEE \n \n ");
                }
            }

            // Sort our averageDistanceToRoadSection
            averageDistanceToRoadSection = averageDistanceToRoadSection
                    .entrySet().stream().sorted(comparingByValue()).collect(
                            Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue,
                                    (e1, e2) -> e2, LinkedHashMap::new));

            Map.Entry<RoadSection, Double> firstEntry = averageDistanceToRoadSection.entrySet().iterator().next();
            bestRoadSection = firstEntry.getKey();
            bestAverageDistance = firstEntry.getValue();
            bestMergedPart = new Pair<>(startIndex, i);
            bestRoadEdgePart = new Pair<>(
                    startingIndexRoadSection.get(bestRoadSection), lastIndexRoadSection.get(bestRoadSection));
        }

        if (bestRoadSection == null ||
                (bestMergedPart.getFirst().equals(bestMergedPart.getSecond()) && bestAverageDistance > maxMergeDistance)) {
            return null;
        }

        // We avoid roadSections with a larger error margin than the averageDistance to create new roads...
        if (bestAverageDistance > STORAGE.getEvolutionDiagram().getBestEpsilon(bundleStreet.getBundleClass())) {

            // Here we check if we can split up the parts into two
            boolean shouldNotBeMerged = DrawGeneralFunctions.checkIfBundleShouldNotBeMerged(
                    bestRoadSection, bestMergedPart, pointList, bundleStreet);
            if (shouldNotBeMerged) {
                triedRoadSections.add(bestRoadSection);
                return null;
            }
        }


        return new MergedBundleStreetPart(bestRoadSection, pointList, distanceToRoadSection.get(bestRoadSection),
                new ArrayList<>(Arrays.asList(bestMergedPart)), new ArrayList<>(Arrays.asList(bestRoadEdgePart)));
    }

    /**
     * After we found which parts were close to already present RoadEdges, we try to draw the parts that were not
     * close enough and combine this with the roadMap that is already present.
     *
     * @param pointList       list of points which we should have merged with mergeRanges
     *                        assumption is made that it starts at startCV location and ends at endCV location!
     * @param distances       the minimum distances from a pointList to a considered roadEdge.
     * @param mergeRanges     the ranges of the pointList that are considered to be in range/part of a
     *                        RoadSection already present. Hence those ranges will not be drawn.
     * @param bundleStreet    the bundleStreet we are actually drawing now
     * @param noIntersections the number of intersections at the bundleStreet ends.
     */
    private void createAdditionalStreets(List<Point2D> pointList, List<Double> distances,
                                         List<MergedBundleStreetPart> mergeRanges, BundleStreet bundleStreet,
                                         MergeType noIntersections) {
        DrawGeneralFunctions.drawNewRoadEdgesBasedOnMergeRanges(pointList, distances,
                mergeRanges, bundleStreet, roadMap, noIntersections);
    }

}
