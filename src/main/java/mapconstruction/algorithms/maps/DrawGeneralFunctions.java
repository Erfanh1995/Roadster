package mapconstruction.algorithms.maps;

import com.google.common.collect.Range;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.algorithms.maps.containers.MergeType;
import mapconstruction.algorithms.maps.containers.MergedBundleStreetPart;
import mapconstruction.algorithms.maps.mapping.ConnectionVertex;
import mapconstruction.algorithms.maps.mapping.RoadMap;
import mapconstruction.algorithms.maps.mapping.RoadSection;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;
import mapconstruction.util.Triplet;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.Collectors;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;
import static mapconstruction.algorithms.maps.mapping.SubtrajectoryBundleStreetCombiner.SubBSCombiner;

/**
 * This class is responsible for smaller functions used by the class DrawOnRoadMap
 *
 * @author Jorricks
 */
public class DrawGeneralFunctions {
    /**
     * Removes duplicate points on the pointList at the start or at the end.
     *
     * @param pointList the list of points.
     */
    public static void removeDuplicatePoints(List<Point2D> pointList) {
        for (int i = 0; i < pointList.size() - 1; i++) {
            if (pointList.get(i).equals(pointList.get(i + 1))) {
                pointList.remove(i + 1);
                i = 0;
            }
        }
    }

    /**
     * Merges the bestPointDistance of the mergedBundleStreet with the mergedList bestPointDistance
     *
     * @param mergedBundleStreetPart a mergedBundleStreetPart.
     * @param bestPointDistance      a list of distances to the closest roadEdges
     */
    public static void processMergedBundleStreetIntoBestPointDistance(MergedBundleStreetPart mergedBundleStreetPart,
                                                                      List<Double> bestPointDistance) {
        if (mergedBundleStreetPart != null) {
            List<Double> bestDistanceOfMergedBSP = mergedBundleStreetPart.getBestPointDistance();
            for (int i = 0; i < bestDistanceOfMergedBSP.size(); i++) {
                if (bestDistanceOfMergedBSP.get(i) < bestPointDistance.get(i)) {
                    bestPointDistance.set(i, bestDistanceOfMergedBSP.get(i));
                }
            }
        }
    }

    /**
     * Given the roadSection, the pointList and the starting point of the pointList, we compare the whole roadSection with
     * the specific point of the pointList at index pointListIndex. We try to find the edge on the roadSection that is
     * closest to our point. Here we only compare with parts of the roadSection that has a heading direction similar to
     * either the edge before or after the point.
     *
     * @param roadSection                   the roadSection we are comparing our pointList with
     * @param pointList                     the pointList
     * @param pointListIndex                the index indicating which point we are looking at in our pointlist.
     * @param maxHeadingDirectionDifference the maximum difference in angle between the point and roadEdges part.
     * @return (Distance, Index) a pair containing the distance and the index of the point closest to the point with
     * a heading direction similar to that around the point of the roadSection.
     */
    public static Pair<Double, Double> findBestMerge(RoadSection roadSection, List<Point2D> pointList,
                                                     int pointListIndex, double maxHeadingDirectionDifference) {
        Trajectory ret = roadSection.getTrajectory();
        Point2D pointToMerge = pointList.get(pointListIndex);
        double minDistance = Double.MAX_VALUE;
        double bestIndex = -1.0;
        double indexOnRet;

        if (pointListIndex > 0) {
            Line2D edgeBeforePTM = new Line2D.Double(pointList.get(pointListIndex - 1), pointList.get(pointListIndex));
            double hd1 = GeometryUtil.getHeadingDirection(edgeBeforePTM);
            indexOnRet = GeometryUtil.getIndexOfTrajectoryClosestToPointByAngle(ret, pointToMerge, hd1,
                    maxHeadingDirectionDifference, true);
            Point2D pointOnRet = GeometryUtil.getTrajectoryDecimalPoint(ret, indexOnRet);
            minDistance = pointOnRet.distance(pointToMerge);
            bestIndex = indexOnRet;
        }
        if (pointListIndex < pointList.size() - 1) {
            Line2D edgeAfterPTM = new Line2D.Double(pointList.get(pointListIndex), pointList.get(pointListIndex + 1));
            double hd2 = GeometryUtil.getHeadingDirection(edgeAfterPTM);
            indexOnRet = GeometryUtil.getIndexOfTrajectoryClosestToPointByAngle(ret, pointToMerge, hd2,
                    maxHeadingDirectionDifference, true);

            Point2D pointOnRet = GeometryUtil.getTrajectoryDecimalPoint(ret, indexOnRet);
            if (pointOnRet.distance(pointToMerge) < minDistance) {
                minDistance = pointOnRet.distance(pointToMerge);
                bestIndex = indexOnRet;
            }
        }

        return new Pair<>(minDistance, bestIndex);
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
     * @param roadMap         the current roadMap
     * @param noIntersections the number of intersections at the bundleStreet ends.
     */
    public static void drawNewRoadEdgesBasedOnMergeRanges(List<Point2D> pointList, List<Double> distances,
                                                          List<MergedBundleStreetPart> mergeRanges,
                                                          BundleStreet bundleStreet, RoadMap roadMap, MergeType noIntersections) {
        // @ToDo write a function that instead of splitSectionAndGetVertex, adds the vertex as extra point to the
        //      roadSection, if the distance to the edge is less than 1 but the distance to the point is more.
        //      Then test test test..

        ConnectionVertex startCV = roadMap.getConnectionVertex(pointList.get(0));
        ConnectionVertex endCV = roadMap.getConnectionVertex(pointList.get(pointList.size() - 1));

        double maxSPSODTR = ALGOCONSTANTS.getMidMaxSinglePointDistance();
        if (noIntersections == MergeType.Loner) {
            maxSPSODTR = ALGOCONSTANTS.getLargeMaxSinglePointDistance();
        }

        // Now what if there were no RoadEdges in the RoadMap yet? Then we should just draw the complete pointList
        // starting at startCV and ending at endCV.
        if (mergeRanges.size() == 0) {
            pointList.set(0, startCV.getLocation());
            pointList.set(pointList.size() - 1, endCV.getLocation());
            roadMap.addRoadSection(pointList, startCV, endCV, bundleStreet);
            return;
        }

        // This is a vertex of the roadEdge which we'll connect to part of our pointList
        Triplet<Point2D, Integer, Boolean> splitVertexTriplet = null;
        Triplet<Point2D, Integer, Boolean> mergeVertexTriplet;

        RoadSection splitVertexRoadSection = null;
        // This is the last index of the pointList that is considered to be merged.
        // This means that splitVertex is connected to the point at 'lastMergeIndex + 1'.
        int lastMergeIndex = -1;
        // Sometimes gap appeared in our final roadMap. This is because the whole range of the trajectory would be
        // within mergeRanges, however, the roadSections changed over time with no connection between them.
        // Therefore we specifically check for this now.
        MergedBundleStreetPart lastMergeRange = null;


        for (MergedBundleStreetPart mergeRange : mergeRanges) {
            for (int j = 0; j < mergeRange.getCoveredParts().size(); j++) { // Note: coverparts.size() =(always) 1

                Pair<Integer, Integer> pair = mergeRange.getCoveredParts().get(j);
                int mergeStart = pair.getFirst();
                int mergeEnd = pair.getSecond();

                Pair<Double, Double> pair1 = mergeRange.getRoadEdgeIndexesByParts().get(j);
                double mergeStartIndexOnRoadEdge = pair1.getFirst();
                double mergeEndIndexOnRoadEdge = pair1.getSecond();


                if (splitVertexTriplet != null) {
                    // Here we check that if the next mergedRange (of the next MergedBundleStreetPart, and thus the
                    // next RoadSection), should be considered a continuation of the previous merged part, if so we
                    // continue our search.
                    if (lastMergeIndex >= mergeStart - 1) {
                        lastMergeIndex = mergeEnd;
                        // This would only makes sense if the edge is longer than 15
                        splitVertexTriplet = findBestPlaceOnRoadEdgeToConnect(pointList, mergeEnd, mergeEndIndexOnRoadEdge,
                                mergeRange.getRoadSection(), false, bundleStreet);

                        splitVertexRoadSection = mergeRange.getRoadSection();
                        checkIfConnectionBetweenRoadEdgesShouldBeDrawn(
                                lastMergeRange, mergeRange, bundleStreet, noIntersections, roadMap);
                        continue;
                    }

                    // When it is not a continuation of the previous mergeRange, but actually a new mergeRange.
                    // Then we want to draw the part that was in between the two mergeRanges.
                    // 1. The case when we need to draw an unmergedPart inbetween two mergedParts.

                    mergeVertexTriplet = findBestPlaceOnRoadEdgeToConnect(pointList, mergeStart, mergeStartIndexOnRoadEdge,
                            mergeRange.getRoadSection(), true, bundleStreet);

                    List<Point2D> partPointList = getUpdatedPointListBasedOnTriplets(pointList, splitVertexTriplet, mergeVertexTriplet);
//                    partPointList.add(splitVertexLocation);
//                    for (int i = lastMergeIndex + 1; i < mergeStart; i++) {
//                        partPointList.add(pointList.get(i));
//                    }
//                    partPointList.add(mergeVertexLocation);

                    // If we are sure we actually want to split.
                    if (shouldPointBeAdded(partPointList, distances, maxSPSODTR)) {
                        ConnectionVertex splitVertex = roadMap.splitSectionAndGetVertex(
                                splitVertexRoadSection, splitVertexTriplet.getFirst(), null);
                        ConnectionVertex mergeVertex = roadMap.splitSectionAndGetVertex(
                                mergeRange.getRoadSection(), mergeVertexTriplet.getFirst(), null);

                        addEdgeToRoadMapWithSpecialMerge(partPointList, splitVertex, mergeVertex,
                                bundleStreet, roadMap, noIntersections);
                    } else {
                        checkIfConnectionBetweenRoadEdgesShouldBeDrawn(
                                lastMergeRange, mergeRange, bundleStreet, noIntersections, roadMap);
                    }

                } else {
                    // 2. This is in the case that we need to draw an unmergedPart from the start to a mergedPart.
                    if (mergeStart > 0) {
                        if (mergeStart == 1 && distances.get(0) <= maxSPSODTR) {
                            continue;
                        }

                        mergeVertexTriplet = findBestPlaceOnRoadEdgeToConnect(pointList, mergeStart, mergeStartIndexOnRoadEdge,
                                mergeRange.getRoadSection(), true, bundleStreet);


                        List<Point2D> partPointList = getUpdatedPointListBasedOnTriplets(pointList, null, mergeVertexTriplet);
//                        for (int i = 0; i < mergeStart; i++) {
//                            partPointList.add(pointList.get(i));
//                        }
//                        partPointList.add(mergeVertexLocation);

                        if (shouldPointBeAdded(partPointList, null, maxSPSODTR)) {
                            ConnectionVertex mergeVertex = roadMap.splitSectionAndGetVertex(
                                    mergeRange.getRoadSection(), mergeVertexTriplet.getFirst(), null);

                            addEdgeToRoadMapWithSpecialMerge(partPointList, startCV, mergeVertex,
                                    bundleStreet, roadMap, noIntersections);
                        }
                    }
                }
                if (mergeEnd != pointList.size() - 1) {
                    splitVertexTriplet = findBestPlaceOnRoadEdgeToConnect(pointList, mergeEnd, mergeEndIndexOnRoadEdge,
                            mergeRange.getRoadSection(), false, bundleStreet);

                    splitVertexRoadSection = mergeRange.getRoadSection();
                }

                lastMergeIndex = mergeEnd;
                if (mergeEnd == pointList.size() - 1) {
                    splitVertexTriplet = null;
                    break;
                }
                lastMergeRange = mergeRange;
            }
            lastMergeRange = mergeRange;
        }

        // 3. This is in the case that we had a last merged part and nothing comes after it.
        if (splitVertexTriplet != null) {

            if (lastMergeIndex >= pointList.size() - 1 || (lastMergeIndex == pointList.size() - 2 &&
                    distances.get(pointList.size() - 1) <= maxSPSODTR)) {
                return;
            }

            List<Point2D> partPointList = getUpdatedPointListBasedOnTriplets(pointList, splitVertexTriplet, null);

//            partPointList.add(splitVertexLocation);
//            for (int i = lastMergeIndex + 1; i < pointList.size(); i++) {
//                partPointList.add(pointList.get(i));
//            }

            // Protection that if there is just one point, we stop.

            if (shouldPointBeAdded(partPointList, null, maxSPSODTR)) {
                ConnectionVertex splitVertex = roadMap.splitSectionAndGetVertex(
                        splitVertexRoadSection, splitVertexTriplet.getFirst(), null);

                addEdgeToRoadMapWithSpecialMerge(partPointList, splitVertex, endCV,
                        bundleStreet, roadMap, noIntersections);
            }
        }
    }





    private static List<Point2D> getUpdatedPointListBasedOnTriplets(
            List<Point2D> pointList, Triplet<Point2D, Integer, Boolean> splitVertexTriplet,
            Triplet<Point2D, Integer, Boolean> mergeVertexTriplet) {
        int startIndex = 0;
        int endIndex = pointList.size();
        boolean startIndexComputedBySubs = false;
        boolean endIndexComputedBySubs = false;
        List<Point2D> newPointList = new ArrayList<>();

        if (splitVertexTriplet != null) {
            startIndex = splitVertexTriplet.getSecond();
            startIndexComputedBySubs = splitVertexTriplet.getThird();
        }

        if (mergeVertexTriplet != null) {
            endIndex = mergeVertexTriplet.getSecond();
            endIndexComputedBySubs = mergeVertexTriplet.getThird();
        }

        if (startIndex >= endIndex){
            throw new IllegalArgumentException("Not allowed");
        }


        for (int i = startIndex + 1; i < endIndex; i++) {
            newPointList.add(pointList.get(i));
        }

        if (startIndexComputedBySubs) {
            mergeStreetBeginningWithIntersection(newPointList, splitVertexTriplet.getFirst(), false);
        } else if (splitVertexTriplet != null) {
            newPointList.add(0, splitVertexTriplet.getFirst());
        } else {
            newPointList.add(0, pointList.get(startIndex));
        }

        if (endIndexComputedBySubs) {
            Collections.reverse(newPointList);
            mergeStreetBeginningWithIntersection(newPointList, mergeVertexTriplet.getFirst(), false);
            Collections.reverse(newPointList);
        } else if (mergeVertexTriplet != null) {
            newPointList.add(mergeVertexTriplet.getFirst());
        }
//        else {
//            // This is by default true, hence commented out
//            newPointList.add(pointList.get(endIndex));
//        }

        return newPointList;
    }

    /**
     * This function checks whether the startCV and endCV are correct.
     * It checks for the the first line, A, whether there is another edge, B, starting at the first point, that ends
     * before the end of A, meaning, we should start at the end of B instead of at startCV.
     * We do the exact same thing also the other way around.
     *
     * @param oriPointList    the pointList
     * @param startCV         the starting connectionVertex
     * @param endCV           the ending connectionVertex
     * @param bundleStreet    the bundleStreet we are drawing
     * @param roadMap         the roadMap we are drawing on
     * @param noIntersections the number of intersections at the bundleStreet ends.
     */
    private static void addEdgeToRoadMapWithSpecialMerge(List<Point2D> oriPointList, ConnectionVertex startCV,
                                                         ConnectionVertex endCV, BundleStreet bundleStreet,
                                                         RoadMap roadMap, MergeType noIntersections) {

        List<Point2D> pointList = new ArrayList<>(oriPointList);
        startCV = checkIfSingleEdgeShouldBeMerged(pointList, startCV, roadMap, noIntersections);
        pointList.set(0, startCV.getLocation());

        Collections.reverse(pointList);
        endCV = checkIfSingleEdgeShouldBeMerged(pointList, endCV, roadMap, noIntersections);
        pointList.set(0, endCV.getLocation());
        Collections.reverse(pointList);

        roadMap.addRoadSection(pointList, startCV, endCV, bundleStreet);
    }

    /**
     * We check whether there is another roadSection supporting the first line of the fullPointList. This would mean we
     * could merge these points into the first line of the fullPointList and get a more accurate representation.
     * The assumption is that the startCV is a point that has multiple roadEdges connected to it.
     *
     * @param fullPointList   the points we want to add to the roadmap
     * @param startCV         the starting ConnectionVertex
     * @param roadMap         the roadMap.
     * @param noIntersections the number of intersections at the bundleStreet ends.
     */
    private static ConnectionVertex checkIfSingleEdgeShouldBeMerged(List<Point2D> fullPointList,
                                                                    ConnectionVertex startCV,
                                                                    RoadMap roadMap, MergeType noIntersections) {
        double maxAngleDiff = ALGOCONSTANTS.getMaxMergingHeadingDirectionDifference();
        double maxDistance;
        if (noIntersections == MergeType.DoubleIntersection) {
            maxDistance = ALGOCONSTANTS.getSmallMergeDistance();
        } else if (noIntersections == MergeType.SingleIntersection) {
            maxDistance = ALGOCONSTANTS.getMidMergeDistance();
        } else {
            maxDistance = ALGOCONSTANTS.getLargeMergeDistance();
        }

        if (fullPointList.size() < 2) {
            return startCV;
        }
        List<Point2D> pointList = new ArrayList<>(Arrays.asList(fullPointList.get(0), fullPointList.get(1)));

        if (startCV.getLocation().distance(pointList.get(0)) > pointList.get(1).distance(pointList.get(0))) {
            System.out.println("Error DrawGeneralFunctions.checkIfSingleEdgeShouldBeMerged(). Error weird way around");
        }
        if (pointList.get(0).distance(pointList.get(1)) < 1) {
            System.out.println("Error DrawGeneralFunctions.checkIfSingleEdgeShouldBeMerged(). Distance to close");
            System.out.println(pointList.get(0).equals(pointList.get(1)));
        }

        double bestMaxIndex = -100.0;
        int bestMaxIndexOnRoadSection = 0;
        RoadSection bestMaxRoadSection = null;

        Trajectory trajectory = new FullTrajectory(pointList);
        Line2D line2D = new Line2D.Double(pointList.get(0), pointList.get(1));
        double heading = GeometryUtil.getHeadingDirection(line2D);

        Set<RoadSection> roadSections = roadMap.getRoadSectionsForConnectionVertex(startCV);
        for (RoadSection roadSection : roadSections) {
            List<Point2D> roadSectionPointList = roadSection.getPointList();

            for (int i = 0; i < roadSectionPointList.size(); i++) {
                Point2D currentPoint = roadSectionPointList.get(i);
                if (line2D.ptSegDist(currentPoint) > maxDistance) {
                    continue;
                }

                boolean valid = false;
                if (i > 0) {
                    Line2D lineBefore = new Line2D.Double(roadSectionPointList.get(i - 1), currentPoint);
                    double h = GeometryUtil.getHeadingDirection(lineBefore);
                    if (GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(heading, h) < maxAngleDiff) {
                        valid = true;
                    }
                }
                if (i < roadSectionPointList.size() - 1) {
                    Line2D lineAfter = new Line2D.Double(currentPoint, roadSectionPointList.get(i + 1));
                    double h = GeometryUtil.getHeadingDirection(lineAfter);
                    if (GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(heading, h) < maxAngleDiff) {
                        valid = true;
                    }
                }

                if (valid) {
                    // Getting the index on the line (hence, a value between 0 and 1).
                    double indexOnProposal = GeometryUtil.getIndexOfTrajectoryClosestToPoint(trajectory, currentPoint);
                    if (indexOnProposal > bestMaxIndex) {
                        bestMaxIndex = indexOnProposal;
                        bestMaxIndexOnRoadSection = i;
                        bestMaxRoadSection = roadSection;
                    }
                }
            }
        }

        if (bestMaxIndex > 0.0 && bestMaxIndex < 1.0) {
            return getConnectionVertexOnRoadSection(bestMaxRoadSection, bestMaxIndexOnRoadSection, roadMap);
        }
        return startCV;
    }

    /**
     * Given a roadSection, an index and the roadMap, we split up the edge if necessary and return the ConnectionVertex.
     *
     * @param roadSection the roadSection
     * @param index       the index on the roadSection, if necessary, we split the roadSection into two.
     * @param roadMap     the roadMap
     * @return the vertex requested.
     */
    private static ConnectionVertex getConnectionVertexOnRoadSection(RoadSection roadSection, int index,
                                                                     RoadMap roadMap) {
        Point2D point = roadSection.getPointList().get(index);
        if (index == 0) {
            return roadSection.getStartVertex();
        } else if (index == roadSection.getPointList().size() - 1) {
            return roadSection.getEndVertex();
        } else {
            return roadMap.splitSectionAndGetVertex(roadSection, point, null);
        }
    }

    /**
     * When we are checking whether a proposed pointList should draw something on the roadMap or whether everything
     * should be merged, it is possible that everything would be merged. However, sometimes we have two different
     * roadSections that are not connected to each other at all, however, the pointList doesn't notice this.
     * To fix that, we want to check when we head over to the next roadSection whether this new pointList proves that
     * they should be merged. If so, we merge them.
     *
     * @param previousMergeRange the MergedBundleStreetPart that came before newMergeRange
     * @param newMergeRange      theMergedBundleStreetPart that came after previousMergeRange
     * @param bundleStreet       current BundleStreet
     * @param noIntersections    the number of intersections at the bundleStreet ends.
     * @param roadMap            roadMap.
     */
    private static void checkIfConnectionBetweenRoadEdgesShouldBeDrawn(MergedBundleStreetPart previousMergeRange,
                                                                       MergedBundleStreetPart newMergeRange,
                                                                       BundleStreet bundleStreet,
                                                                       MergeType noIntersections, RoadMap roadMap) {

        // @NotToDo 1. check whether they actually share subtrajectories..
        // This is something I choose not to implement for the following reason:
        //      When we have a sparse dataset, it would happen that we would check whether they share subtrajectories
        //      but because of GPS inaccuracy and low sampling rate, the bundles around specific areas could miss the
        //      so-called overlapping bundles, which connects two larger bundles splitting off from each other.
        //      We miss them because the error vs length ratio of such a bundle would be to high.
        //      Ps. this is also the reason we consider turns for intersections as well..


        // First we have to check that they are truly different
        if (previousMergeRange.equals(newMergeRange) ||
                previousMergeRange.getRoadSection().equals(newMergeRange.getRoadSection())) {
            return;
        }

        //@Done check if these number makes a diff - No noticable difference

        RoadSection previousRoadSection = previousMergeRange.getRoadSection();
        RoadSection newRoadSection = newMergeRange.getRoadSection();

        int highestIndexForPrevious = previousMergeRange.getLastIndexOfCoveredParts();
        int lowestIndexForNew = newMergeRange.getCoveredParts().get(0).getFirst();
        double distanceInBetweenPreviousAndNew = Math.abs(2 * GeometryUtil.getIndexToIndexDistance(
                newMergeRange.getPointList(), highestIndexForPrevious, lowestIndexForNew));

        if (noIntersections != MergeType.Loner) {  // Small and Med (two or one intersections)
            distanceInBetweenPreviousAndNew += ALGOCONSTANTS.getMidMaxSinglePointDistance() * 2 + 100;
        } else {  // Large (no intersections)
            distanceInBetweenPreviousAndNew += ALGOCONSTANTS.getLargeMaxSinglePointDistance() * 2 + 100;
        }

        double indexOnPreviousRoadSection = previousMergeRange.getRoadEdgeIndexesByParts().get(
                previousMergeRange.getRoadEdgeIndexesByParts().size() - 1).getSecond();
        double indexOnNextRoadSection = newMergeRange.getRoadEdgeIndexesByParts().get(0).getFirst();
//        Point2D pointOnPreviousRoadSection = previousMergeRange.getRoad

        boolean thereIsAConnectionWithinReasonableDistance = roadMap.checkIfTwoPointsAreConnectedWithinDistance(
                indexOnPreviousRoadSection, previousRoadSection,
                indexOnNextRoadSection, newRoadSection,
                distanceInBetweenPreviousAndNew);

        if (!thereIsAConnectionWithinReasonableDistance) {
            // First, we check if there are intersections.
            Point2D intersection = GeometryUtil.findIntersectionBetweenPointLists(
                    previousRoadSection.getPointList(), newRoadSection.getPointList());

            int previousPointIndex, newPointIndex;
            // If so, we find that point and merge the indexes.
            if (intersection != null) {
                double previousIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
                        previousRoadSection.getTrajectory(), intersection);
                previousPointIndex = (int) Math.ceil(previousIndex);
                roadMap.splitSectionAtNewlyIntroducedPoint(previousRoadSection, previousPointIndex, intersection);

                double newIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(newRoadSection.getTrajectory(),
                        intersection);
                newPointIndex = (int) Math.ceil(newIndex);
                roadMap.splitSectionAtNewlyIntroducedPoint(newRoadSection, newPointIndex, intersection);
            }
            // If there is not an intersection, we take last & first points, however, we check if they are close to
            // the ending because in that case we might want to take the endings (to avoid annoying small cases).
            else {
                ConnectionVertex startCV, endCV;
                previousPointIndex = (int) Math.round(indexOnPreviousRoadSection);
                newPointIndex = (int) Math.round(indexOnNextRoadSection);

                Point2D pointOnPrevious = GeometryUtil.getTrajectoryDecimalPoint(
                        previousRoadSection.getTrajectory(), indexOnPreviousRoadSection);
                Point2D pointOnNew = GeometryUtil.getTrajectoryDecimalPoint(
                        newRoadSection.getTrajectory(), indexOnNextRoadSection);

                previousPointIndex = whatIndexToUse(previousPointIndex, previousRoadSection, pointOnNew);
                newPointIndex = whatIndexToUse(newPointIndex, newRoadSection, pointOnPrevious);

                startCV = getConnectionVertexOnRoadSection(previousRoadSection, previousPointIndex, roadMap);
                endCV = getConnectionVertexOnRoadSection(newRoadSection, newPointIndex, roadMap);

                //@Done - Totally fine number.
                if (roadMap.checkIfTwoPointsAreConnected(startCV, endCV,
                        startCV.getLocation().distance(endCV.getLocation()) * 2 + 50, new HashSet<>())) {
                    System.out.println("DrawGeneralFunctions.checkIfConnectionBetweenRoadEdgesShouldBeDrawn. Very special case!");
                    return;
                }

                RoadSection addedRoadSection = roadMap.forceAddRoadSection(Arrays.asList(
                        startCV.getLocation(), endCV.getLocation()), startCV, endCV, bundleStreet);
                addedRoadSection.setConnectionBetweenMergeAreas(true);

                MapFiltering.removeRoadEdgesThatAreOneSidedThatCanBeMerged(addedRoadSection, roadMap);
            }
        }
    }

    private static int whatIndexToUse(int currentIndex, RoadSection rs, Point2D shouldBeCloseTo) {
        List<Point2D> pointList = rs.getPointList();
        Trajectory t = rs.getTrajectory();
        double dPreviousToStart = GeometryUtil.getIndexToIndexDistance(pointList, 0, currentIndex);
        double dPreviousToEnd = GeometryUtil.getIndexToIndexDistance(pointList, currentIndex, pointList.size() - 1);

        double d1 = shouldBeCloseTo.distance(GeometryUtil.getTrajectoryDecimalPoint(t, currentIndex));
        double d2 = Double.MAX_VALUE;
        double d3 = Double.MAX_VALUE;
        if (dPreviousToStart < 75) {
            d2 = shouldBeCloseTo.distance(pointList.get(0));
        }
        if (dPreviousToEnd < 75) {
            d3 = shouldBeCloseTo.distance(pointList.get(pointList.size() - 1));
        }

        if (d2 < d3 && d2 < d1) {
            return 0;
        }
        if (d3 < d2 && d3 < d1) {
            return pointList.size() - 1;
        }
        return currentIndex;
    }


    /**
     * Here we apply a number of checks to ensure the pointList should really be added.
     * 1. PointList with 0 or 1 points.
     * 2. We check if it is just a stripe outwards. Meaning, point(0) = A, point(1) = B, point(2) = A.
     * 3. We check whether the pointList consists of one new point, and if so, that it is not just a duller angle.
     * This would avoid annoying cases where we see points
     *
     * @param pointList  the pointList of the road we want to add
     * @param distances  the distances of the pointList to the considered roadEdges.
     * @param maxSPSODTR maxSinglePointSplitOffDistanceToRoadEdge
     * @return false if it should not be drawn on the map, true if it should be drawn.
     */
    private static boolean shouldPointBeAdded(List<Point2D> pointList, List<Double> distances,
                                              double maxSPSODTR) {

        // pointList of size 0 or 1 should give an error.
        if (pointList.size() < 2) {
            throw new IllegalArgumentException("pointList.size() < 2");
        }
        // pointList of size 2 can always occur and shouldn't be a problem.
        // pointList of size 3 are often annoying.
        if (pointList.size() == 3) {
            // Case 2.
            if (pointList.get(0).distance(pointList.get(2)) < 1) {
                System.out.println("DrawBetweenIntsOnRoadMap.shouldPointBeAdded. First and Last point are the same!");
                return false;
            }
            if (distances != null) {
                if (distances.get(1) < maxSPSODTR) {
                    return false;
                }
            }
        }

        if (GeometryUtil.getContinuousLength(pointList) < 100 &&
                pointList.get(0).distance(pointList.get(pointList.size() - 1)) < 1) {
            return false;
        }

        // In any other case, we assume it is true.
        return true;
    }


    private static Triplet<Point2D, Integer, Boolean> findBestPlaceOnRoadEdgeToConnect(
            List<Point2D> pointList, int pointListIndex, double mergeRoadEdgeIndex, RoadSection roadSection,
            boolean startOfMergedPart, BundleStreet bundleStreet) {

        // Get indices on the roadSection.
        double classicIndex = findClosestVertexOnRoadEdgeToPointClassic(
                pointList, pointListIndex, mergeRoadEdgeIndex, roadSection, startOfMergedPart);
        double newIndex = findBestMergeIndexBySubtrajectories(pointList, pointListIndex,
                roadSection, startOfMergedPart, bundleStreet);

        Point2D classicIndexPoint = GeometryUtil.getTrajectoryDecimalPoint(roadSection.getTrajectory(), classicIndex);

        // @ToDo remove
        return new Triplet<>(classicIndexPoint, pointListIndex, false);


//        if (newIndex < 0) {
//            return new Triplet<>(classicIndexPoint, pointListIndex, false);
//        }
//        Point2D newIndexPoint = GeometryUtil.getTrajectoryDecimalPoint(roadSection.getTrajectory(), newIndex);
//
//        int countForNewIndex = 0;
//        Set<Subtrajectory> overlappingSubs = new HashSet<>();
//        for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
//            for (Subtrajectory subtrajectory2 : roadSection.getBestHalfOfBundlesAndReturnTheirSubtrajectories()) {
//                if (subtrajectory1.overlaps(subtrajectory2)) {
//                    overlappingSubs.add(subtrajectory1);
//
//                    double classicSubParentIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subtrajectory1.getParent(), classicIndexPoint);
//                    double newSubParentIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subtrajectory1.getParent(), newIndexPoint);
//
//                    // Note that we are assuming that the subtrajectories in the bundleStreet are in the direction of
//                    // the bundle. Thereby we can simply base our decision whether the newSubIndex makes sense on the
//                    // parameter startOfMergedPart.
//
//                    if (startOfMergedPart) { // start of a mergedPart, the last point of the distinct drawn part
//
//                        // In this case, we want the newSubIndex to be earlier than the classicSubIndex.
//                        if (newSubParentIndex < classicSubParentIndex + 1E-4) {
////                        if (newSubParentIndex < classicSubParentIndex + 1E-4) {
//                            countForNewIndex++;
//                        }
//                    } else { // end of a merged part, hence the first point of our new roadSection
//
//                        // In this case, we want the newSubIndex to be later than the classicSubIndex.
//                        if (newSubParentIndex + 1E-4 > classicSubParentIndex) {
////                        if (newSubParentIndex + 1E-4 > classicSubParentIndex) {
//                            countForNewIndex++;
//                        }
//                    }
//                    break;
//                }
//            }
//        }
//
//        if ((double) countForNewIndex > ((double) overlappingSubs.size()) / 2) {
//
//            double newRepIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
//                    new FullTrajectory(pointList), newIndexPoint);
//            if (startOfMergedPart) { // start of a mergedPart, the last point of the distinct drawn part
////                newRepIndex = Math.floor(newRepIndex);
//                newRepIndex = Math.ceil(newRepIndex);
//                if (newRepIndex < pointListIndex){
//                    newRepIndex = pointListIndex;
//                }
//                // @ToDo changed this but then we have to change the merging of the pointList with mergeLocation as well
//            } else {
////                newRepIndex = Math.ceil(newRepIndex);
//                newRepIndex = Math.floor(newRepIndex);
//                if (newRepIndex > pointListIndex){
//                    newRepIndex = pointListIndex;
//                }
//                // @ToDo changed this but then we have to change the merging of the pointList with mergeLocation as well
//            }
//            return new Triplet<>(newIndexPoint, (int) newRepIndex, true);
//        } else {
//            return new Triplet<>(classicIndexPoint, pointListIndex, false);
//        }
//        return new Triplet<>(classicIndexPoint, pointListIndex, false);

        // 1. We get the roadSection and index on this roadSection for what position we want to merge with. (PROPOSED)
        // 2. Now we check based on subtrajectories, at which point the roadSection contains the same trajectory as our
        //      bundlestreet, and find at what index the subtrajectory appears in the roadSection.
        // 3. For each of these trajectories we check whether the index to the PROPOSED place is further
        //      away from the subtrajectory index on the roadSection.


    }

    /**
     * Based on subtrajectories we try to find the best index on the current road section to which we want to add our
     * roadsection.
     *
     * @param roadSection       the RoadSection where we compared our point with.
     * @param startOfMergedPart whether the given index on the pointList is the end of a merged part or a start
     * @param bundleStreet      the bundleStreet we are trying to add to our roadMap
     * @return the best merge index for this roadsection, based on subtrajectory data.
     */

    private static double findBestMergeIndexBySubtrajectories(List<Point2D> pointList, int pointIndex,
                                                              RoadSection roadSection, boolean startOfMergedPart,
                                                              BundleStreet bundleStreet) {
        int counter = 0;
        int invalidCounter = 0;
        double averageRepIndex = 0.0;
        ArrayList<Double> repIndexes = new ArrayList<>();
        Point2D lastOrFirstMergedPoint = pointList.get(pointIndex);

        for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
            for (Subtrajectory subtrajectory2 : roadSection.getBestHalfOfBundlesAndReturnTheirSubtrajectories()) {
                if (subtrajectory1.overlaps(subtrajectory2)) {
                    if (!subtrajectory1.getParent().equals(subtrajectory2.getParent())) {
                        subtrajectory2 = subtrajectory2.reverse();
                    }

                    double subIndexLastNonMergedPoint = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
                            subtrajectory1.getParent(), lastOrFirstMergedPoint);

                    Point2D indexPoint;
                    if (startOfMergedPart) { // start of a mergedPart, the last point of the distinct drawn part
                        indexPoint = subtrajectory2.getFirstPoint();

                        if (subIndexLastNonMergedPoint > subtrajectory2.getFromIndex()){
                            invalidCounter++;
                            continue;
                        }
                    } else { // end of a merged part, hence the first point of our new roadSection
                        indexPoint = subtrajectory2.getLastPoint();

                        if (subIndexLastNonMergedPoint < subtrajectory2.getFromIndex()){
                            invalidCounter++;
                            continue;
                        }
                    }


                    double repIndexMaxSub = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
                            roadSection.getTrajectory(), indexPoint);

                    averageRepIndex = counter * averageRepIndex / (counter + 1) + repIndexMaxSub / (counter + 1);
                    counter++;
                    repIndexes.add(repIndexMaxSub);

//                    Range<Double> overlapRange =  subtrajectory1.computeOverlap(subtrajectory2);
//                    Subtrajectory subtrajectory3 = new Subtrajectory(subtrajectory1.getParent(), overlapRange.lowerEndpoint(), overlapRange.upperEndpoint());
//                    boolean roadSectionSameDirection = GeometryUtil.checkIfTrajectoryHasSameDirection(subtrajectory3, roadSection.getTrajectory(), 4);
//
//                    Trajectory roadSectionTrajectory;
//                    if (roadSectionSameDirection){
//                        roadSectionTrajectory = roadSection.getTrajectory();
//                    } else {
//                        roadSectionTrajectory = roadSection.getReverseTrajectory();
//                    }

//                    double repIndexMaxSub = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
//                            roadSectionTrajectory, indexPoint);

//                    double repIndexLastNonMergedPoint = GeometryUtil.getIndexOfTrajectoryClosestToPoint(
//                            roadSectionTrajectory, lastOrFirstMergedPoint);
//
//                    if (startOfMergedPart && repIndexLastNonMergedPoint > repIndexMaxSub ||
//                            !startOfMergedPart && repIndexLastNonMergedPoint < repIndexMaxSub){
//                        invalidCounter++;
//                    } else {
//                        averageRepIndex = counter * averageRepIndex / (counter + 1) + repIndexMaxSub / (counter + 1);
//                        counter++;
//                        repIndexes.add(repIndexMaxSub);
//                    }
                }
            }
        }

        if (invalidCounter > counter){
            return -1.0;
        }

        HashMap<Double, Double> distancesToAverage = new HashMap<>();
        for (Double repIndex : repIndexes) {
            distancesToAverage.put(repIndex, Math.abs(repIndex - averageRepIndex));
        }
        distancesToAverage = distancesToAverage.entrySet().stream()
                .sorted(Map.Entry.comparingByValue())
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue,
                        (oldValue, newValue) -> oldValue, LinkedHashMap::new));

        double newAverageIndex = 0.0;
        int i = 0;
        int maxIndex = (int) Math.ceil(((double) distancesToAverage.size()) * 0.75);
        for (Map.Entry<Double, Double> entry : distancesToAverage.entrySet()) {
            if (i < maxIndex) {
                double indexRep = entry.getKey();
                if (i == 0) {
                    newAverageIndex = indexRep;
                } else {
                    newAverageIndex = i * newAverageIndex / (i + 1) + indexRep / (i + 1);
                }
            }
            i++;
        }


        if (startOfMergedPart) {
            return Math.max(newAverageIndex, 0);
        } else {
            return Math.min(newAverageIndex, roadSection.getTrajectory().numPoints() - 1);
        }
    }

    /**
     * After we found which parts were close to already present RoadEdges, we now find exactly which vertex of the
     * RoadSection we should use to connect the start or the end of the unmerged part of the pointList with. This is
     * done by finding the closest point on the roadSection to the first/last point of the unmerged part.
     *
     * @param pointList          the pointList we are currently looking at to add to our RoadMap.
     * @param pointListIndex     the index of the first or last point of the unmerged part.
     * @param mergeRoadEdgeIndex the index of the RoadSection for the found merged part.
     * @param roadSection        the RoadSection where we compared our point with.
     * @param startOfMergedPart  whether the given index on the pointList is the end of a merged part or a start
     * @return the vertex(of the RoadSection), at which it should be pslit, which we will connect to the end of the pointList.
     */
    private static int findClosestVertexOnRoadEdgeToPointClassic(List<Point2D> pointList,
                                                                 int pointListIndex,
                                                                 double mergeRoadEdgeIndex,
                                                                 RoadSection roadSection,
                                                                 boolean startOfMergedPart) {
        // @ToDo
        //      1. We want to base the position on the first place we come across which has the subtrajectories.
        // The approach:
        // 1. We get the roadSection and index on this roadSection for what position we want to merge with. (PROPOSED)
        // 2. Now we check based on subtrajectories, at which point the roadSection contains the same trajectory as our
        //      bundlestreet, and find at what index the subtrajectory appears in the roadSection.
        // 3. For each of these trajectories we check whether the index to the PROPOSED place is further
        //      away from the subtrajectory index on the roadSection.

        Point2D pointListPoint;
        if (startOfMergedPart) { // start of a mergedPart, hence we want to find the perfect fit for the point before
            // the merged part.
            pointListPoint = pointList.get(Math.max(pointListIndex - 1, 0));
        } else { // end of a merged part, hence we are splitting up now.
            pointListPoint = pointList.get(Math.min(pointListIndex + 1, pointList.size() - 1));
        }

        // For pointList point that is at either the start or end of the unmerged part we want to draw we check
        // what the index on the roadSection is for this given point.
        double maxMinIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(roadSection.getTrajectory(), pointListPoint);

        // Now we want to find the optimal index to merge with based on the mergeRoadEdgeIndex. However, we wish to
        // avoid z-artefacts, therefore we use maxMinIndex, to set a bound on the rounding process. Meaning, in the case
        // that the order of the two changes (one becomes larger than the other and wasn't previously due to the
        // rounding of mergeRoadEdgeIndex), then we change the rounded value such that the original order is enforced
        // again.
        double roundedIndex = Math.round(mergeRoadEdgeIndex);
        if (mergeRoadEdgeIndex > maxMinIndex) {
            // Now we check if this reversed the order. In that case, we should up roundedIndex with 1.
            if (roundedIndex < maxMinIndex) {
                roundedIndex += 1;
            }
        } else {
            // mergeRoadEdgeIndex < maxMinIndex
            if (roundedIndex > maxMinIndex) {
                roundedIndex -= 1;
            }
        }

        return (int) roundedIndex;
    }


    /**
     * Check if we can group the subtrajectories into two groups.
     * If so, we check if there is third overlapping group.
     * Based on this behavior of the bundles we decide whether we are seeing bundles which should be merged together
     * or if we have two parallel bundles.
     *
     * @param allBundleStreetsAtLeastTwoHit
     * @param allBundleStreetsOneHit
     * @param addingBundleStreet
     * @return boolean
     */
    public static boolean checkIfBundlesCanBeSplitIntoTwoGroups(
            Set<BundleStreet> allBundleStreetsAtLeastTwoHit, Set<BundleStreet> allBundleStreetsOneHit,
            BundleStreet addingBundleStreet, RoadSection roadSection) {

        Set<BundleStreet> bundlesOfOurTrajectoryGroup = new HashSet<>();
        Set<BundleStreet> bundlesOfTheOtherTrajectoryGroup = new HashSet<>();
        Set<BundleStreet> bundlesOfBothTrajectoryGroups = new HashSet<>();

        Set<Subtrajectory> subtrajectoryInRequiredGroup = new HashSet<>();
        Set<Subtrajectory> unknownSubtrajectories = new HashSet<>();

        for (Subtrajectory subtrajectory : addingBundleStreet.getSubtrajectories()) {
            if (subtrajectory.isReverse()) {
                subtrajectory.reverse();
            }
            subtrajectoryInRequiredGroup.add(subtrajectory);
        }

        int maxBSize = allBundleStreetsAtLeastTwoHit.stream().mapToInt(bundleStreet -> bundleStreet.getSubtrajectories().size()).max().orElse(0);
        if (maxBSize == 0) {
            return false; // Merge
        }
        maxBSize = (int) Math.ceil((double) maxBSize / 2);
        int finalMaxBSize = maxBSize;


        // This is a sort on bestEpsilon
        LinkedList<BundleStreet> bestEpsilonSorted2 = allBundleStreetsAtLeastTwoHit.stream()
                .sorted(Comparator.comparingInt(b -> (int) STORAGE.getEvolutionDiagram().getBestEpsilon(b.getBundleClass())))
                .filter(bundleStreet -> bundleStreet.getBundle().size() < finalMaxBSize)
                .collect(Collectors.toCollection(LinkedList::new));

        // Only keep 75% of the lowest epsilon.
        LinkedList<BundleStreet> bestEpsilonSorted = new LinkedList<>();
//        @ToDo was 0.75
        for (int i = 0; i < (int) Math.ceil(0.75 * (double) bestEpsilonSorted2.size()); i++){
            bestEpsilonSorted.add(bestEpsilonSorted2.get(i));
        }

        for (BundleStreet bundleStreet: bestEpsilonSorted2){
//            @ToDo was 20..
            if (STORAGE.getEvolutionDiagram().getBestEpsilon(bundleStreet.getBundleClass()) <= 20){
                if (!bestEpsilonSorted.contains(bundleStreet)){
                    bestEpsilonSorted.add(bundleStreet);
                }
            }
        }


        // We initiate a run as follows.
        // We check for each bundle that has less than 2 times the size of the maximum size bundle what we are looking
        // at.
        int counter = bestEpsilonSorted.size() * 2;
        while (bestEpsilonSorted.size() != 0 && counter >= 0) {
            counter--;

            BundleStreet bundleStreet = bestEpsilonSorted.get(0);
            bestEpsilonSorted.remove(0);
            if (bundleStreet.getSubtrajectories().size() > maxBSize) {
                continue;
            }

            int numOfOverlapsInRequiredGroup = 0;
            for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                for (Subtrajectory subtrajectory2 : subtrajectoryInRequiredGroup) {
                    if (subtrajectory2.overlaps(subtrajectory1)) {
                        numOfOverlapsInRequiredGroup++;
                    }
                }
            }

            // Here we update each group.
            // If we found trajectories that are present in our requiredGroup than we add them to the group
            // and we remove them from the unknownSubtrajectories.
            // If we did not found trajectories that are present in our requiredGroup than we add them to unknown group
            if (numOfOverlapsInRequiredGroup > 0) {
                // Add subtrajectory1 to subtrajectoryInRequiredGroup
                for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                    boolean alreadyPresent = false;
                    for (Subtrajectory subtrajectory2 : subtrajectoryInRequiredGroup) {
                        if (subtrajectory2.overlaps(subtrajectory1)) {
                            alreadyPresent = true;
                        }
                    }
                    if (!alreadyPresent) {
                        subtrajectoryInRequiredGroup.add(subtrajectory1);
                    }
                }

                // Remove subtrajectory1 from unknownSubtrajectories
                for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                    for (Subtrajectory subtrajectory2 : new HashSet<>(unknownSubtrajectories)) {
                        if (subtrajectory2.overlaps(subtrajectory1)) {
                            unknownSubtrajectories.remove(subtrajectory2);
                        }
                    }
                }
            } else {
                bestEpsilonSorted.add(bundleStreet);

                // Add subtrajectory1 to unknownSubtrajectories
                for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                    boolean alreadyPresent = false;
                    for (Subtrajectory subtrajectory2 : unknownSubtrajectories) {
                        if (subtrajectory2.overlaps(subtrajectory1)) {
                            alreadyPresent = true;
                        }
                    }
                    if (!alreadyPresent) {
                        unknownSubtrajectories.add(subtrajectory1);
                    }
                }
                unknownSubtrajectories.addAll(bundleStreet.getSubtrajectories());
            }
        }

        // Now that we know exactly what our subtrajectoryInRequiredGroup is, we first check whether there is something
        // left at all in our unknownSubtrajectories.
        // @ToDo test if this makes results worse or better...
        if (unknownSubtrajectories.size() < 3) {
            return false; // Merge
        }

        // Now that we know that our unknownSubtrajectories have a decent number as well, we check for each bundle what
        // it is part of.
        for (BundleStreet bundleStreet : allBundleStreetsAtLeastTwoHit) {
            int subsInRequiredGroup = 0;
            int subsInOtherGroup = 0;

            for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                for (Subtrajectory subtrajectory2 : subtrajectoryInRequiredGroup) {
                    if (subtrajectory1.overlaps(subtrajectory2)) {
                        subsInRequiredGroup++;
                    }
                }
                for (Subtrajectory subtrajectory2 : unknownSubtrajectories) {
                    if (subtrajectory1.overlaps(subtrajectory2)) {
                        subsInOtherGroup++;
                    }
                }
            }

            if (subsInRequiredGroup > 0 && subsInOtherGroup > 0) {
                bundlesOfBothTrajectoryGroups.add(bundleStreet);
            } else if (subsInRequiredGroup > 0) {
                bundlesOfOurTrajectoryGroup.add(bundleStreet);
            } else { // Assuming subsInOthergroup > 0
                bundlesOfTheOtherTrajectoryGroup.add(bundleStreet);
            }
        }

        if (bundlesOfTheOtherTrajectoryGroup.size() == 0 || bundlesOfOurTrajectoryGroup.size() == 0) {
            return false; // Merge
        }

        if (bundlesOfBothTrajectoryGroups.size() == 0) {
            boolean subsInRequiredGroup = false;
            boolean subsInOtherGroup = false;
            for (BundleStreet bundleStreet : allBundleStreetsOneHit) {
                for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                    for (Subtrajectory subtrajectory2 : subtrajectoryInRequiredGroup) {
                        if (subtrajectory1.overlaps(subtrajectory2)) {
                            subsInRequiredGroup = true;
                        }
                    }
                    for (Subtrajectory subtrajectory2 : unknownSubtrajectories) {
                        if (subtrajectory1.overlaps(subtrajectory2)) {
                            subsInOtherGroup = true;
                        }
                    }
                }
            }

            if (subsInRequiredGroup && subsInOtherGroup) {
                return false; // Merge
            } else {
                return true; // Don't merge
            }
        }

        double averageBestEpsilonOurGroup = bundlesOfOurTrajectoryGroup
                .stream()
                .mapToDouble(b -> STORAGE.getEvolutionDiagram().getBestEpsilon(b.getBundleClass()))
                .average()
                .orElse(0.0);
        double averageBestEpsilonOtherGroup = bundlesOfTheOtherTrajectoryGroup
                .stream()
                .mapToDouble(b -> STORAGE.getEvolutionDiagram().getBestEpsilon(b.getBundleClass()))
                .average()
                .orElse(0.0);
        double averageBestEpsilonCompleteGroup = bundlesOfBothTrajectoryGroups
                .stream()
                .mapToDouble(b -> STORAGE.getEvolutionDiagram().getBestEpsilon(b.getBundleClass()))
                .average()
                .orElse(0.0);

        if (Math.max(averageBestEpsilonOurGroup + 10, averageBestEpsilonOurGroup * 1.25) < averageBestEpsilonCompleteGroup &&
                Math.max(averageBestEpsilonOtherGroup + 10, averageBestEpsilonOtherGroup * 1.25) < averageBestEpsilonCompleteGroup) {
            if ((bundlesOfOurTrajectoryGroup.contains(roadSection.getDrawnBundleStreet()) &&
                    bundlesOfTheOtherTrajectoryGroup.contains(addingBundleStreet)) ||
                    (bundlesOfTheOtherTrajectoryGroup.contains(roadSection.getDrawnBundleStreet()) &&
                            bundlesOfOurTrajectoryGroup.contains(addingBundleStreet))) {
                return true;
            }
            return false;
            // We check whether our addingBundleStreet(ABS) is in the completeGroup, ourGroup or otherGroup.
            // Then we check whether the roadSection is in the ourGroup or otherGroup.
            //          if ABS and the roadSection are from the same group we return false meaning merge.
            //          if ABS is part of the completeGroup, we return false meaning merge
            //          if aBS and RoadSection from our and other group respectively (or turned around, we don't merge).
        } else {
            return false;
        }
    }

    /**
     * Given the merge routine and the merge values, we prepare for the above function,
     * checkIfBundlesCanBeSplitIntoTwoGroups.
     *
     * @param bestRoadSection            contains the road section we might want to merge our current bundleStreet with.
     * @param bestMergedPart             a pair of indices for the pointList to indicate the mergeable area.
     * @param bundleStreetToAddToRoadmap the bundlestreet we wish to add.
     * @param pointList                  the pointlist of the bundleSteet we will add (including possible extra points).
     * @return boolean
     */
    public static boolean checkIfBundleShouldNotBeMerged(
            RoadSection bestRoadSection, Pair<Integer, Integer> bestMergedPart, List<Point2D> pointList,
            BundleStreet bundleStreetToAddToRoadmap) {

        FullTrajectory ftOfPointList = new FullTrajectory(pointList);

        Set<Subtrajectory> allSubtrajectories = new HashSet<>(bestRoadSection.getSubtrajectories());
        allSubtrajectories.addAll(bundleStreetToAddToRoadmap.getSubtrajectories());

        List<Point2D> pointsStartingMiddleAndEnding = new ArrayList<>();
        pointsStartingMiddleAndEnding.add(pointList.get(bestMergedPart.getFirst()));
        double lengthOfMergedPart = GeometryUtil.getContinuousLength(
                pointList.subList(bestMergedPart.getFirst(), bestMergedPart.getSecond() + 1));
        pointsStartingMiddleAndEnding.add(
                GeometryUtil.getTrajectoryDecimalPoint(
                        ftOfPointList, GeometryUtil.getTrajectoryIndexAfterOffset(
                                ftOfPointList, bestMergedPart.getFirst(), lengthOfMergedPart / 2)));
        pointsStartingMiddleAndEnding.add(pointList.get(bestMergedPart.getSecond()));

        Set<BundleStreet> bundleStreetsInOneOfTheThree = new HashSet<>();
        bundleStreetsInOneOfTheThree.add(bundleStreetToAddToRoadmap);
        bundleStreetsInOneOfTheThree.add(bestRoadSection.getDrawnBundleStreet());

        Set<BundleStreet> bundleStreetsInTwoOfTheThree = new HashSet<>();
        bundleStreetsInTwoOfTheThree.add(bundleStreetToAddToRoadmap);
        bundleStreetsInTwoOfTheThree.add(bestRoadSection.getDrawnBundleStreet());

        for (Subtrajectory subtrajectory : allSubtrajectories) {

            Set<BundleStreet> localBSInOne = new HashSet<>();
            Set<BundleStreet> localBSInTwo = new HashSet<>();
            for (Point2D currentPoint : pointsStartingMiddleAndEnding) {
                double index = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subtrajectory, currentPoint);
                index = GeometryUtil.convertSubIndexToTrajectoryIndex(subtrajectory, index);

                Set<BundleStreet> bundleStreetsContainingThisSubtrajectory =
                        SubBSCombiner.getBundleStreetsForTrajectoryAndIndex(subtrajectory.getParent(), index, 15.0);

                for (BundleStreet bundleStreet1 : bundleStreetsContainingThisSubtrajectory) {
                    if (localBSInOne.contains(bundleStreet1)) {
                        localBSInTwo.add(bundleStreet1);
                    } else {
                        localBSInOne.add(bundleStreet1);
                    }
                }

            }
            // The sets will prevent bundles being added twice.
            bundleStreetsInOneOfTheThree.addAll(localBSInOne);
            bundleStreetsInTwoOfTheThree.addAll(localBSInTwo);
        }

        boolean returncode = checkIfBundlesCanBeSplitIntoTwoGroups(
                bundleStreetsInTwoOfTheThree, bundleStreetsInOneOfTheThree, bundleStreetToAddToRoadmap, bestRoadSection);
        bundleStreetToAddToRoadmap.setWasMergeableWithOtherRoad(returncode);
        bundleStreetToAddToRoadmap.setOnOfThreeBundleStreetsTriedToMergeWith(bundleStreetsInOneOfTheThree);
        bundleStreetToAddToRoadmap.setTwoOfThreebundleStreetsTriedToMergeWith(bundleStreetsInTwoOfTheThree);

        return returncode;
    }

    /**
     * Merges the beginning of the street with the intersection.
     *
     * @param pointList,                          the pointList that we want to merge with the intersection.
     * @param mergePoint,                         the point we want to merge the beginning of the pointList with.
     * @param isPointListWithMergingLocationMerge whether we are using this function to merge the location
     * @return whether we could find a good merge or whether we should consider it bad.
     */
    public static boolean mergeStreetBeginningWithIntersection(List<Point2D> pointList,
                                                               Point2D mergePoint,
                                                               boolean isPointListWithMergingLocationMerge) {

        if (pointList.size() == 0) {
            System.out.println("mergeStreetBeginning had an empty array");
            pointList.add(0, mergePoint);
            return true;
        } else if (pointList.size() == 1) {
            pointList.add(0, mergePoint);
        }

        double maxDistance = ALGOCONSTANTS.getMaxRoadSectionMergeWithIntDistance();
        double maxDistanceFromPreviousPointsToNewEdge = ALGOCONSTANTS.getMaxRoadSectionMergeErrorDistanceToNewlyCreatedEdge();
        double maxAngleDifference = ALGOCONSTANTS.getMaxRoadSectionMergeAngleDifference();
        double minOriginalPointListSize = ALGOCONSTANTS.getMinRoadSectionMergeRemainingOppositeLength();

        FullTrajectory t = new FullTrajectory(pointList);
        double indexAfterMaxDistance = Math.ceil(
                GeometryUtil.getTrajectoryIndexAfterOffset(t, 0, maxDistance));
        double indexAfter50MetersFromTop = Math.floor(
                GeometryUtil.getTrajectoryIndexAfterOffset(t, t.numPoints() - 1, -minOriginalPointListSize));

        int maxEdgeIndex = Math.min((int) indexAfterMaxDistance, (int) indexAfter50MetersFromTop);

        int bestIndexToMergeWith = 0;
        double bestMatchScore = 0;
        double score;

        // This is to keep track of the distance from the line to the mergePoint
        // At the moment we have a distance that is less than 50, we start counting.
        int indexThatIsWithinACertainLineDistance = Integer.MAX_VALUE;

        int i = 0;
        while (i <= maxEdgeIndex) {
            Point2D startPoint = pointList.get(i + 1);
            Line2D lineOnPointList = new Line2D.Double(startPoint, pointList.get(i));
            double angleOfEdge = GeometryUtil.getHeadingDirection(lineOnPointList);
            double segmentDistanceToLocation = lineOnPointList.ptLineDist(mergePoint);
            if (segmentDistanceToLocation < maxDistanceFromPreviousPointsToNewEdge) {
                indexThatIsWithinACertainLineDistance = Math.min(indexThatIsWithinACertainLineDistance, i);
            } else {
                i++;
                continue;
            }

            Line2D lineFromFirstPToMergeP = new Line2D.Double(startPoint, mergePoint);
            double angleToTurn = GeometryUtil.getHeadingDirection(lineFromFirstPToMergeP);


            // From the point that is connected to a line, which has a line distance to the mergePoint of less than 50,
            // we start keeping track of the proposed line which might replace the first few points.
            // If this line is to far of the point, that means we would be missing out on details of the actual road.
            boolean toFar = false;
            for (int j = indexThatIsWithinACertainLineDistance; j < i; j++) {
                if (lineFromFirstPToMergeP.ptSegDist(pointList.get(j)) > maxDistanceFromPreviousPointsToNewEdge) {
                    toFar = true;
                }
            }
            if (toFar) {
                i++;
                continue;
            }


            score = GeometryUtil.getAbsoluteAngleDifference(angleOfEdge, angleToTurn);
            if (score <= maxAngleDifference) {
//                // This is the original
//                // This makes the range go from 1 to 3 in principle.
//                // Range is 25 -> perfect, to 0 -> bad
//                score = (maxAngleDifference - score);
//                // Range is 75 -> perfect, to 25 -> bad
//                score = (score * 2) + maxAngleDifference;
//                // Range is 3 -> perfect, to 1 -> bad;
//                score = score / maxAngleDifference;
//                // Range of inverseRange is 1 is close, 1/6 is maxDistance.
////                double inverseRangeScore = Math.pow(maxDistance, 2) / (Math.pow(distanceTurnLocation, 2) + Math.pow(maxDistance, 2));
//                double distanceTurnLocation = mergePoint.distance(startPoint);
//                double inverseRangeScore = maxDistance / distanceTurnLocation + maxDistance;
//
//                // Score 3 -> perfect, 1/6 -> Bad
//                score = Math.pow(score, 2) * inverseRangeScore;
//
//                if (score > bestMatchScore) {
//                    bestIndexToMergeWith = i + 1;
//                    bestMatchScore = score;
//                }

                // Improved scoring.. Previous made no sense..
                double distanceConnectLocation = mergePoint.distance(startPoint);

                // Score between 1 and 2.
                double angleScore = (maxAngleDifference - score) / maxAngleDifference;
                angleScore = angleScore + 1.0;

                // Score between 1 and 2
                double distanceScore = Math.max(1, 2 - (distanceConnectLocation / maxDistance));

//                @ToDo check if distanceSCore to the pow 2 works better..
                score = Math.pow(angleScore, 2) * Math.pow(distanceScore, 2);
                if (score > bestMatchScore) {
                    bestIndexToMergeWith = i + 1;
                    bestMatchScore = score;
                }

                // Old score, that was working pretty well..
//                score = score * (1 / Math.sqrt(distanceTurnLocation));
//                if (score < bestMatchScore) {
//                    bestIndexToMergeWith = i + 1;
//                    bestMatchScore = score;
//                }
                // Note: Both of these ideas deliver very bad results in the Athenes dataset.
                // Having one squart less fixes a lot of issues.
                // score = score * (1 / Math.sqrt(Math.sqrt(distanceTurnLocation)));
                // We also increase the score + 5, this is to make sure the distance always matters,
                // as the distance wouldn't do anything for 0.001 score
                // score = (score + 5) * (1 / distanceTurnLocation);
            }

            i++;
        }

        List<Point2D> pointListCopy = new ArrayList<>(pointList);
        pointList.clear();
        pointList.add(mergePoint);
        for (i = bestIndexToMergeWith; i < pointListCopy.size(); i++) {
            pointList.add(pointListCopy.get(i));
        }

        // Here we check whether the merge can be considered a good result, or whether it should be seen as a bad merge.
        // A bad merge is when bestIndexToMergeWith is still 0, as in any other case that we had a good merge, we
        // actually had at least 1.
        return bestIndexToMergeWith > 0;
    }
}
