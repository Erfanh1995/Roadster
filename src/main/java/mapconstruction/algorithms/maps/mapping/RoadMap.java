package mapconstruction.algorithms.maps.mapping;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Range;
import com.google.common.collect.SetMultimap;
import com.google.common.math.DoubleMath;
import com.sun.corba.se.spi.activation.EndpointInfoListHelper;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * A RoadMap is a geometric graph, consisting of vertices and edges, and each edge as an embedding.
 * <p>
 * The RoadMap is undirected by default
 *
 * @author Jorrick Sleijster
 */
public class RoadMap implements Serializable {

    /**
     * Map mapping ConnectionVertices to the adjacent RoadSections, and hence indirectly
     * to the adjacent vertices
     */
    private final SetMultimap<ConnectionVertex, RoadSection> adjacencyList;

    /**
     * List containing all roadSections
     */
    private final List<RoadSection> presentRoadSections;

    /**
     * List containing information about the splits
     */
    private final Map<RoadSection, Map<Range<Integer>, RoadSection>> roadSectionSplitUpList;

    /**
     * Current UID
     */
    private int currentUID = 0;

    public RoadMap() {
        adjacencyList = HashMultimap.create();
        presentRoadSections = new ArrayList<>();
        roadSectionSplitUpList = new HashMap<>();
        currentUID = 0;
    }

    /**
     * Get all ConnectionsVertices
     *
     * @return all ConnectionVertices
     */
    @JsonProperty
    public List<ConnectionVertex> getConnectionVertices() {
        return new ArrayList<>(adjacencyList.keySet());
    }

    /**
     * Get all RoadSections
     *
     * @return all RoadSections
     */
    @JsonIgnore
    public List<RoadSection> getPresentRoadSections() {
        List<RoadSection> roadSections = new LinkedList<>();
        for (int i = 0; i < presentRoadSections.size(); i++){
            if (!presentRoadSections.get(i).isRemoved()){
                roadSections.add(presentRoadSections.get(i));
            }
        }
        return roadSections;
    }

    /**
     * Get all RoadSections
     *
     * @return all RoadSections
     */
    @JsonProperty
    public List<RoadSection> getRoadSections() {
        return new ArrayList<>(presentRoadSections);
    }

    /**
     * Get number of RoadSections present already
     *
     * @return number of RoadSections present in the RoadMap
     */
    @JsonProperty
    public int numberRoadSections() {
        return getPresentRoadSections().size();
    }

    /**
     * Adds a new Road Edge
     *
     * @param pointList     the list of points which represent a trajectory
     * @param startVertex,  one of the two ConnectionVertices at the end.
     * @param endVertex,    one of the two ConnectionVertices at the end.
     * @param bundleStreet, the bundleStreet that caused this RoadSection to be drawn.
     */
    public void addRoadSection(List<Point2D> pointList, ConnectionVertex startVertex, ConnectionVertex endVertex,
                               BundleStreet bundleStreet) {
//        List<Point2D> point2DList = improvePointList(oriPointList);
        RoadSection roadSection = new RoadSection(pointList, startVertex, endVertex, bundleStreet, currentUID);

        if (GeometryUtil.getContinuousLength(roadSection.getPointList()) < ALGOCONSTANTS.getMinBundleStreetLength()) {
            return;
        }
        currentUID++;

        forceAddRoadSection(roadSection);
    }

    /**
     * Adds a new RoadSection to the graph without any extra checks
     *
     * @param pointList     the list of points which represent a trajectory
     * @param startVertex,  one of the two ConnectionVertices at the end.
     * @param endVertex,    one of the two ConnectionVertices at the end.
     * @param bundleStreet, the bundleStreet that caused this RoadSection to be drawn.
     */
    public RoadSection forceAddRoadSection(List<Point2D> pointList, ConnectionVertex startVertex, ConnectionVertex endVertex,
                                    BundleStreet bundleStreet) {
        RoadSection roadSection = new RoadSection(pointList, startVertex, endVertex, bundleStreet, currentUID);
        currentUID++;
        forceAddRoadSection(roadSection);
        return roadSection;
    }

    /**
     * Adds a new edge to the graph without any extra checks
     *
     * @param roadSection, the RoadSection to be added to the graph
     */
    public void forceAddRoadSection(RoadSection roadSection) {
        if (roadSection.getStartVertex() == null) {
            throw new IllegalArgumentException("ConnectionVertex is not allowed not be null");
        }
        if (roadSection.getEndVertex() == null) {
            throw new IllegalArgumentException("ConnectionVertex is not allowed not be null");
        }

        if (STORAGE.getDatasetConfig().isWalkingDataset()) {
            checkIfWeShouldRemoveEarlierRoadEdges(roadSection);
        }

        adjacencyList.put(roadSection.getStartVertex(), roadSection);
        if (roadSection.getStartVertex() != roadSection.getEndVertex()) {
            adjacencyList.put(roadSection.getEndVertex(), roadSection);
        }
        // This is to keep track of the order at which things happen.
        roadSection.setRemoved(false);
        presentRoadSections.add(roadSection);
    }

    /**
     * Removes an edge from the graph
     *
     * @param roadSection, the RoadSection to be added to the graph
     */
    public void removeRoadSection(RoadSection roadSection) {
        adjacencyList.remove(roadSection.getStartVertex(), roadSection);
        checkIfRoadSectionsShouldBeMerged(adjacencyList.get(roadSection.getStartVertex()));

        if (roadSection.getStartVertex() != roadSection.getEndVertex()) {
            adjacencyList.remove(roadSection.getEndVertex(), roadSection);
            checkIfRoadSectionsShouldBeMerged(adjacencyList.get(roadSection.getEndVertex()));
        }
        // This is to keep track of the order at which things happen.
//        @ToDo change later on
//        presentRoadSections.remove(roadSection);
        roadSection.setRemoved(true);
    }

    /**
     * Force remove an edge from the graph, meaning, we don't check if it should be merged.
     *
     * @param roadSection roadSection
     */
    private void forceRemoveRoadSection(RoadSection roadSection) {
        adjacencyList.remove(roadSection.getStartVertex(), roadSection);
        if (roadSection.getStartVertex() != roadSection.getEndVertex()) {
            adjacencyList.remove(roadSection.getEndVertex(), roadSection);
        }
        // This is to keep track of the order at which things happen.
        presentRoadSections.remove(roadSection);
        roadSection.setRemoved(true);
    }

    /**
     * Split an edge from the graph on a specific point and return that point as
     * a newly created ConnectionVertex
     *
     * @param roadSection,      the roadSection we want to split up.
     * @param pointToSplitAt,   the point to split at in the roadSection pointList.
     * @param replacementPoint, the point to replace the pointToSplitAt with.
     *                          (If null, pointToSplitAt is used for the ConnectionVertex)
     * @return the newly created ConnectionVertex at the place of the replacementPoint
     */
    public ConnectionVertex splitSectionAndGetVertex(RoadSection roadSection, Point2D pointToSplitAt, Point2D replacementPoint) {
        List<Point2D> firstPoint2DList = new ArrayList<>();
        List<Point2D> secondPoint2DList = new ArrayList<>();
        if (replacementPoint == null) {
            replacementPoint = pointToSplitAt;
        }

        // First, we find which index the roadSection has.
        double splitUpIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(roadSection.getTrajectory(), pointToSplitAt);

        if (splitUpIndex == -1) {
            throw new IllegalArgumentException("RoadMap.splitSectionAndGetVertex(). Given point is not in  pointList.");
        }

        if (DoubleMath.fuzzyEquals(splitUpIndex - Math.floor(splitUpIndex), 0, 1E-3)){
            splitUpIndex = Math.round(splitUpIndex);
        } else {
            Point2D addedPoint = GeometryUtil.getTrajectoryDecimalPoint(roadSection.getTrajectory(), splitUpIndex);
            double oldIndex = splitUpIndex;
            splitUpIndex = Math.ceil(splitUpIndex);
            if (roadSection.getPointList().get((int) Math.floor(oldIndex)).distance(addedPoint) < 1){
                splitUpIndex = Math.floor(splitUpIndex);
            } else if (roadSection.getPointList().get((int) Math.ceil(oldIndex)).distance(addedPoint) < 1){
                splitUpIndex = Math.ceil(oldIndex);
            } else {
                roadSection.addNewPointToPointList((int) splitUpIndex, addedPoint);
            }
        }


//        List<Point2D> point2DList = roadSection.getPointList();
//        for (int i = 0; i < point2DList.size(); i++) {
//            if (point2DList.get(i).distance(pointToSplitAt) < 1) {
//                splitUpIndex = i;
//                break;
//            }
//        }

        // Second, we check whether the roadSection has been replaced in the mean time, and if so, get the one containing
        // this index.
        RoadSection oldRoadSection = roadSection;
        roadSection = replaceRoadIfItHadBeenSplitUp(roadSection, (int) splitUpIndex);

        // Third, we find again the given point in the new roadSection and act accordingly
        if (oldRoadSection != roadSection) {
            splitUpIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(roadSection.getTrajectory(), pointToSplitAt);
            if (DoubleMath.fuzzyEquals(splitUpIndex % 0, 0, 1E-3)) {
                splitUpIndex = Math.round(splitUpIndex);
            } else {
                Point2D addedPoint = GeometryUtil.getTrajectoryDecimalPoint(roadSection.getTrajectory(), splitUpIndex);
                double oldIndex = splitUpIndex;
                splitUpIndex = Math.ceil(splitUpIndex);
                if (roadSection.getPointList().get((int) Math.floor(oldIndex)).distance(addedPoint) < 1){
                    splitUpIndex = Math.floor(splitUpIndex);
                } else if (roadSection.getPointList().get((int) Math.ceil(oldIndex)).distance(addedPoint) < 1){
                    splitUpIndex = Math.ceil(oldIndex);
                } else {
                    roadSection.addNewPointToPointList((int) splitUpIndex, addedPoint);
                }
            }
        }


        oldRoadSection = roadSection;
        roadSection = replaceRoadIfItHadBeenSplitUp(roadSection, (int) splitUpIndex);

        // Third, we find again the given point in the new roadSection and act accordingly
        if (oldRoadSection != roadSection) {
            splitUpIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(roadSection.getTrajectory(), pointToSplitAt);
            if (DoubleMath.fuzzyEquals(splitUpIndex % 0, 0, 1E-3)) {
                splitUpIndex = Math.round(splitUpIndex);
            } else {
                Point2D addedPoint = GeometryUtil.getTrajectoryDecimalPoint(roadSection.getTrajectory(), splitUpIndex);
                double oldIndex = splitUpIndex;
                splitUpIndex = Math.ceil(splitUpIndex);
                if (roadSection.getPointList().get((int) Math.floor(oldIndex)).distance(addedPoint) < 1){
                    splitUpIndex = Math.floor(splitUpIndex);
                } else if (roadSection.getPointList().get((int) Math.ceil(oldIndex)).distance(addedPoint) < 1){
                    splitUpIndex = Math.ceil(oldIndex);
                } else {
                    roadSection.addNewPointToPointList((int) splitUpIndex, addedPoint);
                }
            }
        }
//
//        splitUpIndex = -1;
//        point2DList = roadSection.getPointList();
//        for (int i = 0; i < point2DList.size(); i++) {
//            if (point2DList.get(i).distance(pointToSplitAt) < 1.1) {
//                splitUpIndex = i;
//                break;
//            }
//        }
        // If first or lasts, we simply return the given points.
        List<Point2D> roadSectionPointList = roadSection.getPointList();
        if (DoubleMath.fuzzyEquals(splitUpIndex, 0, 1E-4)) {
            return roadSection.getVertex(roadSectionPointList.get((int) splitUpIndex));
        }
        if (DoubleMath.fuzzyEquals(splitUpIndex, roadSectionPointList.size() - 1, 1E-4)) {
            return roadSection.getVertex(roadSectionPointList.get((int) splitUpIndex));
        }

        // Create the pointLists
        for (int i = 0; i < roadSectionPointList.size(); i++) {
            if (i <= splitUpIndex) {
                firstPoint2DList.add(roadSectionPointList.get(i));
            }
            if (i >= splitUpIndex) {
                secondPoint2DList.add(roadSectionPointList.get(i));
            }
        }

        ConnectionVertex middleConnectedVertex = getConnectionVertex(replacementPoint);
        int id = roadSection.getUid();
        if (roadSection.getStartVertex().getLocation().distance(middleConnectedVertex.getLocation()) < 1){
            System.out.println("Error RoadMap splitting roadSection! - StartVertex");
            return middleConnectedVertex;
        }
        if (middleConnectedVertex.getLocation().distance(roadSection.getEndVertex().getLocation()) < 1){
            System.out.println("Error RoadMap splitting roadSection! - EndVertex");
            return middleConnectedVertex;
        }

        RoadSection firstRoadSection = new RoadSection(firstPoint2DList, roadSection.getStartVertex(), middleConnectedVertex,
                roadSection.getDrawnBundleStreet(), id, roadSection);
        RoadSection secondRoadSection = new RoadSection(secondPoint2DList, middleConnectedVertex, roadSection.getEndVertex(),
                roadSection.getDrawnBundleStreet(), id, roadSection);


        updateRoadSectionSplitUps(roadSection, (int) splitUpIndex, firstRoadSection, secondRoadSection);
        forceRemoveRoadSection(roadSection);

        forceAddRoadSection(firstRoadSection);
        forceAddRoadSection(secondRoadSection);
        return middleConnectedVertex;
    }

    /**
     * Split a RoadSection by adding a new point to the roadEdge and splitting on that. Returning the
     * newly created ConnectionVertex
     *
     * @param roadSection,      the roadSection we want to split up.
     * @param indexToSplitAt, the point to split at in the roadSection pointList.
     * @param addedPoint,       the point to replace the pointToSplitAt with.
     *                          (If null, pointToSplitAt is used for the ConnectionVertex)
     * @return the newly created ConnectionVertex at the place of the added point
     */
    public ConnectionVertex splitSectionAtNewlyIntroducedPoint(RoadSection roadSection, int indexToSplitAt, Point2D addedPoint){
        List<Point2D> firstPoint2DList = new ArrayList<>();
        List<Point2D> secondPoint2DList = new ArrayList<>();

        if (indexToSplitAt == 0){
            return roadSection.getStartVertex();
        } else if (indexToSplitAt == roadSection.getPointList().size() - 1){
            return roadSection.getEndVertex();
        }
        if (indexToSplitAt < 0 || indexToSplitAt >= roadSection.getPointList().size()) {
            System.out.println(indexToSplitAt);
            throw new IllegalArgumentException("RoadMap.splitSectionAndGetVertex(). Given index is invalid.");
        }
        Point2D toSplitAt = roadSection.getPointList().get(indexToSplitAt);
        List<Point2D> originalPointList = roadSection.getPointList();

        // Second, we check whether the roadSection has been replaced in the mean time, and if so, get the one containing
        // this index.
        roadSection = replaceRoadIfItHadBeenSplitUp(roadSection, indexToSplitAt);

        // Third, we find again the given point in the new roadSection and act accordingly
        int splitUpIndex = -1;
        List<Point2D> point2DList = roadSection.getPointList();
        for (int i = 0; i < point2DList.size(); i++) {
            if (point2DList.get(i).distance(toSplitAt) < 1.1) {
                splitUpIndex = i;
                break;
            }
        }
        // If first or lasts, we simply return the given points.
        if (splitUpIndex == 0) {
            return roadSection.getVertex(point2DList.get(splitUpIndex));
        }
        if (splitUpIndex == point2DList.size() - 1) {
            return roadSection.getVertex(point2DList.get(splitUpIndex));
        }

        // Create the pointLists
        secondPoint2DList.add(addedPoint);
        for (int i = 0; i < point2DList.size(); i++) {
            if (i < splitUpIndex) {
                firstPoint2DList.add(point2DList.get(i));
            }
            if (i >= splitUpIndex) {
                secondPoint2DList.add(point2DList.get(i));
            }
        }
        firstPoint2DList.add(addedPoint);

        ConnectionVertex middleConnectedVertex = getConnectionVertex(addedPoint);
        int id = roadSection.getUid();
//        id = 2345;2
        RoadSection firstRoadSection = new RoadSection(firstPoint2DList, roadSection.getStartVertex(), middleConnectedVertex,
                roadSection.getDrawnBundleStreet(), id, roadSection);
        RoadSection secondRoadSection = new RoadSection(secondPoint2DList, middleConnectedVertex, roadSection.getEndVertex(),
                roadSection.getDrawnBundleStreet(), id, roadSection);

        updateRoadSectionSplitUps(roadSection, splitUpIndex, firstRoadSection, secondRoadSection);
        forceRemoveRoadSection(roadSection);

        forceAddRoadSection(firstRoadSection);
        forceAddRoadSection(secondRoadSection);
        return middleConnectedVertex;
    }

    /**
     * We check whether a split up road section could be merged again because we removed a RoadSection at the point
     * where it was split up.
     *
     * @param roadSections the RoadSections that are still remaining at a specific vertex.
     */
    private void checkIfRoadSectionsShouldBeMerged(Set<RoadSection> roadSections) {
        if (roadSections == null || roadSections.size() != 2) {
            return;
        }

        for (Map.Entry<RoadSection, Map<Range<Integer>, RoadSection>> roadSectionMapEntry :
                roadSectionSplitUpList.entrySet()) {
            RoadSection originalRoadSection = roadSectionMapEntry.getKey();
            Map<Range<Integer>, RoadSection> rangeRoadSectionMap = roadSectionMapEntry.getValue();
            int foundRoadSections = 0;

            for (RoadSection roadSection : rangeRoadSectionMap.values()) {
                if (roadSections.contains(roadSection)) {
                    foundRoadSections++;
                }
            }
            if (foundRoadSections == 2) {
                ArrayList<RoadSection> roadSectionArray = new ArrayList<>(roadSections);
                forceRemoveRoadSection(roadSectionArray.get(0));
                forceRemoveRoadSection(roadSectionArray.get(1));
                roadSectionArray.get(0).setRemoved(true);
                roadSectionArray.get(1).setRemoved(true);

                // Fully remove them to prevent adding them twice later on...
                presentRoadSections.remove(roadSectionArray.get(0));
                presentRoadSections.remove(roadSectionArray.get(1));


                forceAddRoadSection(originalRoadSection);
                originalRoadSection.setRemoved(false);
                roadSectionSplitUpList.remove(originalRoadSection);
                return;
            }
        }
    }

    /**
     * If the roadSection is in the roadSectionSplitUpList hashMap, then it has been split up, and we return the split up
     * version.
     *
     * @param roadSection  the roadSection we are currently looking at
     * @param splitUpIndex the index of the point we should split at in the original roadSection
     * @return the roadSection that contains the point at pointToSplitAt.
     */
    private RoadSection replaceRoadIfItHadBeenSplitUp(RoadSection roadSection, int splitUpIndex) {
        if (roadSectionSplitUpList.containsKey(roadSection)) {
            for (Range<Integer> range : roadSectionSplitUpList.get(roadSection).keySet()) {
                if (range.contains(splitUpIndex)) {
                    return roadSectionSplitUpList.get(roadSection).get(range);
                }
            }
            throw new IllegalArgumentException("RoadMap.replaceRoadIfItHadBeenSplitUp. " +
                    "splitUpIndex is not present in roadSection.");
        }
        return roadSection;
    }

    /**
     * We keep track of split ups in our roadMap.
     *
     * @param originalRoadSection the roadSection we are looking for.
     * @param splitUpIndex        the index of the point (in the originalRoadSection) at which we split up.
     * @param firstRoadSection    the first roadSection which contains the points from 0 to splitUpIndex.
     * @param secondRoadSection   the second roadSection which contains the points from splitUpIndex to the last point.
     */
    private void updateRoadSectionSplitUps(RoadSection originalRoadSection, int splitUpIndex,
                                           RoadSection firstRoadSection, RoadSection secondRoadSection) {
//        if (roadSectionSplitUpList.containsKey(originalRoadSection)) {
//            throw new IllegalArgumentException("RoadMap.updateRoadSectionSplitUps(). Already contains originalRoadSection");
//        }

        HashMap<Range<Integer>, RoadSection> hashMap = new HashMap<>();
        Range<Integer> first = Range.closed(0, splitUpIndex - 1);
        Range<Integer> second = Range.closed(splitUpIndex, originalRoadSection.getPointList().size() - 1);
        hashMap.put(first, firstRoadSection);
        hashMap.put(second, secondRoadSection);
        firstRoadSection.setRemoved(false);
        secondRoadSection.setRemoved(false);
        originalRoadSection.setRemoved(true);

        roadSectionSplitUpList.put(originalRoadSection, hashMap);
    }

    /**
     * For a specific Connection Vertex, we get all RoadSections.
     *
     * @param connectionVertex, get all RoadSections.
     * @return return all RoadSections which are connected to a specific connectionVertex.
     * (emptySet if does not contain key)
     */
    public Set<RoadSection> getRoadSectionsForConnectionVertex(ConnectionVertex connectionVertex) {
        Set<RoadSection> roadSections = new HashSet<>();
        for(RoadSection roadSection: adjacencyList.get(connectionVertex)){
            if (!roadSection.isRemoved()){
                roadSections.add(roadSection);
            }
        }
        return roadSections;
    }

    /**
     * Returns a ConnectionVertex containing the given point. If none exists yet at that position, we create one.
     *
     * @param point2D the location of the ConnectionVertex
     * @return the connectionVertex at the given location (either one that already existed or a new one).
     */
    public ConnectionVertex getConnectionVertex(Point2D point2D) {
        if (point2D == null) {
            throw new IllegalArgumentException("Point2D is null");
        }
        for (ConnectionVertex vertex : getConnectionVertices()) {
            if (vertex.getLocation().distance(point2D) < 1) {
                return vertex;
            }
        }
        return new ConnectionVertex(point2D, null);
    }

    /**
     * Check whether the secondPoint can be reached starting from the firstPoint within maxPathLength meters.
     *
     * @param firstIndex        the index on the firstRoadSection on the roadMap.
     * @param firstRoadSection  the roadSection the firstPoint is a part of.
     * @param secondIndex       the index on the secondRoadSection on the roadMap.
     * @param secondRoadSection the roadSection the secondPoint is a part of.
     * @param maxPathLength     the maxPathLength.
     * @return whether there is a path of maxPathLength.
     */
    public boolean checkIfTwoPointsAreConnectedWithinDistance(double firstIndex, RoadSection firstRoadSection,
                                                              double secondIndex, RoadSection secondRoadSection,
                                                              double maxPathLength) {

        if (firstRoadSection.equals(secondRoadSection)) {
            double distance = GeometryUtil.getIndexToIndexDistance(firstRoadSection.getPointList(), firstIndex, secondIndex);
            return (distance < maxPathLength);
        }

        // We calculate if the distance from firstPoint to end of secondRoadSection (passing startingVertex) is
        // less than maxPathLength in continuous length.
        double distanceCovered = GeometryUtil.getIndexToIndexDistance(secondRoadSection.getPointList(),
                0, secondIndex);
        if (checkIfTwoPointsAreConnectedLoopForRoadSection(
                firstIndex, firstRoadSection, secondRoadSection.getStartVertex(),
                maxPathLength - distanceCovered)) {
            return true;
        }

        // We calculate if the distance from firstPoint to end of secondRoadSection (passing endVertex) is
        // less than maxPathLength in continuous length.
        distanceCovered = GeometryUtil.getIndexToIndexDistance(secondRoadSection.getPointList(),
                secondIndex, secondRoadSection.getPointList().size() - 1);
        return checkIfTwoPointsAreConnectedLoopForRoadSection(
                firstIndex, firstRoadSection, secondRoadSection.getEndVertex(),
                maxPathLength - distanceCovered);
    }

    /**
     * Tries to find a path of less than maxRemainingPathLength meters from firstPoint to endVertex, if found
     * we return true. (firstPoint = firstIndex on firstRoadSection.getPointList()).
     *
     * @param firstIndex             the index on the firstRoadSection of the firstPoint
     * @param firstRoadSection       the roadSection the firstPoint is a part of.
     * @param endVertex              the final vertex we want to end at
     * @param maxRemainingPathLength the remaining distance we can travel before we should find our endVertex.
     * @return whether there is a path of less than maxRemainingPathLength meters from firstPoint to endVertex.
     */
    private boolean checkIfTwoPointsAreConnectedLoopForRoadSection(double firstIndex, RoadSection firstRoadSection,
                                                                   ConnectionVertex endVertex,
                                                                   double maxRemainingPathLength) {
        if (maxRemainingPathLength < 0) {
            return false;
        }
        double distanceCovered = GeometryUtil.getIndexToIndexDistance(firstRoadSection.getPointList(),
                0, firstIndex);
        if (checkIfTwoPointsAreConnected(firstRoadSection.getStartVertex(), endVertex,
                maxRemainingPathLength - distanceCovered, new HashSet<>())) {
            return true;
        }

        // We calculate if the distance from firstPoint to end of secondRoadSection (passing endVertex) is
        // less than maxPathLength in continuous length.
        distanceCovered = GeometryUtil.getIndexToIndexDistance(firstRoadSection.getPointList(),
                firstIndex, firstRoadSection.getPointList().size() - 1);
        return checkIfTwoPointsAreConnected(firstRoadSection.getEndVertex(), endVertex,
                maxRemainingPathLength - distanceCovered, new HashSet<>());
    }

    /**
     * Check whether the secondIndex can be reached starting from the firstPoint within maxPathLength meters.
     *
     * @param startCV                the current connectionVertex we are trying to move away from to goalCV.
     * @param goalCV                 the goalConnectionVertex.
     * @param maxRemainingPathLength the remaining distance we are allowed to cover before we return false
     * @return true if we can go from startCV to goalCV within the given maxRemainingPathLenght, false otherwise.
     */
    public boolean checkIfTwoPointsAreConnected(ConnectionVertex startCV, ConnectionVertex goalCV,
                                                double maxRemainingPathLength, Set<RoadSection> settledSections) {
        if (maxRemainingPathLength < 0) {
            return false;
        }

        if (startCV.equals(goalCV)) {
            return true;
        }


        Set<RoadSection> roadSections = getRoadSectionsForConnectionVertex(startCV);
        for (RoadSection roadSection : roadSections) {
            if (settledSections.contains(roadSection)) {
                continue;
            }
            Set<RoadSection> copySettled = new HashSet<>(settledSections);
            copySettled.add(roadSection);
            if (checkIfTwoPointsAreConnected(roadSection.getOppositeVertex(startCV), goalCV,
                    maxRemainingPathLength - roadSection.getContinuousLength(), copySettled)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the first found roadSection containing the given point. If none, return null.
     *
     * @param point2D the point we want to find on a roadSection
     * @return either the roadSection if there is one containing this point, otherwise we return null.
     */
    private RoadSection findRoadSectionContainingPoint(Point2D point2D) {
        for (RoadSection roadSection : getPresentRoadSections()) {
            for (int i = 0; i < roadSection.getPointList().size(); i++) {
                if (roadSection.getPointList().get(i).distance(point2D) < 1) {
                    return roadSection;
                }
            }
        }
        return null;
    }

    /**
     * Here we check if there is another roadSection which consists of exactly two points and has become useless.
     * This avoids having multiple edges for the same connection.
     */
    private void checkIfWeShouldRemoveEarlierRoadEdges(RoadSection roadSection){
        Set<RoadSection> roadSections1 = getRoadSectionsForConnectionVertex(roadSection.getStartVertex());
        Set<RoadSection> roadSections2 = getRoadSectionsForConnectionVertex(roadSection.getEndVertex());
        for (RoadSection roadSection1: roadSections1){
            if (!roadSections2.contains(roadSection1)){
                continue;
            }

            // @BugFix
            if (roadSection.getStartVertex() == roadSection.getEndVertex() && roadSection1.getStartVertex() != roadSection1.getEndVertex()){
                continue;
            }

            if (roadSection1.getPointList().size() > 2){
                continue;
            }

            if (STORAGE.getDatasetConfig().isWalkingDataset()) {
                removeRoadSection(roadSection1);
            }
        }
    }

//    /**
//     * Improves the pointList such that it contains more points. This is not useful for the athens dataset..
//     * @deprecated does not work well for athens dataset...
//     */
//    public List<Point2D> improvePointList(List<Point2D> originalPointList) {
//        if (GeometryUtil.getContinuousLength(originalPointList) < 100) {
//            return originalPointList;
//        }
//        List<Point2D> modifiedPointList = new ArrayList<>(originalPointList);
//
//        double maxDistance = 50.0;
//        double idealDistance = 25.0;
//        double minimumDistanceFromConnectionVertex = 50.0;
//        double maxDistanceFromConnectionVertex = 75.0;
//        List<Point2D> newList = new ArrayList<>();
//        Line2D firstLine = new Line2D.Double(modifiedPointList.get(0), modifiedPointList.get(1));
//        if (GeometryUtil.lineLength(firstLine) > maxDistanceFromConnectionVertex) {
//            Point2D fiftyMetersFromFirst = GeometryUtil.getPointOnLine(firstLine,
//                    minimumDistanceFromConnectionVertex / GeometryUtil.lineLength(firstLine));
//            modifiedPointList.set(0, fiftyMetersFromFirst);
//        }
//
//        Line2D lastReversedLine = new Line2D.Double(modifiedPointList.get(modifiedPointList.size() - 1),
//                modifiedPointList.get(modifiedPointList.size() - 2));
//        if (GeometryUtil.lineLength(lastReversedLine) > maxDistanceFromConnectionVertex) {
//            Point2D fiftyMetersFromLast = GeometryUtil.getPointOnLine(lastReversedLine,
//                    minimumDistanceFromConnectionVertex / GeometryUtil.lineLength(lastReversedLine));
//            modifiedPointList.set(modifiedPointList.size() - 1, fiftyMetersFromLast);
//        }
//
//        newList.add(originalPointList.get(0));
//
//        for (int i = 0; i < modifiedPointList.size() - 1; i++) {
//            Point2D currentPoint = modifiedPointList.get(i);
//            Point2D nextPoint = modifiedPointList.get(i + 1);
//
//            newList.add(currentPoint);
//            if (currentPoint.distance(nextPoint) > maxDistance) {
//                Line2D line2D = new Line2D.Double(currentPoint, nextPoint);
//                double distance = currentPoint.distance(nextPoint);
//
//                double residu = distance % idealDistance;
//                double nop = (distance - residu) / idealDistance;
//                double perItem = distance / nop;
//                double percentagePerItem = perItem / distance;
//
//                for (int j = 0; j < nop; j++) {
//                    newList.add(GeometryUtil.getPointOnLine(line2D, Math.max(0, Math.min(1, percentagePerItem * j))));
//                }
//            }
//        }
//        newList.add(modifiedPointList.get(modifiedPointList.size() - 1));
//        newList.add(originalPointList.get(originalPointList.size() - 1));
//
//        if (newList.size() < originalPointList.size()) {
//            throw new IllegalArgumentException("Something wrong...");
//        }
//        return newList;
//    }


    @Override
    public String toString() {
        return "RoadMap{" + adjacencyList + '}';
    }

    @JsonProperty
    public int numberOfBundlesPresentInRoadMap(){
        Set<Bundle> bundles = new HashSet<>();
        for (RoadSection roadSection : getPresentRoadSections()){
            bundles.add(roadSection.getDrawnBundleStreet().getBundle());
        }
        return bundles.size();
    }

}
