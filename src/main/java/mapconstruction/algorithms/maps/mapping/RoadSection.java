package mapconstruction.algorithms.maps.mapping;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.collect.Range;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;
import java.util.stream.Collectors;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * A RoadSection is an element which connects two ConnectionVertices by an improved trajectory.
 *
 * @author Jorrick Sleijster
 */
public class RoadSection implements Serializable {

    private List<Point2D> pointList;
    private ConnectionVertex startVertex;
    private ConnectionVertex endVertex;
    private BundleStreet drawnBundleStreet;
    private Set<Subtrajectory> subtrajectories;
    private HashMap<BundleStreet, Range<Double>> bundleStreets;
    private int uid;
    private boolean isConnectionBetweenMergeAreas;
    private boolean isRemoved;

    /**
     * Create an edge connected between two ConnectionVertexes.
     *
     * @param paramPointList the list of points which represent a trajectory
     * @param vertex1        one of the two ConnectionVertices at the end.
     * @param vertex2        one of the two ConnectionVertices at the end.
     * @param bundleStreet   the bundleStreet that caused this RoadSection to be drawn.
     */
    RoadSection(List<Point2D> paramPointList, ConnectionVertex vertex1, ConnectionVertex vertex2, BundleStreet bundleStreet,
                int uid) {
        if (paramPointList.size() < 2) {
            throw new IllegalArgumentException("Incorrect paramPointList");
        }
        pointList = paramPointList;
        for (int i = 0; i < pointList.size(); i++) {
            if (GeometryUtil.isPointNan(pointList.get(i))) {
                throw new IllegalArgumentException("Contains NAN");
            }
        }

        // @ToDo to remove
        for (int i = 0; i < pointList.size() - 1; i++) {
            if (pointList.get(i).distance(pointList.get(i + 1)) > 400) {
//                System.out.println("Weird long edge for walking");
            }
        }

        Point2D vertex1l = vertex1.getLocation();
        Point2D vertex2l = vertex2.getLocation();

        Point2D startV = pointList.get(0);
        Point2D endV = pointList.get(pointList.size() - 1);

        if (pointList.size() <= 1) {
            System.out.println("RoadSection.RoadSection() pointList is to small");
            throw new IllegalArgumentException("RoadSection.RoadSection() pointList is to small");
        }
        if (vertex1l == null) {
            System.out.println("RoadSection.RoadSection() vertex1l is null");
        }
        if (vertex2l == null) {
            System.out.println("RoadSection.RoadSection() vertex2l is null");
        }
        if (vertex1.getLocation().distance(startV) >= 1) {
            throw new IllegalArgumentException("RoadSection.RoadSection. Invalid vertex1");
        }
        if (vertex2.getLocation().distance(endV) >= 1) {
            throw new IllegalArgumentException("RoadSection.RoadSection. Invalid vertex2");
        }

        this.startVertex = vertex1;
        this.endVertex = vertex2;

        drawnBundleStreet = bundleStreet;
        this.bundleStreets = new HashMap<>();
        this.subtrajectories = new HashSet<>();

        addNewBundleStreet(bundleStreet, Range.closed(0.0, (double) pointList.size() - 1));

        this.uid = uid;
        this.isRemoved = false;
    }


    /**
     * Create an edge connected between two ConnectionVertexes.
     * This routine is only meant for when we are splitting up roadsections.
     *
     * @param pointList          the list of points which represent a trajectory
     * @param vertex1,           one of the two ConnectionVertices at the end.
     * @param vertex2,           one of the two ConnectionVertices at the end.
     * @param bundleStreet,      the BundleStreet that caused this RoadSection to be drawn.
     * @param oldRoadSection,    the old RoadSection object from which we get the RoadSection
     */
    RoadSection(List<Point2D> pointList, ConnectionVertex vertex1, ConnectionVertex vertex2, BundleStreet bundleStreet,
                int uid, RoadSection oldRoadSection) {
        this(pointList, vertex1, vertex2, bundleStreet.getSubBundleStreet(vertex1.getLocation(), vertex2.getLocation()), uid);

        HashMap<BundleStreet, Range<Double>> oldBundleStreetHashMap = oldRoadSection.getBundleStreetsWithRanges();

        for (Map.Entry<BundleStreet, Range<Double>> entry : oldBundleStreetHashMap.entrySet()) {
            BundleStreet bundleStreet1 = entry.getKey();
            Range<Double> range1 = entry.getValue();
            List<Point2D> oldPointList = oldRoadSection.getPointList();

            if (range1.lowerEndpoint().equals(range1.upperEndpoint())){
                continue;
            }

            double startIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(new FullTrajectory(oldPointList), vertex1.getLocation());
            double endIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(new FullTrajectory(oldPointList), vertex2.getLocation());

            if (startIndex > endIndex){
                double tempIndex = startIndex;
                startIndex = endIndex;
                endIndex = tempIndex;
            }

            if (!range1.contains(startIndex) && !range1.contains(endIndex)){
                continue;
            } else if (!range1.contains(startIndex)){
                startIndex = range1.lowerEndpoint();
            } else if (!range1.contains(endIndex)){
                endIndex = range1.upperEndpoint();
            }

            BundleStreet bundleStreet2 = bundleStreet1.getSubBundleStreet(vertex1.getLocation(), vertex2.getLocation());
            if (bundleStreet2.getRepresentativeSubtrajectory().numEdges() < 1 ||
                    DoubleMath.fuzzyEquals(startIndex, endIndex, 1E-3)){
                continue;
            }
            addNewBundleStreet(bundleStreet2, Range.closed(startIndex, endIndex));
        }
    }

    /**
     * When a new BundleStreet get's added to the graph, but instead of drawing it, we merge it into this roadSections.
     * @param range, the range of the points on this RoadSection for which this bundleStreet is considered merged.
     * @param bundleStreet, the part of the bundle that is considered drawn in the road map because of this roadSection.
     */
    public void addNewBundleStreet(BundleStreet bundleStreet, Range<Double> range) {
        bundleStreets.put(bundleStreet, range);

        for (Subtrajectory sub1 : bundleStreet.getSubtrajectories()) {
            if (sub1.getFromIndex() == sub1.getToIndex()){
                continue;
            }

            if (sub1.isReverse()) {
                sub1 = sub1.reverse();
            }

            Subtrajectory alreadyPresentSub = null;
            for (Subtrajectory sub2 : subtrajectories) {
                if (sub1.getParent().equals(sub2.getParent())) {
                    alreadyPresentSub = sub2;
                    break;
                }
            }
            if (alreadyPresentSub != null) {
                double subStartIndex = Math.min(alreadyPresentSub.getFromIndex(), sub1.getFromIndex());
                double subEndIndex = Math.max(alreadyPresentSub.getToIndex(), sub1.getToIndex());
                sub1 = new Subtrajectory(sub1.getParent(), subStartIndex, subEndIndex);
            }

            Point2D startPoint = GeometryUtil.getTrajectoryDecimalPoint(bundleStreet.getBundle().getRepresentative(), bundleStreet.getStartIndex());
            Point2D endPoint = GeometryUtil.getTrajectoryDecimalPoint(bundleStreet.getBundle().getRepresentative(), bundleStreet.getEndIndex());

            double startIndex1 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub1, startPoint);
            double endIndex1 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub1, endPoint);

            startIndex1 = GeometryUtil.convertSubIndexToTrajectoryIndex(sub1, startIndex1);
            endIndex1 = GeometryUtil.convertSubIndexToTrajectoryIndex(sub1, endIndex1);

            if (endIndex1 < startIndex1) {
                double temp = startIndex1;
                startIndex1 = endIndex1;
                endIndex1 = temp;
            }


            double startIndex2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub1, pointList.get(0));
            double endIndex2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub1, pointList.get(pointList.size() - 1));

            startIndex2 = GeometryUtil.convertSubIndexToTrajectoryIndex(sub1, startIndex2);
            endIndex2 = GeometryUtil.convertSubIndexToTrajectoryIndex(sub1, endIndex2);


            if (endIndex2 < startIndex2) {
                double temp = startIndex2;
                startIndex2 = endIndex2;
                endIndex2 = temp;
            }

            double startIndex = Math.max(startIndex1, startIndex2);
            double endIndex = Math.min(endIndex1, endIndex2);

            if (endIndex < startIndex) {
                double temp = startIndex;
                startIndex = endIndex;
                endIndex = temp;
            }

            Subtrajectory sub3 = new Subtrajectory(sub1.getParent(), startIndex, endIndex);
            if (GeometryUtil.getContinuousLength(sub3) < 0.1){
                continue;
            }

            if (alreadyPresentSub != null) {
                double distanceAlreadySub = Math.min(alreadyPresentSub.getFirstPoint().distance(pointList.get(0)) +
                                alreadyPresentSub.getLastPoint().distance(pointList.get(pointList.size() - 1)),
                        alreadyPresentSub.getLastPoint().distance(pointList.get(0)) +
                                alreadyPresentSub.getFirstPoint().distance(pointList.get(pointList.size() - 1)));

                double distanceSub3 = Math.min(sub3.getFirstPoint().distance(pointList.get(0)) +
                                sub3.getLastPoint().distance(pointList.get(pointList.size() - 1)),
                        sub3.getLastPoint().distance(pointList.get(0)) +
                                sub3.getFirstPoint().distance(pointList.get(pointList.size() - 1)));

                if (distanceSub3 < distanceAlreadySub) {
                    subtrajectories.remove(alreadyPresentSub);
                    subtrajectories.add(sub3);
                }
            } else {
                subtrajectories.add(sub3);
            }
        }
    }

    /**
     * Get's both connectionVertices
     *
     * @return the start and end vertex.
     */
    @JsonIgnore
    public List<ConnectionVertex> getConnectionVertices() {
        List<ConnectionVertex> vertices = new ArrayList<>();
        vertices.add(this.startVertex);
        vertices.add(this.endVertex);
        return vertices;
    }

    /**
     * Get's the start vertex
     *
     * @return start vertex
     */
    @JsonProperty
    public ConnectionVertex getStartVertex() {
        return startVertex;
    }

    /**
     * Get's the end vertex
     *
     * @return end vertex
     */
    @JsonProperty
    public ConnectionVertex getEndVertex() {
        return endVertex;
    }

    /**
     * Get's the Connection Vertex object given the location of the connection vertex
     *
     * @param location location of either the start or end vertex
     * @return the connection vertex object at the given location.
     */
    @JsonProperty
    public ConnectionVertex getVertex(Point2D location) {
        if (startVertex.getLocation().distance(location) < 1) {
            return startVertex;
        } else if (endVertex.getLocation().distance(location) < 1) {
            return endVertex;
        }
        throw new IllegalArgumentException("RoadSection.getVertex. ConnectionVertex unknown.");
    }

    /**
     * Get's the Connection Vertex object that is not at the given location
     *
     * @param connectionVertex location of either the start or end vertex
     * @return if it is the start vertexes location, we return the end vertex, and the other way around.
     */
    @JsonIgnore
    public ConnectionVertex getOppositeVertex(ConnectionVertex connectionVertex) {
        if (getStartVertex().equals(connectionVertex)) {
            return endVertex;
        } else if (getEndVertex().equals(connectionVertex)) {
            return startVertex;
        }
        throw new IllegalArgumentException("RoadSection.getOppositeVertex. ConnectionVertex unknown.");
    }

    /**
     * Get's the Connection Vertex object that is not at the given location
     *
     * @param location location of either the start or end vertex
     * @return if it is the start vertexes location, we return the end vertex, and the other way around.
     */
    @JsonProperty
    public ConnectionVertex getOppositeVertex(Point2D location) {
        if (startVertex.getLocation().distance(location) < 1) {
            return endVertex;
        } else if (endVertex.getLocation().distance(location) < 1) {
            return startVertex;
        }
        throw new IllegalArgumentException("RoadSection.getOppositeVertex. ConnectionVertex distance to high.");
    }


    /**
     * Get the PointList with the startConnectionVertex at the start
     *
     * @param startConnectionVertex, the connectionVertex where the pointList should start with
     * @return the PointList with startConnectionVertex as first point.
     */
    public List<Point2D> getPointListWithVertexAtStart(ConnectionVertex startConnectionVertex) {
        List<Point2D> newPointList = new ArrayList<>(getPointList());
        if (startConnectionVertex.equals(this.endVertex)) {
            Collections.reverse(newPointList);
        }
        return newPointList;
    }

    /**
     * Get's the point list
     *
     * @return point list
     */
    @JsonProperty
    public synchronized List<Point2D> getPointList() {
        return new ArrayList<>(pointList);
    }

    public synchronized void addNewPointToPointList(int index, Point2D pointToAdd){
        if (pointToAdd.distance(pointList.get(Math.min(pointList.size() - 1, index))) < 0.1){
            System.out.println("ERROR, adding same point again and again?!");
        }
        pointList.add(index, pointToAdd);
    }

    /**
     * Get's the reversed point list
     *
     * @return reversed point list
     */
    @JsonIgnore
    private List<Point2D> getReversedPointList() {
        List<Point2D> newPointList = new ArrayList<>(getPointList());
        Collections.reverse(newPointList);
        return newPointList;
    }

    /**
     * Returns the pointList as a trajectory
     *
     * @return trajectory.
     */
    @JsonIgnore
    public Trajectory getTrajectory() {
        return new FullTrajectory(getPointList());
    }

    /**
     * Returns the pointList as a reversed trajectory
     *
     * @return trajectory
     */
    @JsonIgnore
    public Trajectory getReverseTrajectory() {
        return new FullTrajectory(getReversedPointList());
    }

    /**
     * Returns the pointList with startConnectionVertex at the start as a trajectory
     *
     * @param startConnectionVertex, the ConnectionVertex the trajectory should start with.
     * @return the trajectory with startConnectionVertex at the start as a trajectory
     */
    @JsonIgnore
    public Trajectory getTrajectoryWithVertexAtStart(ConnectionVertex startConnectionVertex) {
        return new FullTrajectory(getPointListWithVertexAtStart(startConnectionVertex));
    }

    /**
     * Get's the bundle street which made us draw this RoadSection.
     *
     * @return the drawn bundle street.
     */
    @JsonIgnore
    public BundleStreet getDrawnBundleStreet() {
        return drawnBundleStreet;
    }

    /**
     * Get's the bundle streets including the ranges
     * @return the hashmap
     */
    @JsonIgnore
    private HashMap<BundleStreet, Range<Double>> getBundleStreetsWithRanges(){
        return bundleStreets;
    }

    /**
     * Get's all bundle streets that made us draw this RoadSection and that merged into it.
     *
     * @return the bundleStreet.
     */
    @JsonIgnore
    public List<BundleStreet> getBundleStreets() {
        List<BundleStreet> streetList = new ArrayList<>();
        streetList.add(drawnBundleStreet);
        streetList.addAll(bundleStreets.keySet());
        return streetList;
    }

    /**
     * Get's the bundleClasses of the bundlestreets which made us draw this RoadSection.
     *
     * @return the bundleStreet.
     */
    @JsonProperty
    public Integer getDrawnBundleStreetClass() {
        return STORAGE.getClassFromBundle(drawnBundleStreet.getBundle());
    }

    /**
     * Get's the bundleClasses of the bundlestreets which made us draw this RoadSection and merged into it.
     *
     * @return the bundleStreet.
     */
    @JsonProperty
    public List<Integer> getBundleStreetsClasses() {
        List<Integer> bundleStreetClasses = new ArrayList<>();
        for (BundleStreet bundleStreet : bundleStreets.keySet()) {
            bundleStreetClasses.add(STORAGE.getClassFromBundle(bundleStreet.getBundle()));
        }
        return bundleStreetClasses;
    }

    /**
     * The distance from the start to the end of the pointList
     *
     * @return the continuous length of the whole point list.
     */
    @JsonProperty
    public double getContinuousLength() {
        return GeometryUtil.getContinuousLength(getPointList());
    }

    /**
     * Get's the id
     *
     * @return the id
     */
    @JsonProperty
    public int getUid() {
        return uid;
    }

    /**
     * Returns the index of the given point, only integer indexes support.
     *
     * @param point2D the point we want the index of.
     * @return the index.
     */
    @JsonIgnore
    public int getIndexOfPoint(Point2D point2D) {
        for (int i = 0; i < getPointList().size(); i++) {
            if (getPointList().get(i).distance(point2D) < 1) {
                return i;
            }
        }
        return -1;
    }

    @JsonProperty
    public boolean isConnectionBetweenMergeAreas() {
        return isConnectionBetweenMergeAreas;
    }

    @JsonProperty
    public void setConnectionBetweenMergeAreas(boolean connectionBetweenMergeAreas) {
        this.isConnectionBetweenMergeAreas = connectionBetweenMergeAreas;
    }

    /**
     * For each bundle, we take all subtrajectories and make them as large as possible with regards to the end point
     * of the bundle and the end point of the roadSection. Therefore the subtrajectory will always be at most as long as
     * the smallest of the two(the roadSection or the bundle).
     * @return All of the subtrajectories
     */
    @JsonProperty
    public Set<Subtrajectory> getSubtrajectories() {
        return subtrajectories;
    }

    @JsonProperty
    public boolean isRemoved() {
        return isRemoved;
    }

    public void setRemoved(boolean removed) {
        isRemoved = removed;
    }

    /**
     * Returns the parents of all the subtrajectories.
     * @return the parents
     */
    @JsonIgnore
    public Set<Trajectory> getParentsOfSubtrajectories() {
        Set<Trajectory> parents = new HashSet<>();
        for (Subtrajectory subtrajectory: getSubtrajectories()){
            parents.add(subtrajectory.getParent());
        }
        return parents;
    }

    /**
     * Returns the BundleStreets with the 50% lowest bestEpsilons or all bundles with less than 25 bestEpsilon value.
     * Whichever is higher is returned.
     */
    @JsonIgnore
    public List<BundleStreet> getBestHalfOfBundlesStreets() {
        List<BundleStreet> bundleStreets = getBundleStreets();

        bundleStreets = bundleStreets.stream()
                .sorted(
                Comparator.comparingDouble(b -> STORAGE.getEvolutionDiagram().getBestEpsilon(b.getBundleClass())))
                .collect(Collectors.toList());

        int firstBundleIndexBestEpsToMuch = 0;
        while (firstBundleIndexBestEpsToMuch < bundleStreets.size()){
            if (STORAGE.getEvolutionDiagram().getBestEpsilon(
                    bundleStreets.get(firstBundleIndexBestEpsToMuch).getBundleClass()) > 25){
                break;
            }
            firstBundleIndexBestEpsToMuch++;
        }

        List<BundleStreet> bestBundleStreets = bundleStreets.subList(0,
                (int) Math.max(firstBundleIndexBestEpsToMuch, Math.ceil(((double) bundleStreets.size()) / 2)));
        return bestBundleStreets;
    }

    @JsonProperty
    public List<Integer> getBestHalfOfBundlesStreetsClasses(){
        List<Integer> integers = new ArrayList<>();
        for (BundleStreet bundleStreet: getBestHalfOfBundlesStreets()){
            integers.add(bundleStreet.getBundleClass());
        }
        return integers;
    }

    /**
     * Returns all subtrajectories present for this halfOfBundleStreets.
     */
    @JsonIgnore
    public Set<Subtrajectory> getBestHalfOfBundlesAndReturnTheirSubtrajectories() {
        Set<Subtrajectory> bestHalfOfSubtrajectories = new HashSet<>();

        for (BundleStreet bundleStreet : getBestHalfOfBundlesStreets()) {
            for (Subtrajectory subtrajectory1 : bundleStreet.getSubtrajectories()) {
                for (Subtrajectory subtrajectory2 : getSubtrajectories()) {
                    if (subtrajectory1.overlaps(subtrajectory2)) {
                        bestHalfOfSubtrajectories.add(subtrajectory2);
                    }
                }
            }
        }
        return bestHalfOfSubtrajectories;
    }
}

