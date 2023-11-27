package mapconstruction.algorithms.maps.containers;

import mapconstruction.algorithms.maps.mapping.RoadSection;
import mapconstruction.util.Pair;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.List;

/**
 * This data object contains information about where our pointList is close to an existing RoadSection in the RoadMap.
 * Therefore it contains a list coveredParts with pairs. This contains the starting and ending index of the PointList
 * where which is a range of indexes where it is close to roadSection, which is already part of the RoadMap.
 *
 * @author Jorrick Sleijster
 */
public class MergedBundleStreetPart implements Serializable {

    private RoadSection roadSection;

    private List<Point2D> pointList;

    private List<Double> pointDistance;

    private List<Pair<Integer, Integer>> coveredParts;

    // Given the #-th pair of coveredParts, the #-th pair are the indexes on the roadEdges.
    private List<Pair<Double, Double>> roadEdgeIndexesByParts;

    public MergedBundleStreetPart(RoadSection roadSection, List<Point2D> pointList, List<Double> bestPointDistance,
                                  List<Pair<Integer, Integer>> coveredParts, List<Pair<Double, Double>> roadEdgeIndexesByParts) {
        this.roadSection = roadSection;
        this.pointList = pointList;
        this.pointDistance = bestPointDistance;
        this.coveredParts = coveredParts;
        this.roadEdgeIndexesByParts = roadEdgeIndexesByParts;
    }

    public RoadSection getRoadSection() {
        return roadSection;
    }

    public List<Point2D> getPointList() {
        return pointList;
    }

    public List<Double> getBestPointDistance(){
        return this.pointDistance;
    }

    public List<Pair<Integer, Integer>> getCoveredParts() {
        return coveredParts;
    }

    public List<Pair<Double, Double>> getRoadEdgeIndexesByParts(){
        return roadEdgeIndexesByParts;
    }

    public Integer getLastIndexOfCoveredParts(){
        if (coveredParts.size() == 0){
            return null;
        }
        return coveredParts.get(coveredParts.size() - 1).getSecond();
    }
}
