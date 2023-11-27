package mapconstruction.algorithms.straightener;

import mapconstruction.trajectories.SimplifiedTrajectory;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Straightener based on heading. We try to remove points that are incorrect and create z-artifacts.
 *
 * @author Jorrick
 */
public class TrajectoryStraightener {

    private final double maxDistance;

    public TrajectoryStraightener(double maxDistance) {
        this.maxDistance = maxDistance;
    }


    /**
     * General function which calls the two algorithms
     * @param t, the trajectory we want to remove the z-artifacts from.
     * @return a SimplifiedTrajectory.
     */
    public Trajectory straighten(Trajectory t) {
        Trajectory originalTrajectory = null;
        if (t instanceof SimplifiedTrajectory){
            SimplifiedTrajectory simply = (SimplifiedTrajectory) t;
            originalTrajectory = simply.getOriginal();
        } else {
            originalTrajectory = t;
        }

        List<Point2D> newTrajectory = removeOverlappingEdges(t);
        newTrajectory = removeZArtifacts(newTrajectory);

        ArrayList<Integer> indices = new ArrayList<>(originalTrajectory.numPoints());
        for (int i = 0; i < originalTrajectory.numPoints(); i++){
            if (newTrajectory.contains(originalTrajectory.getPoint(i))){
                indices.add(i);
            }
        }

        return new SimplifiedTrajectory(originalTrajectory, indices, 0.0);
    }

    /**
     * Removing behaviour of walking up and down all the time.
     * This function checks for every edge(a), all edges that come after it until we reach an edge that has a point that
     * is 50 meters away or further. We check whether for edge a, if any of the other intersect with a, and if so, if it
     * has a heading direction difference with a of less than 90 degrees. In such a case, we store this and we try to
     * find the furthest edge for which this is true for out point a. Then we simply connect the start of a with the end
     * of the intersecting edge.
     * @param t the trajectory we will apply this algorithm on
     * @return the pointList after we applied the algorithm.
     */
    public List<Point2D> removeOverlappingEdges(Trajectory t){
        List<Point2D> newList = new ArrayList<>();

        int i = 0;
        while (i < t.numPoints() - 2){
            newList.add(t.getPoint(i));
            Line2D line1 = new Line2D.Double(t.getPoint(i), t.getPoint(i + 1));
            line1 = GeometryUtil.cropLineByPercentage(line1, 0.01);

            int bestJ = i + 1;
            int j = i + 1;
            while (true) {
                if (t.getPoint(i).distance(t.getPoint(j)) > maxDistance &&
                        t.getPoint(i + 1).distance(t.getPoint(j)) > maxDistance ) {
                    break;
                }

                Line2D line2 = new Line2D.Double(t.getPoint(j), t.getPoint(j + 1));
                if (line1.intersectsLine(line2)) {
                    double hd1 = GeometryUtil.getHeadingDirection(line1);
                    double hd2 = GeometryUtil.getHeadingDirection(line2);
                    if (GeometryUtil.getAbsoluteAngleDifference(hd1, hd2) <= 90) {
                        bestJ = j;
                    }
                }

                j++;
                if (j >= t.numPoints() - 1){
                    break;
                }
            }
            i = bestJ;
        }
        while (i < t.numPoints()){
            newList.add(t.getPoint(i));
            i++;
        }
        return newList;
    }

    /**
     * Removing the Z artifacts.
     * Here we check whether we have a Z artifact within 50 meters of the starting point. A z-artifact is considered to
     * be more than 240 degrees in turns within 50 meters continuous length (where we only calculate turns of 91 degrees
     * or higher). In this case we remove the points in between the start and end point
     *
     * @param t the pointList we got from the previous algorithm
     * @return the modified pointList
     */
    private List<Point2D> removeZArtifacts(List<Point2D> t) {

        List<Point2D> newList = new ArrayList<>();

        int bestIPrevious = -1;
        int bestJPrevious = -1;
        int i = 0;
        for (i = 0; i < t.size() - 2; i++) {
            if (bestJPrevious >= i){
                newList.add(t.get(bestIPrevious));
                i = bestJPrevious + 1;

                bestIPrevious = -1;
                bestJPrevious = -1;
            }
            Line2D line1 = new Line2D.Double(t.get(i), t.get(i + 1));
            double hd1 = GeometryUtil.getHeadingDirection(line1);
            double lastExtremeAngle = hd1;
            double angleDifference = 0.0;
            List<Point2D> smallList = new ArrayList<>(Arrays.asList(t.get(i + 1)));

//            int bestJ = i;
            int j = i + 1;
            int bestJ = -1;
            while (true) {
                if (j >= t.size() - 2){
                    break;
                }
                smallList.add(t.get(j + 1));
                if (GeometryUtil.getContinuousLength(smallList) > maxDistance) {
                    break;
                }

                Line2D tempLine1 = new Line2D.Double(t.get(j - 1), t.get(j));
                Line2D tempLine2 = new Line2D.Double(t.get(j), t.get(j + 1));
                double hdTemp1 = GeometryUtil.getHeadingDirection(tempLine1);
                double hdTemp2 = GeometryUtil.getHeadingDirection(tempLine2);
                if (GeometryUtil.getAbsoluteAngleDifference(hdTemp1, hdTemp2) > 91 &&
                        GeometryUtil.getAbsoluteAngleDifference(hdTemp2, lastExtremeAngle) > 91){
                    lastExtremeAngle = hdTemp2;
                    angleDifference = angleDifference + GeometryUtil.getAbsoluteAngleDifference(hdTemp1, hdTemp2);
                    if (angleDifference >= 240) {
                        if (GeometryUtil.getAbsoluteAngleDifference(hd1, hdTemp2) < 90){
                            bestJ = j;
                            break;
                        }
                    }
                }
                j++;
            }

            if (bestJ != -1 && bestJ <= bestJPrevious){
                for (int k = bestIPrevious; k < i; i++){
                    newList.add(t.get(k));
                }

                bestJPrevious = bestJ;
                bestIPrevious = i;
            } else if (bestJPrevious == -1 && bestJ > -1){
                bestJPrevious = bestJ;
                bestIPrevious = i;
            } if (bestJ == - 1 && bestJ > bestJPrevious){
                newList.add(t.get(bestIPrevious));
                i = bestJPrevious + 1;
                bestIPrevious = -1;
                bestJPrevious = -1;
            } else if (bestJ == -1 && bestJPrevious == -1){
                newList.add(t.get(i));
            }
        }
        if (bestJPrevious != -1){
            newList.add(t.get(bestIPrevious));
//            newList.add(t.get(bestJPrevious));
            for (int j = bestJPrevious; j < t.size(); j++){
                newList.add(t.get(j));
            }
        } else {
            for (int j = i; j < t.size(); j++){
                newList.add(t.get(j));
            }
        }

        int counter0 = 0;
        for (int k = 0; k < t.size() - 1; k++){
            if (t.get(k).equals(t.get(k+1))){
                counter0++;
            }
        }
        int counter1 = 0;
        for (int k = 0; k < newList.size() - 1; k++){
            if (newList.get(k).equals(newList.get(k+1))){
                counter1++;
            }
        }
        if (counter0 == 0 && counter1 >= 1){
            System.out.println("INCORRECT StraightenFunction");
        }

        return newList;
    }
}
