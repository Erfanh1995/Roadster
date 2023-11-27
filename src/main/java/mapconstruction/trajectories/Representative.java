package mapconstruction.trajectories;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class Representative extends Trajectory implements Serializable {

    /**
     * List of all the points
     */
    private List<Point2D> points;

    /**
     * The bundle
     */
    private Bundle parentBundle;

    /**
     * Whether it is a reverse of the bundle.
     */
    private Representative originalRep;

    Representative(List<Point2D> points, Bundle parentBundle) {
        this.points = points;
        this.parentBundle = parentBundle;
        this.originalRep = null;
    }

    Representative(List<Point2D> points, Bundle parentBundle, Representative originalRep) {
        this.points = points;
        this.parentBundle = parentBundle;
        this.originalRep = originalRep;
    }


    /**
     * Return all points on the representativeSubtrajectory
     *
     * @return all points
     */
    @JsonProperty
    public List<Point2D> getPoints() {
        return this.points;
    }

    /**
     * Get an edge from the representativeSubtrajectory at a specific index.
     *
     * @param index, the index of the edge
     * @return the edge at the given index
     */
    public Line2D getEdge(int index) {
        if (index > points.size() - 2 || index < 0) {
            System.out.println("Error ForceRepresentative.getEdge. Index requested " + index +
                    " but size: " + this.points.size());
            return null;
        }
        return new Line2D.Double(getPoint(index), getPoint(index + 1));
    }

    @Override
    public Representative reverse() {
        if (isReverse()){
            return originalRep;
        }

        List<Point2D> reverse = new ArrayList<>(this.points);
        Collections.reverse(reverse);
        return new Representative(reverse, getParentBundle(), this);
    }

    @Override
    public boolean isReverse() {
        return (originalRep != null);
    }

    @Override
    public String getLabel() {
        return "";
    }

    @Override
    public int numPoints() {
        return this.points.size();
    }

    /**
     * Get a point from the representative at a specific index.
     *
     * @param index, the index of the edge
     * @return the point at the given index
     */
    public Point2D getPoint(int index) {
        if (index > points.size() - 1 || index < 0) {
            System.out.println("Error ForceRepresentative.getEdge. Index requested " + index +
                    " but size: " + this.points.size());
            return null;
        }
        return this.points.get(index);
    }

    /**
     * Get a point possible on the edge from the representative.
     *
     * @param index, the index of the edge
     * @return the point possibly half way on an edge.
     */
    public Point2D getPoint(double index) {
        return GeometryUtil.getTrajectoryDecimalPoint(this, index);
    }

    /**
     * Get the bundle this representative is part of
     */
    @JsonIgnore
    public Bundle getParentBundle() {
        return parentBundle;
    }

    /**
     * Get the bundleClass for this representative
     */
    @JsonProperty
    public int getParentBundleClass(){
        return STORAGE.getClassFromBundle(getParentBundle());
    }
}
