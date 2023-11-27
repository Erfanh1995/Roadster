package mapconstruction.algorithms.representative.containers;

import mapconstruction.trajectories.Subtrajectory;

import java.awt.geom.Point2D;
import java.io.Serializable;

/**
 * Class containing info about an intersection of a subtrajectory with an orthogonal on the representativeSubtrajectory.
 * <p>
 * The intersection is the actual point where the orthogonal on the representativeSubtrajectory intersects a specific part of the
 * edge. Here we keep track on what angle the edge makes, the subtrajectory, the actual intersection point and the edge
 * index on the subtrajectory.
 *
 * @author Jorrick Sleijster
 */
public class OrthogonalIntersection implements Serializable {
    private Point2D point;
    private double angle;
    private Subtrajectory subtrajectory;
    private int edgeIndex;
    private double calculatedForce;

    /**
     * Initializes the class. This class stores information about found intersections.
     * An intersection is found between the orthogonal of AC going through B and the edge of a subtrajectory.
     *
     * @param point,         the actual point of intersection on the subtrajectory
     * @param angle,         the angle the edge(of the intersection) of the subtrajectory makes with the line AC.
     * @param subtrajectory, the subtrajectory which intersected with the orthogonal of AC.
     * @param edgeIndex,     the index of the edge of the subtrajectory.
     */
    public OrthogonalIntersection(Point2D point, double angle, Subtrajectory subtrajectory, int edgeIndex) {
        this.point = point;
        this.angle = angle;
        this.subtrajectory = subtrajectory;
        this.edgeIndex = edgeIndex;
        calculatedForce = -1.0;
    }

    public Point2D getPoint() {
        return point;
    }

    public double getAngle() {
        return angle;
    }

    public Subtrajectory getSubtrajectory() {
        return subtrajectory;
    }

    public int getEdgeIndex() {
        return edgeIndex;
    }

    public double getCalculatedForce() {
        return calculatedForce;
    }

    public void setCalculatedForce(double calculatedForce) {
        this.calculatedForce = calculatedForce;
    }
}
