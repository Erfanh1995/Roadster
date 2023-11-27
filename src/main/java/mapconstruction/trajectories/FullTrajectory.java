package mapconstruction.trajectories;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.google.common.math.DoubleMath;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Class representing full trajectories, as given in the input.
 * <p>
 * Note on equality:
 * Each created trajectory will get a unique identifier.
 * This identifier is a positive long integer.
 * Two trajectories are considered equal if they have the same identifier.
 * <p>
 * Trajectories can be reversed. In that case the reversed trajectory will get
 * the negative of the identifier. In this way reversing twice yields the same trajectory.
 *
 * @author Roel
 */
public class FullTrajectory extends Trajectory implements Serializable {
    private static final long serialVersionUID = 1L;
    private static long nextid = 1;
    /**
     * Identifier
     */
    private long id;
    /**
     * Points in this trajectory
     */
    private List<Point2D> points;

    /**
     * number of points in the trajectoy.
     */
    private int numPoints;

    /**
     * Label of the trajectory
     */
    private String label;

    /**
     * Creates a FullTrajectory with the given list of points and ID.
     * The points are NOT copied, the list is used as-is.
     *
     * @param points List of points in proper order representing the trajectory.
     * @param id     Unique id for the trajectory.
     * @throws NullPointerException if {@code points == null}
     */
    private FullTrajectory(List<Point2D> points, long id) {
        Preconditions.checkNotNull(points, "points == null");
        this.points = points;
        this.id = id;
        this.numPoints = points.size();
    }

    /**
     * Creates a fullTrajectory with the given list of points.
     * The points are NOT copied, the list is used as-is.
     *
     * @param points List of points in proper order representing the trajectory.
     * @throws NullPointerException if {@code points == null}
     */
    public FullTrajectory(List<Point2D> points) {
        this(points, nextid++);
    }


    @Override
    public int hashCode() {
        int hash = 3;
        hash = 89 * hash + (int) (this.id ^ (this.id >>> 32));
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final FullTrajectory other = (FullTrajectory) obj;
        if (this.id != other.id) {
            return false;
        }
        return true;
    }

    @JsonProperty
    public long getId() {
        return id;
    }

    @JsonProperty
    public List<Point2D> getPoints() {
        return points;
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public FullTrajectory reverse() {
        FullTrajectory rev = new FullTrajectory(Lists.reverse(points), -id);
        if (label != null) {
            rev.setLabel(label);
        }
        return rev;
    }

    private void writeObject(java.io.ObjectOutputStream out) throws IOException {
        out.writeLong(id);
        out.writeObject(new ArrayList<>(points));
        out.writeInt(numPoints);
        out.writeObject(label);
    }

    private void readObject(java.io.ObjectInputStream in) throws IOException, ClassNotFoundException {
        id = in.readLong();
        Object object = in.readObject();
        if (object != null) {
            points = (List<Point2D>) object;
        } else {
            points = new ArrayList<>();
        }
        numPoints = in.readInt();
        label = (String) in.readObject();
        nextid = Math.max(Math.abs(id) + 1, nextid);
    }


    @Override
    @JsonProperty
    public boolean isReverse() {
        return id < 0;
    }

    @Override
    @JsonProperty
    public int numPoints() {
        return numPoints;
    }


    @Override
    public Point2D getPoint(int pos) {
        Preconditions.checkPositionIndex(pos, numPoints(), "pos");
        if (pos >= numPoints()){
            System.out.println("Error! FullTrajectory.GetPoint out of range");
        }
        return points.get(pos);
    }

    /**
     * Returns the label of this trajectory.
     * <p>
     * If no label was set, the label is the identifier.
     *
     * @return
     */
    @Override
    @JsonProperty
    public String getLabel() {
        return "<" + (label == null ? Long.toString(id) : label) + ">" + (isReverse() ? "_(r)" : "");
    }

    /**
     * Sets the label of this trajectory;
     *
     * @param label
     */
    public void setLabel(String label) {
        this.label = label;
    }


}
