package mapconstruction.trajectories;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Class representing a trajectory that was simplified. Is a view on the
 * original trajectory.
 * <p>
 * <p>
 * Two simplified trajectories are equal if the originals are equal, and the
 * error value is equal.
 *
 * @author Roel
 */
public class SimplifiedTrajectory extends Trajectory implements Serializable {
    private static final long serialVersionUID = 1L;

    /**
     * Original trajectory.
     */
    private Trajectory original;

    /**
     * List of indices pointing to points in the original trajectory, in proper
     * order.
     */
    private List<Integer> indices;

    /**
     * Error value with which this trajectory was created.
     */
    private double error;

    /**
     * Creates a simplified trajectory.
     *
     * @param original Original trajectory that was simplified.
     * @param indices  Indices of points of the original trajectory that are still present in the simplification.
     * @param error    Error used when simplifying.
     * @throws NullPointerException     if {@code original == null || indices == null}
     * @throws IllegalArgumentException if {@code error < 0}
     */
    public SimplifiedTrajectory(Trajectory original, List<Integer> indices, double error) {
        Preconditions.checkNotNull(original, "original == null");
        Preconditions.checkNotNull(indices, "indices == null");
        Preconditions.checkArgument(error >= 0, "Negative error: %s", error);
        this.original = original;
        this.indices = indices;
        this.error = error;

//        For debugging reasons this was added
//        if (indices.size() == 0){
//            throw new IllegalArgumentException("SimplifiedTrajectory has a size of 0 points.");
//        }
    }

    @Override
    @JsonProperty
    public int numPoints() {
        return indices.size();
    }

    @JsonProperty
    public List<Integer> getIndices() {
        return indices;
    }

    @JsonProperty
    public List<Point2D> getPoints() {
        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numPoints(); i++) {
            points.add(original.getPoint(indices.get(i)));
        }
        return points;
    }


    @Override
    public SimplifiedTrajectory reverse() {
        Trajectory origRev = original.reverse();
        final int totPoints = original.numPoints();
        List<Integer> newInd = Lists.transform(indices, idx -> totPoints - idx - 1);
        newInd = Lists.reverse(newInd);

        return new SimplifiedTrajectory(origRev, newInd, error);
    }

    private void writeObject(java.io.ObjectOutputStream out) throws IOException {
        out.writeObject(original);
        out.writeObject(new ArrayList<>(indices));
        out.writeDouble(error);
    }

    //
    private void readObject(java.io.ObjectInputStream in) throws IOException, ClassNotFoundException {
        original = (Trajectory) in.readObject();
        indices = (List<Integer>) in.readObject();
        error = in.readDouble();
    }

    @Override
    public boolean isReverse() {
        return original.isReverse();
    }

    @Override
    public Point2D getPoint(int pos) {
        Preconditions.checkPositionIndex(pos, numPoints(), "pos");
        return original.getPoint(indices.get(pos));
    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 71 * hash + Objects.hashCode(this.original);
        hash = 71 * hash + (int) (Double.doubleToLongBits(this.error) ^ (Double.doubleToLongBits(this.error) >>> 32));
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
        final SimplifiedTrajectory other = (SimplifiedTrajectory) obj;
        if (Double.doubleToLongBits(this.error) != Double.doubleToLongBits(other.error)) {
            return false;
        }
        if (!Objects.equals(this.original, other.original)) {
            return false;
        }
        return true;
    }

    /**
     * Returns the original trajectory.
     *
     * @return
     */
    @JsonProperty
    public Trajectory getOriginal() {
        return original;
    }

    /**
     * Translates the given index to an index in the original trajectory, such
     * that the point at the given index in the simplified trajectory
     * corresponds to the same point at the found index in the original
     * trajectory.
     *
     * @param idx index in the simplified trajectory
     * @return Corresponding index in the original trajectory.
     * @throws IndexOutOfBoundsException if {@code idx < 0 || idx >= numPoints}
     */
    public int getOriginalIndex(int idx) {
        Preconditions.checkPositionIndex(idx, numPoints(), "idx");
        return indices.get(idx);
    }

    @Override
    @JsonProperty
    public String getLabel() {
        return original.getLabel() + "_(s)";
    }

    public double getError() {
        return error;
    }


}
