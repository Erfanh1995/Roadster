package mapconstruction.trajectories;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.google.common.collect.Range;
import com.google.common.math.DoubleMath;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Collection;
import java.util.Objects;

/**
 * Class representing subtrajectories of complete trajectories. Subtrajectories
 * have a reference to its 'parent' which is trajectory they are part of. Note
 * that this may again be a subtrajectory of some other trajectory.
 * <p>
 * We use a {@code from} and {@code to} index to indicate the start and end
 * position of the subtrajectory with repsect to its parent If these indices are
 * integers, then the start and endpoints of the subtrajectory correspond
 * exactly to concrete datapoints of the parent.
 * <p>
 * However, the indices may be real. In that case the fractional part indicates
 * the offset of the point along the edge indicated by the integer part. This is
 * done by mapping taking the parametric representation of the edge, and the
 * fractional part then indicates the parameter along the edge.
 * <p>
 * Examples (Let T be the parent trajectory)
 * <ul>
 * <li> {@code T[0, 4]} is the subtrajectory running from the 0th to the 4th
 * point of T.
 * <li> {@code T[0.5, 4]} is the subtrajectory starting at the point halfway the
 * edge between the 0th-1th point and ends at the 4th point of the parent.
 * <li> {@code T[1, 4.5]} is the subtrajectory starting at the 1st point and and
 * ends at a point halfway the edge between the 4th and the 5th point.
 * <li> {@code T[1.25, 2.75]} starts on the edge between the 1st and 2nd point,
 * at 0.25 the length of the edge away from the 1st point. and ends on the edge
 * between the 2nd and 3rd point, 0.75 the length of the edge away from the
 * second point.
 * </ul>
 * <p>
 * We make sure we do not introduce duplicate endpoints. For example T[0,4] will
 * have 5 points, whereas T[0,4.5] will have 6, and T[0.5,4.5] will have 7.
 * <p>
 * Subtrajectories are equal if they have the same parent and same indices.
 */
public class Subtrajectory extends Trajectory implements Serializable {

    private static final long serialVersionUID = 1L;

    /**
     * Parent of this trajectory.
     */
    private final Trajectory parent;
    /**
     * to index of this sub trajectory with respect to the parent. (inclusive)
     */
    private final double toIndex;
    /**
     * From index of this sub trajectory with respect to the parent. (inclusive)
     */
    private double fromIndex;

    /**
     * Creates a subtrajectory from the given trajectory and the given bounds.
     * <p>
     * We use a {@code from} and {@code to} index to indicate the start and end
     * position of the subtrajectory with respect to its parent If these indices
     * are integers, then the start and endpoints of the subtrajectory
     * correspond exactly to concrete datapoints of the parent.
     * <p>
     * However, the indices may be real. In that case the fractional part
     * indicates the offset of the point along the edge indicated by the integer
     * part. This is done by mapping taking the parametric representation of the
     * edge, and the fractional part then indicates the parameter along the
     * edge.
     * <p>
     * If {@code to < from} then we have an empty subtrajectory Howver, we must
     * have {@code to - from >= -1 && to < parent.numPoiints}
     *
     * @param parent trajectory to take the subtrajectory from.
     * @param from   start index (inclusive)
     * @param to     end index (inclusive)
     * @throws IllegalArgumentException {@code if (from - to >= -1 && to < parent.numPoints)}
     * @throws NullPointerException     {@code if parent == null}
     */
    public Subtrajectory(Trajectory parent, double from, double to) {
        super();
        if (parent == null) {
            throw new NullPointerException(this.getClass() + " Constructor: parent == null");
        }

//      Could allow for better understanding what is going on
//        to - from < 0
        if (to - from < -1 || to >= parent.numPoints()) {
            throw new IllegalArgumentException(this.getClass() + " Constructor: "
                    + "Arguments out of bounds. from: " + from + " to: " + to + " on: " + parent.getLabel() + "(" + parent.numPoints() + ")");
        }
        this.parent = parent;
        this.fromIndex = from;
        this.toIndex = to;
    }

    public Subtrajectory(Trajectory parent) {
        this.parent = parent;
        this.fromIndex = 0;
        this.toIndex = parent.numPoints() - 1;
    }

    /**
     * @return the direct parent of this trajectory.
     */
    @JsonProperty
    public Trajectory getParent() {
        return parent;
    }

    // drops the first vertex in the subsegment
    public void dropInPlace(int i) {
        assert (fromIndex + i < toIndex);
        fromIndex += i;
    }

    /**
     * Returns the from index of this sub trajectory with respect to the parent
     * (inclusive).
     *
     * @return fromIndex
     */
    @JsonProperty
    public double getFromIndex() {
        return fromIndex;
    }

    /**
     * Returns the to index of this sub trajectory with respect to the parent
     * (inclusive).
     *
     * @return toIndex
     */
    @JsonProperty
    public double getToIndex() {
        return toIndex;
    }

    /**
     * Computes whether this trajectory has the given trajectory a lambda
     * subtrajectory.
     * <p>
     * Quicker computation than for normal trajectories
     *
     * @param other  Trajectory to check
     * @param lambda Error margin.
     * @return {@code true} if this trajectory has the given trajectory a lambda
     * subtrajectory. {@code false} otherwise.
     * @throws NullPointerException if {@code other == null}
     */
    public boolean hasAsLambdaSubtrajectory(Subtrajectory other, double lambda) {
        Preconditions.checkNotNull(other, "other == null");
        //return super.hasAsLambdaSubtrajectory(other, lambda);
        if (!parent.equals(other.getParent())) {
            return false;
        }

        // compute points at which the 'prefix' ends and the 'suffix' starts
        // the 'prefix' start and 'suffix' endpoints are simply the bounds on other
        double overlapStart = Math.min(this.getFromIndex(), other.getToIndex()) - other.getFromIndex();
        double overlapEnd = Math.max(this.getToIndex(), other.getFromIndex()) - other.getFromIndex();

        // compute whether these exceed lambda
        double l = 0;

        if (overlapStart > 0d && overlapEnd < other.getToIndex() - other.getFromIndex()) return false;

        if (overlapStart > 0d) {
            Point2D start = other.getPointAt(overlapStart), end;
            for (int i = (int) overlapStart; i >= 0; i--) {
                end = other.getPoint(i);
                l += start.distance(end);
                start = end;
                // return false if l is larger than lambda
                if (DoubleMath.fuzzyCompare(l, lambda, 1E-6) > 0) return false;
            }
        }

        if (overlapEnd < other.getToIndex() - other.getFromIndex()) {
            Point2D start = other.getPointAt(overlapEnd), end;
            for (int i = (int) overlapEnd + 1; i < other.numPoints(); i++) {
                end = other.getPoint(i);
                l += start.distance(end);
                start = end;
                // return false if l is larger than lambda
                if (DoubleMath.fuzzyCompare(l, lambda, 1E-6) > 0) return false;
            }
        }

        return true; //DoubleMath.fuzzyCompare(l, lambda, 1E-6) <= 0;
    }

    public boolean hasAsLambdaSimilar(Subtrajectory other, double lambda) {
        Point2D f = other.getFirstPoint();
        Point2D l = other.getLastPoint();

//        return this.parent.getEdgesAround(f, lambda).stream().anyMatch(i -> fromIndex <= i && i <= toIndex) &&
//               this.parent.getEdgesAround(l, lambda).stream().anyMatch(i -> fromIndex <= i && i <= toIndex);

        boolean first = false, last = false;
        for (Line2D e : this.edges()) {
            first = first || DoubleMath.fuzzyCompare(e.ptLineDist(f), lambda, 1E-6) <= 0;
            last = last || DoubleMath.fuzzyCompare(e.ptLineDist(l), lambda, 1E-6) <= 0;
            if (first && last) return true;
        }
        return false;
    }

    public boolean hasAsLambdaSibling(Subtrajectory other, double lambda) {
        return other.points().stream().allMatch(p -> this.isLambdaClose(p, lambda));
    }

    private boolean isLambdaClose(Point2D point, double lambda) {
        for (Line2D e : this.edges()) {
            if (DoubleMath.fuzzyCompare(e.ptLineDist(point), lambda, 1E-6) <= 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Computes whether this trajectory has the given trajectory a
     * subtrajectory.
     * <p>
     * Quicker computation than for normal trajectories, as we can use the
     * following logic:
     * <p>
     * {@code other} is a subtrajectory of {@code this} iff
     * <p>
     * {@code this.root == other.root && this.rootFrom <= other.rootFrom  && other.rootTo <= this.rootTo}
     *
     * @param other
     * @return
     * @throws NullPointerException if {@code other == null}
     */
    public boolean hasAsSubtrajectory(Subtrajectory other) {
        Preconditions.checkNotNull(other, "other == null");
        return parent.equals(other.getParent())
                && this.getFromIndex() <= other.getFromIndex()
                && other.getToIndex() <= this.getToIndex();
    }

    /**
     * Computes whether this Subtrajectory overlaps the other Subtrajectory
     * <p>
     * That is, the parents are equal, and the intervals overlap.
     *
     * @param other
     * @return
     * @throws NullPointerException if {@code other == null}
     */
    public boolean overlaps(Subtrajectory other) {
        Preconditions.checkNotNull(other, "other == null");
        return computeOverlap(other) != null;
    }

    /**
     * Computes whether this Subtrajectory overlaps any other Subtrajectory
     *
     * @param subtrajectories set
     * @return true when there is an overlap with one of the subtrajectories
     */
    public boolean overlapsOneItemInList(Collection<Subtrajectory> subtrajectories) {
        for (Subtrajectory sub : subtrajectories) {
            if (overlaps(sub)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Computes the overlap this Subtrajectory has with the other Subtrajectory
     * <p>
     * The parents must be equal.
     *
     * @param other, the subtrajectory we are comparing with.
     * @return null if there is no overlap
     * @throws NullPointerException if {@code other == null}
     */
    public Range<Double> computeOverlap(Subtrajectory other) {
        Preconditions.checkNotNull(other, "other == null");
        if (!parent.equals(other.getParent())) {

            // Added functionality where is automatically checks for reverse parent and if so, returns whether they
            // contain overlap
            if (parent.isReverse() != other.isReverse()) {
                if (parent.isReverse()) {
                    return computeOverlap(other.reverse());
                } else {
                    return reverse().computeOverlap(other.reverse());
                }
            }
            return null;
        }
        Range<Double> thisRange = Range.closed(fromIndex, toIndex);
        Range<Double> otherRange = Range.closed(other.fromIndex, other.toIndex);

        if (!thisRange.isConnected(otherRange)) {
            // no overlap
            return null;
        }
        return thisRange.intersection(otherRange);
    }

    /**
     * Computes whether this Subtrajectory completely overlaps/overshadows the other Subtrajectory
     * <p>
     * That is, the parents are equal, and the intervals is equal to the other Subtrajectory endings
     *
     * @param other, the other subtrajectory
     * @return whether it completely overshadows
     * @throws NullPointerException if {@code other == null}
     */
    public boolean completelyOvershadows(Subtrajectory other) {
        Preconditions.checkNotNull(other, "other == null");
        Range<Double> overlap = computeOverlap(other);
        if (overlap == null){
            return false;
        }
        return DoubleMath.fuzzyEquals(overlap.lowerEndpoint(), other.fromIndex, 1E-5) &&
                DoubleMath.fuzzyEquals(overlap.upperEndpoint(), other.toIndex, 1E-5);
    }

    /**
     * Computes whether this Subtrajectory completely overlaps/overshadows any other Subtrajectory
     *
     * So there shouldn't be any part of the subtrajectory that is uncovered
     *
     * @param subtrajectories set of other subtrajectories.
     * @return true when there is an overlap with one of the subtrajectories
     */
    public boolean completelyOvershadowsOneItemInList(Collection<Subtrajectory> subtrajectories) {
        for (Subtrajectory sub : subtrajectories) {
            if (completelyOvershadows(sub)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 97 * hash + Objects.hashCode(this.parent);
        hash = 97 * hash + (int) (Double.doubleToLongBits(this.fromIndex) ^ (Double.doubleToLongBits(this.fromIndex) >>> 32));
        hash = 97 * hash + (int) (Double.doubleToLongBits(this.toIndex) ^ (Double.doubleToLongBits(this.toIndex) >>> 32));
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
        final Subtrajectory other = (Subtrajectory) obj;
            if (Double.doubleToLongBits(this.fromIndex) != Double.doubleToLongBits(other.fromIndex)) {
                return false;
        }
        if (Double.doubleToLongBits(this.toIndex) != Double.doubleToLongBits(other.toIndex)) {
            return false;
        }
        if (!Objects.equals(this.parent, other.parent)) {
            return false;
        }
        return true;
    }

    @Override
    public Subtrajectory reverse() {
        Trajectory reverseParent = parent.reverse();
        double from = parent.numPoints() - this.toIndex - 1;
        double to = parent.numPoints() - this.fromIndex - 1;
        return new Subtrajectory(reverseParent, from, to);
    }

    @Override
    public boolean isReverse() {
        return parent.isReverse();
    }

    @Override
    public int numPoints() {
        // for an integer x, floor(x) = x = ceil(x)
        return fromIndex > toIndex ? 0 : (int) Math.ceil(toIndex) - (int) Math.floor(fromIndex) + 1;
    }

    @Override
    public Point2D getPoint(int pos) {
        if (pos < 0 || pos >= numPoints()) {
            throw new IndexOutOfBoundsException(String.format("Position out of bounds at %d while range is [0,%d)", pos, numPoints()));
        }
        if (pos == 0) {
            return getFirstPoint();
        } else if (pos == numPoints() - 1) {
            return getLastPoint();
        } else {
            return parent.getPoint(DoubleMath.roundToInt(fromIndex, RoundingMode.FLOOR) + pos);
        }
    }

    private Point2D getPointAt(double pos) {
        if (pos < 0 || pos > numEdges()) {
            throw new IndexOutOfBoundsException(String.format("Position out of bounds at %.2f while range is [0,%d]", pos, numEdges()));
        }

        if (DoubleMath.isMathematicalInteger(pos)) {
            return getPoint((int) pos);
        } else {
            return GeometryUtil.getPointOnLine(getEdge((int) pos), pos % 1);
        }
    }

    @Override
    public synchronized Line2D getEdge(int pos) {
        Preconditions.checkPositionIndex(pos, numEdges(), "pos");
        if (pos == 0 || pos == this.numEdges() - 1) {
            int i = pos == 0 ? 0 : 1;
            if (edges == null) {
                edges = new Line2D[2];
            }

            if (edges[i] == null) {
                edges[i] = new Line2D.Double(getPoint(pos), getPoint(pos+1));
            }

            if (edges.length > 2 && i > 0) {
                // Catch 'old' savedstates
                return edges[this.numEdges() - 1];
            }

            return edges[i];
        } else {
            // TODO: when loading in a saved state, using a parent.getEdge reference produces different results
            return parent.getEdge((int) fromIndex + pos);
//          return new Line2D.Double(getPoint(pos), getPoint(pos + 1));
        }
    }

    /**
     * Gets the first point of this trajectory.
     *
     * @return
     */
    @JsonProperty
    public Point2D getFirstPoint() {
        if (DoubleMath.isMathematicalInteger(fromIndex)) {
            return parent.getPoint((int) fromIndex);
        } else {
            return GeometryUtil.getPointOnLine(parent.getEdge((int) fromIndex), fromIndex % 1);
        }
    }

    /**
     * Gets the last point of this trajectory.
     *
     * @return
     */
    @JsonProperty
    public Point2D getLastPoint() {
        if (DoubleMath.isMathematicalInteger(toIndex)) {
            return parent.getPoint((int) toIndex);
        } else {
            return GeometryUtil.getPointOnLine(parent.getEdge((int) toIndex), toIndex % 1);
        }
    }

    /**
     * Returns the label of the trajectory.
     * <p>
     * Is the label of the parent, appended with [from, to]
     *
     * @return
     */
    @Override
    @JsonProperty
    public String getLabel() {
        DecimalFormat format = new DecimalFormat("0.###");
        return String.format("%s[%s, %s]", parent.getLabel(), format.format(fromIndex), format.format(toIndex));
    }

    /**
     * Creates a new subtrajectory for which a part of the given length is
     * removed from the start of the trajectory.
     * <p>
     * The subtrajectory will be a subtrajectory of the parent, not of this
     * trajectory.
     * <p>
     * length can be at most the length of the subtrajectory.
     *
     * @param length length to remove from the start of the trajectory.
     * @return Subtrajectory for which a part of the given length is removed
     * from the start.
     * @throws IllegalArgumentException if {@code length > this.euclideanLength}
     */
    public Subtrajectory trimStart(double length) {
        Preconditions.checkArgument(length <= this.euclideanLength(), "Length to remove (%s) cannot be more than the length of the subtrajectory (%s.)", length, this.euclideanLength());

        // We have to determine a new start index.
        double newFrom = this.fromIndex;

        // accumulated length
        double accumulatedLength = 0;
        // iterate over all edges from start to end.
        for (Line2D edge : edges()) {
            double edgeLength = GeometryUtil.lineLength(edge);
            if (accumulatedLength + edgeLength > length) {
                // desired length exceeded. Need to determine end position
                // somewhere on the current edge.
                double parentEdgeLength = GeometryUtil.lineLength(parent.getEdge((int) newFrom));
                double offset = ((length - accumulatedLength) / parentEdgeLength);

                newFrom += offset;
                break;
            } else {
                // not yet enough, increase accumulated length
                accumulatedLength += edgeLength;

                if (DoubleMath.isMathematicalInteger(newFrom)) {
                    // from is an integer index, hence increase by 1
                    newFrom += 1;
                } else {
                    // round upwards
                    newFrom = DoubleMath.roundToInt(newFrom, RoundingMode.CEILING);
                }
            }
        }

        return new Subtrajectory(parent, newFrom, toIndex);
    }

    /**
     * Creates a new subtrajectory for which a part of the given length is
     * removed from the end of the trajectory.
     * <p>
     * The subtrajectory will be a subtrajectory of the parent, not of this
     * subtrajectory.
     * <p>
     * length can be at most the length of the subtrajectory.
     *
     * @param length length to remove from the end of the trajectory.
     * @return Subtrajectory for which a part of the given length is removed
     * from the end.
     * @throws IllegalArgumentException if {@code length > this.euclideanLength}
     */
    public Subtrajectory trimEnd(double length) {
        Preconditions.checkArgument(length <= this.euclideanLength(),
                "Length to remove (%s) cannot be more than the length of the subtrajectory (%s).", length, this.euclideanLength());

        // We have to determine a new start index.
        double newTo = this.toIndex;

        // accumulated length
        double accumulatedLength = 0;

        // iterate over all edges from start to end.
        for (Line2D edge : Lists.reverse(edges())) {
            double edgeLength = GeometryUtil.lineLength(edge);

            if (accumulatedLength + edgeLength > length) {
                // desired length exceeded. Need to determine end position
                // somewhere on the current edge.
                double parentEdgeLength = GeometryUtil.lineLength(parent.getEdge(DoubleMath.roundToInt(newTo, RoundingMode.CEILING) - 1));
                double offset = ((length - accumulatedLength) / parentEdgeLength);

                newTo -= offset;
                break;
            } else {
                // not yet enough, increase accumulated length
                accumulatedLength += edgeLength;

                if (DoubleMath.isMathematicalInteger(newTo)) {
                    // from is an integer index, hence decrease by 1
                    newTo -= 1;
                } else {
                    // round down
                    newTo = DoubleMath.roundToInt(newTo, RoundingMode.FLOOR);
                }
            }
        }

        return new Subtrajectory(parent, fromIndex, newTo);
    }

    /**
     * Creates a new subtrajectory for which a part of the given length is
     * removed from both the start and the end of the trajectory.
     * <p>
     * The subtrajectory will be a subtrajectory of the parent, not of this
     * subtrajectory.
     * <p>
     * length can be at most half the length of the subtrajectory.
     *
     * @param length length to remove from both ends of the trajectory.
     * @return Subtrajectory for which a part of the given length is removed
     * from both ends.
     * @throws IllegalArgumentException if
     *                                  {@code 2 * length > this.euclideanLength}
     */
    public Subtrajectory trim(double length) {
        Preconditions.checkArgument(2 * length <= this.euclideanLength(), "Length to remove (%s) cannot be more than half the length of the subtrajectory (%s).", length, this.euclideanLength());
        return trimStart(length).trimEnd(length);
    }

}
