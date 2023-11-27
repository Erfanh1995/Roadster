package mapconstruction.algorithms.separation;

import com.google.common.collect.Sets;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.math.RoundingMode;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Determines how well a bundle could be separated into two separate bundles.
 * <p>
 * This is done as follows:
 *
 * <ul>
 * <li> We create two lists of all subtrajectories in the bundle
 * <ul>
 * <li> The first list sorted by start points, in the direction of the movement
 * <li> The first list sorted by end points, in the direction of the movement
 * </ul>
 * <li> We split both lists at the same location for different positions
 * <li> for each position we count the number of trajectories that end up in the
 * same set of the split.
 * <li> positions are only take such that each side of the split has enough
 * trajectories.
 * <li> The maximum encountered value will be the separability score.
 * </ul>
 *
 * @author Roel
 */
public class PermutationBasedSeparabilityComputer implements SeparabilityComputer {

    /**
     * minimum fraction of input trajectories that should be in one of the
     * partitions.
     */
    private static final double MIN_PARTITION_FRACTION = 1d / 4d;

    /**
     * Computes the separability score of the given bundle.
     *
     * @param bundle
     * @return Double between 0 and 1 indicating the separability. 1 if the
     * bundle is perfectly separable, 0 if it is completely unseparable.
     * <p>
     * Returns 0 if {@code bundle.size <= 1}
     */
    @Override
    public double computeSeparability(Bundle bundle) {
        if (bundle.size() <= 1) {
            return 0;
        }

        /*
         * List of subtrajectories of the bundle, sorted at the incoming position
         *  (start points), in the direction of the movement
         */
        List<Subtrajectory> incoming = GeometryUtil.getIncoming(bundle);

        /*
         * List of subtrajectories of the bundle, sorted at the outgoing position
         *  (end points), in the direction of the movement
         */
        List<Subtrajectory> outgoing = GeometryUtil.getOutgoing(bundle);

        return computeSeparationPair(incoming, outgoing).score;
    }

    /**
     * Computes the separation distance for the given bundle.
     * <p>
     * The separation distance is the distance between the starts or ends
     * (whichever is smallest) of the trajectories directly adjacent to
     * the separation point.
     * <p>
     * This value is needed along with the separability score to
     * determine the separability.
     *
     * @param bundle
     * @return
     */
    public double computeSeparationDistance(Bundle bundle) {
        if (bundle.size() <= 1) {
            return 0;
        }

        /*
         * List of subtrajectories of the bundle, sorted at the incoming position
         *  (start points), in the direction of the movement
         */
        List<Subtrajectory> incoming = GeometryUtil.getIncoming(bundle);

        /*
         * List of subtrajectories of the bundle, sorted at the outgoing position
         *  (end points), in the direction of the movement
         */
        List<Subtrajectory> outgoing = GeometryUtil.getOutgoing(bundle);

        int splitIndex = computeSeparationPair(incoming, outgoing).splitIndex;


        DistanceMatrix m1 = new DistanceMatrix(incoming.get(splitIndex - 1), incoming.get(splitIndex));
        double startDist = m1.getEdgeDistance(0, 0);
        DistanceMatrix m2 = new DistanceMatrix(outgoing.get(splitIndex - 1), outgoing.get(splitIndex));
        double endDist = m2.getEdgeDistance(m2.getT1().numEdges() - 1, m2.getT2().numEdges() - 1);
        return Math.min(startDist, endDist);
    }

    /**
     * Computes the separability score of the given bundle, along with the
     * split-index giving the score.
     *
     * @param bundle
     */
    private SeparationPair computeSeparationPair(List<Subtrajectory> incoming, List<Subtrajectory> outgoing) {
        final int splitStart = DoubleMath.roundToInt(MIN_PARTITION_FRACTION * incoming.size(), RoundingMode.CEILING);
        final int splitEnd = DoubleMath.roundToInt((1 - MIN_PARTITION_FRACTION) * incoming.size(), RoundingMode.FLOOR);

        int maxSplit = splitStart;
        double maxScore = 0;
        for (int i = splitStart; i <= splitEnd; i++) {
            double score = computeScore(incoming, outgoing, i);
            if (score > maxScore) {
                maxScore = score;
                maxSplit = i;
            }
        }
        return new SeparationPair(maxScore, maxSplit);
    }


    private double computeScore(List<Subtrajectory> incoming, List<Subtrajectory> outgoing, int split) {
        // split the sets
        Set<Subtrajectory> in1 = new HashSet<>(incoming.subList(0, split));
        Set<Subtrajectory> in2 = new HashSet<>(incoming.subList(split, incoming.size()));
        Set<Subtrajectory> out1 = new HashSet<>(outgoing.subList(0, split));
        Set<Subtrajectory> out2 = new HashSet<>(outgoing.subList(split, outgoing.size()));

        int correct = Sets.intersection(in1, out1).size() + Sets.intersection(in2, out2).size();

        return 1.0 * correct / incoming.size();
    }


    /**
     * Gets virtual index of the point halfway the trajectory, in terms of
     * length
     *
     * @param s
     * @return
     */
    private double getHalfwayIndex(Subtrajectory s) {
        final double halfLength = s.euclideanLength() / 2;
        double lengthSoFar = 0;

        // find edge 
        int edgeExceedingLength = 0;
        for (Line2D line : s.edges()) {
            double newLength = lengthSoFar + GeometryUtil.lineLength(line);

            if (newLength > halfLength) {
                break;
            } else {
                lengthSoFar = newLength;
                edgeExceedingLength++;
            }
        }

        // assume constant movement on edge
        return edgeExceedingLength + (halfLength - lengthSoFar) / (GeometryUtil.lineLength(s.getEdge(edgeExceedingLength)));
    }

    /**
     * Gets virtual point halfway the trajectory, in terms of length
     *
     * @param s
     * @return
     */
    private Point2D getHalfwayPoint(Subtrajectory s) {
        double index = getHalfwayIndex(s);
        return GeometryUtil.getPointOnLine(s.getEdge((int) index), index % 1);
    }

    private static class SeparationPair {

        final double score;
        final int splitIndex;

        public SeparationPair(double score, int splitIndex) {
            this.score = score;
            this.splitIndex = splitIndex;
        }

    }
}
