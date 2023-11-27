package mapconstruction.algorithms.segmentation;

import com.google.common.collect.TreeMultiset;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.util.Map.Entry;
import java.util.TreeMap;

/**
 * Segmentation based on heading. A segment T[i,j] of trajectory T is valid if
 * we the space of headings spans at most a given angle.
 * <p>
 * Angles are computed using degrees.
 *
 * @author Roel
 */
public class HeadingSegmenter extends GreedySegmenter {

    private final double maxAngle;

    // datastructures required to efficiently compute the
    // angles that the space of headings span.
    // The datastructure keeps track of the gaps between
    // two consecutive heading angles
    /**
     * Ordered multiset containing the sizes of the gaps
     */
    private final TreeMultiset<Double> gapSizes;

    /**
     * Map containing the gaps, sorted by start value
     */
    private final TreeMap<Double, AngularRange> gaps;

    private HeadingSegmenter(double maxAngle) {
        this.maxAngle = maxAngle;
        gapSizes = TreeMultiset.create();
        gaps = new TreeMap<>();
    }

    public static HeadingSegmenter degrees(double maxAngle) {
        return new HeadingSegmenter(maxAngle);
    }

    public static HeadingSegmenter radians(double maxAngle) {
        return new HeadingSegmenter(Math.toDegrees(maxAngle));
    }

    private void reset() {
        gapSizes.clear();
        gaps.clear();
    }

    @Override
    protected boolean check(Trajectory t, int i, int j) {
        if (i == j) {
            // For 0 edges the criterion is trivially satisfied.
            // Also, if i and j are equal, then we just split of 
            // a segment so the datastructure should reset.
            reset();
            return true;
        } else if (j - i == 1) {
            // single edge. We add a single range of size 360.
            // is trivially true
            Line2D edge = t.getEdge(i);

            double heading = computeHeading(edge);

            AngularRange gap = new AngularRange(heading, heading + 360);

            addGap(gap);
            return true;
        } else {
            // at least two edges

            // get last edge
            Line2D edge = t.getEdge(j - 1);
            double heading = computeHeading(edge);

            // Get the gap containing the heading
            AngularRange containing = gapContaining(heading);

            // Split the gap (unless the heading equals the start to avoid empty ranges
            if (heading != containing.start) {
                if (heading < containing.start) {
                    heading += 360;
                }
                AngularRange split1 = new AngularRange(containing.start, heading);
                AngularRange split2 = new AngularRange(heading, containing.end);

                // Remove old range
                removeGap(containing);

                // add new gaps
                addGap(split1);
                addGap(split2);
            }

            // get largest gapsize
            double largest = gapSizes.lastEntry().getElement();

            // Smallest span is at most given angle.
            return 360 - largest <= maxAngle;
        }
    }

    private void addGap(AngularRange gap) {
        gaps.put(gap.start, gap);
        gapSizes.add(gap.size());
    }

    private void removeGap(AngularRange gap) {
        gaps.remove(gap.start);
        gapSizes.remove(gap.size());
    }

    /**
     * Gets the gap containing the given value.
     *
     * @return
     */
    private AngularRange gapContaining(double angle) {
        // get the gap whose start point is the highes at most the given angle.
        Entry<Double, AngularRange> entry = gaps.lowerEntry(angle);
        if (entry == null) {
            // angle is lower than the lowest key. This means the angle is in the
            // last interval, which spans across 360 degrees
            return gaps.lowerEntry(angle + 360).getValue();
        } else {
            return entry.getValue();
        }
    }

    /**
     * Computes the heading of the given edge, which is the angle it makes with
     * the positive x-axis.
     *
     * @param edge
     * @return
     */
    private double computeHeading(Line2D edge) {
        // direction of edge
        double dirx = edge.getX2() - edge.getX1();
        double diry = edge.getY2() - edge.getY1();

        double angle = Math.toDegrees(Math.atan2(diry, dirx));

        // map [-180, 180] range to [0 - 360]
        return (angle + 360) % 360;
    }

    private static class AngularRange {

        /**
         * Start of the angular range, in degrees.
         */
        double start;

        /**
         * End of the angular range, in degrees.
         */
        double end;

        public AngularRange(double start, double end) {
            this.start = start;
            this.end = end;
        }

        public double size() {
            return end - start;
        }

        @Override
        public String toString() {
            return "[" + start + ", " + end + "]";
        }


    }

}
