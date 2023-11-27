package mapconstruction.algorithms.maps.merge;

import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.OrthogonalUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.List;

public class Merger {
    /**
     * Whether the dominantSub overlaps the smallSub within the given errorMargin and directionMargin.
     * <p>
     * In other words, whether the smallSub can be merged into the dominantSub.
     *
     * @param dominantSub,     the dominant sub trajectory. Has to start at the same intersection as smallSub
     * @param smallSub,        the small sub trajectory. Has to start at the same intersection as dominantSub
     * @param errorMargin,     maximum distance the smallSub may have from the dominantSub.
     * @param directionMargin, maximum direction difference the smallSub may have at that point from the dominantSub.
     * @param maySkipEnds,     whether can cut the errorMargin of the start of the smallSub.
     * @return yes if it can be merged, no if it can not be merged.
     */
    public static boolean wouldBundleStreetsBeMerged(Subtrajectory dominantSub, Subtrajectory smallSub,
                                                     double errorMargin, double directionMargin, double maySkipEnds) {
        // For every point on the smallSub we check whether it's closest point on the dominantSub is < errorMargin.
        // For every point on the smallSub, and it's corresponding closest point on dominantSub, we check whether the
        //     heading difference over at least 50 meters, is less than directionMargin.

        int minSmallSubIndex = 0;
        int maxSmallSubIndex = smallSub.numPoints();

        if (maySkipEnds > 0) {
            double index = GeometryUtil.getTrajectoryIndexAfterOffset(smallSub, 0, maySkipEnds);
            minSmallSubIndex = (int) (Math.ceil(index));

            index = GeometryUtil.getTrajectoryIndexAfterOffset(smallSub, smallSub.numPoints() - 1, -maySkipEnds);
            maxSmallSubIndex = (int) (Math.floor(index));
        }

        int currentJ = 0;
        for (int i = minSmallSubIndex; i < maxSmallSubIndex; i++) {
            Point2D currentPoint = smallSub.getPoint(i);

            while (true) {
                if (currentJ > dominantSub.numEdges() - 1) {
                    return false;
                }

                Line2D currEdge = dominantSub.getEdge(currentJ);
                if (currEdge.ptSegDist(currentPoint) < errorMargin) {
                    if (checkHeadingDirectionIsSimilar(dominantSub, currentJ, smallSub, i,
                            directionMargin, 25)) {
                        break;
                    }
                }

                currentJ++;
            }
        }

        return true;
    }

    /**
     * Check whether the Heading direction at two Subtrajectories are similar.
     *
     * @param dominantSub,          the dominant sub trajectory. Has to start at the same intersection as smallSub
     * @param dominantSubIndex,     the index of the dominant sub.
     * @param smallSub,             the small sub trajectory. Has to start at the same intersection as dominantSub
     * @param smallSubIndex,        the index of the small sub.
     * @param directionErrorMargin, the direction error margin.
     * @param minACSize,            the minimum length we cover starting from index(in both ways) to determine the
     *                              heading direction. This means it does this length /2 to the left and /2 to the right
     * @return whether the lines created around the given index actually are at most of the given direction error
     */
    private static boolean checkHeadingDirectionIsSimilar(Subtrajectory dominantSub, int dominantSubIndex,
                                                          Subtrajectory smallSub, int smallSubIndex,
                                                          double directionErrorMargin, double minACSize) {

        Line2D dominantLine = OrthogonalUtil.getACLine(dominantSub, dominantSubIndex, minACSize / 2);
        Line2D smallSubLine = OrthogonalUtil.getACLine(smallSub, smallSubIndex, minACSize / 2);

        return GeometryUtil.getHeadingDirectionDifference(dominantLine, smallSubLine) < directionErrorMargin;

    }

    /**
     * Merges the beginning of the street with the intersection.
     * @param trajectory, the trajectory that we want to merge with the intersection.
     * @param intersection, the intersection.
     * @return
     */
    public static List<Point2D> mergeStreetBeginningWithIntersection(List<Point2D> trajectory,
                                                                           Point2D intersection) {
        return trajectory;
    }

    /**
     * Draw a representative that is not connected to any intersection.
     *
     * @param representative, the representative we want to draw.
     */
    public static void drawRepresentativeNotConnectedToIntersection(Representative representative){
        return;
    }

    /**
     * Draw a new bundleStreet.
     */
    public static void drawNewStreet(List<Point2D> newStreet){
        return;
    }

}
