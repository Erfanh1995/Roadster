package mapconstruction.algorithms.representative;

import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.OrthogonalUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Collection;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class CutEnd {

    /**
     * Cut's off all bundles representatives of a given list.
     *
     * @param bList the list to cut off.
     */
    public static void cutOffBundlesEndsForSet(Collection<Bundle> bList) {
        if (ALGOCONSTANTS.isEnableCutOff()) {
            long start = System.currentTimeMillis();
            System.out.println("Starting cutting off bundles");
            /*
            for (Bundle b : bList) {
                if (!b.isBundleRepCutOff()) {
                    cutOffBundleEndWhenUnrepresented(b);
                    b.setBundleRepCutOff(true);
                }
            }
            */
            long end = System.currentTimeMillis();
            Log.log(LogLevel.INFO, "Cutting representatives", "Cutting off bundles time: %d ms", end - start);
            System.out.println("Finished cutting off bundles");
        }
    }

    /**
     * Given a bundle, we trim/cut the endings of the RepresentativeSubtrajectory where it is badly represented.
     * <p>
     * This is done because often we have a RepresentativeSubtrajectory that is just longer then the rest of the
     * subtrajectories. This causes a lot of noise and bad behavior in our force representative.
     *
     * @param b the bundle that might get it's RepresentativeSubtrajectory endings trimmed.
     */
    private static void cutOffBundleEndWhenUnrepresented(Bundle b) {
        int bundleClass = STORAGE.getClassFromBundle(b);
        double bestEpsilon = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);

        Subtrajectory repSub = b.getOriginalRepresentative();

        double cutSubFromIndex = getNewStartingIndex(b);
        double cutSubToIndex = getNewEndingIndex(b);

        if (cutSubFromIndex < 0 || cutSubFromIndex > repSub.numEdges() || cutSubToIndex < 0 || cutSubToIndex > repSub.numEdges()) {
            System.out.println("Error CutEnd.cutOffBundleEndWhenUnrepresented. Incorrect values");
            System.out.printf("%f, %f, %f, %f\n", cutSubFromIndex, repSub.getFromIndex(), cutSubToIndex, repSub.getToIndex());
        }

        double oldFromIndex = repSub.getFromIndex();
        double oldToIndex = repSub.getToIndex();

        Point2D oldFromLocation = repSub.getPoint(0);
        Point2D oldToLocation = repSub.getPoint(repSub.numPoints() - 1);
        Point2D newFromLocation = GeometryUtil.getTrajectoryDecimalPoint(repSub, cutSubFromIndex);
        Point2D newToLocation = GeometryUtil.getTrajectoryDecimalPoint(repSub, cutSubToIndex);

        double maxDiff = Math.max(50, bestEpsilon * 1.25);
        double fromLocationDiff = oldFromLocation.distance(newFromLocation);
        if (fromLocationDiff > maxDiff || fromLocationDiff > 0.5 * oldFromLocation.distance(oldToLocation)) {
            cutSubFromIndex = 0;
//            System.out.println("New from location was to far off");
        }

        double toLocationDiff = oldToLocation.distance(newToLocation);
        if (toLocationDiff > maxDiff || toLocationDiff > 0.5 * oldFromLocation.distance(oldToLocation)) {
            cutSubToIndex = repSub.numPoints() - 1;
//            System.out.println("New to location was to far off");
        }

        if (cutSubFromIndex >= cutSubToIndex) {
            cutSubFromIndex = 0;
            cutSubToIndex = repSub.numPoints() - 1;
            if (cutSubFromIndex  > cutSubToIndex){
                System.out.println("Error in CutBundlesEndings. There was a negative distance.");
            }
        }


        double newFromIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(repSub, cutSubFromIndex);
        double newToIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(repSub, cutSubToIndex);

        Subtrajectory sub = new Subtrajectory(repSub.getParent(), newFromIndex, newToIndex);
        b.setNewRepresentativeSubtrajectory(sub);
    }

    /**
     * Cuts off the beginning of the bundle and returns the new fromIndex (Subtrajectory index, not! Trajectory index).
     *
     * @param b the bundle that might get it's start cut off.
     * @return a better fromIndex for the Subtrajectory(in subTrajectory indexes).
     */
    private static double getNewStartingIndex(Bundle b) {
        Subtrajectory repSub = b.getOriginalRepresentative();
        double newIndex = 0;

        for (int i = 0; i < repSub.numEdges(); i++) {
            double lineIndex = getCutOffIndexOnLine(b, repSub.getEdge(i), false);
            if (lineIndex >= 0 && lineIndex <= 1) {
                newIndex = i + lineIndex;
                break;
            }
        }

        return newIndex;
    }

    /**
     * Cuts off the ending of the bundle and returns the new toIndex (Subtrajectory index, not! Trajectory index).
     *
     * @param b the bundle that might get it's end cut off.
     * @return a better toIndex for the Subtrajectory(in subTrajectory indexes).
     */
    private static double getNewEndingIndex(Bundle b) {
        Subtrajectory repSub = b.getOriginalRepresentative();
        double newIndex = repSub.numEdges() - 1;

        for (int i = (int) newIndex; i >= 0; i--) {
            Line2D edge = repSub.getEdge(i);
            edge = new Line2D.Double(edge.getP2(), edge.getP1());
            double lineIndex = getCutOffIndexOnLine(b, edge, true);
            if (lineIndex >= 0 && lineIndex <= 1) {
                newIndex = i + 1 - lineIndex;
                break;
            }
        }
        return newIndex;
    }

    /**
     * Get's the number of intersections between the subtrajectories of Bundle b and the orthogonal line.
     * <p>
     * Only counts the intersections that have a heading angle difference of less then 60 degrees between the
     * acLine and the edge of the subtrajectory we are intersecting with.
     * <p>
     * Note: acLine should be in the same direction as the orthogonal!
     *
     * @param b,        the bundle
     * @param acLine,   the direction of the representative at the given point.
     * @param orthLine, the orthogonal line on the acLine which should intersections with b's subtrajectories.
     * @return the number of meaningful intersections.
     */
    private static int getNumberOfMeaningfulIntersection(Bundle b, Line2D acLine, Line2D orthLine) {
        int meaningfulIntersections = 1;
        for (Subtrajectory sub : b.getSubtrajectories()) {
            if (sub.equals(b.getOriginalRepresentative())) {
                continue;
            }
            for (int i = 0; i < sub.numEdges(); i++) {
                if (orthLine.intersectsLine(sub.getEdge(i)) &&
                        GeometryUtil.getHeadingDirectionDifference(acLine, sub.getEdge(i)) < 50) {
                    meaningfulIntersections++;
                    break;
                }
            }
        }
        return meaningfulIntersections;
    }

    /**
     * Given a line, we calculate at which index ([0, 1]) approximately there is an orthogonal line on our acLine
     * such that it intersects all subtrajectories of our b.
     *
     * @param b,          the bundle
     * @param acLine,     the line we are looking at for our acLine
     * @param acReversed, whether the ac line is reversed. This is used in specific cases.
     * @return an index that can range from 0 to 1. Or -1 in the case there wasn't a good point on the line.
     */
    private static double getCutOffIndexOnLine(Bundle b, Line2D acLine, boolean acReversed) {
        double mainLineLength = acLine.getP1().distance(acLine.getP2());
        double toIncreaseLengthBy = 2.5;  // 2.5 meters every step.
        double indexStep = toIncreaseLengthBy / mainLineLength;

        Line2D absoluteACLine;
        if (acReversed) {
            absoluteACLine = new Line2D.Double(acLine.getP2(), acLine.getP1());
        } else {
            absoluteACLine = acLine;
        }

        double index = 0;

        while (true) {
            if (index >= 1) {
                index = 1;
            }

            Line2D orthLine = OrthogonalUtil.getPerpendicularOnACThroughB(
                    acLine, GeometryUtil.getPointOnLine(acLine, index), ALGOCONSTANTS.getCutEndOrthogonalLineLength());
            int meaningfulIntersections = getNumberOfMeaningfulIntersection(b, absoluteACLine, orthLine);

            if ((double) meaningfulIntersections >= (double) 0.9 * b.size()) {
                break;
            } else {
                if (index == 1) {
                    return -1;
                }
                index += indexStep;
            }
        }

        if ((index != -1 && index < 0) || index > 1) {
            System.out.println("Error CutEnd.getCutOffIndexOnLine! Incorrect value for index");
        }
        return index;
    }
}
