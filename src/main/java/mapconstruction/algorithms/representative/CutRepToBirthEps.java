package mapconstruction.algorithms.representative;

import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class CutRepToBirthEps {
    /**
     * Cut's off all bundles representatives of a given list.
     * Now instead of by looking if our other trajectories are around and intersect an orthogonal,
     * we base our decision to cut off parts on the birth epsilon of our bundle.
     *
     * @param b the bundle to cut off.
     */
    public static Pair<Subtrajectory, Set<Subtrajectory>> cutOffBundleEndBasedOnEpsilon(Bundle b, Subtrajectory subRep){
        int bundleClass = STORAGE.getClassFromBundle(b);

        double cutSubFromIndex = getNewStartOrEndingIndex(b, subRep, true);
        double cutSubToIndex = getNewStartOrEndingIndex(b, subRep, false);

        if (cutSubFromIndex < 0 || cutSubFromIndex > subRep.numEdges() || cutSubToIndex < 0 || cutSubToIndex > subRep.numEdges()) {
            System.out.println("Error CutEnd.cutOffBundleEndBasedOnEpsilon. Incorrect values");
            System.out.printf("%f, %f \n", cutSubFromIndex, cutSubToIndex);
        }

        if (cutSubFromIndex >= cutSubToIndex) {
            cutSubFromIndex = 0;
            cutSubToIndex = subRep.numPoints() - 1;
            if (cutSubFromIndex  > cutSubToIndex){
                System.out.println("Error in CutEnd.cutOffBundleEndBasedOnEpsilon. There was a negative distance.");
            }
        }

        Subtrajectory newSubRep = new Subtrajectory(subRep, cutSubFromIndex, cutSubToIndex);

        Set<Subtrajectory> subtrajectories = getNewSubtrajectoryList(b, newSubRep);

        return new Pair<>(newSubRep, subtrajectories);
    }

    /**
     * Cuts off the beginning of the bundle and returns the new fromIndex (Subtrajectory index, not! Trajectory index).
     *
     * @param b the bundle that might get it's start cut off.
     * @param repSub, the subtrajectory of the representative we are using now
     * @param start, whether it is the start of end of the bundle we have to check.
     * @return a better fromIndex for the Subtrajectory(in subTrajectory indexes).
     */
    private static double getNewStartOrEndingIndex(Bundle b, Subtrajectory repSub, boolean start) {
        double newIndex = 0;

        if (!start){
            repSub = repSub.reverse();
        }

        for (int i = 0; i < repSub.numEdges(); i++) {
            double lineIndex = getCutOffIndexOnLine(b, repSub.getEdge(i));
            if (lineIndex >= 0 && lineIndex <= 1) {
                newIndex = i + lineIndex;
                break;
            }
        }

        if (!start){
            newIndex = repSub.numPoints() - 1 - newIndex;
        }

        return newIndex;
    }

    /**
     * Given a line, we calculate at which index ([0, 1]) approximately there is an orthogonal line on our acLine
     * such that it intersects all subtrajectories of our b.
     *
     * @param b,          the bundle
     * @param acLine,     the edge we are looking at for our
     * @return an index that can range from 0 to 1. Or -1 in the case there wasn't a good point on the line.
     */
    private static double getCutOffIndexOnLine(Bundle b, Line2D acLine) {
        double mainLineLength = acLine.getP1().distance(acLine.getP2());
        double toIncreaseLengthBy = 10;  // 5 meters every step.
        double indexStep = toIncreaseLengthBy / mainLineLength;
        double birthEpsilon = STORAGE.getEvolutionDiagram().getBirthMoment(STORAGE.getClassFromBundle(b));
        double errorMargin = Math.min(birthEpsilon + 10, birthEpsilon * 2);

        double index = 0;

        while (true) {
            if (index >= 1) {
                index = 1;
            }

            int subsWithinRange = 0;
            Point2D currentPoint = GeometryUtil.getPointOnLine(acLine, index);
            for (Subtrajectory subtrajectory : b.getSubtrajectories()){
                Point2D point2D = GeometryUtil.getTrajectoryPointClosestToLocationAfterOffset(subtrajectory, currentPoint, 0.0);
                double distance = point2D.distance(currentPoint);

                if (distance < errorMargin){
                    subsWithinRange++;
                }
            }

            if ((double) subsWithinRange >= 0.9 * b.size()) {
                break;
            } else {
                if (index == 1) {
                    return -1;
                }
                index += indexStep;
            }
        }

        if ((index != -1 && index < 0) || index > 1) {
            System.out.println("Error CutRepToBirthEps.getCutOffIndexOnLine! Incorrect value for index");
        }
        return index;
    }

    /**
     * Given a new subtrajectory of the representative, we find the values for the subtrajectories representing this
     * bundle.
     * @param bundle the bundle we are creating a new subtrajectory list for
     * @param subRep the subrepresentative indicating the cut off on both endings.
     */
    public static Set<Subtrajectory> getNewSubtrajectoryList(Bundle bundle, Subtrajectory subRep){
        Set<Subtrajectory> newSubtrajectories = new HashSet<>();
        Point2D startPoint = subRep.getFirstPoint();
        Point2D endPoint = subRep.getLastPoint();

        for (Subtrajectory subtrajectory: bundle.getSubtrajectories()){
            double startIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subtrajectory, startPoint);
            double endIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(subtrajectory, endPoint);

            startIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subtrajectory, startIndex);
            endIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(subtrajectory, endIndex);

            if (startIndex > endIndex){
                double tempIndex = startIndex;
                startIndex = endIndex;
                endIndex = tempIndex;
            }

            if (startIndex == endIndex){
                continue;
            }
            newSubtrajectories.add(new Subtrajectory(subtrajectory.getParent(), startIndex, endIndex));

        }

        return newSubtrajectories;
    }
}
