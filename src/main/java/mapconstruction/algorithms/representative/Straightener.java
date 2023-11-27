package mapconstruction.algorithms.representative;


import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class Straightener {
    /**
     * Get's several properties of how straight a line is.
     * <p>
     * This function adds all the measures to the dict.
     * - FullEndToEndMax
     * - FullEndToEndAverage
     * - FullLineLength
     * - CutEndToEndMax
     * - CutEndToEndAverage
     * - CutLineLength
     *
     * @param representative
     * @param dict
     */
    public static void calculateStraightness(List<Point2D> representative,
                                             Map<String, Object> dict) {
        calculateMaxAndAverage(representative, dict, "Full");

        List<Point2D> representativeStripped = new ArrayList<>();
        for (int i = 1; i < representative.size() - 1; i++) {
            representativeStripped.add(representative.get(i));
        }

        calculateMaxAndAverage(representativeStripped, dict, "Cut");
    }

    /**
     * Calculate several metrics for how straight the representativeSubtrajectory is.
     * <p>
     * It modifies the dict object and hence does not return anything.
     *
     * @param representative, representativeSubtrajectory we are checking the straightness off
     * @param dict,           the dict which is going to store our values
     * @param identifier,     identifies our current representativeSubtrajectory
     */
    public static void calculateMaxAndAverage(List<Point2D> representative, Map<String, Object> dict,
                                              String identifier) {

        double maxDistance = 0;
        double averageDistance = 0;
        double accumulatedLength = 0;
        double straightLineLength = 0;

        if (representative.size() > 0) {
            Line2D straightLine = new Line2D.Double(representative.get(0), representative.get(representative.size() - 1));
            straightLineLength = representative.get(0).distance(representative.get(representative.size() - 1));

            for (int i = 0; i < representative.size(); i++) {
                Point2D currentPoint = representative.get(i);
                double distanceToLine = straightLine.ptLineDist(currentPoint);

                maxDistance = Math.max(maxDistance, distanceToLine);

                averageDistance = (averageDistance * i) / (i + 1);
                averageDistance += distanceToLine / (i + 1);

                if (i < representative.size() - 1) {
                    accumulatedLength += representative.get(i).distance(representative.get(i + 1));
                }
            }
        }

        dict.put(identifier + "EndToEndMax", maxDistance);
        dict.put(identifier + "EndToEndAverage", averageDistance);
        if (accumulatedLength == 0) {
            dict.put(identifier + "LineLength", 0);
        } else {
            dict.put(identifier + "LineLength", straightLineLength / accumulatedLength);
        }
    }
}
