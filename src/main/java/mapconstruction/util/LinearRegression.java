package mapconstruction.util;

import com.google.common.math.DoubleMath;
import mapconstruction.trajectories.Subtrajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.IntStream;

/**
 * Class which applies linear regression to several problems.
 * <p>
 * It contains a function which does the actual linear regression but also the wrappers for the different problems.
 *
 * @author Jorrick Sleijster
 */
public class LinearRegression {
    /**
     * We apply linear regression on the points after which a finite line is returned.
     *
     * @param subTurnParts all points used in the linear regression
     * @param dict         place where we store our info in.
     * @param focusPoint   when we have our linear regression line, draw it around this point.
     * @param lineLength   the length of the final returned line.
     * @return the two points that form the line of the representative.
     */
    public static Line2D applyLinearRegressionNextToTurn(List<List<Point2D>> subTurnParts,
                                                         Map<String, Object> dict,
                                                         Point2D focusPoint,
                                                         double lineLength) {

        List<Point2D> points = new ArrayList<>();
        int numberOfVertical = 0;
        for (List<Point2D> pointsOutOfTurn : subTurnParts) {
            points.addAll(pointsOutOfTurn);
            Line2D line = new Line2D.Double(pointsOutOfTurn.get(0), pointsOutOfTurn.get(pointsOutOfTurn.size() - 1));
            if (GeometryUtil.isHeadingVertical(line)) {
                numberOfVertical += 1;
            }
        }

        applyLinearRegression(points, dict, (numberOfVertical >= subTurnParts.size() / 2));
        double beta1 = (double) dict.get("beta1");
        double beta0 = (double) dict.get("beta0");

        return FlipIfOppositeDirection(subTurnParts,
                CreateFiniteLine(beta1, beta0, points, lineLength / 2, focusPoint)
        );
    }

    /**
     * We create a straight line through all the points of the representative with ending approximately at the endings
     * of the representative.
     *
     * @param representative the line we are at
     * @param dict           the place where we store our info of the final line.
     * @return 4 points illustrating the two lines segments line([0, 1]) and line([2, 3]).
     */
    public static List<Point2D> createLinearRegressionLineForRepresentative(List<Point2D> representative,
                                                                            Map<String, Object> dict) {

        // Calculate whether the average location is vertical.
        Line2D longLine = new Line2D.Double(representative.get(0), representative.get(representative.size() - 1));
        applyLinearRegression(representative, dict, GeometryUtil.isHeadingVertical(longLine));
        double beta1 = (double) dict.get("beta1");
        double beta0 = (double) dict.get("beta0");

        return FindCutOffLineByTwoEndings(beta1, beta0, representative.get(0), representative.get(representative.size() - 1));
    }

    /**
     * Shape fits a part/road that is attached to an intersection.
     *
     * @param bundlePart the parts we apply shape fitting on
     * @param focusPoint when we have our linear regression line, draw it around this point.
     * @param lineLength the length of the final returned line.
     * @return a line that represents the line.
     */
    public static Line2D applyLinearRegressionForIntersection(List<Subtrajectory> bundlePart, Point2D focusPoint, double lineLength) {
        List<Point2D> allPoints = new ArrayList<>();
        List<Subtrajectory> bundlePartCopy = new ArrayList<>(bundlePart);

        /**
         * First we start by filtering out at most 20% which has to most inaccurate angle.
         */
        List<Double> angles = new ArrayList<>();
        double averageAngle = 0;
        for (int i = 0; i < bundlePartCopy.size(); i++) {
            Subtrajectory subtrajectory = bundlePartCopy.get(i);
            double angle = GeometryUtil.getHeadingDirection(new Line2D.Double(subtrajectory.getPoint(0),
                    subtrajectory.getPoint(subtrajectory.numPoints() - 1)));
            averageAngle = GeometryUtil.getNewAverageAngle(averageAngle, angle, i);
            angles.add(angle);
        }

        List<Double> angleDifferences = new ArrayList<>();
        for (Double angle : angles) {
            angleDifferences.add(GeometryUtil.getAbsoluteAngleDifference(angle, averageAngle));
        }

        while (angles.size() - 1 > bundlePart.size() * 0.8) {
            int indexOfMax = IntStream.range(0, angleDifferences.size()).boxed().max(Comparator.comparingDouble(angleDifferences::get)).orElseThrow(NoSuchElementException::new);
            angleDifferences.remove(indexOfMax);
            angles.remove(indexOfMax);
            bundlePartCopy.remove(indexOfMax);
        }


        int numberOfVertical = 0;
        for (Subtrajectory subtrajectory : bundlePartCopy) {
            for (int i = 0; i < subtrajectory.numPoints(); i++) {
                double indexT = GeometryUtil.convertSubIndexToTrajectoryIndex(subtrajectory, i);
                if (DoubleMath.fuzzyEquals(indexT % 1, 0, 1E-4)) {
                    allPoints.add(subtrajectory.getPoint(i));
                }
            }

            Line2D line = new Line2D.Double(subtrajectory.getPoint(0),
                    subtrajectory.getPoint(subtrajectory.numPoints() - 1));
            if (GeometryUtil.isHeadingVertical(line)) {
                numberOfVertical += 1;
            }
        }

        Map<String, Object> dict = new HashMap<>();

        applyLinearRegression(allPoints, dict, (numberOfVertical >= bundlePart.size() / 2));
        double beta1 = (double) dict.get("beta1");
        double beta0 = (double) dict.get("beta0");

        return CreateFiniteLine(beta1, beta0, allPoints, lineLength / 2, focusPoint);
    }

    /**
     * Apply straight line linear regression
     * <p>
     * Credits go partially to Robert Sedgewick and Kevin Wayne
     * https://introcs.cs.princeton.edu/java/97data/LinearRegression.java.html
     *
     * @param points the points we want to apply linear regression on
     * @param dict   contains info about our linear regression line
     */
    private static void applyLinearRegression(List<Point2D> points, Map<String, Object> dict, boolean verticalLine) {
        // For faster speed we assume MAXN = 1000;
        int MAXN = 1000;
        int n = 0;
        double[] x = new double[MAXN];
        double[] y = new double[MAXN];

        // first pass: read in data, compute xbar and ybar
        double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
        for (Point2D point : points) {
            if (n >= 1000) {
                break;
            }
            if (verticalLine) {
                x[n] = point.getY();
                y[n] = point.getX();
                sumx += x[n];
                sumx2 += x[n] * x[n];
                sumy += y[n];
            } else {
                x[n] = point.getX();
                y[n] = point.getY();
                sumx += x[n];
                sumx2 += x[n] * x[n];
                sumy += y[n];
            }
            n++;
        }
        double xbar = sumx / n;
        double ybar = sumy / n;

        // second pass: compute summary statistics
        double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
        for (int i = 0; i < n; i++) {
            xxbar += (x[i] - xbar) * (x[i] - xbar);
            yybar += (y[i] - ybar) * (y[i] - ybar);
            xybar += (x[i] - xbar) * (y[i] - ybar);
        }
        double beta1 = xybar / xxbar;
        double beta0 = ybar - beta1 * xbar;

        // print results
//        System.out.println("y   = " + beta1 + " * x + " + beta0);
        // analyze results
        int df = n - 2;
        double rss = 0.0;      // residual sum of squares
        double ssr = 0.0;      // regression sum of squares
        for (int i = 0; i < n; i++) {
            double fit = beta1 * x[i] + beta0;
            rss += (fit - y[i]) * (fit - y[i]);
            ssr += (fit - ybar) * (fit - ybar);
        }
        double R2 = ssr / yybar;
        double svar = rss / df;
        double svar1 = svar / xxbar;
        double svar0 = svar / n + xbar * xbar * svar1;

        /* Now we have x = ay +b, and we need to convert it to y = ax + b
         * Hence y = (1/a)*x - (b/a) */
        if (verticalLine) {
            beta0 = -beta0 / beta1; // beta0 = b
            beta1 = 1 / beta1;     // beta1 = a
        }

        dict.put("beta0", beta0);
        dict.put("beta1", beta1);
        dict.put("R^2", R2);
        dict.put("std error of beta_1", Math.sqrt(svar1));
        dict.put("std error of beta_0", Math.sqrt(svar0));
        dict.put("SSTO", yybar);
        dict.put("SSE", rss);
        dict.put("SSR", ssr);
    }


    /**
     * Given a regression line, cut it off at around the two points mentioned.
     *
     * @param a   the slope of the found regression line
     * @param b   another identifier for the found regression line
     * @param p1  point where the regression line should start approximately
     * @param p2, point where the regression line should end approximately
     * @return Line2D with the regression line cut off.
     */
    private static List<Point2D> FindCutOffLineByTwoEndings(double a, double b, Point2D p1, Point2D p2) {
        /* Our current line is: y = a * x + b */
        double aO, bO;

        /* We now want to find the orthogonal line for p1. */
        aO = -(1 / a);
        bO = p1.getY() - (aO * p1.getX());

        double intersectP1X = (bO - b) / (a - aO);
        double intersectP1Y = a * intersectP1X + b;

        /* We now want to find the orthogonal line for p2. */
        aO = -(1 / a);
        bO = p2.getY() - (aO * p2.getX());

        double intersectP2X = (bO - b) / (a - aO);
        double intersectP2Y = a * intersectP2X + b;

        return new ArrayList<Point2D>(Arrays.asList(
                new Point2D.Double(intersectP1X, intersectP1Y),
                new Point2D.Double(intersectP2X, intersectP2Y)
        ));
    }

    /**
     * Given a regression line y=ax+b, make it a line of length line_length centered around the middlePoint.
     *
     * @param a,          the slope of the found regression line
     * @param b,          another identifier for the found regression line
     * @param points,     all points.
     * @param line_length the length of the line to draw
     * @param middlePoint at which point we should focus the trajectory to draw.
     * @return Line2D with the regression line cut off.
     */
    private static Line2D CreateFiniteLine(double a, double b, List<Point2D> points, double line_length,
                                           Point2D middlePoint) {
        double averageX = 0.0, averageY = 0.0;

        if (middlePoint == null) {
            for (int i = 0; i < points.size(); i++) {
                averageX = (averageX / (i + 1)) * i + points.get(i).getX() / (i + 1);
                averageY = (averageY / (i + 1)) * i + points.get(i).getY() / (i + 1);
            }
        } else {
            averageX = middlePoint.getX();
            averageY = middlePoint.getY();
        }

        if (a < -1 || a > 1) {
            /* Making sure it is exactly 2*MAX_EPS big */
            double length = line_length * Math.sqrt(1 + (1 / a) * (1 / a));
            double lengthToAppend = line_length * line_length / length;

            double point1Y = averageY - lengthToAppend;
            double point2Y = averageY + lengthToAppend;

            double point1X = (point1Y - b) / a;
            double point2X = (point2Y - b) / a;

            return new Line2D.Double(new Point2D.Double(point1X, point1Y), new Point2D.Double(point2X, point2Y));
        } else { /* Same here. To avoid overflow the first x is set to + MAX_EPS, and the second x to - MAX_EPS*/
            double length = line_length * Math.sqrt(1 + a * a);
            double lengthToAppend = line_length * line_length / length;

            double point1X = averageX - lengthToAppend;
            double point2X = averageX + lengthToAppend;

            double point1Y = a * point1X + b;
            double point2Y = a * point2X + b;

            return new Line2D.Double(new Point2D.Double(point1X, point1Y), new Point2D.Double(point2X, point2Y));
        }
    }

    /**
     * Checks whether the listLine goes in the same direction as the line [startPoint, endPoint]
     *
     * @param subTurnParts, several parts before or after a turn.
     * @param lrLine,       lrLine containing two points representing a lrLine.
     * @return
     */
    private static Line2D FlipIfOppositeDirection(List<List<Point2D>> subTurnParts, Line2D lrLine) {

        // In this case we have a lrLine that is in the opposite direction. Hence we flip the list.
//        if (startPoint.distance(lrLine.getP2()) + endPoint.distance(lrLine.getP1()) <
//                startPoint.distance(lrLine.getP1()) + endPoint.distance(lrLine.getP2())) {
//        int countVotesForOpposite = 0;
//        for (List<Point2D> part : subTurnParts){
//
//            Line2D referenceLine = new Line2D.Double(part.get(0), part.get(part.size() - 1));
//            if (GeometryUtil.getHeadingDirectionDifference(referenceLine, lrLine) > 90){
//                countVotesForOpposite++;
//            }
//        }
//
//        if (countVotesForOpposite > subTurnParts.size()){
//            return new Line2D.Double(lrLine.getP2(), lrLine.getP1());
//        }
        double diffX = 0;
        double diffY = 0;
        for (List<Point2D> part : subTurnParts) {
            Line2D referenceLine = new Line2D.Double(part.get(0), part.get(part.size() - 1));
            diffX += referenceLine.getX2() - referenceLine.getX1();
            diffY += referenceLine.getY2() - referenceLine.getY2();
        }

        if (Math.abs(diffX) > Math.abs(diffY)) {
            if (diffX > 0 && lrLine.getX2() - lrLine.getX1() < 0 ||
                    diffX < 0 && lrLine.getX2() - lrLine.getX1() > 0) {
                return new Line2D.Double(lrLine.getP2(), lrLine.getP1());
            }
        } else {
            if (diffY > 0 && lrLine.getY2() - lrLine.getY1() < 0 ||
                    diffY < 0 && lrLine.getY2() - lrLine.getY1() > 0) {
                return new Line2D.Double(lrLine.getP2(), lrLine.getP1());
            }
        }


        return lrLine;
    }
}
