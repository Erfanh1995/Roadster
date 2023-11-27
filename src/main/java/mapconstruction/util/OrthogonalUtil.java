package mapconstruction.util;

import com.google.common.math.DoubleMath;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class OrthogonalUtil {

    /**
     * This get's a line on a trajectory at a given index. This line will always be at least the length of minDistance.
     *
     * @param t,           trajectory
     * @param index,       index of the trajectory we want the ac line for
     * @param minDistance, minimum distance
     * @return an AC line.
     */
    public static Line2D getACLine(Trajectory t, int index, double minDistance) {
        Point2D pointA = t.getPoint(index);
        Point2D pointB = t.getPoint(index);
        Point2D pointC = t.getPoint(index);

        double dA = 0.0;
        double dC = 0.0;
        double firstAAngle = 0.0;
        double firstCAngle = 0.0;
        int indexA = index;
        int indexC = index;
        boolean somethingChanged = true;

        while (somethingChanged) {
            somethingChanged = false;

            if (dA < minDistance) {
                if (indexA > 0) {
                    indexA--;

                    double angleToPoint = GeometryUtil.getHeadingDirection(new Line2D.Double(
                            t.getPoint(indexA), pointA));
                    if (firstAAngle == 0.0) {
                        firstAAngle = angleToPoint;
                    }
                    double angleDifference = GeometryUtil.getAbsoluteAngleDifference(firstAAngle, angleToPoint);
                    // SuperSharpCheck: Preventing more than 125 degree turns of making our representative super ugly..
                    if (pointB.distance(t.getPoint(indexA)) < dA || angleDifference > 125) {
                        break;
                    }
                    pointA = t.getPoint(indexA);
                    dA = pointB.distance(pointA);

                    somethingChanged = true;
                } else {
                    pointA = t.getPoint(0);
                    dA = pointB.distance(pointA);
                }
            }

            if (dC < minDistance) {
                if (indexC < t.numPoints() - 1) {
                    indexC++;
                    double angleToPoint = GeometryUtil.getHeadingDirection(new Line2D.Double(
                            pointC, t.getPoint(indexC)
                    ));

                    if (firstCAngle == 0.0) {
                        firstCAngle = angleToPoint;
                    }

                    double angleDifference = GeometryUtil.getAbsoluteAngleDifference(firstCAngle, angleToPoint);
                    // SuperSharpCheck: Preventing more than 125 degree turns of making our representative super ugly..
                    if (pointB.distance(t.getPoint(indexC)) < dC || angleDifference > 90) {
                        break;
                    }

                    pointC = t.getPoint(indexC);
                    dC = pointB.distance(pointC);

                    somethingChanged = true;
                } else {
                    pointC = t.getPoint(t.numPoints() - 1);
                    dC = pointB.distance(pointC);
                }
            }
        }
        if (dA >= minDistance && dC > dA) {
            // We are cutting off PointC.
            Point2D pointBeforeC = t.getPoint(indexC - 1);
            double leftDistance = dA - pointBeforeC.distance(pointB);
            Line2D lastLine = new Line2D.Double(pointBeforeC, pointC);
            double lineLength = pointC.distance(pointB) - pointBeforeC.distance(pointB);

            double cutOff = Math.max(Math.min(1, leftDistance / lineLength), 0);
            if (!DoubleMath.fuzzyEquals(cutOff, leftDistance / lineLength, 1E-5)) {
                System.out.println("Error Forces.GetACLine. leftDistance was less then 0 or?");
            }
            pointC = GeometryUtil.getPointOnLine(lastLine, cutOff);

        } else if (dC >= minDistance && dA > dC) {
            // We are cutting off pointA
            Point2D pointAfterA = t.getPoint(indexA + 1);
            double leftDistance = dC - pointAfterA.distance(pointB);
            Line2D lastLine = new Line2D.Double(pointAfterA, pointA);
            double lineLength = pointA.distance(pointB) - pointAfterA.distance(pointB);

            double cutOff = Math.max(Math.min(1, leftDistance / lineLength), 0);
            if (!DoubleMath.fuzzyEquals(cutOff, leftDistance / lineLength, 1E-5)) {
                System.out.println("Error Forces.GetACLine. leftDistance was less then 0 or?");
            }
            pointA = GeometryUtil.getPointOnLine(lastLine, cutOff);
        }

        return new Line2D.Double(pointA, pointC);
    }


    /**
     * Calculates a perpendicular line of 2*MAX_EPS length on lineAC which has B exactly in the middle.
     *
     * @param lineAC,      calculates the lineAC.
     * @param pointB,      calculates point B.
     * @param totalLength, length on both sides of B.
     * @return the perpendicular line. Has a total length of 2xtotalLength
     */
    public static Line2D getPerpendicularOnACThroughB(Line2D lineAC, Point2D pointB, double totalLength) {
        // Calculate the slope
        double diffX = lineAC.getX1() - lineAC.getX2();
        double diffY = lineAC.getY1() - lineAC.getY2();
        /* In the case the difference of X-axis is 0, the line is equal to Line2D(pointB, pointB + some on y-axis) */
        if (diffX == 0.0) {
            return new Line2D.Double(
                    new Point2D.Double(pointB.getX() - totalLength, pointB.getY()),
                    new Point2D.Double(pointB.getX() + totalLength, pointB.getY()));
        } else if (diffY == 0.0) { // Same applies for Y.
            return new Line2D.Double(
                    new Point2D.Double(pointB.getX(), pointB.getY() - totalLength),
                    new Point2D.Double(pointB.getX(), pointB.getY() + totalLength));
        } else {
            /* First we calculate the slope */
            double slope = diffY / diffX;
            slope = -1 / slope;

            /* Finding the y-intersect. The formula of this equation is y=slope*x + b, we calculate b = -slope*x + y */
            double b = -slope * pointB.getX() + pointB.getY();

            /* To avoid overflows, we set the first y to + MAX_EPS compared to pointB, and the second - MAX_EPS.
             * From there we calculate the other required point. */
            if (slope < -1 || slope > 1) {
                /* Making sure it is exactly 2*MAX_EPS big */
                double length = totalLength * Math.sqrt(1 + (1 / slope) * (1 / slope));
                double lengthToAppend = totalLength * totalLength / length;

                double point1Y = pointB.getY() - lengthToAppend;
                double point2Y = pointB.getY() + lengthToAppend;

                double point1X = (point1Y - b) / slope;
                double point2X = (point2Y - b) / slope;
                return new Line2D.Double(point1X, point1Y, point2X, point2Y);
            } else { /* Same here. To avoid overflow the first x is set to + MAX_EPS, and the second x to - MAX_EPS*/
                double length = totalLength * Math.sqrt(1 + slope * slope);
                double lengthToAppend = totalLength * totalLength / length;

                double point1X = pointB.getX() - lengthToAppend;
                double point2X = pointB.getX() + lengthToAppend;

                double point1Y = slope * point1X + b;
                double point2Y = slope * point2X + b;
                return new Line2D.Double(point1X, point1Y, point2X, point2Y);
            }
        }
    }
}
