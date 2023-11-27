package mapconstruction.algorithms.maps.intersections;

import com.google.common.base.Preconditions;
import com.google.common.math.DoubleMath;
import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Representative;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.LinearRegression;
import mapconstruction.util.Pair;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;

public class ComputeIntersectionLocation {

    /**
     * This function computes a preciser intersection location
     *
     * @param intersection, location of the intersection
     * @return the intersection
     */
    public static Point2D computeExactIntersectionLocation(Intersection intersection) {
        Set<Bundle> bAtIntersection = intersection.getAllBundlesAroundIntersection();
        Set<Bundle> bCrossingIntersection = intersection.getAllBundlesCrossingTheIntersection();

        Bundle straightBundle = getStraightestBundle(intersection.getApproximateLocation(), 150, bCrossingIntersection);
        intersection.setStraightestBundle(straightBundle);

        List<List<Subtrajectory>> bundlesParts = divideIntersectionIntoStreetParts(intersection);
        if (straightBundle != null) {
            removeBundlePartsFromStraightestBundle(bundlesParts, straightBundle, intersection.getApproximateLocation());
        }
        intersection.setBundlePartsBundled(bundlesParts);

        List<Line2D> shapeFittingLines = applyShapeFittingOnBundleParts(intersection, true);
//        List<Line2D> shapeFittingLines = applyShapeFittingOnBundleParts(intersection, false);

        // Now we check if the shape fitting lines are similar in direction.
        // If so, we merge the two together.
        // @Done check impact - Had no impact anymore.
//        mergeSimilarShapeFittingLines(shapeFittingLines, bundlesParts, intersection);
//        intersection.setBundlePartsBundled(bundlesParts);

//        shapeFittingLines = applyShapeFittingOnBundleParts(intersection, true);
        intersection.setShapeFittingLines(shapeFittingLines);


        Point2D newLocation = calculateIntersectionLocation(intersection, straightBundle);
        if (newLocation.distance(intersection.getApproximateLocation()) > 100) {
            return intersection.getApproximateLocation();
        }
        return newLocation;
    }

    /**
     * Looks for all bundles, whether their representative is straight at the point of the intersection. Takes the
     * bundles with the largest size that is straight.
     *
     * @param location,              the location of the intersection
     * @param offset,                offset from the location
     * @param bCrossingIntersection, bundles crossing the intersection;
     * @return
     */
    private static Bundle getStraightestBundle(Point2D location, double offset, Set<Bundle> bCrossingIntersection) {
        Iterator<Bundle> bundleIterator = bCrossingIntersection.iterator();
        List<Bundle> bundleList = new ArrayList<>(bCrossingIntersection);

        Bundle maxBundle = null;
        double minScore = 1.0;
        double maxRepDistance = 15.0;

        while (bundleIterator.hasNext()) {
            Bundle b1 = bundleIterator.next();

            Representative rep = b1.getRepresentative();

            double midIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(rep, location);
            if (GeometryUtil.getTrajectoryDecimalPoint(rep, midIndex).distance(location) > 25) {
                bundleIterator.remove();
                continue;
            }

            double fromIndex = GeometryUtil.getTrajectoryIndexAfterOffset(rep, midIndex, -offset);
            double endIndex = GeometryUtil.getTrajectoryIndexAfterOffset(rep, midIndex, offset);
//
//            Point2D fromPoint = GeometryUtil.getTrajectoryDecimalPoint(rep, fromIndex);
//            Point2D endPoint = GeometryUtil.getTrajectoryDecimalPoint(rep, endIndex);

            Subtrajectory subRep = new Subtrajectory(rep, fromIndex, endIndex);
            double euclideanLength = subRep.euclideanLength();
            double discreteLength = subRep.discreteLength();
            double ratio = discreteLength / euclideanLength;

            // We check if there is a larger bundle nearby this Representative.
            boolean hasLargerBundle = false;
            for (Bundle b2 : bundleList) {
                Representative b2rep = b2.getRepresentative();

                if (b1.size() >= b2.size()) {
                    continue;
                }

                Set<Subtrajectory> b1Subs = b1.getNonReverseSubtrajectories();
                int countSubsCovered = 0;
                for (Subtrajectory sub : b1Subs) {
                    if (sub.overlapsOneItemInList(b2.getNonReverseSubtrajectories())) {
                        countSubsCovered++;
                    }
                }
                if (countSubsCovered == 0) {
                    continue;
                }

//                double fromIndexB2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(b2rep, fromPoint);
//                double endIndexB2 = GeometryUtil.getIndexOfTrajectoryClosestToPoint(b2rep, endPoint);

//                Point2D fromPointB2 = GeometryUtil.getTrajectoryDecimalPoint(b2rep, fromIndexB2);
//                Point2D endPointB2 = GeometryUtil.getTrajectoryDecimalPoint(b2rep, endIndexB2);

//                double distance1 = fromPoint.distance(fromPointB2);
//                double distance2 = endPoint.distance(endPointB2);
//
//                double coveredDistanceByRep = GeometryUtil.getIndexToIndexDistance(b2rep, fromIndexB2, endIndexB2);
//                if (coveredDistanceByRep > 2.5 * offset) {
//                    continue;
//                }
//
//                if (distance1 < maxRepDistance && distance2 < maxRepDistance ||
//                        distance1 + distance2 < maxRepDistance * 1.5) {
                    hasLargerBundle = true;
//                }
            }

            if (hasLargerBundle) {
                bundleIterator.remove();
                continue;
            }


            if (ratio > 1 || ratio < 0) {
                if (DoubleMath.fuzzyEquals(ratio, 1.0, 1E-6)) {
                    ratio = 1.0;
                } else if (DoubleMath.fuzzyEquals(ratio, 0.0, 1E-6)) {
                    ratio = 0.0;
                } else {
                    throw new IllegalStateException("Margin is not allowed to be < 0 || > 1. Margin was " + Double.toString(ratio));
                }
            }

            if (ratio < 0.95) {
                bundleIterator.remove();
                continue;
            }

            double score;
            score = (1 - ratio) * (1 - ratio);
            score = score / b1.size();

            if (score < minScore) {
                minScore = score;
                maxBundle = b1;
            }
        }

        return maxBundle;
    }


    /**
     * This function divides bundles that cover the same part into clusters, then we cluster their subtrajectories into
     * part around the intersection.
     *
     * @param intersection, the intersection object
     * @return street parts.
     */
    private static List<List<Subtrajectory>> divideIntersectionIntoStreetParts(Intersection intersection) {
        List<List<Pair<Boolean, Bundle>>> streetParts = new ArrayList<>();
        HashMap<Pair<Boolean, Bundle>, Pair<Double, Double>> anglesStreetPartMap = new HashMap<>();

        Set<Bundle> bundles = intersection.getAllBundlesCrossingTheIntersection();
        Point2D location = intersection.getApproximateLocation();

        // This is the actual function which merges the bundle parts.
        for (Bundle bundle : bundles) {
            mergeBundleParts(streetParts, anglesStreetPartMap, bundle, false, intersection);
            mergeBundleParts(streetParts, anglesStreetPartMap, bundle, true, intersection);
        }

        // For this we are actually collecting the parts that should be considered to be part of the bundle.
        double minOff = 15;
        double offset = 50;

        List<List<Subtrajectory>> streetPartsSubs = new ArrayList<>();
        for (List<Pair<Boolean, Bundle>> streetPart : streetParts) {
            List<Subtrajectory> streetPartSubs = new ArrayList<>();
            streetPartsSubs.add(streetPartSubs);

            for (Pair<Boolean, Bundle> streetBundlePart : streetPart) {
                boolean afterIntersection = streetBundlePart.getFirst();
                Bundle bundle = streetBundlePart.getSecond();

                int multiplier = afterIntersection ? 1 : -1;

                for (Subtrajectory sub : bundle.getSubtrajectories()) {
                    double midIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub, location);

                    double oriEndIndex = GeometryUtil.getTrajectoryIndexAfterOffset(sub, midIndex, multiplier * minOff);

                    double endIndex = Math.floor(oriEndIndex);
                    if (afterIntersection) {
                        endIndex = Math.ceil(oriEndIndex);
                    }
                    double startIndex = GeometryUtil.getTrajectoryIndexAfterOffset(sub, endIndex, multiplier * offset);

                    double pEndIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(sub, endIndex);
                    double pStartIndex = GeometryUtil.convertSubIndexToTrajectoryIndex(sub, startIndex);

                    if (pEndIndex % 1 != 0) {
                        continue;
                    }

                    Subtrajectory newSub;
                    if (afterIntersection) {
                        newSub = new Subtrajectory(sub.getParent(), pEndIndex, pStartIndex);
                        newSub = newSub.reverse();
                    } else {
                        newSub = new Subtrajectory(sub.getParent(), pStartIndex, pEndIndex);
                    }

                    if (!newSub.overlapsOneItemInList(streetPartSubs)) {
                        streetPartSubs.add(newSub);
                    }
                }
            }
        }

        return streetPartsSubs;
    }

    /**
     * For a given bundle which is close to a specific intersection, we calculate which 'bundleParts' should be part of
     * the streetPart. Meaning, for a given bundle we check if it can be merged with any of the already created
     * streetParts. If not, we create a new streetPart.
     * same cluster
     *
     * @param streetParts all merged 'streets'
     * @param anglesStreetPartMap to keep track of all the angles of bundleParts
     * @param bundle the bundle we are currently trying to add to one of the streetParts
     * @param afterIntersection whether we are checking the bundlepart before or after the intersection.
     * @param intersection the intersection object we are currently looking at.
     */
    private static void mergeBundleParts(List<List<Pair<Boolean, Bundle>>> streetParts,
                                         HashMap<Pair<Boolean, Bundle>, Pair<Double, Double>> anglesStreetPartMap,
                                         Bundle bundle, Boolean afterIntersection, Intersection intersection) {
        Representative bRep = bundle.getRepresentative();
        Point2D location = intersection.getApproximateLocation();
        double midIndex = GeometryUtil.getIndexOfTrajectoryClosestToPoint(bRep, location);

        double so = 50; // Starting offset
        double off = 50; // Total meters we always look over
        int multiplier = afterIntersection ? 1 : -1;

        double index1 = GeometryUtil.getTrajectoryIndexAfterOffset(bRep, midIndex, multiplier * off);
        double index2 = GeometryUtil.getTrajectoryIndexAfterOffset(bRep, midIndex, multiplier * so);
        double index3 = GeometryUtil.getTrajectoryIndexAfterOffset(bRep, index2, multiplier * off);
        Point2D point1 = GeometryUtil.getTrajectoryDecimalPoint(bRep, index1);
        Point2D point2 = GeometryUtil.getTrajectoryDecimalPoint(bRep, index2);
        Point2D point3 = GeometryUtil.getTrajectoryDecimalPoint(bRep, index3);
        double angle1 = GeometryUtil.getHeadingDirection(new Line2D.Double(point1, location));
        double angle2 = GeometryUtil.getHeadingDirection(new Line2D.Double(point2, point3));

        boolean merged = false;
        for (List<Pair<Boolean, Bundle>> streetPart : streetParts) {
            for (Pair<Boolean, Bundle> streetBundle : streetPart) {
                Pair<Double, Double> angles = anglesStreetPartMap.get(streetBundle);
//                if (GeometryUtil.getAbsoluteAngleDifference(angle1, angles.getFirst()) +
//                        GeometryUtil.getAbsoluteAngleDifference(angle2, angles.getSecond()) < 60 &&
//                        GeometryUtil.getAbsoluteAngleDifference(angle2, angles.getSecond()) < 20) {
                if (GeometryUtil.getAbsoluteAngleDifference(angle1, angles.getFirst()) < 30 &&
                        GeometryUtil.getAbsoluteAngleDifference(angle2, angles.getSecond()) < 20) {
                    merged = true;

                    Pair<Boolean, Bundle> bundlePair = new Pair<>(afterIntersection, bundle);
                    streetPart.add(bundlePair);
                    anglesStreetPartMap.put(bundlePair, new Pair<>(angle1, angle2));
                    break;
                }
            }
            if (merged) {
                break;
            }
        }
        if (!merged) {
            List<Pair<Boolean, Bundle>> streetPart = new ArrayList<>();
            streetParts.add(streetPart);

            Pair<Boolean, Bundle> bundlePair = new Pair<>(afterIntersection, bundle);
            streetPart.add(bundlePair);
            anglesStreetPartMap.put(bundlePair, new Pair<>(angle1, angle2));
        }
    }

    /**
     * Applys shape fitting on bundle parts
     *
     * @param intersection,           the intersection we should apply shape fitting to.
     * @param filterOutToFarDistance, filter out bundle parts that have a distance of more than 100 meters to the first
     *                                drawn line. This ensures incorrectly merged bundleParts of not getting changed
     *                                to much
     * @return a list of line in the same order as the bundleParts.
     */
    private static List<Line2D> applyShapeFittingOnBundleParts(Intersection intersection, boolean filterOutToFarDistance) {
        double maxFilterOutDistance = 100;

        List<Line2D> shapes = new ArrayList<>();
        List<List<Subtrajectory>> bundleParts = intersection.getBundlePartsBundled();
        for (List<Subtrajectory> bundlePart : bundleParts) {
            Line2D line = LinearRegression.applyLinearRegressionForIntersection(bundlePart, intersection.getApproximateLocation(), 200);

            // @ToDo check if this makes a large difference
            if (filterOutToFarDistance) {
                for (int i = 0; i < bundlePart.size(); i++) {
                    Subtrajectory sub = bundlePart.get(i);
                    if (line.ptLineDist(sub.getLastPoint()) > maxFilterOutDistance) {
                        bundlePart.remove(i);
                        i--;
                    }
                }
                line = LinearRegression.applyLinearRegressionForIntersection(bundlePart, intersection.getApproximateLocation(), 200);
            }

            shapes.add(line);
        }
        return shapes;
    }

    /**
     * Removes the bundle parts that are from the straightest bundle.
     *
     * @param bundleParts,            the bundle parts that are not part of the straightest bundle
     * @param straightBundle,         the straightest bundle
     * @param approximateIntLocation, the approximate location
     */
    private static void removeBundlePartsFromStraightestBundle(List<List<Subtrajectory>> bundleParts,
                                                               Bundle straightBundle,
                                                               Point2D approximateIntLocation) {
        Preconditions.checkNotNull(straightBundle, "straightBundle is null");

        for (int i = 0; i < bundleParts.size(); i++) {
            int counter = 0;
            for (Subtrajectory sub : straightBundle.getNonReverseSubtrajectories()) {
                List<Subtrajectory> bundlePart = bundleParts.get(i);
                if (sub.overlapsOneItemInList(bundlePart)) {
                    counter++;
                }
            }

            // This is changed because it didn't seem to have any impact on the results and is a better general measure.
            if (counter > 0.5 * straightBundle.getNonReverseSubtrajectories().size()){
                bundleParts.remove(i);
                i--;
            }
        }
    }

    /**
     * Merges shape fitting lines that have the same direction.
     *
     * @param shapeFittingLines, the generated shape fitting lines
     * @param bundleParts,       the bundle parts
     */
    private static void mergeSimilarShapeFittingLines(List<Line2D> shapeFittingLines,
                                                      List<List<Subtrajectory>> bundleParts,
                                                      Intersection intersection) {

        double maxAlwaysMatchAngleDifference = 10;
        double alwaysMatchingMaxDistance = 15;
        double maxAngleDifference = 20;

        int matched = 0;
        int notMatched = 0;

        for (int i = 0; i < shapeFittingLines.size(); i++) {
            Line2D lineI = shapeFittingLines.get(i);
            for (int j = i + 1; j < shapeFittingLines.size(); j++) {
                Line2D lineJe = shapeFittingLines.get(j);
                double dir1 = GeometryUtil.getHeadingDirection(lineI);
                double dir2 = GeometryUtil.getHeadingDirection(lineJe);
                double distance = GeometryUtil.getDiscreteSegmentDistance(lineI, lineJe);

                // Here we check their heading angles are at most 20 degrees
                if (GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(dir1, dir2) < maxAngleDifference) {
                    if (GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(dir1, dir2) < maxAlwaysMatchAngleDifference) {
                        if (distance < alwaysMatchingMaxDistance) {
                            matched++;
                            System.out.println("Matched fitting lines: " + matched);
                            bundleParts.get(i).addAll(bundleParts.get(j));
                            shapeFittingLines.set(i,
                                    LinearRegression.applyLinearRegressionForIntersection(bundleParts.get(i),
                                            intersection.getApproximateLocation(), 200));
                            bundleParts.remove(j);
                            shapeFittingLines.remove(j);
                            j = 0;
                            continue;
                        } else {
                            notMatched++;
                            System.out.println("Due to distance not matched fitting lines: " + notMatched);
                        }
                    }

                    // Didn't change a thing...
                    // Improved Athens dataset at one point and for Chicago dataset
                    // Here we check that it's not the case that we are on the same side of the intersection
                    // that would mean we a merge intersection into a single street..
//                    double averageIAngle = 0.0;
//
//                    if (i >= bundleParts.size()){ // Overflow protection
//                        continue;
//                    }
//                    for (int a = 0; a < bundleParts.get(i).size(); a++) {
//                        Subtrajectory sub = bundleParts.get(i).get(a);
//                        averageIAngle = GeometryUtil.getNewAverageAngle(averageIAngle,
//                                GeometryUtil.getHeadingDirection(
//                                        new Line2D.Double(sub.getPoint(0), sub.getPoint(sub.numPoints() - 1))),
//                                a);
//                    }
//                    double averageJAngle = 0.0;
//                    for (int a = 0; a < bundleParts.get(j).size(); a++) {
//                        Subtrajectory sub = bundleParts.get(j).get(a);
//                        averageJAngle = GeometryUtil.getNewAverageAngle(averageJAngle,
//                                GeometryUtil.getHeadingDirection(
//                                        new Line2D.Double(sub.getPoint(0), sub.getPoint(sub.numPoints() - 1))),
//                                a);
//                    }
//
//                    double headingDiff = GeometryUtil.getAbsoluteAngleDifference(averageIAngle, averageJAngle);
//                    if (headingDiff < 180 - maxAngleDifference || headingDiff > 180 + maxAngleDifference) {
//                        System.out.println(headingDiff);
//                        continue;
//                    }
//
//                    bundleParts.get(i).addAll(bundleParts.get(j));
//                    shapeFittingLines.set(i,
//                            LinearRegression.applyLinearRegressionForIntersection(bundleParts.get(i),
//                                    intersection.getApproximateLocation(), 200));
//                    bundleParts.remove(j);
//                    shapeFittingLines.remove(j);
//                    j = 0;
                }
            }
        }
    }


    /**
     * Calculates a more precise intersection location based on the shape fitting and straightest bundle.
     *
     * @param intersection, the intersection object.
     * @return the exact intersection location
     */
    private static Point2D calculateIntersectionLocation(Intersection intersection, Bundle straightBundle) {
        if (straightBundle != null) {
            return calculateIntersectionLocationForStraightBundle(intersection, straightBundle);
        } else {
            return calculateIntersectionLocationWithoutStraightBundle(intersection);
        }
    }


    /**
     * Calculates a more precise intersection location based on the shape fitting and straightest bundle.
     *
     * @param intersection, the intersection object.
     * @return the exact intersection location
     */
    private static Point2D calculateIntersectionLocationForStraightBundle(Intersection intersection, Bundle straightBundle) {
        Representative straightRep = straightBundle.getRepresentative();
        Point2D approxLocation = intersection.getApproximateLocation();
        List<Pair<Point2D, Double>> weightedPoints = new ArrayList<>();
        List<Line2D> shapeFittingLines = intersection.getShapeFittingLines();
        List<List<Subtrajectory>> bundlesParts = intersection.getBundlePartsBundled();

        double totalScore = 0.0;

        for (int i = 0; i < shapeFittingLines.size(); i++) {
            Line2D line = shapeFittingLines.get(i);
            double headingLine = GeometryUtil.getHeadingDirection(line);

            Pair<Point2D, Double> pair = null;
            for (int j = 0; j < straightRep.numEdges(); j++) {
                Line2D edge = straightRep.getEdge(j);
                if (edge.intersectsLine(line)) {

                    Point2D intersect = GeometryUtil.intersectionPoint(edge, line);
                    if (intersect == null || intersect.distance(approxLocation) > 100) {
                        continue;
                    }

                    if (pair == null || pair.getFirst().distance(approxLocation) > intersect.distance(approxLocation)) {
                        double headingEdge = GeometryUtil.getHeadingDirection(edge);
                        double headingDifference = GeometryUtil.
                                getAngleDifferenceForPossiblyReverseTrajectories(headingEdge, headingLine);
                        if (headingDifference < 40) {
                            continue;
                        }

                        double sizeWeight = Math.pow(bundlesParts.get(i).size(), 4);
                        double angleWeight = Math.pow(headingDifference, 1.5) * sizeWeight;
                        pair = new Pair<>(intersect, angleWeight);
                    }
                }
            }
            weightedPoints.add(pair);
            if (pair != null) {
                totalScore += pair.getSecond();
            }
        }

        intersection.setIntersectionLocationScore(totalScore);
        return calculateWeightedAverage(weightedPoints, intersection);
    }


    /**
     * Calculates a more precise intersection location based on the shape fitting and straightest bundle.
     *
     * @param intersection, the intersection object.
     * @return the exact intersection location
     */
    private static Point2D calculateIntersectionLocationWithoutStraightBundle(Intersection intersection) {
        Point2D approxLocation = intersection.getApproximateLocation();
        List<Pair<Point2D, Double>> weightedPoints = new ArrayList<>();
        List<Line2D> shapeFittingLines = intersection.getShapeFittingLines();
        List<List<Subtrajectory>> bundlesParts = intersection.getBundlePartsBundled();
        double totalScore = 0.0;

        if (shapeFittingLines.size() > 6) {
            return intersection.getApproximateLocation();
        }

        for (int i = 0; i < shapeFittingLines.size(); i++) {
            Line2D line1 = shapeFittingLines.get(i);
            for (int j = i + 1; j < shapeFittingLines.size(); j++) {
                Line2D line2 = shapeFittingLines.get(j);
                if (line1.intersectsLine(line2)) {
                    Point2D intersect = GeometryUtil.intersectionPoint(line1, line2);
                    double headingDifference = GeometryUtil.getAngleDifferenceForPossiblyReverseTrajectories(
                            GeometryUtil.getHeadingDirection(line1),
                            GeometryUtil.getHeadingDirection(line2));
                    if (headingDifference < 40) {
                        continue;
                    }

                    double sizeWeight = Math.pow(bundlesParts.get(i).size(), 2) *
                            Math.pow(bundlesParts.get(j).size(), 2);
                    double angleWeight = Math.pow(headingDifference, 1.5) + sizeWeight;

                    totalScore += angleWeight;

                    Pair<Point2D, Double> pair = new Pair<>(intersect, angleWeight);
                    weightedPoints.add(pair);
                }
            }
        }

        intersection.setIntersectionLocationScore(totalScore);
        return calculateWeightedAverage(weightedPoints, intersection);
    }

    /**
     * Calculate the weighted average of all points
     *
     * @param weightedPoints, every point combined with it's weight.
     * @param intersection,   the intersection
     * @return the average weighted point.
     */
    private static Point2D calculateWeightedAverage(List<Pair<Point2D, Double>> weightedPoints,
                                                    Intersection intersection) {
        for (Pair<Point2D, Double> pair1 : weightedPoints) {
            if (pair1 == null) {
                continue;
            }
            for (Pair<Point2D, Double> pair2 : weightedPoints) {
                if (pair2 == null) {
                    continue;
                }
                if (pair1.equals(pair2)) {
                    continue;
                }

                if (pair1.getFirst().distance(pair2.getFirst()) > 50) {
                    return intersection.getApproximateLocation();
                }
            }
        }


        double averageX = 0.0;
        double averageY = 0.0;
        double totalWeights = 0.0;

        for (Pair<Point2D, Double> pair : weightedPoints) {
            if (pair == null) {
                continue;
            }

            Point2D point2D = pair.getFirst();
            Double weight = pair.getSecond();

            averageX += point2D.getX() * pair.getSecond();
            averageY += point2D.getY() * pair.getSecond();
            totalWeights += weight;
        }
        if (averageX == 0.0) {
            return intersection.getApproximateLocation();
        }

        return new Point2D.Double(averageX / totalWeights, averageY / totalWeights);
    }
}
