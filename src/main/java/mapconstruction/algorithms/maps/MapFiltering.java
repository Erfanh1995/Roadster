package mapconstruction.algorithms.maps;

import mapconstruction.algorithms.maps.containers.BundleStreet;
import mapconstruction.algorithms.maps.mapping.ConnectionVertex;
import mapconstruction.algorithms.maps.mapping.RoadMap;
import mapconstruction.algorithms.maps.mapping.RoadSection;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.Collectors;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * This class is responsible for doing some final make up changes to the final road map.
 * Remove small edges for example.
 *
 * @author Jorricks
 */
public class MapFiltering {
    /**
     * We check whether there are roadEdges, D, that are between intersection A and B such that there is a roadEdge, C,
     * that is also between A and B and covers the same road, that is one completely merged part with D. If so, that
     * means that if we added D first, and added C later, C would never have been added.
     */
    public static void removeRoadEdgesBetweenIntersectionsThatCanBeMerged(RoadMap roadMap) {
        if (STORAGE.getDatasetConfig().isWalkingDataset()) {
            removeRoadEdgesBetweenIntersectionsThatCanBeMergedWalking(roadMap);
        } else {
            removeRoadEdgesBetweenIntersectionsThatCanBeMergedCar(roadMap);
        }
    }

    private static void removeRoadEdgesBetweenIntersectionsThatCanBeMergedWalking(RoadMap roadMap) {
        List<RoadSection> roadSections = new ArrayList<>(roadMap.getPresentRoadSections());
        Collections.sort(roadSections, Comparator.comparingInt(RoadSection::getUid));

        for (int i = 0; i < roadSections.size(); i++) {
            RoadSection roadSection1 = roadSections.get(i);
            Point2D firstPoint1 = roadSection1.getPointList().get(0);
            Point2D lastPoint1 = roadSection1.getPointList().get(roadSection1.getPointList().size() - 1);
            if (roadSection1.getPointList().size() != 2) {
                continue;
            }

            for (int j = i + 1; j < roadSections.size(); j++) {
                RoadSection roadSection2 = roadSections.get(j);
                Point2D firstPoint2 = roadSection2.getPointList().get(0);
                Point2D lastPoint2 = roadSection2.getPointList().get(roadSection2.getPointList().size() - 1);
                if ((roadSection1.getStartVertex() == roadSection2.getStartVertex() &&
                        roadSection1.getEndVertex() == roadSection2.getEndVertex() &&
                        firstPoint1.distance(firstPoint2) < 1 && lastPoint1.distance(lastPoint2) < 1) ||
                        (roadSection1.getStartVertex() == roadSection2.getEndVertex() &&
                                roadSection1.getEndVertex() == roadSection2.getStartVertex() &&
                                firstPoint1.distance(lastPoint2) < 1 && lastPoint1.distance(firstPoint2) < 1)) {
                    if (GeometryUtil.getContinuousLength(roadSection1.getPointList()) * 1.5 >
                            GeometryUtil.getContinuousLength(roadSection2.getPointList()) &&
                            GeometryUtil.getContinuousLength(roadSection1.getPointList()) <
                                    GeometryUtil.getContinuousLength(roadSection2.getPointList())) {
                        roadMap.removeRoadSection(roadSection1);
                    }
                }
            }
        }

    }

    private static void removeRoadEdgesBetweenIntersectionsThatCanBeMergedCar(RoadMap roadMap) {
        double maxDistance = 25.0;
        List<RoadSection> roadSections = new ArrayList<>(roadMap.getPresentRoadSections());
        Collections.sort(roadSections, Comparator.comparingInt(RoadSection::getUid));
        Collections.reverse(roadSections);

        for (int i = 0; i < roadSections.size(); i++) {
            RoadSection newerRoadSection = roadSections.get(i);
            Trajectory newRep = newerRoadSection.getTrajectory();
            double newCL = GeometryUtil.getContinuousLength(newerRoadSection.getPointList());
            Point2D nPoint1 = newerRoadSection.getPointList().get(0);
            Point2D nPoint2 = newerRoadSection.getPointList().get(newerRoadSection.getPointList().size() - 1);

//            List<RoadSection> roadSectionCopy = new ArrayList<>(roadSections);
            for (int j = i + 1; j < roadSections.size(); j++) {
                boolean isEncapsulated = true;

                RoadSection oldRoadSection = roadSections.get(j);
                Point2D oPoint1 = oldRoadSection.getPointList().get(0);
                Point2D oPoint2 = oldRoadSection.getPointList().get(oldRoadSection.getPointList().size() - 1);

                // Here we check that they have at least one of the same ending points.
                if (!(oPoint1 == nPoint1 || oPoint2 == nPoint2 || oPoint1 == nPoint2 || oPoint2 == nPoint1)) {
                    isEncapsulated = false;
                }

                // Now we check that the continuous length doesn't differ to much.
                double oldCL = GeometryUtil.getContinuousLength(oldRoadSection.getPointList());
                if (oldCL > 2.0 * newCL + 100 || newCL > 2.0 * oldCL + 100) {
                    isEncapsulated = false;
                }

                // Here we check for each point of, (j), the earliest, is completely encapsulated within 50 meters
                // of (i), the oldest.
                for (Point2D point2D : oldRoadSection.getPointList()) {
                    double index = GeometryUtil.getIndexOfTrajectoryClosestToPoint(newRep, point2D);
                    Point2D foundPoint = GeometryUtil.getTrajectoryDecimalPoint(newRep, index);

                    if (foundPoint.distance(point2D) > maxDistance) {
                        isEncapsulated = false;
                        break;
                    }
                }

                if (isEncapsulated) {
//                    roadMap.removeRoadSection(oldRoadSection);
//                    roadSections.remove(oldRoadSection);
                    RoadSection removedRoad = newerRoadSection;

                    roadMap.removeRoadSection(removedRoad);
                    roadSections.remove(removedRoad);
                    i--;

                    List<ConnectionVertex> allCVs = roadMap.getConnectionVertices();
                    if (allCVs.contains(removedRoad.getStartVertex()) && allCVs.contains(removedRoad.getEndVertex())) {
                        boolean stillConnected = roadMap.checkIfTwoPointsAreConnected(
                                removedRoad.getStartVertex(), removedRoad.getEndVertex(),
                                removedRoad.getContinuousLength() * 3, new HashSet<>());
                        if (!stillConnected) {
                            roadMap.forceAddRoadSection(removedRoad);
                        }
                    }
                    break;
                }
            }
        }
    }

    /**
     * At the moment we add a roadSection between two already present roadSections because there is proved that they
     * should be connected, then we check whether there was already a roadSection which less or more represented this
     * part but was not connected by the same roadSection.
     *
     * @param roadSection the roadSection we just added
     * @param roadMap     the roadMap
     */
    public static void removeRoadEdgesThatAreOneSidedThatCanBeMerged(RoadSection roadSection, RoadMap roadMap) {
        double maxDistance = 25.0;
        Set<RoadSection> otherRoadSections = new HashSet<>();

        for (RoadSection otherRoadSection : roadMap.getRoadSectionsForConnectionVertex(roadSection.getStartVertex())) {
            ConnectionVertex orsOtherVertex = otherRoadSection.getOppositeVertex(roadSection.getStartVertex());
            if (roadMap.getRoadSectionsForConnectionVertex(orsOtherVertex).size() == 1) {
                otherRoadSections.add(otherRoadSection);
            }
        }

        for (RoadSection otherRoadSection : roadMap.getRoadSectionsForConnectionVertex(roadSection.getEndVertex())) {
            ConnectionVertex orsOtherVertex = otherRoadSection.getOppositeVertex(roadSection.getEndVertex());
            if (roadMap.getRoadSectionsForConnectionVertex(orsOtherVertex).size() == 1) {
                otherRoadSections.add(otherRoadSection);
            }
        }

        Trajectory rep = roadSection.getTrajectory();
        double newCL = GeometryUtil.getContinuousLength(roadSection.getPointList());

        for (RoadSection otherRoadSection : otherRoadSections) {

            // Now we check that the continuous length doesn't differ to much.
            double oldCL = GeometryUtil.getContinuousLength(otherRoadSection.getPointList());
            if (oldCL > 2.0 * newCL + 100 || newCL > 2.0 * oldCL + 100) {
                continue;
            }

            // Here we check for each point of, (j), the earliest, is completely encapsulated within 50 meters
            // of (i), the oldest.
            boolean isEncapsulated = true;
            for (Point2D point2D : otherRoadSection.getPointList()) {
                double index = GeometryUtil.getIndexOfTrajectoryClosestToPoint(rep, point2D);
                Point2D foundPoint = GeometryUtil.getTrajectoryDecimalPoint(rep, index);

                if (foundPoint.distance(point2D) > maxDistance) {
                    isEncapsulated = false;
                    break;
                }
            }

            if (isEncapsulated) {
                roadMap.removeRoadSection(otherRoadSection);
                break;
            }
        }
    }

    /**
     * Removes small edges that are only connected to one other roadSection.
     *
     * @param roadMap the roadMap
     */
    public static void removeSmallOneSidedEdges(RoadMap roadMap) {
        for (RoadSection roadSection : roadMap.getPresentRoadSections()) {
            if (STORAGE.getDatasetConfig().isWalkingDataset() && GeometryUtil.getContinuousLength(roadSection.getPointList()) > 20) {
                continue;
            }
            if (!STORAGE.getDatasetConfig().isWalkingDataset() && GeometryUtil.getContinuousLength(roadSection.getPointList()) > 50) {
                continue;
            }

            int n1 = roadMap.getRoadSectionsForConnectionVertex(roadSection.getStartVertex()).size();
            int n2 = roadMap.getRoadSectionsForConnectionVertex(roadSection.getEndVertex()).size();

            if ((n1 > 1 && n2 == 1) || (n2 > 1 && n1 == 1)) {
                roadMap.removeRoadSection(roadSection);
            }
        }
    }

    /**
     * Given a distance and the roadMap, we connect all lonely endings (a ConnectionVertex that has only one RoadSection
     * connected to it). Then we connect all lonely endings.
     *
     * @param roadMap  the final roadMap
     * @param distance the distance used.
     */
    public static void connectSingleEndings(RoadMap roadMap, double distance) {
        // First we start by finding all lonely ConnectionVertices, which mean they only have one RoadSection connected.
        List<ConnectionVertex> loners = new ArrayList<>();
        for (ConnectionVertex vertex : roadMap.getConnectionVertices()) {
            if (roadMap.getRoadSectionsForConnectionVertex(vertex).size() == 1) {
                RoadSection roadSection = roadMap.getRoadSectionsForConnectionVertex(vertex).iterator().next();
                if (roadSection.getStartVertex() != roadSection.getEndVertex()) {
                    loners.add(vertex);
                }
            }
        }

        // Now for all loners, we check whether there are any other loners(including itself) within 50 meters distance.
        HashMap<ConnectionVertex, Set<ConnectionVertex>> mergeCombinations = new HashMap<>();
        for (ConnectionVertex currentVertex : loners) {
            HashSet<ConnectionVertex> mergeVertices = new HashSet<>();
            for (ConnectionVertex otherVertex : loners) {
                if (currentVertex.getLocation().distance(otherVertex.getLocation()) < distance) {
                    mergeVertices.add(otherVertex);
                }
            }
            mergeCombinations.put(currentVertex, mergeVertices);
        }

        // For each loner, a, and the loners in their neighbourhood, b, we create an edge from connectionvertices in b
        // to the average of b's locations.
        for (Map.Entry<ConnectionVertex, Set<ConnectionVertex>> entry : mergeCombinations.entrySet()) {
            ConnectionVertex currentVertex = entry.getKey();
            Set<ConnectionVertex> otherVertices = entry.getValue();

            List<Point2D> points = otherVertices.stream().map(ConnectionVertex::getLocation).collect(Collectors.toList());
            Point2D averagePoint = GeometryUtil.getAverage(new ArrayList<>(points));
            ConnectionVertex averageCV = roadMap.getConnectionVertex(averagePoint);

            BundleStreet bundleStreet = roadMap.getRoadSectionsForConnectionVertex(currentVertex).iterator().next().getDrawnBundleStreet();

            // Here we create the needed roadSections but we also remove the current vertices from the others.
            for (ConnectionVertex otherVertex : otherVertices) {
                if (otherVertex == currentVertex) {
                    continue;
                }
                roadMap.forceAddRoadSection(Arrays.asList(averagePoint, otherVertex.getLocation()),
                        averageCV, otherVertex, bundleStreet);
                Set<ConnectionVertex> connectionOtherVerticesForOtherVertex = mergeCombinations.get(otherVertex);
                for (ConnectionVertex vertexInOtherVertices : otherVertices) {
                    if (!otherVertex.equals(vertexInOtherVertices)) {
                        connectionOtherVerticesForOtherVertex.remove(vertexInOtherVertices);
                    }
                }
                mergeCombinations.put(otherVertex, connectionOtherVerticesForOtherVertex);
            }
        }
    }
}
