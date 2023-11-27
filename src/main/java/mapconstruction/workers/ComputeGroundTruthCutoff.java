package mapconstruction.workers;

import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;
import com.google.common.collect.MultimapBuilder;
import mapconstruction.GUI.io.DatasetExplorer;
import mapconstruction.algorithms.distance.RTree;
import mapconstruction.algorithms.maps.network.MapEdge;
import mapconstruction.algorithms.maps.network.MapVertex;
import mapconstruction.algorithms.maps.network.RoadNetwork;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.web.Controller;
import mapconstruction.web.config.GeneralConfig;

import java.awt.geom.Line2D;
import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * @author Jorren
 */
public class ComputeGroundTruthCutoff extends AbortableAlgorithmWorker<File, Void> {

    double epsilon;

    private DatasetExplorer datasetExplorer;

    public ComputeGroundTruthCutoff(double epsilon, GeneralConfig config) {
        this.epsilon = epsilon;
        this.datasetExplorer = new DatasetExplorer(config.getDatasetDirectory());
    }

    @Override
    protected File doInBackground() throws Exception {

        List<Trajectory> trajectories = STORAGE.getTrajectories();

        System.out.println("Constructing an RTree for all line segments in Trajectory set");
        Map<Line2D, Line2D> temp = new HashMap<>();
        trajectories.forEach(t -> t.edges().forEach(e -> temp.put(e, e)));
        RTree<Line2D, Line2D> map = new RTree<>(10, temp);

        System.out.println("Filtering ground truth edges based on input trajectories");
        RoadNetwork backgroundMap = datasetExplorer.getGroundTruth(STORAGE.getDatasetConfig().getPath());
        Multimap<MapVertex, MapEdge> validEdges = ArrayListMultimap.create();
//        Map<MapVertex, MapEdge> validEdges = new HashMap<>();
        for (MapEdge edge : backgroundMap.edges()) {
            MapVertex v1 = edge.getV1();
            MapVertex v2 = edge.getV2();

            Set<Line2D> candidates = map.windowQuery(
                    Math.min(v1.getX(), v2.getX()) - epsilon,
                    Math.min(v1.getY(), v2.getY()) - epsilon,
                    Math.max(v1.getX(), v2.getX()) + epsilon,
                    Math.max(v1.getY(), v2.getY()) + epsilon);

            for (Line2D candidate : candidates) {
                if (candidate.ptLineDist(v1.getX(), v1.getY()) <= epsilon || candidate.ptLineDist(v2.getX(), v2.getY()) <= epsilon) {
                    validEdges.put(edge.getV1(), edge);
                    break;
                }
            }
        }

        System.out.println("Matching candidates to generate valid edge pairs");
        File output = datasetExplorer.getValidEdgePairs(STORAGE.getDatasetConfig().getPath());
        try (FileWriter writer = new FileWriter(output)) {
            for (MapEdge fromEdge : validEdges.values()) {
                for (MapEdge toEdge : validEdges.get(fromEdge.getV2())) {
                    writer.write(String.format("%d,%d\n", fromEdge.getId(), toEdge.getId()));
                }
            }
        }
        System.out.println("Saved state to " + output);

        return output;
    }

}
