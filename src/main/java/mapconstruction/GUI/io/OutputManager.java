package mapconstruction.GUI.io;

import mapconstruction.algorithms.maps.mapping.RoadMap;
import mapconstruction.algorithms.maps.mapping.RoadSection;
import mapconstruction.algorithms.maps.network.MapEdge;
import mapconstruction.algorithms.maps.network.MapVertex;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;

import java.awt.geom.Point2D;
import java.io.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;

public class OutputManager {

    private String path;
    private int precision;

    private final static DateFormat DATES = new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss");

    public OutputManager(String path) {
        this(path, 5);
    }

    public OutputManager(String path, int precision) {
        this.path = path;
        this.precision = precision;
    }

    public void saveRoadMap(String name, Date date, RoadMap network) {
        String subPath = name + '/' + DATES.format(date);
        createDirectoryIfAbsent(subPath);

        Map<Point2D, MapVertex> vMap = new HashMap<>();
        Set<MapVertex> vertices = new LinkedHashSet<>();
        Set<MapEdge> edges = new LinkedHashSet<>();

        for (RoadSection section : network.getPresentRoadSections()) {
            List<Point2D> points = section.getPointList();
            MapVertex vj = null;
            for (Point2D pi : points) {
                MapVertex vi = vMap.get(pi);
                if (vi == null) {
                    vi = new MapVertex(pi.getX(), pi.getY(), null, false);
                    vMap.put(pi, vi);
                    vertices.add(vi);
                }

                if (vj != null) {
                    edges.add(new MapEdge(vj, vi, null));
                }

                vj = vi;
            }
        }

        try {
            makeVertexFile(subPath, String.format("%s_vertices.txt", name), vertices);
            makeEdgeFile(subPath, String.format("%s_edges.txt", name), edges);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void makeVertexFile(String subdirectory, String filename, Set<MapVertex> vertices) throws IOException {
        File output = new File(this.path + '/' + subdirectory + '/' + filename).getAbsoluteFile();
        BufferedWriter writer = new BufferedWriter(new FileWriter(output.toString()));
        for (MapVertex v : vertices) {
            Point2D loc = v.getLocation();
            writer.write(String.format(Locale.US, "%d,%." + precision + "f,%." + precision + "f\n", v.getId(), loc.getX(), loc.getY()));
        }
        writer.close();
    }

    private void makeEdgeFile(String subdirectory, String filename, Set<MapEdge> edges) throws IOException {
        File output = new File(this.path + '/' + subdirectory + '/' + filename).getAbsoluteFile();
        BufferedWriter writer = new BufferedWriter(new FileWriter(output.toString()));
        int id = 0;
        for (MapEdge e : edges) {
            writer.write(String.format(Locale.US,"%d,%d,%d\n", id ++, e.getV1().getId(), e.getV2().getId()));
        }
        writer.close();
    }

    private void createDirectoryIfAbsent(String subdirectory) {
        File dir = new File(this.path + '/' + subdirectory + '/');
        boolean mkdirs = dir.mkdirs();
        if (mkdirs) {
            Log.log(LogLevel.INFO, "BenchmarkManager", "Created benchmark directory: %s", dir.toString());
        }
    }

}
