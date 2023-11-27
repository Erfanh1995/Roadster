package mapconstruction.GUI.io;

import mapconstruction.algorithms.maps.network.FilteredRoadNetwork;
import mapconstruction.algorithms.maps.network.MapEdge;
import mapconstruction.algorithms.maps.network.MapVertex;
import mapconstruction.algorithms.maps.network.RoadNetwork;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class DatasetExplorer {
    private String path;

    public DatasetExplorer(String path) {
        this.path = path;
    }

    public List<String> getAllDatasets() {
        List<String> allDatasets = new ArrayList<>();

//        File parent = new File(System.getProperty("user.dir") + this.path);
        File parent = new File(this.path);
        Log.log(LogLevel.INFO, "DatasetExplorer", "Getting all datasets in folder: %s", parent.toPath());
        if (parent.isDirectory()) {
            File[] children = parent.listFiles(File::isDirectory);
            if (children != null) {
                for (File child : children) {
                    if (checkIfFolderHasConfigFile(child)) {
                        allDatasets.add(child.getName());
                    }
                }
            }
        } else {
            boolean mkdirs = parent.mkdirs();
            Log.log(LogLevel.INFO, "DatasetExplorer", "Created data directory: %b", mkdirs);
        }
        return allDatasets;
    }

    private boolean checkIfFolderHasConfigFile(File dir) {
        File[] children = dir.listFiles(file -> file.getName().toLowerCase().endsWith(".yml"));
        if (children != null) {
            return children.length > 0;
        }
        return false;
    }

    public File[] getAllFilesInDataset(String relative_dir) {
//        File datasetFolder = new File(System.getProperty("user.dir") + this.path + '/' + relative_dir);
        File datasetFolder = new File(this.path + '/' + relative_dir);
        return datasetFolder.listFiles(file -> file.getName().toLowerCase().endsWith(".txt"));
    }

    public File getConfigFileInDataset(String relative_dir) {
//        File datasetFolder = new File(System.getProperty("user.dir") + this.path + '/' + relative_dir);
        File datasetFolder = new File(this.path + '/' + relative_dir);
        File[] config_files = datasetFolder.listFiles(file -> file.getName().toLowerCase().endsWith(".yml"));
        assert config_files != null;
        return config_files[0];
    }

    public RoadNetwork getGroundTruth(String relative_dir) throws FileNotFoundException {
//        File verificationFolder = new File(System.getProperty("user.dir") + this.path +
//                "/" + relative_dir + "/verification");
        File verificationFolder = new File(this.path + "/" + relative_dir + "/verification");

        if (!verificationFolder.isDirectory()) {
            throw new FileNotFoundException("Directory verification does not exist.");
        }

        File[] verticesFileList = verificationFolder.listFiles(file -> file.getName().contains("vertices"));
        if (verticesFileList == null || verticesFileList.length < 1) {
            throw new FileNotFoundException("Error");
        }
        File verticesFile = verticesFileList[0];

        File[] edgesFileList = verificationFolder.listFiles(file -> file.getName().contains("edges"));
        if (edgesFileList == null || edgesFileList.length < 1) {
            throw new FileNotFoundException("Error");
        }
        File edgesFile = edgesFileList[0];

        File validFile = null;
        File[] validFileList = verificationFolder.listFiles(file -> file.getName().contains("valid"));
        if (validFileList != null && validFileList.length > 0) {
            validFile = validFileList[0];
        }

        Map<Integer, MapVertex> vertices = new HashMap<>(); // identifier to vertex
        FilteredRoadNetwork backgroundMap = new FilteredRoadNetwork(true);
        try {
            // process vertices.
            List<String> vertexLines = Files.readAllLines(verticesFile.toPath());
            for (String line : vertexLines) {
                String[] elem = line.split(",");
                vertices.put(Integer.parseInt(elem[0]), new MapVertex(Double.parseDouble(elem[1]), Double.parseDouble(elem[2]), null, false));
            }
            // process valid edges if present.
            Set<Integer> validEdges = new HashSet<>();
            if (validFile != null) {
                List<String> validLines = Files.readAllLines(validFile.toPath());
                for (String line : validLines) {
                    for (String elem : line.split(",")) {
                        validEdges.add(Integer.parseInt(elem));
                    }
                }
            }

            // process edges
            List<String> edgeLines = Files.readAllLines(edgesFile.toPath());
            for (String line : edgeLines) {
                String[] elem = line.split(",");
                if (validFile != null) {
                    backgroundMap.addEdge(new MapEdge(
                            Integer.parseInt(elem[0]),
                            vertices.get(Integer.parseInt(elem[1])),
                            vertices.get(Integer.parseInt(elem[2]))),
                            validEdges.contains(Integer.parseInt(elem[0])));
                } else {
                    backgroundMap.addEdge(new MapEdge(
                            Integer.parseInt(elem[0]),
                            vertices.get(Integer.parseInt(elem[1])),
                            vertices.get(Integer.parseInt(elem[2]))));
                }
            }
        } catch (IOException ex) {
            Logger.getLogger(DatasetExplorer.class.getName()).log(Level.SEVERE, null, ex);
        }

        return backgroundMap;
    }

    /**
     * Get a file instance for a dataset file containing valid edge pairs
     * @param relative_dir relative Dataset directory
     * @return The file to store valid edge pairs
     * @throws IOException Generic IO Exception
     */
    public File getValidEdgePairs(String relative_dir) throws IOException {
        File verificationFolder = new File(this.path + "/" + relative_dir + "/verification");

        if (!verificationFolder.isDirectory()) {
            throw new FileNotFoundException("Directory verification does not exist.");
        }

        File validEdgePairs = new File(verificationFolder, relative_dir + "_valid_edge_pairs_co.txt");
        validEdgePairs.createNewFile();

        return validEdgePairs;
    }
}
