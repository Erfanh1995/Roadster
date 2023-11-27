package mapconstruction.GUI.io;

import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class SavedStatesIndexer {
    private String path;

    public SavedStatesIndexer(String path) {
        this.path = path;
    }

    public List<String> getAllSavedStates() {
        List<String> allDatasets = new ArrayList<>();

        createDirectoryIfAbsent();
//        File parent = new File(System.getProperty("user.dir") + this.path);
        File parent = new File(this.path);
        if (this.path == null){
            try {
                throw new Exception("The saved state path is null. Please check your general-config.yml!");
            } catch (Exception e) {
                e.printStackTrace();
            }
            System.exit(0);
        }

        Log.log(LogLevel.INFO, "SaveStatesExplorer", "Getting all saved states in folder: %s", parent.toPath());

        File[] allSavedStates = parent.listFiles(file -> file.getName().endsWith(".savst"));
        for (File file : Objects.requireNonNull(allSavedStates)) {
            allDatasets.add(file.getName());
        }

        return allDatasets;
    }

    public File getSavedState(String filename) throws FileNotFoundException {
//        return new File(System.getProperty("user.dir") + this.path + '/' + filename);
        File file = new File(this.path + '/' + filename);
        if (!file.exists()) {
            throw new FileNotFoundException("Not found");
        }
        return file;
    }

    public String getNewSavedStateFilePath(String filename) {
        createDirectoryIfAbsent();
        return this.path + '/' + filename;
    }

    private void createDirectoryIfAbsent() {
//        File dir = new File(System.getProperty("user.dir") + this.path + '/');
        File dir = new File(this.path + '/');
        boolean mkdirs = dir.mkdirs();
        if (mkdirs) {
            Log.log(LogLevel.INFO, "SaveStatesExplorer", "Created savedStates directory: %s", dir.toString());
        }

    }
}
