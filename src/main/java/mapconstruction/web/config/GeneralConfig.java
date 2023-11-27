package mapconstruction.web.config;

import com.fasterxml.jackson.annotation.JsonProperty;

import static java.lang.String.format;

public class GeneralConfig {
    private int webPagePort;
    private boolean openWebPageOnStart;

    private String googleMapsApiKey;
    private String datasetDirectory;
    private String savedStatesDirectory;
    private String outputDirectory;
    private String benchmarkDirectory;

    private int numOfProcesses;

    public GeneralConfig() {
//        Specific
        numOfProcesses = 4;
    }

    public int getWebPagePort() {
        return webPagePort;
    }

    public void setWebPagePort(int webPagePort) {
        this.webPagePort = webPagePort;
    }

    public boolean isOpenWebPageOnStart() {
        return openWebPageOnStart;
    }

    public void setOpenWebPageOnStart(boolean openWebPageOnStart) {
        this.openWebPageOnStart = openWebPageOnStart;
    }

    @JsonProperty
    public String getGoogleMapsApiKey() {
        return googleMapsApiKey;
    }

    public void setGoogleMapsApiKey(String googleMapsApiKey) {
        this.googleMapsApiKey = googleMapsApiKey;
    }

    public String getDatasetDirectory() {
        return datasetDirectory;
    }

    public void setDatasetDirectory(String datasetDirectory) {
        this.datasetDirectory = datasetDirectory;
    }

    public String getSavedStatesDirectory() {
        return savedStatesDirectory;
    }

    public void setSavedStatesDirectory(String savedStatesDirectory) {
        this.savedStatesDirectory = savedStatesDirectory;
    }

    public String getOutputDirectory() {
        return outputDirectory;
    }

    public void setOutputDirectory(String outputDirectory) {
        this.outputDirectory = outputDirectory;
    }

    public String getBenchmarkDirectory() {
        return benchmarkDirectory;
    }

    public void setBenchmarkDirectory(String benchmarkDirectory) {
        this.benchmarkDirectory = benchmarkDirectory;
    }

    public int getNumOfProcesses() {
        return numOfProcesses;
    }

    public void setNumOfProcesses(int numOfProcesses) {
        this.numOfProcesses = numOfProcesses;
    }

    @Override
    public String toString() {
        return format("1. webPagePort: %s\n", webPagePort) +
               format("2. openWebPageOnStart: %s\n", openWebPageOnStart) +
               format("3. googleMapsApiKey: %s\n", googleMapsApiKey) +
               format("4. datasetDirectory: %s\n", datasetDirectory) +
               format("5. savedStatesDirectory: %s\n", savedStatesDirectory) +
               format("6. outputDirectory: %s\n", outputDirectory) +
               format("7. benchmarkDirectory: %s\n", benchmarkDirectory) +
               format("8. numberOfProcesses: %s\n", numOfProcesses);
    }

}
