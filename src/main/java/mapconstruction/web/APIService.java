package mapconstruction.web;

import com.fasterxml.jackson.annotation.JsonProperty;
import mapconstruction.GUI.io.DatasetExplorer;
import mapconstruction.GUI.io.SavedStatesIndexer;
import mapconstruction.algorithms.maps.ComputeRoadNetwork;
import mapconstruction.algorithms.maps.network.RoadNetwork;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.web.config.DatasetConfig;
import mapconstruction.web.config.GeneralConfig;
import mapconstruction.web.config.YamlConfigRunner;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import java.io.File;
import java.io.FileNotFoundException;
import java.sql.Timestamp;
import java.util.*;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * This class is responsible for all the logic of the API.
 * The class is instantiated by RestServerApp.
 *
 * @author Jorrick Sleijster
 * @since 18/09/2018
 */
@Path("/api")
public class APIService {
    private Controller controller;
    private GeneralConfig config;
    private DatasetExplorer datasetExplorer;
    private SavedStatesIndexer savedStatesIndexer;

    APIService(Controller controller, GeneralConfig config) {
        this.controller = controller;
        this.config = config;
        this.datasetExplorer = new DatasetExplorer(config.getDatasetDirectory());
        this.savedStatesIndexer = new SavedStatesIndexer(config.getSavedStatesDirectory());
    }

    /**
     * Creates a default response.
     *
     * @param responseMessage message to send with the response.
     * @return response with responseMessage.
     */
    private static Response createDefaultResponse(Object responseMessage) {
        return new Response(responseMessage, false, "", STORAGE.getProgressAlgorithm());
    }

    /**
     * Create a default error response
     *
     * @param errorMessage error message to send with the response.
     * @return Reponse with false and error message.
     */
    private static Response createErrorResponse(String errorMessage) {
        return new Response(false, true, errorMessage, STORAGE.getProgressAlgorithm());
    }

    /**
     * Ping the server to get an update on if an algorithm is running.
     *
     * @return DefaultResponse with true.
     */
    @GET
    @Path("/ping")
    @Produces(MediaType.APPLICATION_JSON)
    public Response ping() {
        return createDefaultResponse(true);
    }

    /**
     * Getting all the constants specified in the general-config file.
     * Mainly important because of the Google Mapa API key.
     *
     * @return DefaultResponse with all constants from the config.
     */
    @GET
    @Path("/get_constants")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getConstants() {
        return createDefaultResponse(this.config);
    }

    /**
     * Function which returns whether the dataset is set.
     *
     * @return DefaultResponse with true if dataset is set, otherwise with false.
     */
    @GET
    @Path("/is_dataset_set")
    @Produces(MediaType.APPLICATION_JSON)
    public Response isDatasetSet() {
        return createDefaultResponse(this.controller.isDatasetConfigSet());
    }

    /**
     * Function which gets all datasets.
     *
     * @return DefaultResponse with all different dataset folders.
     */
    @GET
    @Path("/get_all_datasets")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAllDatasets() {
        return createDefaultResponse(datasetExplorer.getAllDatasets());
    }

    /**
     * Function to get all saved states.
     *
     * @return DefaultResponse with all saved states.
     */
    @GET
    @Path("/get_all_saved_states")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAllSavedStates() {
        return createDefaultResponse(savedStatesIndexer.getAllSavedStates());
    }

    /**
     * Choose a specific dataset.
     *
     * @param saved_state_filename the name of the saved state.
     * @return ErrorResponse with the error message that the state does not exist.
     * DefaultResponse with true if the state was loaded in.
     */
    @GET
    @Path("/choose_saved_state")
    @Produces(MediaType.APPLICATION_JSON)
    public Response setChosenSavedState(@QueryParam("saved_state") String saved_state_filename) {
        if (!controller.isDatasetConfigSet()) {
            try {
                File saved_state = savedStatesIndexer.getSavedState(saved_state_filename);
                System.out.println(saved_state);
                controller.loadState(saved_state);
            } catch (FileNotFoundException ex) {
                Log.log(LogLevel.ERROR, "APIService", "Saved state not existent: %s", saved_state_filename);
                return createErrorResponse("Saved state not existent: " + saved_state_filename);
            }
            return createDefaultResponse(true);
        } else {
            return createErrorResponse("A dataset was already set.");
        }
    }

    /**
     * Choose a specific dataset
     *
     * @param dataset_dir the folder of the dataset
     * @return DefaultResponse with true if everything worked out.
     */
    @GET
    @Path("/choose_dataset")
    @Produces(MediaType.APPLICATION_JSON)
    public Response setChosenDataset(@QueryParam("dataset_dir") String dataset_dir,
                                     @QueryParam("enable_walking") Boolean enable_walking_dataset) {

        File[] txtFiles = datasetExplorer.getAllFilesInDataset(dataset_dir);
        if (!controller.isDatasetConfigSet()) {
            controller.enableWalkingProperties(enable_walking_dataset);

            File configFile = datasetExplorer.getConfigFileInDataset(dataset_dir);
            controller.loadTrajectories(txtFiles);
            try {
                STORAGE.setDatasetConfig(YamlConfigRunner.getDatasetConfig(configFile));
                STORAGE.getDatasetConfig().setPath(dataset_dir);
                STORAGE.getDatasetConfig().setWalkingDataset(enable_walking_dataset);
                Log.log(LogLevel.INFO, "APIService", "Loaded dataset : %s", dataset_dir);
            } catch (Exception e) {
                e.printStackTrace();
                Log.log(LogLevel.ERROR, "APIService", "Dataset not existent: %s", dataset_dir);
                System.out.println("Unable to load in the dataset. Please make sure the dataset config is correct.");
                return createErrorResponse("Dataset not existent: " + dataset_dir);
            }
            return createDefaultResponse(true);
        } else {
            return createErrorResponse("A dataset was already set.");
        }
    }

    /**
     * This function get's all objects which were in the original dataset and which were calculated.
     *
     * @return AllGeometricObjects containing the trajectories, bundles, roadNetwork and datasetConfig.
     */
    @GET
    @Path("/get_all_objects")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getDatasetTrajectories() {
        //@ToDo remove this from the process
        System.out.println("APIService.getDatasetTrajectories. Offering data");

        AllGeometricObjects geometricObjects = new AllGeometricObjects();
        geometricObjects.setOriginal(STORAGE.getOriginalTrajectories());
        geometricObjects.setFiltered(STORAGE.getTrajectories());
        geometricObjects.setBundles(STORAGE.getAllBundleProperties());
        geometricObjects.setSettings(STORAGE.getDatasetConfig());

        HashMap<String, Object> network = new HashMap<>();

        ComputeRoadNetwork computeRoadNetwork = controller.returnTheRoadNetworkComputer();
        if (computeRoadNetwork != null){
            network.put("intersections", computeRoadNetwork.getIntersection());
            network.put("intersectionConnections", computeRoadNetwork.getIntersectionsConnectors());
            network.put("roadMap", computeRoadNetwork.getRoadMap());
        } else {
            network.put("intersections", new ArrayList<>());
            network.put("intersectionConnections", new ArrayList<>());
            network.put("roadMap", null);
        }

        geometricObjects.setNetwork(network);
        return createDefaultResponse(geometricObjects);
    }

    /**
     * This function starts the calculation of the bundle evolution diagram.
     *
     * @return DefaultResponse with true.
     */
    @GET
    @Path("/compute_bundles")
    @Produces(MediaType.APPLICATION_JSON)
    public Response computeBundles() {
        controller.computeBundlesEvolutionDiagram();
        return createDefaultResponse(true);
    }

    /**
     * This function starts the calculation of the bundles and the RoadNetwork.
     *
     * @return DefaultResponse with true.
     */
    @GET
    @Path("/compute_network")
    @Produces(MediaType.APPLICATION_JSON)
    public Response computeNetwork() {
        controller.computeTheRoadMap();
        return createDefaultResponse(true);
    }

    /**
     * This function starts the calculation of the bundles and the RoadNetwork.
     *
     * @return DefaultResponse with true.
     */
    @GET
    @Path("/compute_bundles_and_network")
    @Produces(MediaType.APPLICATION_JSON)
    public Response computeBundleRoadNetwork() {
        controller.computeBundlesAndRoadMap();
        return createDefaultResponse(true);
    }

    /**
     * This function resets all data objects to starting position, meaning we have to select a dataset again.
     *
     * @return DefaultResponse with true.
     */
    @GET
    @Path("/reset_to_start")
    @Produces(MediaType.APPLICATION_JSON)
    public Response resetApplication(){
        controller.resetToTheStart();
        return createDefaultResponse(true);
    }

    /**
     * Get ground truth map
     *
     * @return DefaultResponse with all vertices and edges from the ground truth map.
     */
    @GET
    @Path("/get_ground_truth")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getGroundTruth() {
        try {
            RoadNetwork backgroundMap = datasetExplorer.getGroundTruth(STORAGE.getDatasetConfig().getPath());
            return createDefaultResponse(backgroundMap);
        } catch (FileNotFoundException e) {
            Log.log(LogLevel.WARNING, "APIService", "Getting the ground truth map gave an error: %s", e.toString());
            return createDefaultResponse(null);
        } catch (NullPointerException e) {
            Log.log(LogLevel.ERROR, "APIService", "Could not get ground truth map. No dataset specified");
            return createErrorResponse("Please choose a dataset first.");
        }
    }
}

/**
 * This class is used to create a default response.
 * It is created such that all requests have the same way to report errors and can update the user on whether an
 * algorithm is running in the background.
 */
class Response {

    private Object data;
    private Boolean error;
    private String errorMessage;
    private String timestamp;
    private int algorithmProcess;

    public Response() {
        // Needed by Jackson deserialization
    }

    public Response(Object data, Boolean error, String errorMessage, int algorithmProcess) {
        this.data = data;
        this.error = error;
        this.errorMessage = errorMessage;
        this.algorithmProcess = algorithmProcess;

        Date date = new Date();
        long time = date.getTime();
        Timestamp ts = new Timestamp(time);
        this.timestamp = ts.toString();
    }

    @JsonProperty
    public Object getData() {
        return data;
    }

    @JsonProperty
    public Boolean getError() {
        return error;
    }

    @JsonProperty
    public String getErrorMessage() {
        return errorMessage;
    }

    @JsonProperty
    public String getTimestamp() {
        return timestamp;
    }

    @JsonProperty
    public int getAlgorithmProcess() {
        return algorithmProcess;
    }
}

/**
 * A data container class. Containing all the geometric objects of the program.
 */
class AllGeometricObjects {
    private List<Trajectory> original;
    private List<Trajectory> filtered;
    private ArrayList<Map<String, Object>> bundles;
    private Object network;
    private DatasetConfig settings;

    @JsonProperty
    public List<Trajectory> getOriginal() {
        return original;
    }

    public void setOriginal(List<Trajectory> original) {
        this.original = original;
    }

    @JsonProperty
    public List<Trajectory> getFiltered() {
        return filtered;
    }

    public void setFiltered(List<Trajectory> filtered) {
        this.filtered = filtered;
    }

    @JsonProperty
    public ArrayList<Map<String, Object>> getBundles() {
        return bundles;
    }

    public void setBundles(ArrayList<Map<String, Object>> bundles) {
        this.bundles = bundles;
    }

    @JsonProperty
    public Object getNetwork() {
        return network;
    }

    public void setNetwork(Object network) {
        this.network = network;
    }

    @JsonProperty
    public DatasetConfig getSettings() {
        return settings;
    }

    public void setSettings(DatasetConfig settings) {
        this.settings = settings;
    }
}