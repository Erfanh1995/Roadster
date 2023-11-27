package mapconstruction.web;

import com.google.common.math.DoubleMath;
import mapconstruction.GUI.io.*;
import mapconstruction.GUI.listeners.*;
import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.algorithms.maps.ComputeRoadNetwork;
import mapconstruction.algorithms.preprocessing.CompositePreprocessor;
import mapconstruction.algorithms.preprocessing.SegmentationPreprocessor;
import mapconstruction.algorithms.preprocessing.SimplificationPreprocessor;
import mapconstruction.algorithms.preprocessing.StraightenerPreprocessor;
import mapconstruction.algorithms.segmentation.HeadingSegmenter;
import mapconstruction.algorithms.segmentation.SelfSimilaritySementer;
import mapconstruction.algorithms.segmentation.TrajectorySegmenter;
import mapconstruction.algorithms.simplification.SimplificationMethod;
import mapconstruction.algorithms.simplification.TrajectorySimplifier;
import mapconstruction.algorithms.straightener.TrajectoryStraightener;
import mapconstruction.benchmark.Benchmark;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.web.config.DatasetConfig;
import mapconstruction.web.config.GeneralConfig;
import mapconstruction.web.config.YamlConfigRunner;
import mapconstruction.workers.AbortableAlgorithmWorker;
import mapconstruction.workers.ComputeEvolutionDiagram;
import mapconstruction.workers.ComputeGroundTruthCutoff;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;
import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * Controller handles the logic between the API and the actual implementation of the algorithms.
 * Furthermore it handles quite a lot of data flow.
 *
 * @author Jorrick
 * @since 05/11/2018
 */
public class Controller {

    SaveStateDiagramAndNetworkListener saveStateListener;
    /**
     * Keeps track of the time for the save state controllers.
     */
    private long timingStart = System.currentTimeMillis();
    private TxtTrajectoryReader txtTrajReader;
    private IpeTrajectoryReader ipeTrajReader;
    private FileNameExtensionFilterExt txtFilter;
    private FileNameExtensionFilterExt ipeFilter;
    private GeneralConfig generalConfig;
    private SavedStatesIndexer savedStatesIndexer;
    private BenchmarkManager benchmarkManager;
    private OutputManager outputManager;
    private boolean ignoreDirection;
    private boolean useSimplifier;
    private boolean simplifierGreedy;
    private boolean simplifierRDP;
    private boolean walkingDataset;
    private int simplifierError;
    private boolean useSegmenter;
    private boolean segmenterHeading;
    private boolean segmenterSelfSim;
    private boolean cutOffTrajectoryEndings;
    private int segmenterHeadingAngle;
    private int segmenterDistSelfSim;
    private AbortableAlgorithmWorker currentWorker;
    private ComputeRoadNetwork computeRoadNetwork;

    public Controller(GeneralConfig generalConfig) {
        txtFilter = new FileNameExtensionFilterExt(new FileNameExtensionFilter("Text file", "txt"));
        ipeFilter = new FileNameExtensionFilterExt(new FileNameExtensionFilter("Ipe document", "ipe", "xml"));

        txtTrajReader = new TxtTrajectoryReader();
        ipeTrajReader = new IpeTrajectoryReader();

        this.generalConfig = generalConfig;
        savedStatesIndexer = new SavedStatesIndexer(generalConfig.getSavedStatesDirectory());
        outputManager = new OutputManager(generalConfig.getOutputDirectory());
        benchmarkManager = new BenchmarkManager(generalConfig.getBenchmarkDirectory());

        ignoreDirection = true;
        useSimplifier = false;
        simplifierGreedy = false;
        simplifierRDP = false;
        simplifierError = 3;

        useSegmenter = false;
        segmenterHeading = false;
        segmenterSelfSim = false;
        segmenterHeadingAngle = 90;
        segmenterDistSelfSim = 20;

        cutOffTrajectoryEndings = true;
        walkingDataset = false;
        computeRoadNetwork = null;

        ALGOCONSTANTS.setNumThreads(generalConfig.getNumOfProcesses());

        initLog();
    }

    /**
     * Enable walking dataset
     * @param enableWalkingProperties true if we should enable the properties for a walking dataset, false for car
     *                                or anything else.
     */
    public void enableWalkingProperties(boolean enableWalkingProperties){

        if (enableWalkingProperties){
            walkingDataset = true;
            enableSimplifier(simplifierError);
        } else {
            walkingDataset = false;
            useSimplifier = false;
        }
    }

    public void setCutOffRepresentatives(boolean enableCutOff){
        cutOffTrajectoryEndings = false;
        ALGOCONSTANTS.setEnableCutOff(enableCutOff);
    }

    /**
     * Set the use of a segmenter
     * @param segmenterEpsilon
     */
    public void setUseSegmenter(int segmenterEpsilon){
        useSegmenter = true;
        segmenterSelfSim = true;
        segmenterDistSelfSim = segmenterEpsilon;
    }

    /**
     * Starting the logger
     */
    private void initLog() {
        // init log
        Log.addLogUser(new FileLogger());
        new Thread(Log.instance()).start();
    }

    /**
     * To check whether the datasetConfig is set.
     *
     * @return true if the datasetConfig is set, false otherwise.
     */
    public boolean isDatasetConfigSet() {
        return STORAGE.getDatasetConfig() != null;
    }

    /**
     * Function to load in a Dataset.
     *
     * @param datasetDir
     */
    public void loadDataset(String datasetDir) {
        if (!isDatasetConfigSet()) {
            DatasetExplorer datasetExplorer = new DatasetExplorer(generalConfig.getDatasetDirectory());
            if (datasetExplorer.getAllDatasets().indexOf(datasetDir) < 0) {
                System.out.println("Error! This dataset does not exist.");
            }


            File configFile = datasetExplorer.getConfigFileInDataset(datasetDir);

            File[] txtFiles = datasetExplorer.getAllFilesInDataset(datasetDir);
            loadTrajectories(txtFiles);

            try {
                STORAGE.setDatasetConfig(YamlConfigRunner.getDatasetConfig(configFile));
                STORAGE.getDatasetConfig().setPath(datasetDir);
                STORAGE.getDatasetConfig().setWalkingDataset(walkingDataset);
                Log.log(LogLevel.INFO, "APIService", "Loaded dataset : %s", datasetDir);
            } catch (Exception e) {
                e.printStackTrace();
                Log.log(LogLevel.ERROR, "APIService", "Dataset not existent: %s", datasetDir);
                System.out.println("Unable to load in the dataset. Please make sure the dataset config is correct.");
            }
        }

    }

    /**
     * Given a bunch of files, it loads in all these trajectories
     *
     * @param files all the files that should be loaded in.
     */
    public void loadTrajectories(File[] files) {
        // Add files to the list
        // Load all trajectories
        List<Trajectory> trajs = new ArrayList<>();
        for (File f : files) {
            TrajectoryReader reader;
            if (txtFilter.accept(f)) {
                // text file
                reader = txtTrajReader;
            } else if (ipeFilter.accept(f)) {
                reader = ipeTrajReader;
            } else {
                Log.log(LogLevel.WARNING, "Control", "Skipped over unsupported file: %s", f.getName());
                continue;
            }

            trajs.addAll(reader.parse(f));

            Log.log(LogLevel.INFO, "Control", "Trajectories from %s added", f.getName());
        }
        STORAGE.clearBundles();
        STORAGE.setEvolutionDiagram(null);
        STORAGE.setOriginalTrajectories(trajs);
        STORAGE.setTrajectories(trajs);
        preProcess();
    }

    /**
     * Enable simplifier
     */
    public void enableSimplifier(int simplifyError) {
        if (simplifyError <= 0) {
            return;
        }
        this.simplifierError = simplifyError;
        simplifierRDP = true;
        useSimplifier = true;
    }

    /**
     * Function to get the trajectory simplifier as configured.
     *
     * @return TrajectorySimplificationMethod or null
     */
    private TrajectorySimplifier getSimplifier() {
        if (simplifierGreedy) {
            return SimplificationMethod.Greedy;
        } else if (simplifierRDP) {
            return SimplificationMethod.RDP;
        }
        return null;
    }

    /**
     * Function to get the trajectory segmenter as configured.
     *
     * @return TrajectorySegmenter
     */
    private TrajectorySegmenter getSegmenter() {
        if (segmenterHeading) {
            double angle = (double) segmenterHeadingAngle;
            return HeadingSegmenter.degrees(angle);
        } else if (segmenterSelfSim) {
            double dist = (double) segmenterDistSelfSim;
            return new SelfSimilaritySementer(dist, 2 * dist, ignoreDirection);
        }
        return null;
    }

    /**
     * This function pre-processes the trajectories.
     * It create a pipeline after which the specific preprocessors are added.
     */
    public void preProcess() {
        CompositePreprocessor pipeline = new CompositePreprocessor();
        double error = (double) simplifierError;
        if (useSimplifier && !DoubleMath.fuzzyEquals(error, 0, 1E-6)) {
            pipeline.add(new SimplificationPreprocessor(getSimplifier(), error));
        }
        if (walkingDataset){
            pipeline.add(new StraightenerPreprocessor(new TrajectoryStraightener(50.0)));
        }
        if (useSegmenter) {
            pipeline.add(new SegmentationPreprocessor(getSegmenter()));
        }
        if (pipeline.numOfPreprocessors() > 0) {
            STORAGE.setTrajectories(pipeline.run(STORAGE.getTrajectories()));
        }
    }

    /**
     * Raises an error if the current worker is running.
     */
    public void raiseErrorIfWorkerRunning() throws ArithmeticException {
        if (currentWorker != null) {
            if (currentWorker.getState() != SwingWorker.StateValue.DONE) {
                throw new ArithmeticException("A worker is still running.");
            }
        }
    }

    /**
     * Getting whether the worker is still busy
     */
    public boolean isWorkerBusy() {
        return currentWorker != null && !currentWorker.isDone();
    }

    /**
     * Instantiates the computation of the bundle evolution diagram.
     */
    public void computeBundlesEvolutionDiagram() {
        attachListener();
        raiseErrorIfWorkerRunning();
        currentWorker = new ComputeEvolutionDiagram();
        currentWorker.run();

        STORAGE.addBundleListener(new BundleChangeListener() {
            @Override
            public void displayedBundlesChanged(BundleChangeEvent evt) {
                System.out.println("BUNDLES DISPLAY CHANGED: " + evt.getBundles().size());
            }

            @Override
            public void bundlesChanged(BundleChangeEvent evt) {
                System.out.println("BUNDLES CHANGED: " + evt.getBundles().size());
                STORAGE.setProgressAlgorithm(100);
            }
        });

        while (STORAGE.getAllUnfilteredBundles().size() == 0) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        STORAGE.setProgressAlgorithm(100);
//        Unnecessary as DataStorage does take care of this.
//        calculatePropertiesOfAllBundles();
    }

    /**
     * Initiates the computation of the bundle evolution diagram and afterwards the road network.
     */
    public void computeBundlesAndRoadMap() {
        computeBundlesEvolutionDiagram();
        computeTheRoadMap();
    }

    /**
     * Compute the newly created roadNetwork
     */
    public void computeTheRoadMap() {
        attachListener();
        if (computeRoadNetwork == null){
            this.computeRoadNetwork = new ComputeRoadNetwork();
            STORAGE.setRoadMap(computeRoadNetwork.getRoadMap());

            outputManager.saveRoadMap(STORAGE.getDatasetConfig().getPath(), new Date(), computeRoadNetwork.getRoadMap());
            System.out.println("Done computing road network");
        }
    }

    public void computeGroundTruthCutoff(double epsilon, GeneralConfig config) {
        currentWorker = new ComputeGroundTruthCutoff(epsilon, config);
        currentWorker.run();
    }

    public ComputeRoadNetwork returnTheRoadNetworkComputer() {
        return computeRoadNetwork;
    }

    /**
     * Attach the listener
     */
    private void attachListener() {
        if (saveStateListener == null) {
            saveStateListener = new SaveStateDiagramAndNetworkListener();
            STORAGE.addDiagramListener(saveStateListener);
            STORAGE.addNetworkListeners(saveStateListener);
        }
    }

    /**
     * This function loads all relevant variables into a file.
     * <p>
     * This stores the current state of the program into a file, such that we can set all these variables back later.
     * This function is called when the algorithm for calculating the road network is finished.
     *
     * @param out, the file where we write the state too. (Note, this is not the path, this is defined somewhere else)
     */
    private void saveState(File out) {
        try (ObjectOutputStream writer = new ObjectOutputStream(new FileOutputStream(out))) {

            writer.writeObject(STORAGE.getDatasetConfig());
            writer.writeObject(STORAGE.getOriginalTrajectories());
            writer.writeObject(STORAGE.getTrajectories());
            writer.writeObject(STORAGE.getEvolutionDiagram());
            writer.writeObject(this.computeRoadNetwork);
            ParameterSerializable params = ParameterSerializable.create();
            writer.writeObject(params);

            Log.log(LogLevel.INFO, "Control", "State exported to: %s", out.getAbsolutePath());
        } catch (IOException ex) {
            Logger.getLogger(Controller.class.getName()).log(Level.SEVERE, null, ex);
            Log.log(LogLevel.ERROR, "Control", "Failed to export state: %s", ex.toString());
        }
    }

    public void timer(String s) {
        long timingEnd = System.currentTimeMillis();
        float elapsedTimeSec = (timingEnd - timingStart) / 1000F;
        timingStart = timingEnd;

        String str = Float.toString(elapsedTimeSec) + " - " + s;
        System.out.println(str);
        Log.log(LogLevel.INFO, "Control", str);
    }

    /**
     * This function loads all the relevant variables from the saved state back into the storage.
     *
     * @param in, the File
     */
    public void loadState(File in) {
        try (ObjectInputStream reader = new ObjectInputStream(new FileInputStream(in))) {
            timer("Controller - Starting the loading state");
            DatasetConfig datasetConfig = (DatasetConfig) reader.readObject();
            List<Trajectory> originalTrajectories = (List<Trajectory>) reader.readObject();
            List<Trajectory> trajectories = (List<Trajectory>) reader.readObject();
            EvolutionDiagram diagram = (EvolutionDiagram) reader.readObject();
            ComputeRoadNetwork roadMapComputer = (ComputeRoadNetwork) reader.readObject();
            timer("Controller - Loaded the files into main memory");

            try {
                ((ParameterSerializable) reader.readObject()).restore(this);
            } catch (EOFException ex) {
                Log.log(LogLevel.WARNING, "Control", "Parameters could not be restored");
            }

            STORAGE.setDatasetConfig(datasetConfig);
            STORAGE.setOriginalTrajectories(originalTrajectories);
            STORAGE.setTrajectories(trajectories);
            STORAGE.setEvolutionDiagram(diagram);
            this.computeRoadNetwork = roadMapComputer;
            if (this.computeRoadNetwork != null) {
                STORAGE.setRoadMap(computeRoadNetwork.getRoadMap());
            }

            enableWalkingProperties(STORAGE.getDatasetConfig().isWalkingDataset());

            timer("Controller - Set all trajectories and the evolution diagram");

            AbortableAlgorithmWorker lastWorker = new ComputeEvolutionDiagram();
            ((ComputeEvolutionDiagram) lastWorker).updateDiagramBundles();

            System.out.println("Controller - Computing all bundle properties.");
            Log.log(LogLevel.INFO, "Control", "Computing all bundle properties");

            STORAGE.setFilter(((ComputeEvolutionDiagram) lastWorker).predicate);
            timer("Controller - Applied the trajectory filter");

            timer("Controller - Calculated all bundle properties");
            Log.log(LogLevel.INFO, "Control", "State imported from: %s", in.getAbsolutePath());
        } catch (IOException | ClassNotFoundException ex) {
            Logger.getLogger(Controller.class.getName()).log(Level.SEVERE, null, ex);
            Log.log(LogLevel.ERROR, "Control", "Failed to import state: %s", ex.toString());
        }
    }

    /**
     * Function to delete all objects and to restart as with a fresh install
     */
    public void resetToTheStart() {
        STORAGE.setDatasetConfig(null);
        STORAGE.setOriginalTrajectories(new ArrayList<>());
        STORAGE.setTrajectories(new ArrayList<>());
        STORAGE.clearBundles();
        STORAGE.setEvolutionDiagram(null);
        if (computeRoadNetwork != null){
            computeRoadNetwork = null;
            STORAGE.setRoadMap(null);
        }

        currentWorker = new ComputeEvolutionDiagram();
    }

    /**
     * Get an instance of the benchmark manager pointing at the configured benchmark directory
     */
    public BenchmarkManager getBenchmarkManager() {
        return benchmarkManager;
    }

    /**
     * Helper class containing all configured parameters in serializable form.
     * <p>
     * Can be used to easily restore parameters.
     */
    private static class ParameterSerializable implements Serializable {

        private static final long serialVersionUID = 1L;

        private double sweep_size;

        private ParameterSerializable() {

        }

        public static ParameterSerializable create() {
            ParameterSerializable o = new ParameterSerializable();
            o.sweep_size = 0.0;
            return o;
        }

        public void restore(Controller controller) {
//            Skipp
        }
    }

////////////////////////////////////////////////////////////////////////////////
//////////////////////// Utility inner classes /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

    /**
     * Class to save the state of the program when the RoadNetwork was changed.
     */
    private class SaveStateDiagramAndNetworkListener implements NetworkChangeListener, DiagramChangeListener {
        @Override
        public void diagramChanged(DiagramChangeEvent evt) {
            String date = new SimpleDateFormat("yyyy-MM-dd_HH-mm").format(new Date());
            String walking = STORAGE.getDatasetConfig().isWalkingDataset()? "walking_" : "car_";
            String fileName = "diagram_" + walking + STORAGE.getDatasetConfig().getPath() + "_" + date + ".savst";
            saveState(new File(savedStatesIndexer.getNewSavedStateFilePath(fileName)));
        }

        @Override
        public void networkChanged(NetworkChangeEvent evt) {
//            DISABLED! There is a very weird error when running from this saved state.
//            Probably has to do with the fact that not everything from the intersection is stored, but who knows..
//            String date = new SimpleDateFormat("yyyy-MM-dd_HH:mm").format(new Date());
//            String walking = STORAGE.getDatasetConfig().isWalkingDataset()? "walking_" : "car_";
//            String fileName = "network_" + walking +  STORAGE.getDatasetConfig().getPath() + "_" + date + ".savst";
//            saveState(new File(savedStatesIndexer.getNewSavedStateFilePath(fileName)));
        }
    }
}
