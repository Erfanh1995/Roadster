package mapconstruction.starter;

import mapconstruction.algorithms.maps.intersections.containers.Intersection;
import mapconstruction.benchmark.Benchmark;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.web.Controller;
import mapconstruction.web.config.GeneralConfig;
import mapconstruction.web.config.YamlConfigRunner;
import org.apache.commons.cli.*;

import java.io.File;
import java.io.IOException;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class Starter {

    /**
     * Contains the commandLine instance after launch
     */
    private static CommandLine commandLine;
    /**
     * Contains the HelpFormatter
     */
    private static HelpFormatter formatter;
    /**
     * Contains general configuration
     */
    private GeneralConfig yamlConfig;
    /**
     * Contains the controller.
     */
    private Controller controller;

    /**
     * Epsilon for Ground Truth cutoff (if present)
     */
    private static double cutoffEpsilon;

    /**
     * Actual instantiation of the algorithms.
     *
     * @param configPath,        the config path.
     * @param datasetSelected,   whether the dataset was selected, if false a saved state path was given.
     * @param path,              path to either the dataset or the saved state
     * @param computationOption, what is to be calculated. 1 = bundles, 2 = network, 3 = both.
     * @param simplifyDistance,  the distance of the simplification, if > 0, enable simplification
     */
    private Starter(String configPath, boolean datasetSelected, String path, int computationOption, int simplifyDistance,
                    boolean enableWalkingDataset, int segmentation, boolean enableRepCutOff) {
        initialize(configPath);

        if (enableWalkingDataset){
            controller.enableWalkingProperties(true);
        } else if (simplifyDistance > 0) {
            controller.enableSimplifier(simplifyDistance);
        }
        if (segmentation > 0){
            controller.setUseSegmenter(segmentation);
        }
        if (!enableRepCutOff){
//            The standard value is true, hence when we set the tag, we want to change it to false.
            controller.setCutOffRepresentatives(false);
        }

        Benchmark.memMonitor(true);

        if (datasetSelected) {
            controller.loadDataset(path);
        } else {
            controller.loadState(new File(yamlConfig.getSavedStatesDirectory(), path));
        }

        if (computationOption == -1) {
            Benchmark.start(controller.getBenchmarkManager(), path);
            controller.computeBundlesEvolutionDiagram();
            Benchmark.stopAndReport();
        } else if (computationOption == 1) {
            controller.computeBundlesEvolutionDiagram();

            while (STORAGE.getProgressAlgorithm() != 100 || STORAGE.getDisplayedBundles().size() == 0) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else if (computationOption == 2) {
            controller.computeTheRoadMap();
        } else if (computationOption == 4) {
            controller.computeGroundTruthCutoff(cutoffEpsilon, yamlConfig);
        } else {
            controller.computeBundlesAndRoadMap();
        }

        System.out.println("Exiting the MapConstruction program now.");
        System.exit(0);
    }

    /**
     * The function that is called
     */
    public static void main(String[] args) {
        CommandLineParser parser = new DefaultParser();
        formatter = new HelpFormatter();
        formatter.setOptionComparator(null);  // To disable sorting them on key.

        try {
            commandLine = parser.parse(getCommandLineOptions(), args);
            afterParseProcess();
        } catch (ParseException e) {
            System.out.println(e.getMessage());
            formatter.printHelp("Starter.java", getCommandLineOptions());
            System.exit(1);
        }
    }

    /**
     * Specifies all the options we have at the command line.
     *
     * @return all the options
     */
    private static Options getCommandLineOptions() {
        Options options = new Options();

        Option generalConfig = new Option("p", "configPath", true, "General config path");
        generalConfig.setRequired(true);
        options.addOption(generalConfig);

        Option dataset = new Option("d", "dataset", true, "Dataset input directory");
        Option savedState = new Option("s", "savedState", true, "SavedState input file");
        OptionGroup startingPoint = new OptionGroup();
        startingPoint.addOption(dataset);
        startingPoint.addOption(savedState);
        startingPoint.setRequired(true);
        options.addOptionGroup(startingPoint);

        Option benchmark = new Option("bm", "benchmark", false, "Compute bundles and enable benchmarking");
        Option computeBundles = new Option("cb", "computeBundles", false, "Compute bundles");
        Option computeRoadNetwork = new Option("crn", "computeRoadNetwork", false, "Compute road network");
        Option computeAll = new Option("call", "computeAll", false, "Compute all");
        Option computeCutoff = new Option("coff", "computeCutoff", true, "Compute Ground Truth cutoff");
        OptionGroup computeGroup = new OptionGroup();
        computeGroup.addOption(benchmark);
        computeGroup.addOption(computeBundles);
        computeGroup.addOption(computeRoadNetwork);
        computeGroup.addOption(computeAll);
        computeGroup.addOption(computeCutoff);
        computeGroup.setRequired(true);
        options.addOptionGroup(computeGroup);

        Option useSimplificationRDP = new Option("sim", "simplify", true, "Use RDP simplifier");
        Option useWalkingDatasetSetting = new Option("walk", "walkSettings", false, "Use Walking dataset configuration");
        OptionGroup simplifiers = new OptionGroup();
        simplifiers.addOption(useSimplificationRDP);
        simplifiers.addOption(useWalkingDatasetSetting);
        options.addOptionGroup(simplifiers);

        Option segmentation = new Option("seg", "segmentation", true, "Use segmentation");
        options.addOption(segmentation);

        Option disableCutOff = new Option("cut", "disableCutOff", false, "Disable cutting off representatives");
        options.addOption(disableCutOff);

        return options;
    }

    /**
     * We look at what actually was returned.
     */
    private static void afterParseProcess() throws ParseException {
        boolean datasetSelected = false;
        String path, infoString = "";
        String configPath = commandLine.getOptionValue("configPath");
        int simplifyDistance = 0;
        int segmentation = 0;
        boolean enableWalkingDataset = false;
        boolean enableRepCutOff = true;


        if (commandLine.hasOption("walk")) {
            enableWalkingDataset = true;
            infoString += "You have selected walking dataset configuration settings. \n";
        } else if (commandLine.hasOption("sim")) {
            try {
                simplifyDistance = Integer.parseInt(commandLine.getOptionValue("sim"));
                infoString +=  "You have selected to use the Simplifier with a distance of " + simplifyDistance + " meters. \n";
            } catch (Exception ex){
                throw new ParseException("Please enter an integer for simplifyDistance.");
            }
        }

        // If we want to use segmentation, enable it
        if (commandLine.hasOption("seg")) {
            try {
                segmentation = Integer.parseInt(commandLine.getOptionValue("seg"));
                infoString +=  "You have selected to use the Segmentation with a distance of " + segmentation + " meters. \n";
            } catch (Exception ex){
                throw new ParseException("Please enter an integer for Segmentation value.");
            }
        }

        // If we want to disable the shortening of representative of bundles for which they badly represent their subs.
        if (commandLine.hasOption("cut")){
            enableRepCutOff = false;
        }


        // Whether we want a dataset of saved state
        if (commandLine.hasOption("d")) {
            datasetSelected = true;
            path = commandLine.getOptionValue("d");
            infoString += "You have selected a dataset at path: " + path + "\n";
        } else {
            path = commandLine.getOptionValue("s");
            infoString += "You have selected a savedState at path: " + path + "\n";
        }

        // To find out which computation we are launching.
        int computation = 0;
        if (commandLine.hasOption("bm")) {
            computation = -1;
            infoString += "The benchmark for Bundle generation starts now.";
        } else if (commandLine.hasOption("cb")) {
            computation = 1;
            infoString += "The calculation for the Bundles starts right now.";
        } else if (commandLine.hasOption("crn")) {
            computation = 2;
            infoString += "The calculation for the Road Network starts right now.";
        } else if (commandLine.hasOption("call")) {
            computation = 3;
            infoString += "The calculation for the Bundles starts right now after which " +
                    "the Road Network will be computed.";
        } else if (commandLine.hasOption("coff")) {
            computation = 4;
            cutoffEpsilon = Double.parseDouble(commandLine.getOptionValue("coff"));
            infoString += "The calculation for the Ground Truth cutoff starts right now.";
        }

        // To filter out specific unexpected cases.
        if (datasetSelected && computation == 2) {
            throw new ParseException("It is not possible to compute the RoadNetwork without the bundles.");
        }
        if (!datasetSelected && (computation == 1 || computation == 3 || computation == 4)) {
            throw new ParseException("It is not possible to recompute the bundles for a saved state.");
        }

        // Starting the actual computation
        System.out.println(infoString);
        new Starter(configPath, datasetSelected, path, computation, simplifyDistance, enableWalkingDataset, segmentation, enableRepCutOff);
    }

    /**
     * Load the general config of our application
     *
     * @param configPath, path to the general config file.
     */
    private void initialize(String configPath) {
        try {
            yamlConfig = YamlConfigRunner.getGeneralConfig(configPath);
        } catch (IOException ex) {
            System.out.println("Problem getting the config. Please set it at the right place and configure it correctly.");
            System.out.println(ex.toString());
            Log.log(LogLevel.INFO, "APIService", "Problem getting the config. Please set it at the right place.");
            System.exit(0);
        }
        controller = new Controller(yamlConfig);
    }

    /**
     * Here we deleted the ArtemisJob
     */
    private void shutDownArtemisJob() {
        String jobID = System.getenv("PBS_O_JOBID");
        if (jobID != null) {
            try {
                Runtime.getRuntime().exec(new String[]{"qdel " + jobID});
            } catch (Exception e) {
                System.out.println("HEY Buddy ! U r Doing Something Wrong ");
                e.printStackTrace();
            }
        }
    }


}
