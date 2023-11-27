package mapconstruction.GUI.datastorage;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import mapconstruction.GUI.filter.BooleanTriPredicates;
import mapconstruction.GUI.filter.TriPredicate;
import mapconstruction.GUI.listeners.*;
import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.algorithms.maps.mapping.RoadMap;
import mapconstruction.algorithms.representative.CutEnd;
import mapconstruction.attributes.BundleClassAttributes;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.web.config.DatasetConfig;

import java.util.*;

/**
 * Singleton instance storing the relevant data for the GUI.
 *
 * @author Roel
 */
public enum DataStorage {
    STORAGE;

    /**
     * List of (unfiltered) loaded trajectories.
     */
    private final List<Trajectory> originalTrajectories;
    /**
     * List of (filtered) loaded trajectories.
     */
    private final List<Trajectory> trajectories;
    /**
     * Listeners to trajectory changes
     */
    private final Set<TrajectoryChangeListener> trajListeners;
    /**
     * Listeners to bundle changes
     */
    private final Set<BundleChangeListener> bundleListeners;
    /**
     * Listeners to diagram changes
     */
    private final Set<DiagramChangeListener> diagramListeners;
    /**
     * Listeners to diagram changes
     */
    private final Set<NetworkChangeListener> networkListeners;
    /**
     * Progress of the algorithm
     */
    private int progressAlgorithm;
    /**
     * Information of the dataset
     */
    private DatasetConfig datasetConfig;
    /**
     * Filters set on the bundel.
     */
    private TriPredicate<EvolutionDiagram, Integer, Double> filter = BooleanTriPredicates.alwaysTrue();


    /**
     * All currently generated bundles
     */
    private BiMap<Bundle, Integer> allBundlesWithClasses;

    /**
     * All currently generated bundles
     */
    private BiMap<Bundle, Integer> allBundlesWithClassesUnfiltered;

    /**
     * All currently generated bundles
     */
    private BiMap<Bundle, Integer> displayedBundlesWithClasses;

    /**
     * Evolution diagram.
     */
    private EvolutionDiagram evolutionDiagram;

    /**
     * Road network
     */
    private RoadMap roadMap;

    private DataStorage() {
        trajListeners = new LinkedHashSet<>();
        bundleListeners = new LinkedHashSet<>();
        diagramListeners = new LinkedHashSet<>();
        networkListeners = new LinkedHashSet<>();

        originalTrajectories = new ArrayList<>();
        trajectories = new ArrayList<>();
        allBundlesWithClasses = HashBiMap.create();
        allBundlesWithClassesUnfiltered = HashBiMap.create();
        displayedBundlesWithClasses = HashBiMap.create();

        datasetConfig = null;
        roadMap = null;
    }

    /**
     * Sets and gets the progress of the algorithm currently executes.
     * Assumes at most one algorithm is running at a time.
     *
     * @return int, containg the progress
     */
    public int getProgressAlgorithm() {
        return progressAlgorithm;
    }

    /**
     * Sets the progress of the algorithm
     *
     * @param progressAlgorithm, the algorithms process. Range from 0 to 100.
     */
    public void setProgressAlgorithm(int progressAlgorithm) {
        if (progressAlgorithm < 0 || progressAlgorithm > 100) {
            System.out.println("Progress of algorithm is incorrectly set: " + Integer.toString(progressAlgorithm));
        }
        this.progressAlgorithm = progressAlgorithm;
    }

    /**
     * Returns read-only view on the list of trajectories.
     * <p>
     * Changes to the lists must be done using this Class' methods
     *
     * @return
     */
    @JsonProperty
    public synchronized List<Trajectory> getTrajectories() {
        return Collections.unmodifiableList(trajectories);
    }

    public synchronized void setTrajectories(List<Trajectory> trajectories) {
        List<Trajectory> old = new ArrayList<>(this.trajectories);
        this.trajectories.clear();
        this.trajectories.addAll(trajectories);
        notifyTrajectoryListeners(old, trajectories);
    }

    @JsonProperty
    public synchronized List<Trajectory> getOriginalTrajectories() {
        return Collections.unmodifiableList(originalTrajectories);
    }

    public synchronized void setOriginalTrajectories(List<Trajectory> trajectories) {
        this.originalTrajectories.clear();
        this.originalTrajectories.addAll(trajectories);
    }

    public synchronized void addTrajectory(Trajectory t) {
        trajectories.add(t);
        notifyTrajectoryListeners(Collections.singletonList(t), Collections.EMPTY_LIST);
    }

    public synchronized void addAllTrajectories(List<Trajectory> trajectories) {
        this.trajectories.addAll(trajectories);
        notifyTrajectoryListeners(trajectories, Collections.EMPTY_LIST);
    }

    public synchronized void removeTrajectory(Trajectory t) {
        trajectories.remove(t);
        notifyTrajectoryListeners(Collections.EMPTY_LIST, Collections.singletonList(t));
    }

    public synchronized void removeAllTrajectories(List<Trajectory> trajectories) {
        this.trajectories.removeAll(trajectories);
        notifyTrajectoryListeners(Collections.EMPTY_LIST, trajectories);
    }

    public synchronized void clearTrajectories() {
        List<Trajectory> old = new ArrayList<>(this.trajectories);
        trajectories.clear();
        notifyTrajectoryListeners(Collections.EMPTY_LIST, old);
    }

    @JsonProperty
    public DatasetConfig getDatasetConfig() {
        return datasetConfig;
    }

    public void setDatasetConfig(DatasetConfig datasetConfig) {
        this.datasetConfig = datasetConfig;
    }

    /**
     * Sets the map of all generated bundles with their classes. Also resets the
     * displayed classes to the one given.
     *
     * @param bundlesWithClasses
     */
    public synchronized void setBundlesWithClasses(BiMap<Bundle, Integer> bundlesWithClasses) {
        this.allBundlesWithClassesUnfiltered = HashBiMap.create(bundlesWithClasses);
        postprocessAllBundles();
        computeSomeBundleProperties();
    }

    /**
     * Sets the generated bundles without explicit classes. The classes are
     * generated in the order of the given collection. Also resets the displayed
     * classes to the one given.
     *
     * @param bundles
     */
    public synchronized void setBundles(Collection<Bundle> bundles) {
        this.allBundlesWithClassesUnfiltered = HashBiMap.create(bundles.size());
        int i = 0;
        for (Bundle b : bundles) {
            this.allBundlesWithClassesUnfiltered.put(b, i++);
        }
        postprocessAllBundles();
    }

    /**
     * Returns the set of all unfiltered bundles.
     */
    public synchronized Set<Bundle> getAllUnfilteredBundles() {
        return allBundlesWithClassesUnfiltered.keySet();
    }

    /**
     * Sets the map of all generated bundles with their classes.
     *
     * @param displayedBundlesWithClasses
     */
    public synchronized void setDiplayedBundlesWithClasses(BiMap<Bundle, Integer> displayedBundlesWithClasses) {
        this.displayedBundlesWithClasses = HashBiMap.create(displayedBundlesWithClasses);
        notifyDisplayedBundleListeners(this.displayedBundlesWithClasses);
    }

    /**
     * Returns a the set of bundles representing the displayed classes.
     *
     * @return
     */
    public synchronized Set<Bundle> getDisplayedBundles() {
        return Collections.unmodifiableSet(this.displayedBundlesWithClasses.keySet());
    }


    /**
     * Sets the displayed bundles without explicit classes. The classes are
     * extracted from the set of available bundles.
     *
     * @param bundles
     */
    public synchronized void setDisplayedBundles(Collection<Bundle> bundles) {
        this.displayedBundlesWithClasses = HashBiMap.create(bundles.size());
        bundles.forEach(b -> this.displayedBundlesWithClasses.put(b, this.allBundlesWithClasses.get(b)));
        notifyDisplayedBundleListeners(this.displayedBundlesWithClasses);
    }

    /**
     * Returns a the set of  displayed classes.
     *
     * @return
     */
    public synchronized Set<Integer> getDisplayedClasses() {
        return Collections.unmodifiableSet(this.displayedBundlesWithClasses.values());
    }

    /**
     * Sets the displayed bundles by classes
     *
     * @param classes
     */
    public synchronized void setDisplayedClasses(Collection<Integer> classes) {
        this.displayedBundlesWithClasses = HashBiMap.create(classes.size());
        classes.forEach(c -> this.displayedBundlesWithClasses.inverse().put(c, this.allBundlesWithClasses.inverse().get(c)));
        notifyDisplayedBundleListeners(this.displayedBundlesWithClasses);
    }

    @JsonProperty
    public synchronized BiMap<Bundle, Integer> getDisplayedBundlesWithClasses() {
        return displayedBundlesWithClasses;
    }

    public synchronized BiMap<Bundle, Integer> getAllBundlesWithClasses() {
        return allBundlesWithClasses;
    }

    public synchronized void setFilter(TriPredicate<EvolutionDiagram, Integer, Double> predicate) {
        this.filter = predicate;
        postprocessAllBundles();
    }

    private synchronized void postprocessAllBundles() {
        Log.log(LogLevel.WARNING, "DataStorage", "Execute postProcess size=%d", allBundlesWithClassesUnfiltered.size());
        this.allBundlesWithClasses = HashBiMap.create(Maps.filterValues(allBundlesWithClassesUnfiltered, val -> filter.test(evolutionDiagram, val, evolutionDiagram.getBestEpsilon(val))));
        this.displayedBundlesWithClasses = allBundlesWithClasses;
        Log.log(LogLevel.WARNING, "DataStorage", "Executed postProcess size=%d", allBundlesWithClasses.size());

        notifyBundleListeners(this.allBundlesWithClasses);
        notifyDisplayedBundleListeners(this.displayedBundlesWithClasses);

    }

    /**
     * Function is called by setBundlesWithClasses, which is the last function called in ComputeEvolutionDiagram.
     */
    private synchronized void computeSomeBundleProperties(){
        // First we cut off the bundle ends to make sure they are fitting well.
        CutEnd.cutOffBundlesEndsForSet(this.displayedBundlesWithClasses.keySet());
        // Then we calculate the bundle properties.
        System.out.println("DataStorage - Computing all bundle properties.");
        long start = System.currentTimeMillis();

        for (Bundle b : this.allBundlesWithClasses.keySet()) {
            b.calculateForceProperties();
        }

        long end = System.currentTimeMillis();

        Log.log(LogLevel.INFO, "ForceRepresentative", "Calculated forcerep for no bundles: %d", this.allBundlesWithClasses.keySet().size());
        Log.log(LogLevel.INFO, "ForceRepresentative", "Force representative time: %d ms", end - start);
        start = System.currentTimeMillis();

        for (Bundle b : this.allBundlesWithClasses.keySet()) {
            b.calculateTurnProperties();
        }

        end = System.currentTimeMillis();
        Log.log(LogLevel.INFO, "TurnRepresentative", "Turn detection time: %d ms", end - start);
    }


    /**
     * Returns the class of the given bundle.
     *
     * @param b
     * @return
     */
    public synchronized int getClassFromBundle(Bundle b) {
        if (this.allBundlesWithClasses.get(b) == null) {
            return -1;
        } else {
            return this.allBundlesWithClasses.get(b);
        }
    }

    /**
     * Returns the bundle of the given class;
     *
     * @param c
     * @return
     */
    public synchronized Bundle getBundleFromClass(int c) {
        return this.allBundlesWithClasses.inverse().get(c);
    }


    public synchronized void clearBundles() {
        allBundlesWithClasses.clear();
        displayedBundlesWithClasses.clear();
        notifyBundleListeners(allBundlesWithClasses);
        notifyDisplayedBundleListeners(displayedBundlesWithClasses);
    }

    /**
     * This functions gets all possible properties of the bundles.
     */
    @JsonProperty
    public synchronized ArrayList<Map<String, Object>> getAllBundleProperties() {
        ArrayList<Map<String, Object>> allBundlesWithProperties = new ArrayList<>();
        ArrayList<String> attributes = new ArrayList<String>(BundleClassAttributes.names());

        Set<Integer> bundleClasses = Collections.unmodifiableSet(allBundlesWithClasses.inverse().keySet());
        ArrayList<Integer> sortedClasses = Lists.newArrayList(bundleClasses);
        sortedClasses.sort(Comparator.naturalOrder());

        for (Integer bundleClass : sortedClasses) {
            double bestEps = evolutionDiagram.getBestEpsilon(bundleClass);
            Map<String, Object> properties = new HashMap<String, Object>();

            for (int col = 0; col < attributes.size() + 2; col++) {
                switch (col) {
                    case 0:
                        properties.put("BundleClass", bundleClass);
                        break;
                    case 1:
                        properties.put("Bundle", allBundlesWithClasses.inverse().get(bundleClass));
                        break;
                    default:
                        properties.put(attributes.get(col - 2),
                                BundleClassAttributes.get(attributes.get(col - 2))
                                        .applyAsDouble(evolutionDiagram, bundleClass, bestEps));
                        break;
                }
            }

            allBundlesWithProperties.add(properties);
        }
        return allBundlesWithProperties;
    }


    public synchronized RoadMap getRoadMap() {
        return roadMap;
    }

    public synchronized void setRoadMap(RoadMap roadMap) {
        RoadMap old = this.roadMap;
        this.roadMap = roadMap;
        notifyNetworkListeners(old, this.roadMap);
    }


    /**
     * Notifies all instances listening to trajectory changes.
     */
    private synchronized void notifyTrajectoryListeners(List<Trajectory> added, List<Trajectory> removed) {
        TrajectoryChangeEvent evt = new TrajectoryChangeEvent(this, getTrajectories(), added, removed);
        trajListeners.forEach((TrajectoryChangeListener l) -> l.trajectoriesChanged(evt));
    }

    /**
     * Notifies all instances listening to bundle changes.
     */
    private synchronized void notifyBundleListeners(BiMap<Bundle, Integer> bundlesWithClasses) {
        BundleChangeEvent evt = new BundleChangeEvent(this, bundlesWithClasses);
        bundleListeners.forEach((BundleChangeListener l) -> l.bundlesChanged(evt));
    }

    /**
     * Notifies all instances listening to bundle changes.
     */
    private synchronized void notifyDisplayedBundleListeners(BiMap<Bundle, Integer> bundlesWithClasses) {
        BundleChangeEvent evt = new BundleChangeEvent(this, bundlesWithClasses);
        bundleListeners.forEach((BundleChangeListener l) -> l.displayedBundlesChanged(evt));
    }

    /**
     * Notifies all instances listening to diagram changes.
     */
    private synchronized void notifyDiagramListeners(EvolutionDiagram oldD, EvolutionDiagram newD) {
        DiagramChangeEvent evt = new DiagramChangeEvent(this, oldD, newD);
        diagramListeners.forEach((DiagramChangeListener l) -> l.diagramChanged(evt));
    }

    /**
     * Notifies all instances listening to network changes.
     *
     * @param oldRN  the old RoadNetwork.
     * @param newRN, the new Roadnetwork.
     */
    private synchronized void notifyNetworkListeners(RoadMap oldRN, RoadMap newRN) {
        NetworkChangeEvent evt = new NetworkChangeEvent(this, oldRN, newRN);
        networkListeners.forEach((NetworkChangeListener l) -> l.networkChanged(evt));
    }


    public synchronized void addTrajectoryListener(TrajectoryChangeListener listener) {
        trajListeners.add(listener);
    }

    public synchronized void addBundleListener(BundleChangeListener listener) {
        bundleListeners.add(listener);
    }

    public synchronized void addDiagramListener(DiagramChangeListener listener) {
        diagramListeners.add(listener);
    }

    public synchronized void addNetworkListeners(NetworkChangeListener listener) {
        networkListeners.add(listener);
    }

    public synchronized EvolutionDiagram getEvolutionDiagram() {
        return evolutionDiagram;
    }

    public synchronized void setEvolutionDiagram(EvolutionDiagram evolutionDiagram) {
        EvolutionDiagram old = this.evolutionDiagram;
        this.evolutionDiagram = evolutionDiagram;
        notifyDiagramListeners(old, evolutionDiagram);
    }


}
