package mapconstruction.algorithms.diagram;

import com.google.common.collect.*;
import mapconstruction.GUI.filter.TriPredicate;
import mapconstruction.attributes.BundleAttribute;
import mapconstruction.trajectories.Bundle;

import java.io.Serializable;
import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;

/**
 * Class keeping track of the evolution diagram, abstractly. It does not
 * actually draw the diagram, but keeps track of the different bundles that are
 * alive for different epsilon.
 *
 * @author Roel
 */
public class EvolutionDiagram implements Serializable {

    private static final long serialVersionUID = 1L;

    /**
     * Least slope of the length-eps diagram to consider it significant.
     */
    private static final int MIN_SIG_SLOPE = 10;

    /**
     * Maximum value for bestEps if possible, otherwise the birth eps is chosen.
     */
    private static final int MAX_BEST_EPS = 75;
    private static final double MIN_PLATEAU = 20;
    /**
     * States of the diagram for increasing epsilon.
     */
    private final TreeMap<Double, DiagramState> states;
    private final Map<Integer, Double> birthMoments;
    private final Map<Integer, Double> mergeMoments;
    /**
     * Cache for best epsilons per class.
     */
    private transient Map<Integer, Double> classBestEps;

    public EvolutionDiagram() {
        this.states = new TreeMap<>();
        birthMoments = new HashMap<>();
        mergeMoments = new HashMap<>();
        classBestEps = new HashMap<>();
    }

    /**
     * Dummy evolution diagram for only running the sweep line algorithm
     *
     * @param bundles
     * @param eps
     * @return
     */
    public static EvolutionDiagram createDummy(Iterable<Bundle> bundles, double eps) {
        EvolutionDiagram diagram = new EvolutionDiagram();
        int counter = 0;
        BiMap<Bundle, Integer> bundleClasses = HashBiMap.create();

        for (Bundle b : bundles) {
            diagram.addBirthMoment(counter, eps);
            bundleClasses.put(b, counter++);
        }

        DiagramState state = new DiagramState(bundleClasses, Collections.emptySet(), Collections.emptyMap());
        diagram.addState(eps, state);
        diagram.addState(2 * eps, new DiagramState(HashBiMap.create(), Collections.emptySet(), Collections.emptyMap()));
        return diagram;
    }

    /**
     * Returns the values for epsilon for which states have been computed.
     *
     * @return
     */
    public NavigableSet<Double> getEpsilons() {
        return Collections.unmodifiableNavigableSet((NavigableSet<Double>) states.keySet());
    }

    /**
     * Gets the bundle classes alive for the given epsilon. If for the given
     * epsilon no explicit state was computed, the state for the first epsilon
     * below the given value will be returned.
     *
     * @param epsilon
     * @return
     */
    public BiMap<Bundle, Integer> getBundleClasses(double epsilon) {
        return Maps.unmodifiableBiMap(states.floorEntry(epsilon).getValue().getBundleClasses());
    }

    /**
     * Gets births for the given epsilon. If for the given epsilon no explicit
     * state was computed, the state for the first epsilon below the given value
     * will be returned.
     *
     * @param epsilon
     * @return
     */
    public Set<Integer> getBirths(double epsilon) {
        return Collections.unmodifiableSet(states.floorEntry(epsilon).getValue().getBirths());
    }

    /**
     * Gets merges for the given epsilon. If for the given epsilon no explicit
     * state was computed, the state for the first epsilon below the given value
     * will be returned.
     *
     * @param epsilon
     * @return
     */
    public Map<Integer, Integer> getMerges(double epsilon) {
        return Collections.unmodifiableMap(states.floorEntry(epsilon).getValue().getMerges());
    }

    void addState(double epsilon, DiagramState state) {
        states.put(epsilon, state);
    }

    void addBirthMoment(int bundleClass, double epsilon) {
        birthMoments.put(bundleClass, epsilon);
    }

    void addMergeMoment(int bundleClass, double epsilon) {
        mergeMoments.put(bundleClass, epsilon);
    }

    /**
     * Returns the birth moment of the given class.
     *
     * @param bundleClass
     * @return
     */
    public double getBirthMoment(int bundleClass) {
        return nullIsNaN(birthMoments.get(bundleClass));
    }

    private double nullIsNaN(Double val) {
        return val == null ? Double.NaN : val;
    }

    /**
     * Returns the merge moment of the given class.
     * <p>
     * Returns the highest key if it is the last surviving bundle. Returns NaN
     * if the bundle calss does not merge, but is not the last one surviving.
     *
     * @param bundleClass
     * @return
     */
    public double getMergeMoment(int bundleClass) {
        Double result = mergeMoments.get(bundleClass);
        if (result == null) {
            if (getBundleClasses(states.lastKey()).containsValue(bundleClass)) {
                return states.lastKey();
            } else {
                return Double.NaN;
            }
        }
        return result;
    }

    /**
     * Returns the total age of the given class.
     *
     * @param bundleClass
     * @return
     */
    public double getLifeSpan(int bundleClass) {
        double birth = getBirthMoment(bundleClass);
        double merge = getMergeMoment(bundleClass);

        return merge - birth;
    }

    /**
     * Returns the state with an espilon striclty smaller than the given value
     *
     * @param epsilon
     * @return
     */
    DiagramState getPrevious(double epsilon) {
        return states.lowerEntry(epsilon).getValue();
    }

    /**
     * Returns whether the diagram is empty
     *
     * @return
     */
    public boolean isEmpty() {
        return states.isEmpty();
    }

    public int numClasses() {
        return birthMoments.keySet().stream().max(Comparator.naturalOrder()).orElse(0) + 1;
    }

    /**
     * Gets the last bundle of the given bundle class up to the given epsilon.
     *
     * @param bundleClass
     * @param epsilon
     * @return
     */
    public Bundle getBundleUpToLevel(int bundleClass, double epsilon) {
        double merge = getMergeMoment(bundleClass);
        DiagramState state;
        if (Double.isNaN(merge)) {
            double next = getBirthMoment(bundleClass);
            BiMap<Bundle, Integer> classes = getBundleClasses(next);
            while (true) {
                Entry<Double, DiagramState> entry = states.higherEntry(next);
                BiMap<Bundle, Integer> newClasses = entry.getValue().getBundleClasses();
                if (newClasses.containsValue(bundleClass)) {
                    classes = newClasses;
                    next = entry.getKey();
                } else {
                    return classes.inverse().get(bundleClass);
                }
            }
        } else {
            if (merge > epsilon) {
                state = states.floorEntry(epsilon).getValue();
            } else if (states.lastEntry().getValue().getBundleClasses().containsValue(bundleClass)) {
                state = states.lastEntry().getValue();
            } else {
                state = states.lowerEntry(merge).getValue();
            }
            Bundle result = state.getBundleClasses().inverse().get(bundleClass);
            return result;
        }
    }

    /**
     * Returns a bimap mapping bundles to their classes for all classes
     * discovered up to the given epsilon.
     * <p>
     * The representing bundles are the bundles for which the class was present
     * last.
     *
     * @param epsilon
     * @return
     */
    public BiMap<Bundle, Integer> getLastBundlesWithClassesUpToLevel(double epsilon) {
        BiMap<Bundle, Integer> result = HashBiMap.create();
        for (double eps : getEpsilons().descendingSet()) {
            if (eps <= epsilon) {
                for (Entry<Bundle, Integer> entry : getBundleClasses(eps).entrySet()) {
                    if (!result.containsValue(entry.getValue())) {
                        result.put(entry.getKey(), entry.getValue());
                    }
                }
            }
        }
        return result;
    }

    /**
     * Returns a bimap mapping bundles to their classes for all classes
     * discovered up to the given epsilon.
     * <p>
     * The representing bundles are the bundles for which the class was present
     * first.
     *
     * @param epsilon
     * @return
     */
    public BiMap<Bundle, Integer> getFirstBundlesWithClassesUpToLevel(double epsilon) {
        BiMap<Bundle, Integer> result = HashBiMap.create();
        for (double eps : getEpsilons()) {
            if (eps <= epsilon) {
                for (Entry<Bundle, Integer> entry : getBundleClasses(eps).entrySet()) {
                    if (!result.containsValue(entry.getValue())) {
                        result.put(entry.getKey(), entry.getValue());
                    }
                }
            }
        }
        return result;
    }

    /**
     * Constructs a new diagram by removing all the bundle classes that do not
     * satisfy the given predicate.
     * <p>
     * The additional parameter predEps is to indicate which level to use to
     * compute bundle-attributes.
     *
     * @param predicate
     * @param predEps
     * @return
     */
    public EvolutionDiagram filter(TriPredicate<EvolutionDiagram, Integer, Double> predicate, double predEps) {
        EvolutionDiagram diagram = new EvolutionDiagram();

        // bundle class associated with classes that were merged into it, that still have to be remerged.
        Multimap<Integer, Integer> stillToMerge = HashMultimap.create();

        for (Entry<Double, DiagramState> entry : this.states.entrySet()) {
            double eps = entry.getKey();
            DiagramState state = entry.getValue();

            // Compute new set of births, removing all unimportant bundles.
            Set<Integer> births = state.getBirths().stream()
                    .filter(bundleClass -> predicate.test(this, bundleClass, predEps))
                    .collect(Collectors.toSet());

            // Compute new bundle classes, removing all unimportant ones.
            BiMap<Bundle, Integer> bundleClasses = HashBiMap.create();
            for (Entry<Bundle, Integer> classEntry : state.getBundleClasses().entrySet()) {
                if (predicate.test(this, classEntry.getValue(), predEps)) {
                    bundleClasses.put(classEntry.getKey(), classEntry.getValue());
                }
            }

            // Compute new merges
            Map<Integer, Integer> merges = new HashMap<>();
            for (Entry<Integer, Integer> mergeEntry : state.getMerges().entrySet()) {
                int sourceBundleClass = mergeEntry.getKey();

                if (predicate.test(this, mergeEntry.getValue(), predEps)) {
                    // merge is ok, target satisfies predicate.
                    if (stillToMerge.containsKey(sourceBundleClass)) {
                        for (int i : stillToMerge.get(sourceBundleClass)) {
                            merges.put(i, mergeEntry.getValue());
                        }
                    } else if (predicate.test(this, sourceBundleClass, predEps)) {
                        merges.put(sourceBundleClass, mergeEntry.getValue());
                    }

                } else if (stillToMerge.containsKey(sourceBundleClass)) {
                    for (int i : stillToMerge.get(sourceBundleClass)) {
                        stillToMerge.put(mergeEntry.getValue(), i);
                        bundleClasses.put(getBundleUpToLevel(i, eps), i);
                    }
                    stillToMerge.removeAll(sourceBundleClass);
                } else if (predicate.test(this, sourceBundleClass, predEps)) {
                    stillToMerge.put(mergeEntry.getValue(), sourceBundleClass);
                }

            }
            for (int i : stillToMerge.values()) {
                bundleClasses.put(getBundleUpToLevel(i, eps), i);
            }

            DiagramState newState = new DiagramState(bundleClasses, births, merges);
            diagram.addState(eps, newState);
            for (int c : newState.getBirths()) {
                // Process birth moments
                diagram.addBirthMoment(c, eps);
            }

            for (int c : newState.getMerges().keySet()) {
                // Process merges
                diagram.addMergeMoment(c, eps);
            }

        }
        return diagram;

    }

    /**
     * Computes the best value for epsilon for the given bundle class.
     * <p>
     * It the the epsilon at the start of the first plateau in the length-eps
     * graph, after a signicant jump in length (steep slope).
     * <p>
     * Birth eps if no such epsilon can be found.
     *
     * @param bundleclass
     * @return
     */
    private double computeBestEpsFirstPlateau(int bundleclass) {
        double birth = getBirthMoment(bundleclass);
        double lastEps = birth;
        double bestEps = birth;

        boolean foundSteep = false;
        Bundle lastBundle = getBundleAt(bundleclass, lastEps);
        NavigableSet<Double> epsilons = getEpsilons();
        while (lastEps != epsilons.last()) {
            double nextEps = epsilons.higher(lastEps);
            if (nextEps > MAX_BEST_EPS) {
                break;
            }

            double diffEps = (nextEps - lastEps);
            Bundle nextBundle = getBundleAt(bundleclass, nextEps);

            if (nextBundle == null) {
                break;
            } else {
                double lastLength = BundleAttribute.AvgContinuousLength.applyAsDouble(lastBundle);
                double nextLength = BundleAttribute.AvgContinuousLength.applyAsDouble(nextBundle);
                double lengthDiff = (nextLength - lastLength);
                if (lengthDiff >= MIN_SIG_SLOPE * diffEps) {
                    // steep slope significant
                    foundSteep = true;
                    bestEps = nextEps;
                } else if (foundSteep && nextEps - bestEps < MIN_PLATEAU) {
                    // Now at a plateau, check if stable at least for MIN_LATEAU eps.

                } else if (foundSteep && nextEps - bestEps >= MIN_PLATEAU) {
                    // now at a stable plateu
                    return bestEps;
                }
            }
            lastEps = nextEps;
            lastBundle = nextBundle;
        }
        return bestEps;

    }

    /**
     * Returns the best value of epsilon for the given bundleclass.
     *
     * @param bundleClass
     * @return
     */
    public double getBestEpsilon(int bundleClass) {
        if (classBestEps == null) {
            classBestEps = new HashMap<>();
        }
        return classBestEps.computeIfAbsent(bundleClass, this::computeBestEpsFirstPlateau);
    }

    /**
     * Gets the bundles for all classes at their best epsilon.
     *
     * @return
     */
    public BiMap<Bundle, Integer> bundlesWithClassesAtBest() {
        BiMap<Bundle, Integer> map = HashBiMap.create(numClasses());
        for (int bundleClass : birthMoments.keySet()) {
            map.put(getBundleAt(bundleClass, getBestEpsilon(bundleClass)), bundleClass);
        }
        return map;
    }

    /**
     * Gets the bundle for the given class at the given epsilon.
     *
     * @param bundleclass
     * @param epsilon
     * @return
     */
    public Bundle getBundleAt(int bundleclass, double epsilon) {
        return getBundleClasses(epsilon).inverse().get(bundleclass);
    }

    public Set<Integer> getClasses() {
        return Collections.unmodifiableSet(birthMoments.keySet());
    }

}
