package mapconstruction.algorithms.diagram;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;
import mapconstruction.algorithms.AbstractTrajectoryAlgorithm;
import mapconstruction.algorithms.bundles.BundleGenerationAlgorithm;
import mapconstruction.algorithms.bundles.KLSubbundleAlgorithm;
import mapconstruction.algorithms.bundles.MaximalSubbundleAlgorithm;
import mapconstruction.algorithms.bundles.graph.GeneratingSemiWeakFDLabelledGraph;
import mapconstruction.algorithms.bundles.sweep.FurthestEndpointSweep;
import mapconstruction.algorithms.distance.KdTree;
import mapconstruction.benchmark.Benchmark;
import mapconstruction.exceptions.AlgorithmAbortedException;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Point2D;
import java.io.*;
import java.util.*;
import java.util.Map.Entry;
import java.util.concurrent.*;
import java.util.function.DoubleUnaryOperator;
import java.util.function.IntUnaryOperator;

import static mapconstruction.GUI.datastorage.ConstantsStorage.ALGOCONSTANTS;

/**
 * Builder for the diagram that tracks evolution of bundles.
 * <p>
 * This builder does not actually draw the diagram, but keeps track of the
 * intermediate states.
 *
 * @author Roel (original author)
 * @author Jorrick Sleijster (modified by)
 */
public class EvolutionDiagramBuilder extends AbstractTrajectoryAlgorithm<EvolutionDiagram> {

    static final String LOGTAG = "Evolution";
    static final String LOGTAG1 = "DigDeep";
    /**
     * The value of lambda to use will be labdaFactor * epsilon
     */
    private final double lambdaFactor;
    /**
     * Maximum epsilon size for termination.
     */
    private final double maxEps;
    /**
     * Minimum epsilon size to start
     */
    private final double minEps;
    /**
     * Whether direction should be ignored when generating bundles.
     */
    private final boolean ignoreDirection;
    /**
     * Unary operator in the current value of epsilon that computes the next
     * value.
     */
    private final DoubleUnaryOperator nextEpsilon;
    /**
     * Type of increments
     */
    private final IncrType incrType;
    /**
     * value of incementer.
     */
    private final double incrementer;
    /**
     * Initial diagram to use. Used to extend a diagram with additional levels.
     */
    private final EvolutionDiagram initialDiagram;
    private final IntUnaryOperator kStep;
    /**
     * Whether we want to output message in the console whether the merges failed
     */
    private final boolean debugMerges = false;
    /**
     * ID of the next bundle class.
     */
    private int nextClassNumber = 0;
    /**
     * Algorithm used to generate the bundles in each step.
     */
    private BundleGenerationAlgorithm algo;
    private Set<Bundle> encounteredBundles;
    //To save data to avoid recomputation of the sets in digdeep
    //private MultiKeyMap savedsweeplines=new MultiKeyMap();
    private SortedMap<Double,result_var> digdeepresults = new TreeMap<Double,result_var>();

    private Map<Double, Set<Bundle>> savedmaxbun = new TreeMap<Double,Set<Bundle>>();
    private Map<Double, Map<Bundle,Bundle>> savedmaxmerge = new TreeMap<Double, Map<Bundle,Bundle>>();


    /**
     * Constructs a diagram builder with the given parameters
     *
     * @param lambdaFactor      factor for multiplying the current distance value to
     *                          get lambda.
     * @param maxEps            maximum epsilon
     * @param ignoreDirection
     */
    private EvolutionDiagramBuilder(double lambdaFactor, double minEps, double maxEps, boolean ignoreDirection, IncrType incrType, double incrementer, EvolutionDiagram initialDiagram, IntUnaryOperator kStep) {
        super();
        this.lambdaFactor = lambdaFactor;
        this.maxEps = maxEps;
        this.ignoreDirection = ignoreDirection;
        this.incrType = incrType;
        this.incrementer = incrementer;
        this.minEps = minEps;
        this.initialDiagram = initialDiagram;
        this.kStep = kStep;


        nextEpsilon = d -> {
            switch (incrType) {
                case Additive:
                    return d + incrementer;
                case Multiplicative:
                    return d == 0 ? 1 : d * incrementer;
                default:
                    throw new IllegalStateException();
            }
        };
    }

    public static EvolutionDiagramBuilder additive(double incrementer, double lambdaFactor, double minEps, double maxEps, boolean ignoreDirection, IntUnaryOperator kStep) {

        return new EvolutionDiagramBuilder(lambdaFactor, minEps, maxEps, ignoreDirection, IncrType.Additive, incrementer, new EvolutionDiagram(), kStep);
    }

    public static EvolutionDiagramBuilder multiplicative(double incrementer, double lambdaFactor, double minEps, double maxEps, boolean ignoreDirection, IntUnaryOperator kStep) {
        if (incrementer <= 1) {
            throw new IllegalArgumentException("Cannot build the diagram in Multiplicative mode if the  incrementer is not greater than 1.");
        }
        return new EvolutionDiagramBuilder(lambdaFactor, minEps, maxEps, ignoreDirection, IncrType.Multiplicative, incrementer, new EvolutionDiagram(), kStep);
    }

    public static EvolutionDiagramBuilder additive(double incrementer, double lambdaFactor, double minEps, double maxEps, boolean ignoreDirection, IntUnaryOperator kStep, EvolutionDiagram initialDiagram) {

        return new EvolutionDiagramBuilder(lambdaFactor, minEps, maxEps, ignoreDirection, IncrType.Additive, incrementer, initialDiagram, kStep);
    }

    public static EvolutionDiagramBuilder multiplicative(double incrementer, double lambdaFactor, double minEps, double maxEps, boolean ignoreDirection, IntUnaryOperator kStep, EvolutionDiagram initialDiagram) {
        if (incrementer <= 1) {
            throw new IllegalArgumentException("Cannot build the diagram in Multiplicative mode if the  incrementer is not greater than 1.");
        }
        return new EvolutionDiagramBuilder(lambdaFactor, minEps, maxEps, ignoreDirection, IncrType.Multiplicative, incrementer, initialDiagram, kStep);
    }

    /**
     * Build the diagram for the given list of trajectories in parallel
     *
     * @param trajectories
     * @return
     */
    public EvolutionDiagram runAlgorithmParallel(List<Trajectory> trajectories) {
        EvolutionDiagram diagram = initialDiagram;

        double epsilon;
        this.encounteredBundles = new HashSet<>();
        if (initialDiagram.isEmpty()) {
            Log.log(LogLevel.STATUS, LOGTAG, "Starting to build diagram from scratch");
            epsilon = minEps;
        } else {
            Log.log(LogLevel.STATUS, LOGTAG, "Starting to build diagram, extending an existing one");
            epsilon = nextEpsilon.applyAsDouble(diagram.getEpsilons().last());
            for (double e : diagram.getEpsilons()) {
                encounteredBundles.addAll(diagram.getBundleClasses(epsilon).keySet());
            }
            nextClassNumber = diagram.numClasses();
        }

        Log.log(LogLevel.INFO, LOGTAG, "Parameters for evolution diagram: lambdaFactor=%.2f, incr=%.2f, incrType=%s, minEps=%.2f, maxEps=%.2f, ignoreDir=%b", lambdaFactor, incrementer, incrType.name(), minEps, maxEps, ignoreDirection);

        // Assign all remaining threads to subtasks, keep one free for the current thread.
        ExecutorService executor = Executors.newFixedThreadPool(ALGOCONSTANTS.getNumThreads() - 1);
//        CompletionService<String> manager = new ExecutorCompletionService<>(executor);

//        Map<Double, Future<String>> results = new HashMap();
        Map<Double,Future<Pair<Set<Bundle>, Map<Bundle, Bundle>>>> results = new LinkedHashMap<>();

        Log.log(LogLevel.STATUS, LOGTAG, "Starting Threads to find all bundles.");
        Benchmark.push("Bundle generation");

        trajectories = Collections.synchronizedList(trajectories);

        // start workers to compute bundles
        while (epsilon <= maxEps /*&& result.size() > 1*/ && !aborted) {
            try {
                Callable<Pair<Set<Bundle>, Map<Bundle, Bundle>>> worker = new FindAllBundles(trajectories,
//                        new MaximalSubbundleAlgorithm(epsilon, epsilon * lambdaFactor, this.ignoreDirection, kStep)
                        new KLSubbundleAlgorithm(epsilon, epsilon * lambdaFactor, this.ignoreDirection)
                );
                results.put(epsilon, executor.submit(worker));

                // proper incrementing
                if (epsilon < maxEps && nextEpsilon.applyAsDouble(epsilon) > maxEps) {
                    epsilon = maxEps;
                } else {
                    epsilon = nextEpsilon.applyAsDouble(epsilon);
                }

                setProgress((int) (100 * epsilon / (maxEps - minEps + 1) / 2));
            } catch (AlgorithmAbortedException ex) {
                // algorithm aborted return partial diagram
                Log.log(LogLevel.WARNING, LOGTAG, "Algorithm aborted. Showing partial diagram");
                break;
            }
        }

        Benchmark.split("Evolution diagram");

        for (Map.Entry<Double,Future<Pair<Set<Bundle>, Map<Bundle, Bundle>>>> fep : results.entrySet()) {
            try {
//                String fp = ffp.get();
//
//                ObjectInputStream ois;
//                ois = new ObjectInputStream(new FileInputStream(fp));
//                Pair<Set<Bundle>, Map<Bundle, Bundle>> p = (Pair<Set<Bundle>, Map<Bundle, Bundle>>) ois.readObject();
//                ois.close();
                epsilon = fep.getKey();
                Pair<Set<Bundle>, Map<Bundle, Bundle>> p = fep.getValue().get();

                Log.log(LogLevel.INFO, LOGTAG, "Starting processBundles bundl=%d eps=%f", p.k.size(), epsilon);

                DiagramState state = processBundles(p.k, p.v, epsilon, diagram);

                // Add the state
                Log.log(LogLevel.INFO, LOGTAG, "Got processBundles births=%d merges=%d", state.getBirths().size(), state.getMerges().size());

                diagram.addState(epsilon, state);

                for (int c : state.getBirths()) {
                    // Process birth moments
                    diagram.addBirthMoment(c, epsilon);
                }

                for (int c : state.getMerges().keySet()) {
                    // Process merges
                    diagram.addMergeMoment(c, epsilon);
                }

                encounteredBundles.addAll(state.getBundleClasses().keySet());

                setProgress((int) (100 * epsilon / (maxEps - minEps + 1) / 2) + 50);
            } catch (AlgorithmAbortedException | InterruptedException e) {
                // algorithm aborted return partial diagram
                Log.log(LogLevel.WARNING, LOGTAG, "Algorithm aborted. Showing partial diagram");
                break;
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        Benchmark.pop();

        Benchmark.addResult("Final", encounteredBundles);

        setProgress(100);
        Log.log(LogLevel.STATUS, LOGTAG, "Diagram finished");
        return diagram;
    }

    @Override
    public EvolutionDiagram runAlgorithm(List<Trajectory> trajectories) {
        return runAlgorithmSequential(trajectories);
    }

    /**
     * Build the diagram for the given list of trajectories. If aborted early,
     * returns a partial diagram.
     *
     * @param trajectories
     * @return
     */
    public EvolutionDiagram runAlgorithmSequential(List<Trajectory> trajectories) {
        EvolutionDiagram diagram = initialDiagram;

        double epsilon;
        this.encounteredBundles = new HashSet<>();
        if (initialDiagram.isEmpty()) {
            Log.log(LogLevel.STATUS, LOGTAG, "Starting to build diagram from scratch");
            epsilon = minEps;
        } else {
            Log.log(LogLevel.STATUS, LOGTAG, "Starting to build diagram, extending an existing one");
            epsilon = nextEpsilon.applyAsDouble(diagram.getEpsilons().last());
            for (double e : diagram.getEpsilons()) {
                encounteredBundles.addAll(diagram.getBundleClasses(epsilon).keySet());
            }
            nextClassNumber = diagram.numClasses();
        }

        // Needed sets for finding stable bundles
        // Stable bundles
        Set<Bundle> result = new HashSet<Bundle>();
        // Stable bundles from previous state
        Set<Bundle> toBeRemoved = new HashSet<Bundle>();
        // Previously found bundles
        Set<Bundle> pr_result = new HashSet<Bundle>();
        // New bundles that we haven't seen before
        Set<Bundle> new_result = new HashSet<Bundle>();
        // Computed bundles in each iteration
        Set<Bundle> B;

        Set<Bundle> savedforlater = new HashSet<Bundle>();

        Set<Bundle> stable = new HashSet<Bundle>();
        Double d = new Double(0);
        //Map<Bundle, Bundle> pr_merges = new HashMap<Bundle, Bundle>();

        Log.log(LogLevel.INFO, LOGTAG, "Parameters for evolution diagram: lambdaFactor=%.2f, incr=%.2f, incrType=%s, minEps=%.2f, maxEps=%.2f, ignoreDir=%b", lambdaFactor, incrementer, incrType.name(), minEps, maxEps, ignoreDirection);

        while (epsilon <= maxEps /*&& result.size() > 1*/ && !aborted) {

            try {
                if (!new_result.isEmpty()) {
                    new_result.clear();
                }
                BundleGenerationAlgorithm lambdaAlgo = new KLSubbundleAlgorithm(epsilon, epsilon * lambdaFactor, this.ignoreDirection);
                algo = lambdaAlgo;
                Set<Bundle> q = new HashSet<Bundle>();
                if (epsilon > 2*minEps && !digdeepresults.subMap(epsilon/4,epsilon/2).isEmpty() && digdeepresults.subMap(epsilon/4,epsilon/2).lastKey() != new Double(epsilon/4)){

                    d= digdeepresults.subMap(new Double(epsilon/4), new Double(epsilon/2)).lastKey();
                    q = savedmaxbun.get(d);
                }
                Log.log(LogLevel.STATUS, LOGTAG, "Finding all bundles.");
                // finding all bundles for a fixed epsilon
                B = algo.run(trajectories);
                for (Bundle b : B) {
                    Bundle i = CheckContinious(mergeSet(pr_result,stable),b,-1,epsilon);

                    if (i != null ) {
                        //adding to stable results
                        result.add(b);
                        //removing from unstable results
                        //pr_result.remove(i);
                        if (pr_result.contains(i)){
                            toBeRemoved.add(i);
                            if (q != null){
                                if (!q.isEmpty()){
                                    Bundle j = CheckContinious(q,i,-1,epsilon/2);
                                    if (j != null){
                                        savedforlater.add(j);
                                    }
                                }

                            }
                        }
                    }
                    else {
                        //newly discovered bundle
                        new_result.add(b);
                    }

                }
                // go further to see if the left bundles in pr_result are stable
                //for (Bundle b : pr_result) {
                Map<Bundle, Bundle> merges = lambdaAlgo.getMerges();


                //}
                if (epsilon == minEps){
                    /*Iterator<Bundle> iterator = merges.values().iterator();
                    while (iterator.hasNext()){
                        Bundle element = iterator.next();
                        if (!result.contains(element))
                        {
                            iterator.remove();
                        }
                    }*/
                    digdeepresults.put(new Double(epsilon),new result_var(epsilon,result,merges));
                }
                else if (epsilon > minEps){

                    if (digdeepresults.containsKey(new Double(epsilon/2))){
                        Set <Bundle> v = digdeepresults.get(new Double(epsilon/2)).getBundles();
                        Map <Bundle,Bundle> temp = digdeepresults.get(new Double(epsilon/2)).getMerges();
                        digdeepresults.put(new Double(epsilon/2), new result_var(epsilon/2,mergeSet(v,toBeRemoved),temp));
                        // if (!digdeepresults.subMap(epsilon/4,epsilon/2).isEmpty() && digdeepresults.subMap(epsilon/4,epsilon/2).lastKey() != new Double(epsilon/4)){
                        if (!savedforlater.isEmpty()){
                            System.out.println(savedforlater);

                            result_var p = digdeepresults.get(d);
                            Set<Bundle> u = p.getBundles();
                            Map<Bundle, Bundle> n = p.getMerges();
                            u = mergeSet(u,savedforlater);
                            digdeepresults.put(d,new result_var(d,u,n));
                            savedforlater.clear();
                        }


                        //   }
                    }

                    digdeepresults.put(new Double(epsilon),new result_var(epsilon,result,merges));
                }
                //really a hack
                /*if (epsilon == 160)
                {
                    Iterator<Bundle> iterator = merges.values().iterator();
                    while (iterator.hasNext()){
                        Bundle element = iterator.next();
                        if (!result.contains(element))
                        {
                            iterator.remove();
                        }
                    }
                    digdeepresults.put(new Double(epsilon),new result_var(epsilon,result,merges));
                } */

                //pr_merges.clear();
                //pr_merges.putAll(merges);

                /*
                Log.log(LogLevel.STATUS, LOGTAG, "Building state");

                DiagramState state = processBundles(result, merges, epsilon, diagram);

                // Add the state
                diagram.addState(epsilon, state);

                for (int c : state.getBirths()) {
                    // Process birth moments
                    diagram.addBirthMoment(c, epsilon);
                }

                for (int c : state.getMerges().keySet()) {
                    // Process merges
                    diagram.addMergeMoment(c, epsilon);
                }

                encounteredBundles.addAll(state.getBundleClasses().keySet());
                */
                pr_result.removeAll(toBeRemoved);

                if (!pr_result.isEmpty() && ((epsilon+(epsilon/2))/4)%1 == 0 ){
                    digdeep((epsilon+(epsilon/2))/4,(epsilon+(epsilon/2))/2, epsilon, pr_result,trajectories);}

                pr_result.clear();
                pr_result.addAll(new_result);
                stable.clear();
                stable.addAll(result);
                toBeRemoved.clear();

                setProgress((int) (100 * epsilon / (maxEps - minEps + 1)));
                // proper incrementing
                if (epsilon < maxEps && nextEpsilon.applyAsDouble(epsilon) > maxEps) {
                    epsilon = maxEps;
                } else {
                    epsilon = nextEpsilon.applyAsDouble(epsilon);
                }
                Log.log(LogLevel.STATUS, LOGTAG, "Next Epsilon");
/*
                if (epsilon < maxEps && state.getBundleClasses().size() <= 1) {
                    epsilon = maxEps;
                }*/
            } catch (AlgorithmAbortedException ex) {
                // algorithm aborted return partial diagram
                Log.log(LogLevel.WARNING, LOGTAG, "Algorithm aborted. Showing partial diagram");
                break;
            }
        }

        Set s = digdeepresults.entrySet();
        Iterator i = s.iterator();
        while (i.hasNext())
        {
            Map.Entry m = (Map.Entry)i.next();
            double key = (Double) m.getKey();
            result_var r = (result_var)m.getValue();
            Set<Entry<Bundle,Bundle>> entryset =  r.getMerges().entrySet();
            Iterator<Entry<Bundle,Bundle>> iterator = entryset.iterator();
            while (iterator.hasNext()){
                Entry<Bundle,Bundle> element = iterator.next();
                if (!r.getBundles().contains(element.getKey()) || !r.getBundles().contains(element.getValue()))
                {
                    iterator.remove();
                }
            }
            Log.log(LogLevel.STATUS, LOGTAG, "Building state");
            DiagramState state = processBundles(r.getBundles(), r.getMerges(), key, diagram);
            diagram.addState(key, state);
            for (int c : state.getBirths()) {
                // Process birth moments
                diagram.addBirthMoment(c, key);
            }

            for (int c : state.getMerges().keySet()) {
                // Process merges
                diagram.addMergeMoment(c, key);
            }
            encounteredBundles.addAll(state.getBundleClasses().keySet());
        }

        setProgress(100);
        Log.log(LogLevel.STATUS, LOGTAG, "Diagram finished");
        return diagram;
    }

    /**
     * Finds the hidden stable bundles
     * and return the updated stable set of bundles
     *
     *
     * @param trajectories, epsilon, epsilon, the hidden bundle and its size, and set of stable bundles
     */
    public void digdeep(double e1, double e2, double e ,Set<Bundle> pr_results, List<Trajectory> trajectories){
        Log.log(LogLevel.STATUS, LOGTAG1, "Digdeep");
        Set<Bundle> stable1 = new HashSet<Bundle>();
        Set<Bundle> stable2 = new HashSet<Bundle>();
        Set<Bundle> stable3 = new HashSet<Bundle>();
        Set<Bundle> case1 = new HashSet<Bundle>();
        Set<Bundle> case2 = new HashSet<Bundle>();
        //Set<Bundle> results = pr_results;

        Set<Bundle> S1 = new HashSet<Bundle>();
        Set<Bundle> S2 = new HashSet<Bundle>();
        Map<Bundle, Bundle> merges2;
        Map<Bundle, Bundle> merges1;


        if (savedmaxbun.containsKey(e1)){
            S1 = savedmaxbun.get(e1);
            merges1 = savedmaxmerge.get(e1);
        }
        else {
            BundleGenerationAlgorithm bundleAlgo1;
            bundleAlgo1 = new KLSubbundleAlgorithm(e1, e1 * lambdaFactor, this.ignoreDirection);
            S1 = bundleAlgo1.run(trajectories);
            savedmaxbun.put(e1, S1);
            merges1 = bundleAlgo1.getMerges();
            savedmaxmerge.put(e1,merges1);
        }
        if (savedmaxbun.containsKey(e2)){
            S2 = savedmaxbun.get(e2);
            merges2 = savedmaxmerge.get(e2);
        }
        else {
            BundleGenerationAlgorithm bundleAlgo2;
            bundleAlgo2 = new KLSubbundleAlgorithm(e2, e2 * lambdaFactor, this.ignoreDirection);
            S2 = bundleAlgo2.run(trajectories);
            savedmaxbun.put(e2, S2);
            merges2 = bundleAlgo2.getMerges();
            savedmaxmerge.put(e2,merges2);
        }

        for (Bundle b : pr_results) {

        /*    int k = b.size();
            if (savedsweeplines.containsKey(e1, k)) {
                S1 = (Set<Bundle>) savedsweeplines.get(e1, k);
            } else {
                SweeplineBundleAlgorithm bundleAlgo1;
                bundleAlgo1 = new SweeplineBundleAlgorithm(e1, k, this.ignoreDirection);
                S1 = bundleAlgo1.run(trajectories);
                savedsweeplines.put(e1, k, S1);
            }
            if (savedsweeplines.containsKey(e2, k)) {
                S2 = (Set<Bundle>) savedsweeplines.get(e2, k);
            } else {
                SweeplineBundleAlgorithm bundleAlgo2;
                bundleAlgo2 = new SweeplineBundleAlgorithm(e2, k, this.ignoreDirection);
                S2 = bundleAlgo2.run(trajectories);
                savedsweeplines.put(e2, k, S2);
            }
*/


            Bundle i = CheckContinious(S1, b, -1,e1);
            Bundle j = CheckContinious(S2, b, 1,e2);

            if (i != null && j != null) {
                // it's stable!!
                stable1.add(i);
                stable2.add(j);
                stable3.add(b);
                //pr_results.remove(b);
                //return ;
            }
            if ((i == null && j != null)) {
                // check further for new e1 and e2
                case1.add(j);
                stable3.add(b);
                //pr_results.remove(b);

                //return s;
            } else if (i != null && j == null) {
                // check further for new e1 and e2
                case2.add(i);
                stable3.add(b);
                //pr_results.remove(b);

                //return s;
            }
            else  {
                //it's not stable.
                //pr_results.remove(b);
                //return s;
            }
            //making an state for the diagram

        }

        double key1 = digdeepresults.tailMap(e1).firstKey();
        double key2 = digdeepresults.tailMap(e2).firstKey();
        for(Bundle b: digdeepresults.get(key1).getBundles() ){
            Bundle i = CheckContinious(S1, b, -1,key1);
            if (i != null)
            {
                stable1.add(i);
            }
        }
        for(Bundle b: digdeepresults.get(key2).getBundles()){
            Bundle i = CheckContinious(S2, b, -1,key2);
            if (i != null)
            {
                stable2.add(i);
            }
        }
        double key3 = digdeepresults.headMap(e1).lastKey();
        double key4 = digdeepresults.headMap(e2).lastKey();
        for(Bundle b: digdeepresults.get(key3).getBundles() ){
            Bundle i = CheckContinious(S1, b, 1,e1);
            if (i != null)
            {
                stable1.add(i);
            }
        }
        for(Bundle b: digdeepresults.get(key4).getBundles()){
            Bundle i = CheckContinious(S2, b, 1,e2);
            if (i != null)
            {
                stable2.add(i);
            }
        }
        Set <Bundle> v = digdeepresults.get(new Double(e)).getBundles();
        Map <Bundle,Bundle> tempmerge = digdeepresults.get(new Double(e)).getMerges();
        digdeepresults.put(new Double(e), new result_var(new Double(e),mergeSet(v,stable3),tempmerge));
        /*
        Iterator<Bundle> iterator1 = merges1.values().iterator();
        while (iterator1.hasNext()){
            Bundle element1 = iterator1.next();
            if (!mergeSet(stable1,case2).contains(element1))
            {
                iterator1.remove();
            }
        }
        Iterator<Bundle> iterator2 = merges2.values().iterator();
        while (iterator2.hasNext()){
            Bundle element2 = iterator2.next();
            if (!mergeSet(stable2,case1).contains(element2))
            {
                iterator2.remove();
            }
        }
*/
        if (!stable1.isEmpty()){

            if(digdeepresults.containsKey(e1))
            {
                result_var temp = digdeepresults.get(e1);
                temp.result = mergeSet(temp.result,mergeSet(stable1,case2));
                temp.merges.putAll(merges1);
                digdeepresults.put(new Double(e1),temp);
            }
            else
            {
                digdeepresults.put(new Double(e1),new result_var(e1,mergeSet(stable1,case2),merges1));
            }
        }
        if (!stable2.isEmpty()){
            if (digdeepresults.containsKey(e2))
            {
                result_var temp = digdeepresults.get(e2);
                temp.result = mergeSet(temp.result,mergeSet(stable2,case1));
                temp.merges.putAll(merges2);
                digdeepresults.put(new Double(e2),temp);
            }
            else
            {
                digdeepresults.put(new Double(e2),new result_var(e2,mergeSet(stable2,case1),merges2));
            }

        }

        //case 1
        if (((e2 + (e2 - e1) / 2) / 2) % 1 == 0 && (e2 + (e2 - e1) / 2)<=maxEps && !case1.isEmpty()){ // limit the number of runs
            digdeep((e2 + (e2 - e1) / 2) / 2, e2 + ((e2 - e1) / 2), e2, case1, trajectories);

        }
        //case 2
        if (((e1 + (e2 - e1) / 2) / 2) % 1 == 0 && (e1 + (e2 - e1) / 2)<=maxEps && !case2.isEmpty()){ // limit the number of runs
            digdeep((e1 + (e2 - e1) / 2) / 2, e1 + ((e2 - e1) / 2), e1, case2, trajectories);

        }
        if (savedmaxbun.containsKey((e2 + (e2 - e1) / 2) / 2))
        {
            Set<Bundle> m1 = savedmaxbun.get((e2 + (e2 - e1) / 2) / 2);
            Set<Bundle> extra1 = digdeepresults.get((e2 + (e2 - e1) / 2) / 2).getBundles();
            Map<Bundle,Bundle> mer1 = digdeepresults.get((e2 + (e2 - e1) / 2) / 2).getMerges();
            Set<Bundle> s1 = new HashSet<Bundle>();
            for (Bundle b: m1)
            {
                Bundle i = CheckContinious(case2, b, -1,(e2 + (e2 - e1) / 2) / 2);
                if (i != null){
                    s1.add(b);
                }
            }
            digdeepresults.put(new Double((e2 + (e2 - e1) / 2) / 2),new result_var((e2 + (e2 - e1) / 2) / 2,mergeSet(extra1,s1),mer1));
        }
        if (savedmaxbun.containsKey((e1 + (e2 - e1) / 2) / 2))
        {
            Set<Bundle> m2 = savedmaxbun.get((e1 + (e2 - e1) / 2) / 2);
            Set<Bundle> extra2 = digdeepresults.get((e1 + (e2 - e1) / 2) / 2).getBundles();
            Map<Bundle,Bundle> mer2 = digdeepresults.get((e1 + (e2 - e1) / 2) / 2).getMerges();
            Set<Bundle> s2 = new HashSet<Bundle>();
            for (Bundle b: m2)
            {
                Bundle i = CheckContinious(case1, b, +1,(e1 + (e2 - e1) / 2) / 2);
                if (i != null){
                    s2.add(b);
                }
            }
            digdeepresults.put(new Double((e1 + (e2 - e1) / 2) / 2),new result_var((e1 + (e2 - e1) / 2) / 2,mergeSet(extra2,s2),mer2));
        }

    }
    // to check if a bundle is equal to another
    private Bundle CheckContinious (Set<Bundle> s, Bundle b, int indicator, double e) {
        //executes if we are looking for a smaller bundle
        if (indicator > 0)
            for (Bundle i: s)
                if (i.size() == b.size() && (i.hasAsSubBundle(b) || i.hasAsLambdaSubBundle(b,e*lambdaFactor) || i.equals(b)))
                    return i;
        if (indicator <= 0) //executes if we are looking for a larger bundle
            for (Bundle i: s)
                if (i.size() == b.size() && (b.hasAsSubBundle(i) || b.hasAsLambdaSubBundle(i,e*lambdaFactor) || b.equals(i)))
                    return i;
        return null;
    }

    /**
     * Processes the given bundles to generate a new state.
     *
     * @param result
     * @param epsilon
     * @param diagram
     * @return
     */
    private DiagramState processBundles(Set<Bundle> result, Map<Bundle, Bundle> bundleMerges, double epsilon, EvolutionDiagram diagram) {

        Log.log(LogLevel.STATUS, "processBundles", "Adding %d bundles", result.size());

        BiMap<Bundle, Integer> bundleClasses = HashBiMap.create();
        Set<Integer> births = new HashSet<>();
        Map<Integer, Integer> merges = new HashMap<>();

        if (diagram.isEmpty()) {
            handleFirstState(result, bundleClasses, births);
        } else {
            // Get the previous state
            DiagramState previousState = diagram.getPrevious(epsilon);
            KdTree<Bundle> kdQuery = new KdTree<>(4);
//            kdQuery.insertAll(previousState.getBundleClasses().keySet(), (k -> {
//                Subtrajectory r = k.getOriginalRepresentative();
//                Point2D s = r.getFirstPoint();
//                Point2D t = r.getLastPoint();
//                return Arrays.asList(s.getX(), s.getY(), t.getX(), t.getY());
//            }));

            for (Bundle bunNew : result) {

                boolean isContinuation = tryFindContinuation(previousState, bunNew, bundleClasses, kdQuery, epsilon);

                if (!isContinuation && !encounteredBundles.contains(bunNew)) {
                    // No old bundle found that the new one continues.
                    // Additionally, it is not a reappearing bundle
                    // add as new class

                    Log.log(LogLevel.STATUS, "processBundles", "Added a bundle");

                    bundleClasses.put(bunNew, nextClassNumber);

                    // Current birth moment
                    births.add(nextClassNumber);
                    nextClassNumber++;
                }

            }

            // Determine where the old classes have merged into
            // Find class numbers of classes that are no longer present.
            Set<Integer> mergedClasses = new HashSet<>(previousState.getBundleClasses().values());
            mergedClasses.removeAll(bundleClasses.values());

            Log.log(LogLevel.STATUS, "processBundles", "mergedClasses size %d", mergedClasses.size());

            for (int c : mergedClasses) {
                tryFindMerge(previousState, c, bundleClasses, epsilon, merges, bundleMerges);

            }

        }
        Log.log(LogLevel.STATUS, "processBundles", "Reported %d bundle", bundleClasses.size());
        return new DiagramState(bundleClasses, births, merges);
    }

    private void tryFindMerge(DiagramState previousState, int bundleClass, BiMap<Bundle, Integer> bundleClasses, double epsilon, Map<Integer, Integer> merges, Map<Bundle, Bundle> bundleMerges) {
        // Find into which classes the bundles have merged.

        Bundle mergedBundle = previousState.getBundleClasses().inverse().get(bundleClass);
        // find other class
        for (Entry<Bundle, Integer> bunClassPair : bundleClasses.entrySet()) {
            // Candidate
            Bundle otherBundle = bunClassPair.getKey();
            int otherClass = bunClassPair.getValue();
            if (otherBundle.hasAsLambdaSubBundle(mergedBundle, epsilon * lambdaFactor)) {
                // We allow a decrease of size
                merges.put(bundleClass, otherClass);
                return; // Found the merge for this class
            }
        }
        // Another attempt to find merges
        // find find correct bundle in bundle merges
        for (Entry<Bundle, Bundle> entry : bundleMerges.entrySet()) {
            Bundle from = entry.getKey();
            Bundle to = entry.getValue();
            if (from.hasAsLambdaSubBundle(mergedBundle, epsilon * lambdaFactor)) {
                // Find proper class
                while (!bundleClasses.containsKey(to) && bundleMerges.containsKey(to)) {
                    to = bundleMerges.get(to);
                    if (to == null) {
                        System.err.println("to == null");
                        Log.log(LogLevel.WARNING, LOGTAG, "to == null");
                        return;
                    }
                }
                if (debugMerges) {
                    System.err.println("WARNING: Second merge attempt used!");
                }
                Log.log(LogLevel.WARNING, LOGTAG, "Second merge attempt used!");
                merges.put(bundleClass, bundleClasses.get(to));
                return;
            }
        }
        if (debugMerges) {
            System.err.println("WARNING: No merge found!");
        }
        Log.log(LogLevel.WARNING, LOGTAG, "No merge found!");
    }

    private boolean tryFindContinuation(DiagramState previousState, Bundle bunNew, BiMap<Bundle, Integer> bundleClasses, KdTree<Bundle> kdQuery, double epsilon) {
//        Subtrajectory representative = bunNew.getOriginalRepresentative();
//        Point2D s = representative.getFirstPoint();
//        Point2D t = representative.getLastPoint();
        Set<Bundle> candidates = previousState.getBundleClasses().keySet(); // kdQuery.rangeQuery(2 * epsilon, s.getX(), s.getY(), t.getX(), t.getY());

        // For every new bundle, we have to check to every previous bundle
        // whether it is a continuation of a previous bundle.
        // If so, we can copy the class.
        // Otherwise we add a new class
        // We prefer proper subbundles over lambda subbundles. First attempt,
        // proper subbundles
        for (Bundle bunOld : candidates) {
            int classNumber = previousState.getBundleClasses().get(bunOld);
            if (bunNew.size() == bunOld.size() && !bundleClasses.containsValue(classNumber)) {
                // finally check for subbundle, as this is the most expensive check
                if (bunNew.hasAsSubBundle(bunOld)) {
                    // Continuation of old bundle
                    bundleClasses.put(bunNew, classNumber);
                    return true;
                }
            }
        }
        // Second attempt, with lambda subbundle.
        for (Bundle bunOld : candidates) {
            int classNumber = previousState.getBundleClasses().get(bunOld);
            if (bunNew.size() == bunOld.size() && !bundleClasses.containsValue(classNumber)) {
                // finally check for lambdasubbundle, as this is the most expensive check
                if (bunNew.hasAsLambdaSubBundle(bunOld, epsilon * lambdaFactor)) {
                    // Continuation of old bundle
                    bundleClasses.put(bunNew, classNumber);
                    return true;
                }
            }
        }

        return false;
    }

    private void handleFirstState(Set<Bundle> result, BiMap<Bundle, Integer> bundleClasses, Set<Integer> births) {
        // No previous states yet to compare to
        Log.log(LogLevel.STATUS, "FirstState", "Adding %d bundles.", result.size());
        for (Bundle b : result) {
            // Each bundle its own class
            bundleClasses.put(b, nextClassNumber);

            // Current birth moment
            births.add(nextClassNumber);

            // Note, we do not add any merges, as no previous clss is present.
            nextClassNumber++;
        }
    }

    @Override
    public void abort() {
        super.abort(); //To change body of generated methods, choose Tools | Templates.
        algo.abort();
    }

    /**
     * Enum indicating ways to increment epsilon.
     */
    private enum IncrType {
        Additive,
        Multiplicative
    }

    private static <T> Set<T> mergeSet(Set<T> a, Set<T> b)
    {
        return new HashSet<T>() {{
            addAll(a);
            addAll(b);
        } };
    }

}

class Pair<K, V> implements Serializable {
    private static final long serialVersionUID = -7115961138233562435L;
    K k;
    V v;

    public Pair(K k, V v) {
        this.k = k;
        this.v = v;
    }
}

class result_var {

    Set<Bundle> result;
    double epsilon;
    Map<Bundle, Bundle> merges;


    public result_var (double epsilon,Set<Bundle> result,Map<Bundle, Bundle> merges){
        this.result = result;
        this.epsilon = epsilon;
        this.merges = merges;
    }

    public double getEpsilon(){
        return this.epsilon;
    }

    public Set<Bundle> getBundles(){
        return this.result;
    }

    public Map<Bundle, Bundle> getMerges(){
        return this.merges;
    }

}

class FindAllBundles implements Callable<Pair<Set<Bundle>, Map<Bundle, Bundle>>> {

    List<Trajectory> trajectories;
    BundleGenerationAlgorithm lambdaAlgo;

    FindAllBundles(List<Trajectory> trajectories, BundleGenerationAlgorithm lambdaAlgo) {
        this.trajectories = trajectories;
        this.lambdaAlgo = lambdaAlgo;
    }


    @Override
    public Pair<Set<Bundle>, Map<Bundle, Bundle>> call() throws Exception {
        Set<Bundle> result;
        Log.log(LogLevel.STATUS, EvolutionDiagramBuilder.LOGTAG, "Finding all bundles.");
        result = lambdaAlgo.run(trajectories);
        Log.log(LogLevel.STATUS, EvolutionDiagramBuilder.LOGTAG, "Found %d bundles.", result.size());
        Pair<Set<Bundle>, Map<Bundle, Bundle>> p = new Pair<Set<Bundle>, Map<Bundle, Bundle>>(result, lambdaAlgo.getMerges());

        return p;

//        File f = File.createTempFile("bundle", "bundle");
//        ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(f));
//        oos.writeObject(p);
//        oos.close();
//
//        return f.getAbsolutePath();
    }


}
	
	
	
