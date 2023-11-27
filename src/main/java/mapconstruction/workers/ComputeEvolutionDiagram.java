package mapconstruction.workers;

import com.google.common.collect.BiMap;
import mapconstruction.GUI.filter.AttributeTriPredicate;
import mapconstruction.GUI.filter.QuantifiedTriPredicate;
import mapconstruction.GUI.filter.TriPredicate;
import mapconstruction.algorithms.AbstractTrajectoryAlgorithm;
import mapconstruction.algorithms.bundles.MaximalSubbundleAlgorithm;
import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.algorithms.diagram.EvolutionDiagramBuilder;
import mapconstruction.attributes.BundleClassAttributes;
import mapconstruction.benchmark.Benchmark;
import mapconstruction.exceptions.AlgorithmAbortedException;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;
import mapconstruction.web.Controller;

import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.IntUnaryOperator;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Collectors;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

public class ComputeEvolutionDiagram extends AbortableAlgorithmWorker<EvolutionDiagram, Void> {
    static final double delta = 10;
    static final double epsSeg = 20;
    static final int minEps = 5;
    static final int maxEps = 200;
    static final double deltaEps = 2; // epsStep
    static final double minLength = 3;
    static final double lambdaFactor = 2;
    static final double minRelativeLifespan = 1;
    static final double minAbsoluteLifespan = 10;
    // these are actually already encoded somewhere else
    static final double bigH = 10;
    static final double bigC = 20;
    static final boolean fixConnections = true;
    final int minSize = MaximalSubbundleAlgorithm.minK;
    // maxBestEps = 75, set somewhere else
    public final TriPredicate<EvolutionDiagram, Integer, Double> predicate =
            new QuantifiedTriPredicate<EvolutionDiagram, Integer, Double>(QuantifiedTriPredicate.Quantifier.All, Arrays.asList(

                    new AttributeTriPredicate(BundleClassAttributes.get("Size"), (a, b, c) -> (double) minSize, AttributeTriPredicate.Operator.GreaterThanOrEqual)
                    , new AttributeTriPredicate(BundleClassAttributes.get("RelativeLifeSpan"), (a, b, c) -> minRelativeLifespan, AttributeTriPredicate.Operator.GreaterThan)
                    , new AttributeTriPredicate(BundleClassAttributes.get("LifeSpan"), (a, b, c) -> minAbsoluteLifespan, AttributeTriPredicate.Operator.GreaterThanOrEqual)
                    , new AttributeTriPredicate(BundleClassAttributes.get("MinContinuousLength"), (a, b, bEps) -> minLength * (double) bEps, AttributeTriPredicate.Operator.GreaterThanOrEqual)
            ));

    final IntUnaryOperator kStep = k -> k + 1;

    public ComputeEvolutionDiagram() {
        algo = EvolutionDiagramBuilder.multiplicative(deltaEps, lambdaFactor, minEps, maxEps, true, kStep);
    }

    @Override
    protected EvolutionDiagram doInBackground() {
        try {
            ///STORAGE.setEvolutionDiagram(null);
            AbstractTrajectoryAlgorithm<EvolutionDiagram> algo2 = (AbstractTrajectoryAlgorithm<EvolutionDiagram>) algo;

            STORAGE.setProgressAlgorithm(1);
            algo2.addListener(() -> {
                STORAGE.setProgressAlgorithm(algo2.getProgress());
            });
            Log.log(LogLevel.INFO, "ComputeEVO", "==> BuildDiagramWorker.class Starting to build diagram @ %s", Calendar.getInstance().getTime());
            Log.log(LogLevel.INFO, "ComputeEVO", "Using trajectories: %s", getTrajectoryLabels());
            List<Trajectory> trajectories = STORAGE.getTrajectories();

            int numPoints = trajectories.stream().mapToInt(Trajectory::numPoints).sum();
            Log.log(LogLevel.INFO, "ComputeEVO", "Total number of points: % d", numPoints);
            System.out.println("Starting computation of the evolution diagram");

            Benchmark.push("Multiplicative");
            long start = System.currentTimeMillis();
            EvolutionDiagram result = algo2.run(trajectories);
            long end = System.currentTimeMillis();
            Benchmark.pop();

            Log.log(LogLevel.INFO, "ComputeEVO", "Total running time:  %d ms", end - start);
            return result;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    @Override
    protected void done() {
        Benchmark.push("Finalizing");
        try {
            super.done();
            EvolutionDiagram diagram = get();

            int bundleCount = 0;
            for (double eps : diagram.getEpsilons()) {
                bundleCount += diagram.getBundleClasses(eps).size();
            }
            Log.log(LogLevel.INFO, "ComputeEVO", "Total number of epsilons: %d", diagram.getEpsilons().size());
            Log.log(LogLevel.INFO, "ComputeEVO", "Number of bundles before filtering: %d", bundleCount);

            STORAGE.setEvolutionDiagram(diagram);
            // Note: for the following two lines the order is of the utmost importance!!!
            updateDiagramBundles();
            STORAGE.setFilter(predicate);

            // TODO this computation is unused?
//            Set<Bundle> bundles = STORAGE.getDisplayedBundlesWithClasses()
//                    .entrySet()
//                    .stream()
//                    .map(Map.Entry::getKey)//.trim(STORAGE.getEvolutionDiagram().getBestEpsilon(entry.getValue())))
//                    .collect(Collectors.toSet());
//            Log.log(LogLevel.INFO, "ComputeEVO", "Number of bundles after filtering: %d", bundles.size());
//
//            Log.log(LogLevel.INFO, "ComputeEVO", "==> Evolution Diagram created @ %s", Calendar.getInstance().getTime());
            System.out.println("Computation of the evolution diagram has finished.");
        } catch (InterruptedException | ExecutionException ex) {
            if (!AlgorithmAbortedException.class.equals(ex.getCause().getClass())) {
                // Exception was not because the algorithm was aborted.
                Logger.getLogger(Controller.class.getName()).log(Level.SEVERE, null, ex);
                Log.log(LogLevel.ERROR, "Control", "Exception thrown: " + ex.getClass().getName());
            } else {
                System.out.println(ex.getMessage());
                Log.log(LogLevel.WARNING, "ComputeRN", "Algorithm aborted. No output generated");
            }
        } finally {
            STORAGE.setProgressAlgorithm(0);
        }
        Benchmark.pop();
    }

    public void updateDiagramBundles() {
        BiMap<Bundle, Integer> classBunMapping;

        classBunMapping = STORAGE.getEvolutionDiagram().bundlesWithClassesAtBest();

        Log.log(LogLevel.INFO, "ComputeEvolutionDiagram", "Bundles after updateDiagramBundles %d", classBunMapping.entrySet().size());

        STORAGE.setBundlesWithClasses(classBunMapping);
    }

    private List<String> getTrajectoryLabels() {
        return STORAGE.getTrajectories().stream().map(Trajectory::getLabel).collect(Collectors.toList());
    }
}


