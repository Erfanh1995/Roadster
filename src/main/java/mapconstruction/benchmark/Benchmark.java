package mapconstruction.benchmark;

import mapconstruction.GUI.io.BenchmarkManager;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;

import java.util.*;

public class Benchmark {

    private static String name;
    private static BenchmarkManager instance;
    private static Timing current;
    private static Map<String, Set<Bundle>> result;
    private static boolean enabled;
    private static Thread memMonitor = null;

    /**
     * Start a new global benchmark, overrides any existing benchmarks.
     */
    public static void start(BenchmarkManager bm, String n) {
        name = n;
        instance = bm;
        // preserve order of insertion
        result = new LinkedHashMap<>();
        current = new Timing("Global");
        current.time();
        enabled = true;
        if (memMonitor != null && !memMonitor.isAlive()) memMonitor.start();

        Log.log(LogLevel.INFO, "Benchmark", "Starting benchmark of bundling algorithm at %s", new Date());
    }

    /**
     * Stop benchmarking, usually called at the end of a benchmarked algorithm.
     */
    public static void stop() {
        if (!enabled) return;
        while (pop()) {}
        current.time();
        if (memMonitor != null) memMonitor.interrupt();

        Log.log(LogLevel.INFO, "Benchmark", "Completed benchmark of bundling algorithm at %s", new Date());
    }

    /**
     * Report all findings to the configured benchmark folder.
     */
    public static void report() {
        if (!enabled) return;
        Date now = new Date();
        // create yaml
        for (Map.Entry<String, Set<Bundle>> result : result.entrySet()) {
            // create image
            instance.saveSnapshot(name, now, result.getKey(), result.getValue());
        }
        instance.saveStats(name, now, current, result);

        Log.log(LogLevel.INFO, "Benchmark", "Saved benchmark results to %s/%s", name, now);

    }

    public static void stopAndReport() {
        stop();
        report();
    }

    /**
     * Push a new sub-benchmark onto the timings stack
     */
    public static void push(String format, Object... arguments) {
        if (!enabled) return;
        String name = String.format(format, arguments);
        // create and start a nested timing
        current = current.addSubtiming(name);
        current.time();
    }

    /**
     * Pop from a sub-benchmark on the timings stack
     */
    public static boolean pop() {
        if (enabled && current.hasParent()) {
            current.time(); // pause child timer
            current = current.getParent();
            return true;
        }
        return false;
    }

    /**
     * Create a new parallel timing, which doesn't stop the parent and needs to be closed upon its own. Additionally,
     * a parallel subtiming can be automatically terminated by wrapping it inside a try-catch block.
     */
    public static Timing parallel(String format, Object... arguments) {
        if (!enabled) return null;
        String name = String.format(format, arguments);
        // create an unbound subtiming and start it
        Timing subtiming = current.addSubtiming(name);
        subtiming.time();
        return subtiming;
    }

    /**
     * Create a new split benchmark on the current stack depth
     */
    public static void split(String format, Object... arguments) {
        if (pop()) {
            push(format, arguments);
        }
    }

    /**
     * Report a new (named) result. Used for example when reporting for multiple epsilon.
     */
    public static void addResult(String name, Set<Bundle> bundles) {
        if (!enabled) return;
        result.put(name, bundles);
    }

    public static void memMonitor() {
        memMonitor(false);
    }

    public static void memMonitor(boolean start) {
        memMonitor = new Thread(new MemoryMonitor());
        if (start) {
            memMonitor.start();
        }
    }

    private static class MemoryMonitor implements Runnable {

        @Override
        public void run() {
            Runtime current = Runtime.getRuntime();
            double mb = 1024 * 1024;
            try {
                while (!Thread.interrupted()) {
                    long total = current.totalMemory();
                    long free = current.freeMemory();
                    long max = current.maxMemory();
                    System.out.printf("Memory usage: %10.0f MB / %10.0f MB (%.0f MB)\r", (total - free) / mb, total / mb, max / mb);
                    Thread.sleep(1000);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}
