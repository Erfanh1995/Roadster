package mapconstruction.algorithms.preprocessing;

import mapconstruction.algorithms.simplification.TrajectorySimplifier;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Preprocessor that simplifies trajectories.
 *
 * @author Roel
 */
public class SimplificationPreprocessor extends Preprocessor {

    /**
     * Simplifier to use.
     */
    private final TrajectorySimplifier simplifier;

    /**
     * error to use.
     */
    private final double error;

    public SimplificationPreprocessor(TrajectorySimplifier simplifier, double error) {
        this.simplifier = simplifier;
        this.error = error;
    }

    @Override
    protected List<Trajectory> runAlgorithm(List<Trajectory> trajectories) {
        List<Trajectory> result = new ArrayList<>();
        Log.log(LogLevel.STATUS, "Simplification", "Simplifying trajectories, using: %s.", simplifier.getClass().getSimpleName());
        Log.log(LogLevel.INFO, "Simplification", "Total number of points before: %d", trajectories.stream().mapToInt(Trajectory::numPoints).sum());
        long start = System.currentTimeMillis();

        for (Trajectory t : trajectories) {
            result.add(simplifier.simplify(t, error));
        }

        long end = System.currentTimeMillis();

        Log.log(LogLevel.INFO, "Simplification", "Total number of points after: %d", result.stream().mapToInt(Trajectory::numPoints).sum());
        Log.log(LogLevel.INFO, "Simplification", "Simplification time: %d ms", end - start);

        return result;
    }

}
