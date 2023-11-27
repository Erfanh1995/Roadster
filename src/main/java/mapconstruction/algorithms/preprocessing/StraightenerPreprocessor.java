package mapconstruction.algorithms.preprocessing;

import mapconstruction.algorithms.straightener.TrajectoryStraightener;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Preprocessor that removes annoying / z-artifacts from the trajectories.
 *
 * @author jorricks
 */
public class StraightenerPreprocessor extends Preprocessor {

    /**
     * Straightener to use
     */
    private final TrajectoryStraightener straightener;

    public StraightenerPreprocessor(TrajectoryStraightener straightener){
        this.straightener = straightener;
    }

    @Override
    protected List<Trajectory> runAlgorithm(List<Trajectory> trajectories){
        List<Trajectory> result = new ArrayList<>();
        Log.log(LogLevel.STATUS, "Straightener", "Straightening trajectories");
        Log.log(LogLevel.INFO, "Straightener", "Total number of points before: %d", trajectories.stream().mapToInt(Trajectory::numPoints).sum());
        long start = System.currentTimeMillis();

        for (Trajectory t : trajectories) {
            result.add(straightener.straighten(t));
        }

        long end = System.currentTimeMillis();

        Log.log(LogLevel.INFO, "Straightener", "Total number of points after: %d", result.stream().mapToInt(Trajectory::numPoints).sum());
        Log.log(LogLevel.INFO, "Straightener", "Straightening time: %d ms", end - start);

        return result;
    }
}
