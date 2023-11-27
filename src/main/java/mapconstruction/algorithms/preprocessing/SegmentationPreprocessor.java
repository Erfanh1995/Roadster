/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.algorithms.preprocessing;

import mapconstruction.algorithms.segmentation.TrajectorySegmenter;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Preprocessor that segments trajectories.
 *
 * @author Roel
 */
public class SegmentationPreprocessor extends Preprocessor {

    private final TrajectorySegmenter segmenter;

    public SegmentationPreprocessor(TrajectorySegmenter segmenter) {
        this.segmenter = segmenter;
    }


    @Override
    protected List<Trajectory> runAlgorithm(List<Trajectory> trajectories) {
        List<Trajectory> segmented = new ArrayList<>();

        // Segment all the trajectories and pass them to the algorithm
        Log.log(LogLevel.STATUS, "Segmentation", "Segmenting trajectories, using %s.", segmenter.getClass().getSimpleName());
        Log.log(LogLevel.INFO, "Segmentation", "Number of trajectories before: %d", trajectories.size());

        long start = System.currentTimeMillis();

        for (Trajectory t : trajectories) {
            checkAbort();
            segmented.addAll(segmenter.segment(t));
        }

        long end = System.currentTimeMillis();

        Log.log(LogLevel.INFO, "Segmentation", "Number of trajectories after: %d", segmented.size());
        Log.log(LogLevel.INFO, "Segmentation", "Segmentation time: %d ms", end - start);
        return segmented;
    }


}
