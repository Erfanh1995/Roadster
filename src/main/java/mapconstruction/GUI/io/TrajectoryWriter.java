package mapconstruction.GUI.io;

import mapconstruction.trajectories.Trajectory;

import java.io.File;

/**
 * Interface fro writing trajectories to file.
 *
 * @author Roel
 */
public interface TrajectoryWriter {


    void write(Trajectory trajectory, File file);
}
