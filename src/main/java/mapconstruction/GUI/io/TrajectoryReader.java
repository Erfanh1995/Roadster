package mapconstruction.GUI.io;

import mapconstruction.trajectories.Trajectory;

import java.io.File;
import java.util.List;

/**
 * Interface for reading trajectories from an input file.
 *
 * @author Roel
 */
public interface TrajectoryReader {

    List<? extends Trajectory> parse(File f);

}
