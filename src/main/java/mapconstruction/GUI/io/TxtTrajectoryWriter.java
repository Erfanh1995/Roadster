package mapconstruction.GUI.io;

import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Writes a list of trajectories to separate txt files.
 *
 * @author Roel
 */
public class TxtTrajectoryWriter implements TrajectoryWriter {

    @Override
    public void write(Trajectory trajectory, File file) {
        try (PrintWriter w = new PrintWriter(file)) {
            for (Point2D p : trajectory.points()) {
                w.format(Locale.forLanguageTag("en-US"), "%f %f \n", p.getX(), p.getY());
            }
        } catch (IOException ex) {
            Logger.getLogger(TxtTrajectoryWriter.class.getName()).log(Level.SEVERE, null, ex);
        }
    }


}
