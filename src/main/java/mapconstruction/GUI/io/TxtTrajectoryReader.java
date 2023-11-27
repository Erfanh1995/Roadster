package mapconstruction.GUI.io;

import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Reads Trajectory from a txt file.
 * Expects the following format:
 * <p>
 * - Each line a sequence of 2 floating point numbers and an optiona integer
 * - Each line represents a point on the trajectory
 * - First number is x coord,
 * - second number y coord
 * - (optional) third number is  a timestamp: number of seconds after epoch time.
 *
 * @author Roel
 */
public class TxtTrajectoryReader implements TrajectoryReader {

    /**
     * Reads the trajectory from file.
     *
     * @param f
     * @return Trajectory read from file. Returns null if reading the file failed.
     */
    public FullTrajectory readFile(File f) {
        try (BufferedReader bf = new BufferedReader(new FileReader(f))) {
            // init list of points
            ArrayList<Point2D> points = new ArrayList<>();
            String line;
            while ((line = bf.readLine()) != null) {
                // Split on whitespace
                String[] numbers = line.split("\\s+");

                double x = Double.parseDouble(numbers[0]);
                double y = Double.parseDouble(numbers[1]);

                Point2D point = new Point2D.Double(x, y);

                if (points.isEmpty() || !point.equals(points.get(points.size() - 1))) {
                    // Ignore consecutive duplicate points
                    points.add(point);
                }

            }

            FullTrajectory t = new FullTrajectory(points);
            t.setLabel(f.getName());
            return t;
        } catch (IOException | NumberFormatException ex) {
            Log.log(LogLevel.ERROR, "TrajectoryIO", "Exception while reading trajectory: ", ex.getMessage());
            System.out.println("TrajectoryIO, Exception while reading trajectory: "+ ex.getMessage());
            Logger.getLogger(TxtTrajectoryReader.class.getName()).log(Level.SEVERE, null, ex);
        }
        return null;
    }

    @Override
    public List<? extends Trajectory> parse(File f) {
        Trajectory t = readFile(f);
        if (t == null) {
            return Collections.emptyList();
        } else {
            return Collections.singletonList(t);
        }
    }
}
