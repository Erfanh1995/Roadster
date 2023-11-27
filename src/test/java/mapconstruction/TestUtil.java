package mapconstruction;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Helper functions for testing.
 * @author Roel
 */
public class TestUtil {
    
    public static List<Point2D> doubleArrayToTrajectory(double[][] t) {
        List<Point2D> points = new ArrayList<>();
        Arrays.stream(t).forEachOrdered(xy -> points.add(new Point2D.Double(xy[0], xy[1])));
        return points;
    }
}
