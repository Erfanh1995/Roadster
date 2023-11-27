package mapconstruction.algorithms.simplification;

import mapconstruction.trajectories.SimplifiedTrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

/**
 * Different algorithms for trajectory simplification.
 *
 * @author Roel
 */
public class  SimplificationMethod {

    public static final TrajectorySimplifier RDP = new RDP();
    public static final TrajectorySimplifier Greedy = new Greedy();

    /**
     * Simplifies the given trajectory using Ramer-Douglas-Peucker with the
     * given allowed error.
     */
    private static class RDP implements TrajectorySimplifier {
        @Override
        public SimplifiedTrajectory simplify(Trajectory original, double error) {
            List<Integer> indices = RDP(original, error, 0, original.numPoints() - 1);

            return new SimplifiedTrajectory(original, indices, error);
        }

        private List<Integer> RDP(Trajectory t, double error, int i, int j) {
            Line2D shortcut = new Line2D.Double(t.getPoint(i), t.getPoint(j));

            // find index of the point that lies furthest away from the shortcut
            int furthest = i;
            double furthestDistance = 0;
            for (int loop = i; loop <= j; loop++) {
                Point2D point = t.getPoint(loop);
                double distance = shortcut.ptSegDist(point);
                if (distance > furthestDistance) {
                    furthestDistance = distance;
                    furthest = loop;
                }
            }

            if (furthestDistance <= error) {
                // No need to recurse
                ArrayList<Integer> out = new ArrayList<>();
                out.add(i);
                out.add(j);
                return out;
            } else {
                List<Integer> left = RDP(t, error, i, furthest);
                List<Integer> right = RDP(t, error, furthest, j);
                // merge the lists
                // make sure we do not include the joining point twice
                left.remove(left.size() - 1);
                left.addAll(right);
                return left;
            }

        }

    }

    /**
     * Simplification using a greedy approach.
     * <p>
     * From a given vertex, it removes all points in an error-disk, and then
     * moves to the next point.
     */
    private static class Greedy implements TrajectorySimplifier {
        @Override
        public SimplifiedTrajectory simplify(Trajectory original, double error) {
            ArrayList<Integer> indices = new ArrayList<>(original.numPoints());

            indices.add(0);
            int i = 0;
            while (i < original.numPoints() - 1) {
                Point2D p = original.getPoint(i);

                // Find first point with a distance larger than error
                // from p
                i++;
                while (i < original.numPoints() - 1) {
                    Point2D q = original.getPoint(i);
                    if (p.distance(q) > error) {
                        break;
                    }
                    i++;
                }
                indices.add(i);
            }

            return new SimplifiedTrajectory(original, indices, error);
        }

    }
}
