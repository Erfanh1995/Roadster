package mapconstruction.attributes;

import com.google.common.collect.Iterables;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Subtrajectory;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

/**
 * Different attributes that can be computed for bundles.
 * <p>
 * Enum-strategy pattern.
 *
 * @author Roel
 */
public enum BundleAttribute implements ToDoubleFunction<Bundle> {
    /**
     * Get the number of trajectories in the bundle.
     */
    Size {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.size();
        }

    },
    /**
     * Gets the discrete length of the bundle.
     */
    DiscreteLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.discreteLength();
        }
    },
    /**
     * Gets the continuous length of the bundle.
     */
    ContinuousLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.continuousLength();
        }
    },
    /**
     * Gets the minimum number of vertices of a trajectory in the bundle.
     */
    MinDiscreteLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(Subtrajectory::numPoints)
                    .min()
                    .orElse(Double.POSITIVE_INFINITY);
        }
    },
    /**
     * Gets the minimum euclidean length of a trajectory in the bundle.
     */
    MinContinuousLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(Subtrajectory::euclideanLength)
                    .min()
                    .orElse(Double.POSITIVE_INFINITY);
        }
    },
    /**
     * Gets the maximum number of vertices of a trajectory in the bundle.
     */
    MaxDiscreteLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(Subtrajectory::numPoints)
                    .max()
                    .orElse(0);
        }
    },
    /**
     * Gets the maximum euclidian length of a trajectory in the bundle.
     */
    MaxContinuousLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(Subtrajectory::euclideanLength)
                    .max()
                    .orElse(0);
        }
    },
    /**
     * Gets the average number of vertices of the trajectories in the bundle.
     */
    AvgDiscreteLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(Subtrajectory::numPoints)
                    .average()
                    .orElse(0);
        }
    },
    /**
     * Gets the average euclidian length of the trajectories in the bundle.
     */
    AvgContinuousLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            return bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(Subtrajectory::euclideanLength)
                    .average()
                    .orElse(0);
        }
    },
    /**
     * Gets the standard deviation in the number of vertices of the trajectories in the bundle.
     */
    StdDevDiscreteLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            double mean = AvgDiscreteLength.applyAsDouble(bundle);
            return Math.sqrt(bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(traj -> mean - traj.numPoints())
                    .map(x -> x * x)
                    .average()
                    .orElse(0));
        }
    },
    /**
     * Gets the standard deviation in the number of vertices of the trajectories in the bundle.
     */
    StdDevContinuousLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            double mean = AvgContinuousLength.applyAsDouble(bundle);
            return Math.sqrt(bundle.getSubtrajectories()
                    .stream()
                    .mapToDouble(traj -> mean - traj.euclideanLength())
                    .map(x -> x * x)
                    .average()
                    .orElse(0));
        }
    },
    /**
     * Gets the number of vertices of the representativeSubtrajectory.
     */
    RepDiscreteLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            Subtrajectory rep = bundle.getOriginalRepresentative();
            return rep == null ? Double.NaN : rep.numPoints();
        }
    },
    /**
     * Gets the euclidian length of the representativeSubtrajectory.
     */
    RepContinuousLength {
        @Override
        public double applyAsDouble(Bundle bundle) {
            Subtrajectory rep = bundle.getOriginalRepresentative();
            return rep == null ? Double.NaN : rep.euclideanLength();
        }
    },
    /**
     * Returns the distance between the furthest two points in the bundle.
     */
    Diameter {
        @Override
        public double applyAsDouble(Bundle bundle) {
            List<Point2D> allPoints = new ArrayList<>();

            for (Subtrajectory t : bundle.getSubtrajectories()) {
                Iterables.addAll(allPoints, t.points());
            }

            double diameter = 0;
            for (Point2D p1 : allPoints) {
                for (Point2D p2 : allPoints) {
                    if (p1 != p2) {
                        diameter = Math.max(diameter, p1.distance(p2));
                    }
                }
            }

            return diameter;
        }
    }

}
