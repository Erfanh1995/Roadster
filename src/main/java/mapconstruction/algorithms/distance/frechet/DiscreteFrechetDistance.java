package mapconstruction.algorithms.distance.frechet;

import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.algorithms.distance.TrajectoryDistance;
import mapconstruction.trajectories.Trajectory;

/**
 * Computer for the discrete Frechét distance between two given trajectories.
 * <p>
 * Uses a dynamic programming approach.
 * <p>
 * If d(i,j) denotes the distance between T1[i] and T2[j], and
 * F is the dp table, we have the following recurrence:
 * <p>
 * {@code F[i,j] = max(d(i,j), min(F[i, j-1], F[i-1, j], F[i-1,j-1]}
 *
 * @author Roel
 */
public class DiscreteFrechetDistance implements TrajectoryDistance {

    /**
     * Distance matrix that was used in the last distance computation.
     * Null if not computation has been done yet.
     */
    private DistanceMatrix distanceMatrix;


    /**
     * Dynamic programming table used in the last distance computation.
     * Cell (i,j) indicates the dfd between  T[0,i] and T[0,j]
     * Null if no computation has been done yet.
     */
    private double[][] frechetMatrix;

    public DiscreteFrechetDistance() {
        distanceMatrix = null;
        frechetMatrix = null;
    }


    /**
     * Computes the discrete Frechét distance between the two given trajectories.
     * <p>
     * The used matrixes can afterwards be queried using
     * {@code getDistanceMatrix} and {@code getFrechetMatrix}
     *
     * @param t1 first trajectory
     * @param t2 second trajectory
     * @return
     */
    @Override
    public double compute(Trajectory t1, Trajectory t2) {
        int n = t1.numPoints();
        int m = t2.numPoints();

        // compute distances.
        distanceMatrix = new DistanceMatrix(t1, t2);
        frechetMatrix = new double[n][m];

        // We fill the table row by row,
        // filling each row in increasing order.
        for (int i = 0; i < frechetMatrix.length; i++) {
            for (int j = 0; j < frechetMatrix[i].length; j++) {
                // Compute minimum of bottom left adjacent cells.
                double min = Double.MAX_VALUE;
                if (i > 0) {
                    min = Math.min(min, frechetMatrix[i - 1][j]);
                }
                if (j > 0) {
                    min = Math.min(min, frechetMatrix[i][j - 1]);
                }
                if (i > 0 && j > 0) {
                    min = Math.min(min, frechetMatrix[i - 1][j - 1]);
                }

                double dist = distanceMatrix.getPointDistance(i, j);

                frechetMatrix[i][j] = (i == 0 && j == 0) ? dist : Math.max(dist, min);
            }
        }

        return frechetMatrix[n - 1][m - 1];
    }

    public DistanceMatrix getDistanceMatrix() {
        return distanceMatrix;
    }

    public double[][] getFrechetMatrix() {
        return frechetMatrix;
    }


}
