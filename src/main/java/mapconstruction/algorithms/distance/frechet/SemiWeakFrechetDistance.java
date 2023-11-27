package mapconstruction.algorithms.distance.frechet;

import com.google.common.collect.HashBasedTable;
import com.google.common.collect.Table;
import mapconstruction.algorithms.distance.DistanceMatrix;
import mapconstruction.algorithms.distance.TrajectoryDistance;
import mapconstruction.trajectories.Trajectory;

/**
 * Computer for the Semi-weak Frechét distance between two given trajectories.
 * <p>
 * Uses a dynamic programming approach.
 * <p>
 * If d(v(i),e(j)) denotes the distance between the ith vertex of T1 and jth
 * edge of T2, and F is the dp table, we have the following recurrence:
 * <p>
 * {@code F[v(i),e(j)] = max(d(v(i),e(j)), min(F[v(i-1), e(j)], F[e(i-1), v(j)])}
 * <p>
 * Similarly,
 * {@code F[e(i),v(j)] = max(d(e(i),v(j)), min(F[v(i), e(j-1)], F[e(i), v(j - 1)])}
 * <p>
 * If the number of points in T1 is n and of T2 is m, then the distance is
 * <p>
 * min(F[v(n - 1),e(m - 1)], F[e(n - 2),v(m - 1)]) (assuming zero indexing).
 *
 * @author Roel
 */
public class SemiWeakFrechetDistance implements TrajectoryDistance {

    /**
     * Offset used for indexing. The ith vertex coordinate will be
     * 2i*INDEX_OFFSET, the jth edge coordinate (2j + 1)*INDEX_OFFSET
     */
    private static final int INDEX_OFFSET = 1;

    @Override
    public double compute(Trajectory t1, Trajectory t2) {
        final int n = t1.numPoints();
        final int m = t2.numPoints();

        // compute distances.
        final DistanceMatrix distanceMatrix = new DistanceMatrix(t1, t2);

        final Table<Integer, Integer, Double> frechetTable = HashBasedTable.create(2 * n, 2 * m);

        // we fill the table row by row, starting with a vertex-y-coordinate, and then alternating.
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                // handle coordinates (e(i), v(j))
                if (i < n - 1) {
                    // Compute minimum of bottom left adjacent cells.
                    double min = Double.MAX_VALUE;
                    if (j > 0) {
                        min = Math.min(min, frechetTable.get(v(i), e(j - 1)));
                        min = Math.min(min, frechetTable.get(e(i), v(j - 1)));
                    }
                    double dist = distanceMatrix.getEdgePointDistance(i, j);

                    double value;
                    if (j == 0) {
                        // base case
                        value = dist;
                    } else {
                        value = Math.max(min, dist);
                    }

                    frechetTable.put(e(i), v(j), value);
                }

                // handle coordinates (v(i), e(j))
                if (j < m - 1) {
                    // Compute minimum of bottom left adjacent cells.
                    double min = Double.MAX_VALUE;
                    if (i > 0) {
                        min = Math.min(min, frechetTable.get(v(i - 1), e(j)));
                        min = Math.min(min, frechetTable.get(e(i - 1), v(j)));
                    }
                    double dist = distanceMatrix.getPointEdgeDistance(i, j);

                    double value;
                    if (i == 0) {
                        // base case
                        value = dist;
                    } else {
                        value = Math.max(min, dist);
                    }

                    frechetTable.put(v(i), e(j), value);
                }

            }
        }
        final double d1 = frechetTable.get(v(n - 1), e(m - 2));
        final double d2 = frechetTable.get(e(n - 2), v(m - 1));
        final double result = Math.min(d1, d2);
        return result;

    }

    /**
     * Gets the ith vertex coordinate
     *
     * @param i
     */
    private int v(int i) {
        return 2 * i * INDEX_OFFSET;
    }

    /**
     * Gets the ith edge coordinate
     *
     * @param i
     */
    private int e(int i) {
        return (2 * i + 1) * INDEX_OFFSET;
    }
}
