package mapconstruction.algorithms.bundles;

import junit.framework.TestCase;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class KLSubbundleAlgorithmTest extends TestCase {

    public KLSubbundleAlgorithmTest(String testName) {
        super(testName);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
    }

    @Override
    protected void tearDown() throws Exception {
        super.tearDown();
    }

    private List<Trajectory> makeTrajectories(int[][][] coordinates) {
        List<Trajectory> trajectories = new ArrayList<>();
        for (int[][] coordinate : coordinates) {
            List<Point2D> points = new ArrayList<>();
            for (int[] point : coordinate) {
                points.add(new Point(point[0], point[1]));
            }
            trajectories.add(new FullTrajectory(points));
        }
        return trajectories;
    }



    public void testBundleSize1() {
        int[][][] c = {
            { {0, 0}, {2, 0}, {4, 0} }, // A
            { {0, 1}, {2, 1}, {4, 1} }, // B
            { {4, 2}, {2, 2}, {0, 2} }  // C
        };

        List<Trajectory> trajectories = makeTrajectories(c);


        KLSubbundleAlgorithm algo1 = new KLSubbundleAlgorithm(1d, 0d, true);
        MaximalSubbundleAlgorithm algo2 = new MaximalSubbundleAlgorithm(1d, 0d, true, k -> k+1);
        System.out.println("[KLSubbundle] " + algo1.runAlgorithm(trajectories));
        System.out.println("[MaxSubbundle] " + algo2.runAlgorithm(trajectories));


    }

}
