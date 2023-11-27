package mapconstruction.algorithms.distance.frechet;

import junit.framework.TestCase;
import mapconstruction.TestUtil;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Trajectory;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.List;

/**
 *
 * @author Roel
 */
public class SemiWeakFrechetDistanceTest extends TestCase {
    
    public SemiWeakFrechetDistanceTest(String testName) {
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
    

    /**
     * Verifies the given test case.
     * 
     * t1 and t2 are two trajectories, represented as an array of pairs
     * (x,y coordinates);
     * 
     * expected is the expected frechet distance.
     */
    private void verify(double[][] t1, double[][] t2, double expected) {
        List<Point2D> points1 = TestUtil.doubleArrayToTrajectory(t1);
        List<Point2D> points2 = TestUtil.doubleArrayToTrajectory(t2);
        Trajectory traj1 = new FullTrajectory(points1);
        Trajectory traj2 = new FullTrajectory(points2);
        
        SemiWeakFrechetDistance instance = new SemiWeakFrechetDistance();
        double result = instance.compute(traj1, traj2);
        assertEquals(expected, result);
    }
    
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute1() {
        System.out.println("compute: Test 1: Same trajectory");
        double[][] t1 = new double[][]{
            {0,0}, {0,1}, {0,2}
        };
        double expected = 0;
        
        
        verify(t1, t1, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute2() {
        System.out.println("compute: Test 2: Distance 1");
        double[][] t1 = new double[][]{
            {0,0}, {0,1}, {0,2}
        };
        double[][] t2 = new double[][]{
            {1,0}, {1,1}, {1,2}
        };
        double expected = 1;
        
        
        verify(t1, t2, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute3() {
        System.out.println("compute: Test 3: Long Dist");
        double[][] t1 = new double[][]{
            {0,0}, {-100,1}, {0,2}
        };
        double[][] t2 = new double[][]{
            {1,0}, {100,1}, {1,2}
        };
        double expected = Line2D.ptSegDist(0, 0, -100, 1, 100, 1);
        
        
        verify(t1, t2, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute4() {
        System.out.println("compute: Test 4: Dif num points, same line");
        double[][] t1 = new double[][]{
            {0,0}, {0,2}
        };
        double[][] t2 = new double[][]{
            {0,0}, {0,1}, {0,2}
        };
        double expected = 0;
        
        
        verify(t1, t2, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute4b() {
        System.out.println("compute: Test 4: Dif num points, same line");
        double[][] t1 = new double[][]{
            {0,0}, {0,2}
        };
        double[][] t2 = new double[][]{
            {0,0}, {0,1}, {0,2}
        };
        double expected = 0;
        
        
        verify(t2, t1, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute5() {
        System.out.println("compute: Test 5: Dif num points, parallel line");
        double[][] t1 = new double[][]{
            {0,0}, {0,2}
        };
        double[][] t2 = new double[][]{
            {1,0}, {1,1}, {1,2}
        };
        double expected = 1;
        
        verify(t1, t2, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute5b() {
        System.out.println("compute: Test 5: Dif num points, parallel line");
        double[][] t1 = new double[][]{
            {0,0}, {0,2}
        };
        double[][] t2 = new double[][]{
            {1,0}, {1,1}, {1,2}
        };
        double expected = 1;
        
        verify(t2, t1, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute6() {
        System.out.println("compute: Test 6: offset, parallel");
        double[][] t1 = new double[][]{
            {0,0}, {0,2}, {0,4}, {0,6}
        };
        double[][] t2 = new double[][]{
            {1,1}, {1,3}, {1,5}, {1,7}
        };
        double expected = 1;
        
        verify(t1, t2, expected);
    }
    
    /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute7() {
        System.out.println("compute: Test 6: offset, parallel 2");
        double[][] t1 = new double[][]{
            {0,0}, {0,2}, {0,4}, {0,6}
        };
        double[][] t2 = new double[][]{
            {1,-1}, {1,1}, {1,3}, {1,5}
        };
        double expected = 1;
        
        verify(t1, t2, expected);
    }
    
     /**
     * Test of compute method, of class DiscreteFrechetDistance.
     */
    public void testCompute8() {
        System.out.println("compute: Test 6: extended at ends, parallel");
        double[][] t1 = new double[][]{
            {0,-10}, {0,2}, {0,4}, {0,16}
        };
        double[][] t2 = new double[][]{
            {1,1}, {1,3}, {1,5}
        };
        double expected = 1;
        
        verify(t1, t2, expected);
    }
    
    
}
