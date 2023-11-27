package mapconstruction.trajectories;

import mapconstruction.TestUtil;

import java.awt.geom.Point2D;
import java.util.List;

/**
 *
 * @author Roel
 */
public class FullTrajectoryTest extends TrajectoryTestCases {

    public FullTrajectoryTest(String testName) {
        super(testName);
    }

    @Override
    protected FullTrajectory createInstance(List<Point2D> points) {
        return new FullTrajectory(points);
    }
    
    public void testConstructorExceptions() {
        System.out.println("FullTrajectory constructor exeptions");
        try {
            new FullTrajectory(null);
            fail("Should have thrown exception");
        } catch (NullPointerException e) {
            
        }
    }

    
    
    public void testEquals1() {
        System.out.println("pointEquals: false case, but same points");
        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        setInstance(points);
        List<Point2D> otherPoints = TestUtil.doubleArrayToTrajectory(coords);
        Trajectory other = createInstance(otherPoints);

        boolean expResult = false;
        boolean result = instance.equals(other);
        assertEquals(expResult, result);
    }

    public void tesEquals2() {
        System.out.println("pointEquals: true case, same trajectory");
        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        setInstance(points);

        boolean expResult = true;
        boolean result = instance.equals(instance);
        assertEquals(expResult, result);
    }

    public void testReverse_EX1() {
        System.out.println("reverse: Check ID");
        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}
        };

        double[][] coords2 = new double[][]{
            {3, 0}, {2, 0}, {1, 0}, {0, 0}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        setInstance(points);

        FullTrajectory instance2 = (FullTrajectory) instance;
        FullTrajectory expResult = createInstance(TestUtil.doubleArrayToTrajectory(coords2));
        FullTrajectory result = instance2.reverse();
        assert (result.pointEquals(expResult));
        assertEquals(instance2.getId(), -result.getId());
    }

    public void testReverse_EX2() {
        System.out.println("reverse: reverse twice");
        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        setInstance(points);

        FullTrajectory instance2 = (FullTrajectory) instance;
        FullTrajectory result = instance2.reverse().reverse();
        assertEquals(instance2, result);
    }

}
