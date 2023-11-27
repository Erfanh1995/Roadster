package mapconstruction.trajectories;

import junit.framework.TestCase;
import mapconstruction.TestUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.List;

/**
 *
 * @author Roel
 */
public abstract class TrajectoryTestCases extends TestCase {
    
    public TrajectoryTestCases(String testName) {
        super(testName);
    }
    
    /** Test fixture. */
    protected Trajectory instance;
    
    /**
     * Set instance with the given set of points.
     * @param points 
     */
    protected void setInstance(List<Point2D> points) {
        instance = createInstance(points);
    }
    
    protected abstract Trajectory createInstance(List<Point2D> points);

    
    /**
     * Test of numPoints method, of class Trajectory.
     */
    public void testNumPoints0() {
        System.out.println("numPoints: one point");
        double[][] coords = new double[][]{
            {0,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int expResult = points.size();
        int result = instance.numPoints();
        assertEquals(expResult, result);
    }
    
    /**
     * Test of numPoints method, of class Trajectory.
     */
    public void testNumPoints1() {
        System.out.println("numPoints: 5 points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}, {4,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int expResult = points.size();
        int result = instance.numPoints();
        assertEquals(expResult, result);
    }
    
    /**
     * Test of numEdges method, of class Trajectory.
     */
    public void testNumEdges0() {
        System.out.println("numEdges: no point");
        double[][] coords = new double[][]{
            
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int expResult = 0;
        int result = instance.numEdges();
        assertEquals(expResult, result);
    }
    
    /**
     * Test of numEdges method, of class Trajectory.
     */
    public void testNumEdges2() {
        System.out.println("numEdges: 1 point");
        double[][] coords = new double[][]{
            {0,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int expResult = 0;
        int result = instance.numEdges();
        assertEquals(expResult, result);
    }
    
    /**
     * Test of numEdges method, of class Trajectory.
     */
    public void testNumEdges1() {
        System.out.println("numEdges: 5 points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}, {4,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int expResult = points.size() - 1;
        int result = instance.numEdges();
        assertEquals(expResult, result);
    }

    /**
     * Test of euclideanLength method, of class Trajectory.
     */
    public void testEuclideanLength0() {
        System.out.println("euclideanLength: 2 points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);

        double expResult = 1;
        double result = instance.euclideanLength();
        assertEquals(expResult, result, 0.0);
    }
    
    /**
     * Test of euclideanLength method, of class Trajectory.
     */
    public void testEuclideanLength1() {
        System.out.println("euclideanLength: 3 points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {10,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);

        double expResult = 10;
        double result = instance.euclideanLength();
        assertEquals(expResult, result, 0.0);
    }

    /**
     * Test of getPointsList method, of class Trajectory.
     */
    public void testPoints() {
        System.out.println("Points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        List<Point2D> result = instance.points();
        assertEquals(points, result);
    }
    
    /**
     * Test of getPointsList method, of class Trajectory.
     */
    public void testEdges() {
        System.out.println("Edges");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        
        List<Line2D> edges = Arrays.asList(new Line2D.Double(0, 0, 1, 0), new Line2D.Double(1, 0, 2, 0), new Line2D.Double(2, 0, 3, 0));
        
        List<Line2D> result = instance.edges();
        assertEquals(edges.size(), result.size());
        for (int i = 0; i < edges.size(); i ++) {
            Line2D l1 = edges.get(i);
            Line2D l2 = result.get(i);
            assertEquals(l1.getP1(), l2.getP1());
            assertEquals(l1.getP2(), l2.getP2());
        }
    }

    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetPoint0() {
        System.out.println("getPoint: first points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = 0;
        
        Point2D expResult = new Point2D.Double(0, 0);
        Point2D result = instance.getPoint(pos);
        assertEquals(expResult, result);
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetPoint1() {
        System.out.println("getPoint: second point");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = 1;
        
        Point2D expResult = new Point2D.Double(1, 0);
        Point2D result = instance.getPoint(pos);
        assertEquals(expResult, result);
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetPointException0() {
        System.out.println("getPoint exception: negative");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = -1;
        
        try {
            instance.getPoint(pos);
            fail("Should have thrown exception");
        } catch (IndexOutOfBoundsException e) {
        }
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetPointException1() {
        System.out.println("getPoint exception: too large");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = 4;
        
        try {
            instance.getPoint(pos);
            fail("Should have thrown exception");
        } catch (IndexOutOfBoundsException e) {
        }
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetEdge0() {
        System.out.println("getEdge: first edge");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = 0;
        
        Line2D expResult = new Line2D.Double(0,0,1,0);
        Line2D result = instance.getEdge(pos);
        assertEquals(expResult.getP1(), result.getP1());
        assertEquals(expResult.getP2(), result.getP2());
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetEdge1() {
        System.out.println("getEdge: second Edge");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = 1;
        
        Line2D expResult = new Line2D.Double(1,0,2,0);
        Line2D result = instance.getEdge(pos);
        assertEquals(expResult.getP1(), result.getP1());
        assertEquals(expResult.getP2(), result.getP2());
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetEdgeException0() {
        System.out.println("getEdge exception: negative");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = -1;
        
        try {
            instance.getEdge(pos);
            fail("Should have thrown exception");
        } catch (IndexOutOfBoundsException e) {
        }
    }
    
    /**
     * Test of getPoint method, of class Trajectory.
     */
    public void testGetEdgeException1() {
        System.out.println("getEdge exception: too large");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);   
        setInstance(points);
        int pos = 3;
        
        try {
            instance.getEdge(pos);
            fail("Should have thrown exception");
        } catch (IndexOutOfBoundsException e) {
        }
    }
    
    
    
    public void testPointEquals1() {
        System.out.println("pointEquals: true case, same points");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords); 
        setInstance(points);
        List<Point2D> otherPoints = TestUtil.doubleArrayToTrajectory(coords);  
        Trajectory other = createInstance(otherPoints);
        
        boolean expResult = true;
        boolean result = instance.pointEquals(other);
        assertEquals(expResult, result);
    }
    
    public void testPointEquals2() {
        System.out.println("pointEquals: true case, same trajectory");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords); 
        setInstance(points);
        
        boolean expResult = true;
        boolean result = instance.pointEquals(instance);
        assertEquals(expResult, result);
    }
    
    public void testPointEquals3() {
        System.out.println("pointEquals: false case, same trajectory");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(Arrays.copyOfRange(coords, 1, 3)); 
        setInstance(points);
        List<Point2D> otherPoints = TestUtil.doubleArrayToTrajectory(coords);  
        Trajectory other = createInstance(otherPoints);
        
        boolean expResult = false;
        boolean result = instance.pointEquals(other);
        assertEquals(expResult, result);
    }
    
    public void testReverse1() {
        System.out.println("reverse: empty");
        double[][] coords = new double[][]{
            
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords); 
        setInstance(points);
        
        Trajectory expResult = createInstance(points);
        Trajectory result = instance.reverse();
        assert(result.pointEquals(expResult));
    }
    
    public void testReverse2() {
        System.out.println("reverse");
        double[][] coords = new double[][]{
            {0,0}, {1,0}, {2,0}, {3,0}
        };
        
        double[][] coords2 = new double[][]{
            {3,0}, {2,0}, {1,0}, {0,0}
        };
        
        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords); 
        setInstance(points);
        
        Trajectory expResult = createInstance(TestUtil.doubleArrayToTrajectory(coords2));
        Trajectory result = instance.reverse();
        assert(result.pointEquals(expResult));
    }

  
    
}
