package mapconstruction.trajectories;

import com.google.common.collect.Lists;
import mapconstruction.TestUtil;
import org.junit.*;

import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 *
 * @author Roel
 */
public class SimplifiedTrajectoryTest extends TrajectoryTestCases {

    // reference parent;
    private Trajectory refParent;

    public SimplifiedTrajectoryTest(String testName) {
        super(testName);
        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        refParent = new FullTrajectory(points);
    }

    @BeforeClass
    public static void setUpClass() {
    }

    @AfterClass
    public static void tearDownClass() {
    }

    @Before
    public void setUp() {
    }

    @After
    public void tearDown() {
    }

    public void testConstructorExceptions() {
        System.out.println("SimplifiedTrajectory constructor exeptions");
        try {
            new SimplifiedTrajectory(null, Arrays.asList(0, 2, 4, 6), 0);
            fail("Should have thrown exception");
        } catch (NullPointerException e) {

        }
        
        try {
            new SimplifiedTrajectory(refParent, null, 0);
            fail("Should have thrown exception");
        } catch (NullPointerException e) {

        }
        
        try {
            new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), -1);
            fail("Should have thrown exception");
        } catch (IllegalArgumentException e) {

        }
    }

    /**
     * Test of numPoints method, of class SimplifiedTrajectory.
     */
    @Test
    public void testNumPoints_SUB1() {
        System.out.println("numPoints");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        int expResult = 4;
        int result = instance.numPoints();
        assertEquals(expResult, result);
    }

    /**
     * Test of numPoints method, of class SimplifiedTrajectory.
     */
    @Test
    public void testNumPoints_SUB2() {
        System.out.println("numPoints");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0), 0);
        int expResult = 1;
        int result = instance.numPoints();
        assertEquals(expResult, result);
    }

    /**
     * Test of getPointsList method, of class SimplifiedTrajectory.
     */
    @Test
    public void testPoints_SUB1() {
        System.out.println("Points");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);

        double[][] coords = new double[][]{
            {0, 0}, {2, 0}, {4, 0}, {6, 0}
        };

        List<Point2D> expResult = TestUtil.doubleArrayToTrajectory(coords);
        List<Point2D> result = instance.points();
        assertEquals(expResult, result);
    }

    /**
     * Test of reverse method, of class SimplifiedTrajectory.
     */
    @Test
    public void testReverse_SUB1() {
        System.out.println("reverse");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(1, 3, 5), 12);

        double[][] coords = new double[][]{
            {1, 0}, {3, 0}, {5, 0}
        };

        List<Point2D> expResult = Lists.reverse(TestUtil.doubleArrayToTrajectory(coords));

        SimplifiedTrajectory result = instance.reverse();
        assertEquals(expResult, result.points());
        assertEquals(12, result.getError(), 0);
    }

    /**
     * Test of getPoint method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetPoint_SUB1() {
        System.out.println("getPoint");
        int pos = 0;
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        Point2D expResult = refParent.getPoint(0);
        Point2D result = instance.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetPoint_SUB2() {
        System.out.println("getPoint");
        int pos = 1;
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        Point2D expResult = refParent.getPoint(2);
        Point2D result = instance.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetPoint_SUB3() {
        System.out.println("getPoint");
        int pos = 3;
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        Point2D expResult = refParent.getPoint(6);
        Point2D result = instance.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getOriginal method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetOriginal() {
        System.out.println("getOriginal");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        Trajectory expResult = refParent;
        Trajectory result = instance.getOriginal();
        assertEquals(expResult, result);
    }

    /**
     * Test of getOriginalIndex method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetOriginalIndex1() {
        System.out.println("getOriginalIndex");
        int idx = 0;
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        int expResult = 0;
        int result = instance.getOriginalIndex(idx);
        assertEquals(expResult, result);
    }

    /**
     * Test of getOriginalIndex method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetOriginalIndex2() {
        System.out.println("getOriginalIndex");
        int idx = 1;
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        int expResult = 2;
        int result = instance.getOriginalIndex(idx);
        assertEquals(expResult, result);
    }

    /**
     * Test of getOriginalIndex method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetOriginalIndex3() {
        System.out.println("getOriginalIndex");
        int idx = 3;
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        int expResult = 6;
        int result = instance.getOriginalIndex(idx);
        assertEquals(expResult, result);
    }

    /**
     * Test of equals method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetEquals1() {
        System.out.println("equals");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        boolean expResult = true;
        boolean result = instance.equals(instance);
        assertEquals(expResult, result);
    }

    /**
     * Test of equals method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetEquals2() {
        System.out.println("equals");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        SimplifiedTrajectory instance2 = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        boolean expResult = true;
        boolean result = instance.equals(instance2);
        assertEquals(expResult, result);
    }

    /**
     * Test of equals method, of class SimplifiedTrajectory.
     */
    @Test
    public void testGetEquals3() {
        System.out.println("equals");
        SimplifiedTrajectory instance = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 0);
        SimplifiedTrajectory instance2 = new SimplifiedTrajectory(refParent, Arrays.asList(0, 2, 4, 6), 10);
        boolean expResult = false;
        boolean result = instance.equals(instance2);
        assertEquals(expResult, result);
    }

    @Override
    protected Trajectory createInstance(List<Point2D> points) {
        Trajectory parent = new FullTrajectory(points);
        List<Integer> indices = IntStream.range(0, points.size()).boxed().collect(Collectors.toList());
        return new SimplifiedTrajectory(parent, indices, 0);
    }

}
