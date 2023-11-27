package mapconstruction.trajectories;

import com.google.common.collect.Range;
import mapconstruction.TestUtil;

import java.awt.geom.Point2D;
import java.util.List;

/**
 *
 * @author Roel
 */
public class SubtrajectoryTest extends TrajectoryTestCases {

    // reference parent;
    private Trajectory refParent;

    public SubtrajectoryTest(String testName) {
        super(testName);
        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        refParent = new FullTrajectory(points);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
    }

    @Override
    protected void tearDown() throws Exception {
        super.tearDown();
    }

    public void testConstructorExceptions() {
        System.out.println("Subtrajectory constructor exceptions");
        try {
            new Subtrajectory(null, 0, 1);
            fail("Should have thrown exception");
        } catch (NullPointerException e) {

        }

        double[][] coords = new double[][]{
            {0, 0}, {1, 0}, {2, 0}, {3, 0}
        };
        List<Point2D> otherPoints = TestUtil.doubleArrayToTrajectory(coords);
        Trajectory other = createInstance(otherPoints);

        try {
            new Subtrajectory(other, 2, 0);
            fail("Should have thrown exception");
        } catch (IllegalArgumentException e) {

        }

        try {
            new Subtrajectory(other, 0, 4);
            fail("Should have thrown exception");
        } catch (IllegalArgumentException e) {

        }
    }

    /**
     * Test of getDirectParent method, of class Subtrajectory.
     */
    public void testGetDirectParent() {
        System.out.println("getDirectParent");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0, 2);
        Trajectory expResult = refParent;
        Trajectory result = instance2.getParent();
        assertEquals(expResult, result);
    }

    /**
     * Test of getFromIndex method, of class Subtrajectory.
     */
    public void testGetFromIndex() {
        System.out.println("getFromIndex");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0, 2);
        double expResult = 0;
        double result = instance2.getFromIndex();
        assertEquals(expResult, result);
    }

    /**
     * Test of getFromIndex method, of class Subtrajectory.
     */
    public void testGetFromIndex_fractional1() {
        System.out.println("getFromIndex (real)");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0.5, 2);
        double expResult = 0.5;
        double result = instance2.getFromIndex();
        assertEquals(expResult, result);
    }

    /**
     * Test of getFromIndex method, of class Subtrajectory.
     */
    public void testGetFromIndex_fractional2() {
        System.out.println("getFromIndex (real)");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2.2, 2);
        double expResult = 2.2;
        double result = instance2.getFromIndex();
        assertEquals(expResult, result);
    }

    /**
     * Test of getToIndex method, of class Subtrajectory.
     */
    public void testGetToIndex() {
        System.out.println("getToIndex");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0, 2);
        double expResult = 2;
        double result = instance2.getToIndex();
        assertEquals(expResult, result);
    }

    /**
     * Test of getToIndex method, of class Subtrajectory.
     */
    public void testGetToIndex_fractional1() {
        System.out.println("getToIndex (real)");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0, 2.5);
        double expResult = 2.5;
        double result = instance2.getToIndex();
        assertEquals(expResult, result);
    }

    /**
     * Test of getToIndex method, of class Subtrajectory.
     */
    public void testGetToIndex_fractional2() {
        System.out.println("getToIndex (real)");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0, 0.1);
        double expResult = 0.1;
        double result = instance2.getToIndex();
        assertEquals(expResult, result);
    }

    /**
     * Test of numPoints method, of class Subtrajectory.
     */
    public void testNumPointsWithSubrange0() {
        System.out.println("numPoints: one point");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 2);
        int expResult = 1;
        int result = instance2.numPoints();
        assertEquals(expResult, result);
    }

    /**
     * Test of numPoints method, of class Subtrajectory.
     */
    public void testNumPointsWithSubrange1() {
        System.out.println("numPoints: three points");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);
        int expResult = 3;
        int result = instance2.numPoints();
        assertEquals(expResult, result);
    }

    public void testNumPoints_fracStart() {
        System.out.println("numPoints (frac start): four points");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1.5, 4);
        int expResult = 4;
        int result = instance2.numPoints();
        assertEquals(expResult, result);
    }

    public void testNumPoints_fracEnd() {
        System.out.println("numPoints (frac end): four points");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4.5);
        int expResult = 4;
        int result = instance2.numPoints();
        assertEquals(expResult, result);
    }

    public void testNumPoints_fracBoth1() {
        System.out.println("numPoints (frac both): five points");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1.5, 4.5);
        int expResult = 5;
        int result = instance2.numPoints();
        assertEquals(expResult, result);
    }

    public void testNumPoints_fracBoth2() {
        System.out.println("numPoints (frac both): two points");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1.5, 1.75);
        int expResult = 2;
        int result = instance2.numPoints();
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class Subtrajectory.
     */
    public void testGetPointWithSubrange0() {
        System.out.println("getPoint: single point");
        int pos = 0;
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 2);
        Point2D expResult = new Point2D.Double(2, 0);
        Point2D result = instance2.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class Subtrajectory.
     */
    public void testGetPointWithSubrange1() {
        System.out.println("getPoint");
        int pos = 2;
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Point2D expResult = new Point2D.Double(3, 0);
        Point2D result = instance2.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class Subtrajectory.
     */
    public void testGetPoint_fractionalStart() {
        System.out.println("getPoint, fracStart");

        Subtrajectory instance2 = new Subtrajectory(refParent, 1.5, 5);

        // starting pos
        int pos = 0;
        Point2D expResult = new Point2D.Double(1.5, 0);
        Point2D result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // next pos
        pos = 1;
        expResult = new Point2D.Double(2, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // last
        pos = 4;
        expResult = new Point2D.Double(5, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class Subtrajectory.
     */
    public void testGetPoint_fractionalEnd() {
        System.out.println("getPoint, fracStart");

        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4.5);

        // starting pos
        int pos = 0;
        Point2D expResult = new Point2D.Double(1, 0);
        Point2D result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // next pos
        pos = 1;
        expResult = new Point2D.Double(2, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // 2nd to last
        pos = 3;
        expResult = new Point2D.Double(4, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // last
        pos = 4;
        expResult = new Point2D.Double(4.5, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPoint method, of class Subtrajectory.
     */
    public void testGetPoint_fractionalBoth() {
        System.out.println("getPoint, fracStart");

        Subtrajectory instance2 = new Subtrajectory(refParent, 1.5, 4.5);

        // starting pos
        int pos = 0;
        Point2D expResult = new Point2D.Double(1.5, 0);
        Point2D result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // next pos
        pos = 1;
        expResult = new Point2D.Double(2, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // 2nd to last
        pos = 3;
        expResult = new Point2D.Double(4, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);

        // last
        pos = 4;
        expResult = new Point2D.Double(4.5, 0);
        result = instance2.getPoint(pos);
        assertEquals(expResult, result);
    }

    /**
     * Test of equals method, of class Subtrajectory.
     */
    public void testEquals0() {
        System.out.println("equals: same object");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        boolean expResult = true;
        boolean result = instance2.equals(instance2);
        assertEquals(expResult, result);
    }

    public void testEquals1() {
        System.out.println("equals: same subtrajectory, direct");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Subtrajectory other = new Subtrajectory(refParent, 1, 5);
        boolean expResult = true;
        boolean result = instance2.equals(other);
        assertEquals(expResult, result);
    }

    /**
     * Test of equals method, of class Subtrajectory.
     */
    public void testEquals3() {
        System.out.println("equals: different ranges");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 2, 4);
        boolean expResult = false;
        boolean result = instance2.equals(other);
        assertEquals(expResult, result);
    }

    public void testEquals4() {
        System.out.println("equals: same subtrajectory");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1.3, 5.3);
        Subtrajectory other = new Subtrajectory(refParent, 1.3, 5.3);
        boolean expResult = true;
        boolean result = instance2.equals(other);
        assertEquals(expResult, result);
    }

    public void testHasAsSubtrajectory_SUB1() {
        System.out.println("hasAsSubtrajectory (subtrajectory variant): true, equal");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 1, 3);
        boolean expResult = true;
        boolean result = instance2.hasAsSubtrajectory(other);
        assertEquals(expResult, result);
    }

    public void testHasAsSubtrajectory_SUB2() {
        System.out.println("hasAsSubtrajectory (subtrajectory variant): true, sub left side");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Subtrajectory other = new Subtrajectory(refParent, 1, 3);
        boolean expResult = true;
        boolean result = instance2.hasAsSubtrajectory(other);
        assertEquals(expResult, result);
    }

    public void testHasAsSubtrajectory_SUB3() {
        System.out.println("hasAsSubtrajectory (subtrajectory variant): true, sub Right side");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Subtrajectory other = new Subtrajectory(refParent, 3, 5);
        boolean expResult = true;
        boolean result = instance2.hasAsSubtrajectory(other);
        assertEquals(expResult, result);
    }

    public void testHasAsSubtrajectory_SUB4() {
        System.out.println("hasAsSubtrajectory (subtrajectory variant): true, sub both");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Subtrajectory other = new Subtrajectory(refParent, 2, 4);
        boolean expResult = true;
        boolean result = instance2.hasAsSubtrajectory(other);
        assertEquals(expResult, result);
    }

    public void testHasAsSubtrajectory_SUB5() {
        System.out.println("hasAsSubtrajectory (subtrajectory variant): false, disjoint");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 4, 5);
        boolean expResult = false;
        boolean result = instance2.hasAsSubtrajectory(other);
        assertEquals(expResult, result);
    }

    public void testHasAsSubtrajectory_SUB6() {
        System.out.println("hasAsSubtrajectory (subtrajectory variant): false, overlap");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4);
        Subtrajectory other = new Subtrajectory(refParent, 3, 5);
        boolean expResult = false;
        boolean result = instance2.hasAsSubtrajectory(other);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB1() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, equal, lambda0");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 1, 3);

        double lambda = 0;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB2() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, sub, lambda0");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Subtrajectory other = new Subtrajectory(refParent, 2, 4);

        double lambda = 0;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB3() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, equal, lambda1");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 5);
        Subtrajectory other = new Subtrajectory(refParent, 2, 4);

        double lambda = 1;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB4() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, left lambda 1");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 5);
        Subtrajectory other = new Subtrajectory(refParent, 1, 4);

        double lambda = 1;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB5() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, left lambda 2");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 5);
        Subtrajectory other = new Subtrajectory(refParent, 1, 4);

        double lambda = 2;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB6() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): false, left lambda 1");
        Subtrajectory instance2 = new Subtrajectory(refParent, 3, 5);
        Subtrajectory other = new Subtrajectory(refParent, 1, 4);

        double lambda = 1;
        boolean expResult = false;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB7() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, right lambda 1");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 2, 4);

        double lambda = 1;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB8() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, right lambda 2");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 2, 4);

        double lambda = 2;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB9() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): false, right lambda 1");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);
        Subtrajectory other = new Subtrajectory(refParent, 2, 5);

        double lambda = 1;
        boolean expResult = false;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB10() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true,both lambda 2");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);
        Subtrajectory other = new Subtrajectory(refParent, 1, 5);

        double lambda = 2;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB11() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true, both lambda 3");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);
        Subtrajectory other = new Subtrajectory(refParent, 1, 5);

        double lambda = 3;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB12() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): false, both lambda 1");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);
        Subtrajectory other = new Subtrajectory(refParent, 1, 5);

        double lambda = 1;
        boolean expResult = false;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB13() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): false disjoint");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 2);
        Subtrajectory other = new Subtrajectory(refParent, 3, 4);

        double lambda = 0;
        boolean expResult = false;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB14() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): false disjoint single point");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 2);
        Subtrajectory other = new Subtrajectory(refParent, 3, 3);

        double lambda = 0;
        boolean expResult = false;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB15() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): true single point");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 2);
        Subtrajectory other = new Subtrajectory(refParent, 2, 2);

        double lambda = 0;
        boolean expResult = true;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testHasAsLambdaSubtrajectory_SUB16() {
        System.out.println("hasAsLambdaSubtrajectory (subtrajectory variant): false empty");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 2);
        Subtrajectory other = new Subtrajectory(refParent, 2, 1);

        double lambda = 0;
        boolean expResult = false;
        boolean result = instance2.hasAsLambdaSubtrajectory(other, lambda);
        assertEquals(expResult, result);
    }

    public void testTrimStart1() {
        System.out.println("trimStart: 0 length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);

        Subtrajectory expected = new Subtrajectory(refParent, 1, 3);

        double length = 0;

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart2() {
        System.out.println("trimStart: integer start, integer length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);

        double length = 1;
        Subtrajectory expected = new Subtrajectory(refParent, 2, 3);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart3() {
        System.out.println("trimStart: integer start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4);

        double length = 1.4;
        Subtrajectory expected = new Subtrajectory(refParent, 2.4, 4);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart4() {
        System.out.println("trimStart: fractional start, integer length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0.75, 4);

        double length = 2;
        Subtrajectory expected = new Subtrajectory(refParent, 2.75, 4);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart5() {
        System.out.println("trimStart: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0.75, 4);

        double length = 1.65;
        Subtrajectory expected = new Subtrajectory(refParent, 2.4, 4);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart6() {
        System.out.println("trimStart: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0.75, 4);

        double length = 1.25;
        Subtrajectory expected = new Subtrajectory(refParent, 2, 4);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart7() {
        System.out.println("trimStart: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 0.25, 4);

        double length = 0.4;
        Subtrajectory expected = new Subtrajectory(refParent, 0.65, 4);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart8() {
        System.out.println("trimStart: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 3.1, 3.9);

        double length = 0.5;
        Subtrajectory expected = new Subtrajectory(refParent, 3.6, 3.9);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected, result);
    }

    public void testTrimStart9() {
        System.out.println("trimStart: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 3.2, 4.2);

        double length = 0.9;
        Subtrajectory expected = new Subtrajectory(refParent, 4.1, 4.2);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected.getParent(), result.getParent());
        assertEquals(expected.getFromIndex(), expected.getFromIndex(), 1E-6);
        assertEquals(expected.getToIndex(), expected.getToIndex(), 1E-6);
    }

    public void testTrimStartException() {
        System.out.println("trimStart: Exception");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);

        double length = 4;

        try {
            instance2.trimStart(length);
            fail("Should have thrown exception");
        } catch (IllegalArgumentException e) {

        }
    }

    public void testTrimEnd1() {
        System.out.println("trimEnd: 0 length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);

        Subtrajectory expected = new Subtrajectory(refParent, 1, 3);

        double length = 0;

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd() {
        System.out.println("trimEnd: integer End, integer length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 3);

        double length = 1;
        Subtrajectory expected = new Subtrajectory(refParent, 1, 2);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd3() {
        System.out.println("trimEnd: integer End, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4);

        double length = 1.4;
        Subtrajectory expected = new Subtrajectory(refParent, 1, 2.6);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd4() {
        System.out.println("trimStart: fractional End, integer length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4.75);

        double length = 2;
        Subtrajectory expected = new Subtrajectory(refParent, 1, 2.75);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd5() {
        System.out.println("trimEnd: fractional End, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4.5);

        double length = 1.75;
        Subtrajectory expected = new Subtrajectory(refParent, 1, 2.75);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd6() {
        System.out.println("trimEnd: fractional end, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4.25);

        double length = 1.15;
        Subtrajectory expected = new Subtrajectory(refParent, 1, 3.1);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd7() {
        System.out.println("trimEnd: fractional end, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 1, 4.75);

        double length = 0.45;
        Subtrajectory expected = new Subtrajectory(refParent, 1, 4.30);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd8() {
        System.out.println("trimEnd: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 3.1, 3.9);

        double length = 0.5;
        Subtrajectory expected = new Subtrajectory(refParent, 3.1, 3.4);

        Subtrajectory result = instance2.trimEnd(length);
        assertEquals(expected, result);
    }

    public void testTrimEnd9() {
        System.out.println("trimStart: fractional start, fractional length");
        Subtrajectory instance2 = new Subtrajectory(refParent, 3.8, 4.8);

        double length = 0.9;
        Subtrajectory expected = new Subtrajectory(refParent, 3.8, 3.9);

        Subtrajectory result = instance2.trimStart(length);
        assertEquals(expected.getParent(), result.getParent());
        assertEquals(expected.getFromIndex(), expected.getFromIndex(), 1E-6);
        assertEquals(expected.getToIndex(), expected.getToIndex(), 1E-6);
    }

    public void testTrimEndException() {
        System.out.println("trimEnd: Exception");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);

        double length = 4;

        try {
            instance2.trimEnd(length);
            fail("Should have thrown exception");
        } catch (IllegalArgumentException e) {

        }
    }

//    public void testTrimException() {
//        System.out.println("trim: Exception");
//        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 4);
//
//        double length = 2;
//
//        try {
//            instance2.trim(length);
//            fail("Should have thrown exception");
//        } catch (IllegalArgumentException e) {
//
//        }
//    }
    
    public void testComputeOverlap1() {
        System.out.println("ComputeOverlap, overlap start");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 5);

        Subtrajectory other = new Subtrajectory(refParent, 1, 4);
        Range<Double> expected = Range.closed(2d, 4d);
        
        Range<Double> result = instance2.computeOverlap(other);
        Range<Double> result2 = other.computeOverlap(instance2);
        
        assertEquals(expected, result);
        assertEquals(expected, result2);
    }
    
    public void testComputeOverlap2() {
        System.out.println("ComputeOverlap, overlap end");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 5);

        Subtrajectory other = new Subtrajectory(refParent, 3, 6);
        Range<Double> expected = Range.closed(3d, 5d);
        
        Range<Double> result = instance2.computeOverlap(other);
        Range<Double> result2 = other.computeOverlap(instance2);
        
        assertEquals(expected, result);
        assertEquals(expected, result2);
    }
    
    public void testComputeOverlap3() {
        System.out.println("ComputeOverlap, middle");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 5);

        Subtrajectory other = new Subtrajectory(refParent, 3, 4);
        Range<Double> expected = Range.closed(3d, 4d);
        
        Range<Double> result = instance2.computeOverlap(other);
        Range<Double> result2 = other.computeOverlap(instance2);
        
        assertEquals(expected, result);
        assertEquals(expected, result2);
    }
    
    public void testComputeOverlap4() {
        System.out.println("ComputeOverlap, no");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 5);

        Subtrajectory other = new Subtrajectory(refParent, 0,1);
        Range<Double> expected = null;
        
        Range<Double> result = instance2.computeOverlap(other);
        Range<Double> result2 = other.computeOverlap(instance2);
        
        assertEquals(expected, result);
        assertEquals(expected, result2);
    }
    
    public void testComputeOverlap5() {
        System.out.println("ComputeOverlap, touch");
        Subtrajectory instance2 = new Subtrajectory(refParent, 2, 3);

        Subtrajectory other = new Subtrajectory(refParent, 3,4);
        Range<Double> expected = Range.closed(3d, 3d);
        
        Range<Double> result = instance2.computeOverlap(other);
        Range<Double> result2 = other.computeOverlap(instance2);
        
        assertEquals(expected, result);
        assertEquals(expected, result2);
    }

    @Override
    protected Trajectory createInstance(List<Point2D> points) {
        Trajectory parent = new FullTrajectory(points);
        return new Subtrajectory(parent, 0, parent.numPoints() - 1);
    }

}
