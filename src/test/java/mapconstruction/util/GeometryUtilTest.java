package mapconstruction.util;

import com.google.common.collect.Range;
import mapconstruction.TestUtil;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.trajectories.Trajectory;
import org.junit.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static mapconstruction.util.GeometryUtil.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

/**
 * @author Roel
 */
public class GeometryUtilTest {

    public GeometryUtilTest() {
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

    /**
     * Test of lineLength method, of class GeometryUtil.
     */
    @Test
    public void testLineLength1() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double expResult = 10.0;
        double result = GeometryUtil.lineLength(line);
        assertEquals(expResult, result, 0.0);
    }

    /**
     * Test of lineLength method, of class GeometryUtil.
     */
    @Test
    public void testLineLength2() {
        Line2D line = new Line2D.Double(0, 0, 3, 4);
        double expResult = 5.0;
        double result = GeometryUtil.lineLength(line);
        assertEquals(expResult, result, 0.0);
    }

    /**
     * Test of getPointOnLine method, of class GeometryUtil.
     */
    @Test
    public void testGetPointOnLine1() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double t = 0.0;
        Point2D expResult = new Point2D.Double(0, 0);
        Point2D result = GeometryUtil.getPointOnLine(line, t);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPointOnLine method, of class GeometryUtil.
     */
    @Test
    public void testGetPointOnLine2() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double t = 1.0;
        Point2D expResult = new Point2D.Double(10, 0);
        Point2D result = GeometryUtil.getPointOnLine(line, t);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPointOnLine method, of class GeometryUtil.
     */
    @Test
    public void testGetPointOnLine3() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double t = 0.5;
        Point2D expResult = new Point2D.Double(5, 0);
        Point2D result = GeometryUtil.getPointOnLine(line, t);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPointOnLine method, of class GeometryUtil.
     */
    @Test
    public void testGetPointOnLine4() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double t = 0.75;
        Point2D expResult = new Point2D.Double(7.5, 0);
        Point2D result = GeometryUtil.getPointOnLine(line, t);
        assertEquals(expResult, result);
    }

    /**
     * Test of getPointOnLine method, of class GeometryUtil.
     */
    @Test
    public void testGetPointOnLine_EX1() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double t = -1;
        Class<?> expResult = IllegalArgumentException.class;

        try {
            GeometryUtil.getPointOnLine(line, t);
            fail("Should have thrown exception");
        } catch (Exception e) {
            assertEquals(expResult, e.getClass());
        }


    }

    /**
     * Test of getPointOnLine method, of class GeometryUtil.
     */
    @Test
    public void testGetPointOnLine_EX2() {
        Line2D line = new Line2D.Double(0, 0, 10, 0);
        double t = 2;
        Class<?> expResult = IllegalArgumentException.class;

        try {
            GeometryUtil.getPointOnLine(line, t);
            fail("Should have thrown exception");
        } catch (Exception e) {
            assertEquals(expResult, e.getClass());
        }


    }


    /**
     * Test of segCircIntersectionParams method, of class GeometryUtil.
     */
    @Test
    public void testSegCircIntersectionParams_3args_0sol0int() {
        Line2D line = new Line2D.Double(0, 15, 10, 15);
        Point2D circCenter = new Point2D.Double(4, 10);
        double circRadius = 2.0;
        List<Double> expResult = Collections.emptyList();
        List<Double> result = GeometryUtil.segCircIntersectionParams(line, circCenter, circRadius);
        assertEquals(expResult, result);
    }

    /**
     * Test of segCircIntersectionParams method, of class GeometryUtil.
     */
    @Test
    public void testSegCircIntersectionParams_3args_2sol0int() {
        Line2D line = new Line2D.Double(20, 10, 30, 10);
        Point2D circCenter = new Point2D.Double(4, 10);
        double circRadius = 2.0;
        List<Double> expResult = Collections.emptyList();
        List<Double> result = GeometryUtil.segCircIntersectionParams(line, circCenter, circRadius);
        assertEquals(expResult, result);
    }

    /**
     * Test of segCircIntersectionParams method, of class GeometryUtil.
     */
    @Test
    public void testSegCircIntersectionParams_3args_2sol1int() {
        Line2D line = new Line2D.Double(5, 10, 15, 10);
        Point2D circCenter = new Point2D.Double(5, 10);
        double circRadius = 2.0;
        List<Double> expResult = Collections.singletonList(0.2);
        List<Double> result = GeometryUtil.segCircIntersectionParams(line, circCenter, circRadius);
        assertEquals(expResult, result);

    }

    /**
     * Test of segCircIntersectionParams method, of class GeometryUtil.
     */
    @Test
    public void testSegCircIntersectionParams_3args_2sol2int() {
        Line2D line = new Line2D.Double(0, 10, 10, 10);
        Point2D circCenter = new Point2D.Double(4, 10);
        double circRadius = 2.0;
        List<Double> expResult = Arrays.asList(0.2, 0.6);
        List<Double> result = GeometryUtil.segCircIntersectionParams(line, circCenter, circRadius);
        assertEquals(expResult, result);

    }

    /**
     * Test of segCircIntersectionParams method, of class GeometryUtil.
     */
    @Test
    public void testSegCircIntersectionParams_3args_1sol2int() {
        Line2D line = new Line2D.Double(0, 12, 10, 12);
        Point2D circCenter = new Point2D.Double(5, 10);
        double circRadius = 2.0;
        List<Double> expResult = Arrays.asList(0.5);
        List<Double> result = GeometryUtil.segCircIntersectionParams(line, circCenter, circRadius);
        assertEquals(expResult, result);

    }


    /**
     * Test of segCircIntersections method, of class GeometryUtil.
     */
    @Test
    public void testSegCircIntersections() {
        Line2D line = new Line2D.Double(0, 10, 10, 10);
        Point2D circCenter = new Point2D.Double(4, 10);
        double circRadius = 2.0;
        List<Point2D> expResult = Arrays.asList(new Point2D.Double(2, 10), new Point2D.Double(6, 10));
        List<Point2D> result = GeometryUtil.segCircIntersections(line, circCenter, circRadius);
        assertEquals(expResult, result);

    }

    /**
     * Test of getDirectionInDregrees method, of class GeometryUtil.
     */
    @Test
    public void testGetDirectionInDegrees() {
        Line2D line;
        line = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(5, 5));
        assertEquals(45.0, GeometryUtil.getDirectionInDegrees(line), 1E-4);

        line = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(-3, 5));
        assertEquals(90 + 45.0, GeometryUtil.getDirectionInDegrees(line), 1E-4);

        line = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(-3, -3));
        assertEquals(180 + 45.0, GeometryUtil.getDirectionInDegrees(line), 1E-4);

        line = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(5, -3));
        assertEquals(270 + 45.0, GeometryUtil.getDirectionInDegrees(line), 1E-4);

        line = new Line2D.Double(new Point2D.Double(0, 0), new Point2D.Double(Math.PI / 2, Math.PI));
        assertEquals(63.4349479675293, GeometryUtil.getDirectionInDegrees(line), 1E-4);
    }

    /**
     * Test of getNewAverageAngle method, of class GeometryUtil.
     */
    @Test
    public void testGetNewAverageAngle() {
        double answer;
        answer = GeometryUtil.getNewAverageAngle(180.0, 0, 2);
        assertEquals((180 * 2.0 + 0.0) / 3.0, answer, 1E-5);

        answer = GeometryUtil.getNewAverageAngle(0.0, 359, 5);
        assertEquals((360.0 * 5.0 + 359.0) / 6.0, answer, 1E-5);

        answer = GeometryUtil.getNewAverageAngle(0.0, 1.0, 5);
        assertEquals((0.0 * 5.0 + 1.0) / 6.0, answer, 1E-5);

        answer = GeometryUtil.getNewAverageAngle(359, 350, 5);
        assertEquals((359.0 * 5.0 + 350.0) / 6.0, answer, 1E-5);

        answer = GeometryUtil.getNewAverageAngle(359, 10, 5);
        assertEquals((-1.0 * 5.0 + 10.0) / 6.0, answer, 1E-5);
    }

    /**
     * Test of getHeadingDirectionDifference method, of class GeometryUtil.
     */
    @Test
    public void testDifferenceInDegreesBetweenLines() {
        Line2D a, b;
        a = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(5, 5));
        b = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(4, 4));
        assertEquals(0.0, GeometryUtil.getHeadingDirectionDifference(a, b), 1E-4);


        a = new Line2D.Double(new Point2D.Double(0, 0), new Point2D.Double(4, -1));
        b = new Line2D.Double(new Point2D.Double(0, 0), new Point2D.Double(4, 1));
        assertEquals(14.036243 * 2, GeometryUtil.getHeadingDirectionDifference(a, b), 1E-4);

        a = new Line2D.Double(new Point2D.Double(1, 1), new Point2D.Double(5, 2));
        b = new Line2D.Double(new Point2D.Double(1, -1), new Point2D.Double(-3, 0));
        assertEquals(180 - 14.036243 * 2, GeometryUtil.getHeadingDirectionDifference(a, b), 1E-4);
    }

    /**
     * Tests of getTrajectoryDecimalPoint
     */
    @Test
    public void testGetTrajectoryDecimalPoint() {
        double[][] coords = new double[][]{
                {0, 0}, {2, 2}, {2, 12}, {7, 17}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        FullTrajectory t = new FullTrajectory(points);

        Point2D point = getTrajectoryDecimalPoint(t, 1.5);
        assertEquals(point.getX(), 2, 1E-8);
        assertEquals(point.getY(), 7, 1E-8);

        point = getTrajectoryDecimalPoint(t, 1.25);
        assertEquals(point.getX(), 2, 1E-8);
        assertEquals(point.getY(), 4.5, 1E-8);


        point = getTrajectoryDecimalPoint(t, 1);
        assertEquals(point.getX(), 2, 1E-8);
        assertEquals(point.getY(), 2, 1E-8);

        point = getTrajectoryDecimalPoint(t, 0);
        assertEquals(point.getX(), 0, 1E-8);
        assertEquals(point.getY(), 0, 1E-8);

        point = getTrajectoryDecimalPoint(t, 3);
        assertEquals(point.getX(), 7, 1E-8);
        assertEquals(point.getY(), 17, 1E-8);

        try {
            getTrajectoryDecimalPoint(t, -1);
            fail("Should have thrown exception");
        } catch (IndexOutOfBoundsException e) {
        }

        try {
            getTrajectoryDecimalPoint(t, 3.1);
            fail("Should have thrown exception");
        } catch (IndexOutOfBoundsException e) {
        }
    }

    /**
     * Tests of getIndexOfTrajectoryClosestToPoint
     */
    @Test
    public void testGetIndexOfTrajectoryClosestToPoint() {
        double[][] coords = new double[][]{
                {0, 0}, {2, 2}, {2, 12}, {7, 17}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        FullTrajectory t = new FullTrajectory(points);

        double index = getIndexOfTrajectoryClosestToPoint(t, new Point2D.Double(3.0, 7.0));
        assertEquals(index, 1.5, 1E-8);

        index = getIndexOfTrajectoryClosestToPoint(t, new Point2D.Double(3.0, 2.0));
        assertEquals(index, 1.0, 1E-8);

        index = getIndexOfTrajectoryClosestToPoint(t, new Point2D.Double(5.5, 13.5));
        assertEquals(index, 2.5, 1E-8);

        index = getIndexOfTrajectoryClosestToPoint(t, new Point2D.Double(8.0, 17.0));
        assertEquals(index, 3.0, 1E-8);

        index = getIndexOfTrajectoryClosestToPoint(t, new Point2D.Double(-1.0, 0));
        assertEquals(index, 0.0, 1E-8);

        index = getIndexOfTrajectoryClosestToPoint(t, new Point2D.Double(3.0, 3.0));
        assertEquals(index, 1.1, 1E-8);

    }

    /**
     * Test of getTrajectoryIndexAfterOffset
     */
    @Test
    public void testGetTrajectoryIndexAfterOffset() {
        double[][] coords = new double[][]{
                {0, 0}, {2, 2}, {2, 12}, {7, 17}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        FullTrajectory t = new FullTrajectory(points);


        double index = getTrajectoryIndexAfterOffset(t, 0, Math.sqrt(2 * 2 * 2));
        assertEquals(index, 1.0, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 1, 10);
        assertEquals(index, 2.0, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 1.5, 5 + 0.5 * Math.sqrt(5 * 5 + 5 * 5));
        assertEquals(index, 2.5, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 1.5, -5 - 0.25 * Math.sqrt(2 * 2 * 2));
        assertEquals(index, 0.75, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 2.5, -10 - 0.5 * Math.sqrt(50) - 0.25 * Math.sqrt(8));
        assertEquals(index, 0.75, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 2.5, 0.3 * Math.sqrt(50));
        assertEquals(index, 2.8, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 2.5, -0.3 * Math.sqrt(50));
        assertEquals(index, 2.2, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 1.5, 0.2 * Math.sqrt(100));
        assertEquals(index, 1.7, 1E-8);

        index = getTrajectoryIndexAfterOffset(t, 1.5, -0.2 * Math.sqrt(100));
        assertEquals(index, 1.3, 1E-8);
    }

    /**
     * Test of convertSubIndexToTrajectoryIndex
     */
    @Test
    public void testConvertSubIndexToTrajectoryIndex() {
        double[][] coords = new double[][]{
                {0, 0}, {2, 2}, {2, 12}, {7, 17}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        FullTrajectory t = new FullTrajectory(points);
        Subtrajectory sub = new Subtrajectory(t, 0.25, 2.75);

        double index = convertSubIndexToTrajectoryIndex(sub, 0);
        assertEquals(index, 0.25, 1E-8);

        index = convertSubIndexToTrajectoryIndex(sub, 3);
        assertEquals(index, 2.75, 1E-8);

        index = convertSubIndexToTrajectoryIndex(sub, 2);
        assertEquals(index, 2, 1E-8);

        index = convertSubIndexToTrajectoryIndex(sub, 1);
        assertEquals(index, 1, 1E-8);

        index = convertSubIndexToTrajectoryIndex(sub, 1.5);
        assertEquals(index, 1.5, 1E-8);

        index = convertSubIndexToTrajectoryIndex(sub, (1 / 3.0));
        assertEquals(index, 0.5, 1E-8);

        index = convertSubIndexToTrajectoryIndex(sub, 2 + (1 / 3.0));
        assertEquals(index, 2.25, 1E-8);
    }

    /**
     * Test of getIndexToIndexDistance
     */
    @Test
    public void testGetIndexToIndexDistance() {
        //@ToDo implement this shit.

        double[][] coords = new double[][]{
                {0, 0}, {2, 2}, {2, 12}, {7, 17}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        FullTrajectory t = new FullTrajectory(points);

        helpGetIndexToIndexDistance(t, 0, Math.sqrt(2 * 2 * 2));
        helpGetIndexToIndexDistance(t, 1.0, 10);
        helpGetIndexToIndexDistance(t, 1.5, 5 + 0.5 * Math.sqrt(5 * 5 + 5 * 5));
        helpGetIndexToIndexDistance(t, 1.5, -5 - 0.25 * Math.sqrt(2 * 2 * 2));
        helpGetIndexToIndexDistance(t, 2.5, -10 - 0.5 * Math.sqrt(50) - 0.25 * Math.sqrt(8));
        helpGetIndexToIndexDistance(t, 2.5, 0.3 * Math.sqrt(50));
        helpGetIndexToIndexDistance(t, 2.5, -0.3 * Math.sqrt(50));
        helpGetIndexToIndexDistance(t, 1.5, 0.2 * Math.sqrt(100));
        helpGetIndexToIndexDistance(t, 1.5, -0.2 * Math.sqrt(100));
    }

    /**
     * Helper function for one specific test of getIndexToIndexDistance
     */
    private void helpGetIndexToIndexDistance(Trajectory t, double startIndex, double offset) {
        double endIndex = getTrajectoryIndexAfterOffset(t, startIndex, offset);

        assertEquals(GeometryUtil.getIndexToIndexDistance(t, startIndex, endIndex), Math.abs(offset), 1E-8);
        assertEquals(GeometryUtil.getDirectionalIndexToIndexDistance(t, startIndex, endIndex), offset, 1E-8);
    }

    /**
     * Test of getIndexRangeAroundPointOnTrajectory
     */
    @Test
    public void testGetIndexRangeAroundPointOnTrajectory() {
        double[][] coords = new double[][]{
                {0, 0}, {2, 2}, {2, 12}, {7, 17}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(coords);
        FullTrajectory t = new FullTrajectory(points);

        Range<Double> range = getIndexRangeAroundPointOnTrajectory(t, new Point2D.Double(3.0, 7.0), 5, true);
        assertEquals(range.lowerEndpoint(), 1.0, 1E-8);
        assertEquals(range.upperEndpoint(), 2.0, 1E-8);

        range = getIndexRangeAroundPointOnTrajectory(t, new Point2D.Double(3.0, 7.0), 7, true);
        assertEquals(range.lowerEndpoint(), 1.0 - (2.0 / Math.sqrt(8)), 1E-8);
        assertEquals(range.upperEndpoint(), 2.0 + (2.0 / Math.sqrt(50)), 1E-8);
    }

    /**
     * Test of cutOffSubtrajectoryByRepresentativeRange()
     */
    @Test
    public void testCutOffSubtrajectoryByRepresentativeRange(){
        double[][] subCoords = new double[][]{
                {-1.0, -1.0}, {0, 0}, {2, 2}, {2, 12}, {7, 17}, {2.5, 13}, {1.5, 1}
        };

        double[][] repCoords = new double[][]{
                {0, 1}, {2, 3}, {2, 13}, {7, 18}, {3, 14}, {2, 2}
        };

        List<Point2D> points = TestUtil.doubleArrayToTrajectory(subCoords);
        FullTrajectory fullSub = new FullTrajectory(points);
        Subtrajectory sub = new Subtrajectory(fullSub, 0, fullSub.numPoints() - 1);

        points = TestUtil.doubleArrayToTrajectory(repCoords);
        FullTrajectory rep = new FullTrajectory(points);

        Subtrajectory repSub = new Subtrajectory(rep, 1, 3);
        Subtrajectory subtrajectory = cutOffSubtrajectoryByRepresentativeRange(repSub, sub);
        assertEquals(2.1, subtrajectory.getFromIndex(), 1E-8);
        assertEquals(4.0 , subtrajectory.getToIndex(), 1E-8);

        repSub = new Subtrajectory(rep, 1, 4.0);
        subtrajectory = cutOffSubtrajectoryByRepresentativeRange(repSub, sub);
        assertEquals(2.1, subtrajectory.getFromIndex(), 1E-8);
        assertEquals(4.827586206896552 , subtrajectory.getToIndex(), 1E-8);

        repSub = new Subtrajectory(rep, 1, 5.0);
        subtrajectory = cutOffSubtrajectoryByRepresentativeRange(repSub, sub);
        assertEquals(2.1, subtrajectory.getFromIndex(), 1E-8);
        assertEquals(5.913793103448276 , subtrajectory.getToIndex(), 1E-8);
    }
}
