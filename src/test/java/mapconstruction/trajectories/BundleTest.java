package mapconstruction.trajectories;

import junit.framework.TestCase;
import mapconstruction.TestUtil;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 *
 * @author Roel
 */
public class BundleTest extends TestCase {

    Trajectory t1;
    Trajectory t2;
    Trajectory t3;

    public BundleTest(String testName) {
        super(testName);
        double[][] coords1 = new double[][]{
            {0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}
        };

        double[][] coords2 = new double[][]{
            {0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2}, {5, 2}, {6, 2}
        };

        double[][] coords3 = new double[][]{
            {0, 3}, {1, 3}, {2, 3}, {3, 3}, {4, 3}, {5, 3}, {6, 3}
        };
        
        List<Point2D> points1 = TestUtil.doubleArrayToTrajectory(coords1);
        List<Point2D> points2 = TestUtil.doubleArrayToTrajectory(coords2);
        List<Point2D> points3 = TestUtil.doubleArrayToTrajectory(coords3);
        
        t1 = new FullTrajectory(points1);
        t2 = new FullTrajectory(points2);
        t3 = new FullTrajectory(points3);
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
        System.out.println("Bundle constructor exeptions");
        try {
            Bundle.create(null);
            fail("Should have thrown exception");
        } catch (NullPointerException e) {
            
        }
    }


    /**
     * Test of getSubtrajectories method, of class Bundle.
     */
    public void testGetSubtrajectories() {
        System.out.println("getSubtrajectories");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 0, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 0, 4);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);


        Set<Subtrajectory> result = instance.getSubtrajectories();
        assertEquals(3, result.size());
        assertTrue(result.contains(s1));
        assertTrue(result.contains(s2));
        assertTrue(result.contains(s3));
    }

    /**
     * Test of getFullTrajectories method, of class Bundle.
     */
    public void testGetFullTrajectories() {
        System.out.println("getFullTrajectories");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 0, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 0, 4);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Set<Trajectory> result = instance.getParentTrajectories();
        assertEquals(3, result.size());
        assertTrue(result.contains(t1));
        assertTrue(result.contains(t2));
        assertTrue(result.contains(t3));

    }

    /**
     * Test of size method, of class Bundle.
     */
    public void testSize() {
        System.out.println("size");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 0, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 0, 4);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        int expResult = 3;
        int result = instance.size();
        assertEquals(expResult, result);
    }

    /**
     * Test of length method, of class Bundle.
     */
    public void testLength() {
        System.out.println("length");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 3);
        Subtrajectory s2 = new Subtrajectory(t2, 0, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 1, 4);

       ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);
        int expResult = 5;
        int result = instance.discreteLength();
        assertEquals(expResult, result);
    }




    /**
     * Test of covers method, of class Bundle.
     */
    public void testCovers0() {
        System.out.println("covers: true");
        Subtrajectory s1 = new Subtrajectory(t1, 1, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 1, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 1, 4);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Subtrajectory ss1 = new Subtrajectory(t1, 2, 3);
        Subtrajectory ss2 = new Subtrajectory(t2, 2, 3);
        
        
        ArrayList<Subtrajectory> list2 = new ArrayList<>();
        list2.add(ss1);
        list2.add(ss2);
        Bundle other = Bundle.create(list2);
        boolean expResult = true;
        boolean result = instance.covers(other);
        assertEquals(expResult, result);
    }
    
    /**
     * Test of covers method, of class Bundle.
     */
    public void testCovers1() {
        System.out.println("covers: false - proper overlap other case");
        Subtrajectory s1 = new Subtrajectory(t1, 2, 3);
        Subtrajectory s2 = new Subtrajectory(t2, 2, 3);
        Subtrajectory s3 = new Subtrajectory(t3, 2, 3);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Subtrajectory ss1 = new Subtrajectory(t1, 1, 4);
        Subtrajectory ss2 = new Subtrajectory(t2, 1, 4);
        
        ArrayList<Subtrajectory> list2 = new ArrayList<>();
        list2.add(ss1);
        list2.add(ss2);
        Bundle other = Bundle.create(list2);
        
        boolean expResult = false;
        boolean result = instance.covers(other);
        assertEquals(expResult, result);
    }


    /**
     * Test of hasAsLambdaSubBundle method, of class Bundle.
     */
    public void testHasAsLambdaSubBundle0() {
        System.out.println("hasAsLambdaSubBundle: true, lambda 0 (same as covering)");
        Subtrajectory s1 = new Subtrajectory(t1, 1, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 1, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 1, 4);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Subtrajectory ss1 = new Subtrajectory(t1, 2, 3);
        Subtrajectory ss2 = new Subtrajectory(t2, 2, 3);
        ArrayList<Subtrajectory> list2 = new ArrayList<>();
        list2.add(ss1);
        list2.add(ss2);
        Bundle other = Bundle.create(list2);
        
        
        double lambda = 0;

        boolean expResult = true;
        boolean result = instance.hasAsLambdaSubBundle(other, lambda);
        assertEquals(expResult, result);
    }
    
    /**
     * Test of hasAsLambdaSubBundle method, of class Bundle.
     */
    public void testHasAsLambdaSubBundle1() {
        System.out.println("hasAsLambdaSubBundle: true, proper overlap");
        Subtrajectory s1 = new Subtrajectory(t1, 2, 3);
        Subtrajectory s2 = new Subtrajectory(t2, 2, 3);
        Subtrajectory s3 = new Subtrajectory(t3, 2, 3);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Subtrajectory ss1 = new Subtrajectory(t1, 1, 4);
        Subtrajectory ss2 = new Subtrajectory(t2, 1, 4);
        ArrayList<Subtrajectory> list2 = new ArrayList<>();
        list2.add(ss1);
        list2.add(ss2);
        Bundle other = Bundle.create(list2);
        
        
        double lambda = 2;

        boolean expResult = true;
        boolean result = instance.hasAsLambdaSubBundle(other, lambda);
        assertEquals(expResult, result);
    }
    
    /**
     * Test of hasAsLambdaSubBundle method, of class Bundle.
     */
    public void testHasAsLambdaSubBundle2() {
        System.out.println("hasAsLambdaSubBundle: false, proper overlap");
        Subtrajectory s1 = new Subtrajectory(t1, 2, 3);
        Subtrajectory s2 = new Subtrajectory(t2, 2, 3);
        Subtrajectory s3 = new Subtrajectory(t3, 2, 3);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Subtrajectory ss1 = new Subtrajectory(t1, 1, 4);
        Subtrajectory ss2 = new Subtrajectory(t2, 1, 4);
        ArrayList<Subtrajectory> list2 = new ArrayList<>();
        list2.add(ss1);
        list2.add(ss2);
        Bundle other = Bundle.create(list2);
        
        
        double lambda = 1;

        boolean expResult = false;
        boolean result = instance.hasAsLambdaSubBundle(other, lambda);
        assertEquals(expResult, result);
    }
    
    /**
     * Test of hasAsLambdaSubBundle method, of class Bundle.
     */
    public void testHasAsLambdaSubBundle4() {
        System.out.println("hasAsLambdaSubBundle: true, proper overlap, different sizes in compared");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 3);
        Subtrajectory s2 = new Subtrajectory(t2, 2, 3);
        Subtrajectory s3 = new Subtrajectory(t3, 2, 4);

        ArrayList<Subtrajectory> list = new ArrayList<>();
        list.add(s1);
        list.add(s2);
        list.add(s3);       
        Bundle instance = Bundle.create(list);

        Subtrajectory ss1 = new Subtrajectory(t1, 1, 4);
        Subtrajectory ss2 = new Subtrajectory(t3, 1, 4);
        ArrayList<Subtrajectory> list2 = new ArrayList<>();
        list2.add(ss1);
        list2.add(ss2);
        Bundle other = Bundle.create(list2);
        
        
        double lambda = 1;

        boolean expResult = true;
        boolean result = instance.hasAsLambdaSubBundle(other, lambda);
        assertEquals(expResult, result);
    }

}
