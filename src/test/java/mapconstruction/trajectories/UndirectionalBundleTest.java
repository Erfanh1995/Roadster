package mapconstruction.trajectories;

import java.util.ArrayList;

/**
 *
 * @author Roel
 */
public class UndirectionalBundleTest extends BundleTest {
    
    public UndirectionalBundleTest(String testName) {
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
     * Test of hashCode method, of class UndirectionalBundle.
     */
    public void testHashCode() {
        System.out.println("hashCode");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 0, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 0, 4);
        
        ArrayList<Subtrajectory> tInst = new ArrayList<>();
        tInst.add(s1);
        tInst.add(s2);
        tInst.add(s3);
        
        UndirectionalBundle instance = UndirectionalBundle.create(tInst);
        
        ArrayList<Subtrajectory> tRev1 = new ArrayList<>();
              
        tRev1.add(s1.reverse());
        tRev1.add(s2);
        tRev1.add(s3);
        
        UndirectionalBundle rev1 = UndirectionalBundle.create(tRev1);
        
        
        ArrayList<Subtrajectory> tRev2 = new ArrayList<>();
        tRev2.add(s1);
        tRev2.add(s2.reverse());
        tRev2.add(s3);  

        
        UndirectionalBundle rev2 = UndirectionalBundle.create(tRev2);
        
        
        ArrayList<Subtrajectory> tRev3 = new ArrayList<>();
        tRev3.add(s1);
        tRev3.add(s2);
        tRev3.add(s3.reverse()); 
        
        UndirectionalBundle rev3 = UndirectionalBundle.create(tRev3);
        
        int result = instance.hashCode();
        assertEquals(rev1.hashCode(), result);
        assertEquals(rev2.hashCode(), result);
        assertEquals(rev3.hashCode(), result);
    }

    /**
     * Test of equals method, of class UndirectionalBundle.
     */
    public void testEquals() {
        System.out.println("equals");
        Subtrajectory s1 = new Subtrajectory(t1, 0, 4);
        Subtrajectory s2 = new Subtrajectory(t2, 0, 4);
        Subtrajectory s3 = new Subtrajectory(t3, 0, 4);
        
         ArrayList<Subtrajectory> tInst = new ArrayList<>();
        tInst.add(s1);
        tInst.add(s2);
        tInst.add(s3);
        
        UndirectionalBundle instance = UndirectionalBundle.create(tInst);
        
        ArrayList<Subtrajectory> tRev1 = new ArrayList<>();
              
        tRev1.add(s1.reverse());
        tRev1.add(s2);
        tRev1.add(s3);
        
        UndirectionalBundle rev1 = UndirectionalBundle.create(tRev1);
        
        
        ArrayList<Subtrajectory> tRev2 = new ArrayList<>();
        tRev2.add(s1);
        tRev2.add(s2.reverse());
        tRev2.add(s3);  

        
        UndirectionalBundle rev2 = UndirectionalBundle.create(tRev2);
        
        
        ArrayList<Subtrajectory> tRev3 = new ArrayList<>();
        tRev3.add(s1);
        tRev3.add(s2);
        tRev3.add(s3.reverse()); 
        
        UndirectionalBundle rev3 = UndirectionalBundle.create(tRev3);
        
        assertEquals(rev1, instance);
        assertEquals(rev2, instance);
        assertEquals(rev3, instance);       
    }
    
}
