package mapconstruction.algorithms.representative;

import org.junit.*;

import static org.junit.Assert.assertEquals;

/**
 * @author Jorrick Sleijster
 */
public class ForcesTest {

    public ForcesTest() {
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
     * Test of calculateParabola
     */
    @Test
    public void testCalculateParabola1() {
        double expResult = 1.0;
        double result = Forces.calculateParabola(0, 45,2,1);
        assertEquals(expResult, result, 1E-5);
    }

    /**
     * Test of calculateParabola
     */
    @Test
    public void testCalculateParabola2() {
        double expResult = 3.0;
        double result = Forces.calculateParabola(45, 45,2,1);
        assertEquals(expResult, result, 1E-5);
    }


    /**
     * Test of calculateParabola
     */
    @Test
    public void testCalculateParabola3() {
        double expResult = 4.555555;
        double result = Forces.calculateParabola(60, 45,2,1);
        assertEquals(expResult, result, 1E-5);
    }

    /**
     * Test of calculateParabola
     */
    @Test
    public void testCalculateParabola4() {
        double expResult = 3;
        double result = Forces.calculateParabola(60, 60,3,0);
        assertEquals(expResult, result, 1E-5);
    }

    /**
     * Test of calculateParabola
     */
    @Test
    public void testCalculateParabola5() {
        double expResult = 0;
        double result = Forces.calculateParabola(0, 60,3,0);
        assertEquals(expResult, result, 1E-5);
    }
}
