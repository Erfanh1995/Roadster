package mapconstruction.algorithms.distance;

import junit.framework.TestCase;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class QuadTreeTest extends TestCase {

    public QuadTreeTest(String testName) {
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

    public void testInserts1() {
        QuadTree<Integer> qTree = new QuadTree<>(2, 3);
        qTree.initialize(-10, -10, 10, 10);

        qTree.insert(1,1, 1);
        qTree.insert(1, 2, 2);
        qTree.insert(1, 3, 3);

        TestCase.assertEquals(3, qTree.size());
    }

    public void testRangeQuery1() {
        QuadTree<Integer> qTree = new QuadTree<>(2, 3);
        qTree.initialize(-10, -10, 10, 10);

        qTree.insert(1,1, 1);
        qTree.insert(1, 2, 2);
        qTree.insert(1, 3, 3);


        qTree.insert(1,1, 4);
        qTree.insert(2, 2, 5);
        qTree.insert(2, 3, 6);

        Set<Integer> values = new HashSet<>(Arrays.asList(1, 2, 4));
        TestCase.assertEquals(values, qTree.getInRange(1, 1, 1));
    }

}
