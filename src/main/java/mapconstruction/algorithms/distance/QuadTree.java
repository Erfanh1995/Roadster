package mapconstruction.algorithms.distance;

import java.util.*;
import java.util.stream.Collectors;

public class QuadTree<T extends Comparable<T>> implements Iterable<T> {
    // Variables for determining tree shape
    private int BUCKET_SIZE = 10;
    private boolean DYNAMIC_BUCKETS = false;
    private int DYNAMIC_FREQUENCY = 30;
    private double DYNAMIC_EXPONENT = 1 / 3d;
    private int MAX_DEPTH = 7;

    private int size = 0;
    private Quad root;

    public QuadTree() {
        // use defaults
    }

    public QuadTree(int bucketSize) {
        BUCKET_SIZE = bucketSize;
    }

    public QuadTree(int bucketSize, int maxDepth) {
        BUCKET_SIZE = bucketSize;
        MAX_DEPTH = maxDepth;
    }

    public QuadTree(double dynamicExponent, int dynamicFrequency) {
        DYNAMIC_BUCKETS = true;
        DYNAMIC_EXPONENT = dynamicExponent;
        DYNAMIC_FREQUENCY = dynamicFrequency;
    }

    public QuadTree(double dynamicExponent, int dynamicFrequency, int maxDepth) {
        DYNAMIC_BUCKETS = true;
        DYNAMIC_EXPONENT = dynamicExponent;
        DYNAMIC_FREQUENCY = dynamicFrequency;
        MAX_DEPTH = maxDepth;
    }

    public class Entry {
        private double x, y;
        private T value;

        public Entry(double x, double y, T value) {
            this.x = x;
            this.y = y;
            this.value = value;
        }

        boolean inRange(double x, double y, double r) {
            return Math.pow(this.x - x, 2) + Math.pow(this.y - y, 2) <= Math.pow(r, 2);
        }

        boolean inBounds(double x1, double y1, double x2, double y2) {
            return (x1 <= x && x <= x2) && (y1 <= y && y <= y2);
        }
    }

    public class Quad implements Iterable<Quad> {
        private Quad
            UL = null,
            UR = null,
            LL = null,
            LR = null;
        private int depth;
        private double x1, y1, x2, y2;
        private List<Entry> items = new LinkedList<>();

        Quad(Quad parent, double x1, double y1, double x2, double y2) {
            this.depth = parent == null ? 0 : parent.depth + 1;
            this.x1 = x1;  this.y1 = y1;
            this.x2 = x2;  this.y2 = y2;
        }

        private boolean overlaps(double X1, double Y1, double X2, double Y2) {
            return !(x2 < X1) && !(y2 < Y1) && !(x1 > X2) && !(y1 > Y2);
        }

        /**
         * Check whether the current quad intersects the circle defined by the center point (x,y) and radius r.
         * Solution borrowed from <a href="https://stackoverflow.com/a/402010"> Stack Overflow </a>.
         */
        private boolean overlaps(double x, double y, double r) {
            double cX = Math.abs(x - (x1 + x2) / 2d);
            double cY = Math.abs(y - (y1 + y2) / 2d);

            if (cX > (x2 - x1) / 2d + r) return false;
            if (cY > (y2 - y1) / 2d + r) return false;

            if (cX <= (x2 - x1) / 2d) return true;
            if (cY <= (y2 - y1) / 2d) return true;

            double cD = Math.pow(cX - (x2 - x1) / 2d, 2) +
                    Math.pow(cY - (y2 - y1) / 2d, 2);

            return cD <= r * r;
        }

        private void createChildren() {
            UL = new Quad(this, x1, y1, (x2 + x1) / 2d, (y2 + y1) / 2d);
            UR = new Quad(this, (x2 + x1) / 2d, y1, x2, (y2 + y1) / 2d);
            LL = new Quad(this, x1, (y2 + y1) / 2d, (x2 + x1) / 2d, y2);
            LR = new Quad(this, (x2 + x1) / 2d, (y2 + y1) / 2d, x2, y2);
        }

        private boolean hasChildren() {
            return UL != null;
        }

        void insert(Entry e) {
            if (hasChildren()) {
                // find target quadrant, based on their bounds.
                Quad target = e.x < UL.x2 ? (e.y < UL.y2 ? UL : LL) : (e.y < UL.y2 ? UR : LR);
                target.insert(e);
            } else {
                if (items.size() < BUCKET_SIZE || depth >= MAX_DEPTH) {
                    items.add(e);
                } else {
                    createChildren();
                    // move items to child quads
                    for (Entry i : items) {
                        insert(i);
                    } // remove items from current
                    items = Collections.emptyList();
                    // insert new quad
                    insert(e);
                }
            }
        }

        void rangeQuery(double x, double y, double r, Set<T> resultSet) {
            if (!overlaps(x, y, r)) return;

            for (Entry item : items) {
                if (item.inRange(x, y, r)) {
                    resultSet.add(item.value);
                }
            }
            for (Quad child : this) {
                child.rangeQuery(x, y, r, resultSet);
            }
        }

        void boundQuery(double x1, double y1, double x2, double y2, Set<T> resultSet) {
            if (!overlaps(x1, y1, x2, y2)) return;

            for (Entry item : items) {
                if (item.inBounds(x1, y1, x2, y2)) {
                    resultSet.add(item.value);
                }
            }
            for (Quad child : this) {
                child.boundQuery(x1, y1, x2, y2, resultSet);
            }
        }

        @Override
        public String toString() {
            String d = new String(new char[depth]).replace('\0', '\t');
            if (hasChildren()) {
                return  "{\n" +
                        d + "\tUL: " + UL + "\n" +
                        d + "\tUR: " + UR + "\n" +
                        d + "\tLL: " + LL + "\n" +
                        d + "\tLR: " + LR + "\n" +
                        d + "}";
            }
            return items.stream().map(o -> o.value).collect(Collectors.toList()).toString();
        }

        @Override
        public Iterator<Quad> iterator() {
            if (hasChildren())
                return Arrays.asList(UL, UR, LL, LR).iterator();
            return Collections.emptyIterator();
        }
    }

    /**
     * Initialize the quadtree by determining its bounds, and therefore the outer 'root' Quad. The bounds are defined
     * by a top-left corner ({@code x1},{@code y1}) and a bottom-right corner ({@code x2},{@code y2}).
     * This will also remove any existing entries from the data structure.
     */
    public void initialize(double x1, double y1, double x2, double y2) {
        this.root = new Quad(null, x1, y1, x2, y2);
        this.size = 0;
    }

    /**
     * Insert an element at location ({@code x},{@code y}) with a given value.
     */
    public void insert(double x, double y, T value) {
        Entry e = new Entry(x, y, value);
        root.insert(e);

        size ++; // resize buckets if required
        if (DYNAMIC_BUCKETS && size % DYNAMIC_FREQUENCY == 0) {
            BUCKET_SIZE = Math.max(7, (int) Math.pow(size, DYNAMIC_EXPONENT));
        }
    }

    /**
     * Get the set of values currently present inside the QuadTree.
     * @return A set of values located within the QuadTree.
     */
    public Set<T> values() {
        return getInBounds(root.x1, root.y1, root.x2, root.y2);
    }

    /**
     * Get a sorted set of values which are within distance {@code r} from point ({@code x},{@code y}).
     * The resulting set is sorted by the natural order of the type parameter {@code T}.
     * @return A set of values located within the given range.
     */
    public Set<T> getInRange(double x, double y, double r) {
        Set<T> resultSet = new TreeSet<>();
        if (root != null) {
            root.rangeQuery(x, y, r, resultSet);
        }
        return resultSet;
    }

    /**
     * Get a sorted set of items which are within an axis-aligned rectangle defined by top-left corner ({@code x1},{@code x2})
     * and bottom-right corner ({@code x2},{@code y2}). The resulting set is sorted by the natural order of the type
     * parameter {@code T}.
     * @return A set of values located within the given rectangle.
     */
    public Set<T> getInBounds(double x1, double y1, double x2, double y2) {
        Set<T> resultSet = new TreeSet<>();
        if (root != null) {
            root.boundQuery(x1, y1, x2, y2, resultSet);
        }
        return resultSet;
    }

    /**
     * Get the number of elements stored inside the QuadTree.
     * @return The number of elements.
     */
    public int size() {
        return size;
    }


    @Override
    public Iterator<T> iterator() {
        return values().iterator();
    }

    @Override
    public String toString() {
        return "QuadTree(" + root + ")";
    }
}
