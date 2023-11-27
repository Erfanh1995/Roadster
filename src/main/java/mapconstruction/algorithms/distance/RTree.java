package mapconstruction.algorithms.distance;

import com.google.common.collect.Lists;
import mapconstruction.util.Pair;

import java.awt.*;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;

import static java.lang.Double.isNaN;

public class RTree<K extends Shape, V> {

//    Map<K,V> values;
    int size;
    Node root;

    private final int bucketSize;

    public RTree(int bucketSize) {
        if (bucketSize <= 1) {
            throw new IllegalArgumentException("RTree buckets cannot be smaller than 2");
        }
        this.bucketSize = bucketSize;

//        values = new HashMap<>();
        root = new Node();
        this.size = 0;
    }

    public void insert(K index, V value) {
//        values.put(index, value);
        root.insert(index, value);
        size ++;
    }

    public Collection<V> values() {
        return windowQuery(Double.MIN_VALUE, Double.MIN_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        //return this.values();
    }

    public Rectangle2D getBounds() {
        return root.getBounds2D();
    }

    public Set<V> windowQuery(double x1, double y1, double x2, double y2) {
        return windowQuery(x1, y1, x2, y2, new HashSet<>());
    }

    public Set<V> windowQuery(double x1, double y1, double x2, double y2, Set<V> resultSet) {
        Set<K> indices = new HashSet<>();

        Rectangle2D bounds = new Rectangle2D.Double(x1, y1, x2 - x1, y2 - y1);
        root.findInWindow(bounds, resultSet);
//        resultSet = new HashSet<>(indices.size());
//        indices.stream().map(values::get).forEach(resultSet::add);

        return resultSet;
    }

    public int size() {
        return this.size;
    }

    public RTree(int n, Map<K,V> values) {
        this.bucketSize = n;
        int r = values.size();
        this.size = r;
        List<K> shapes = values.keySet().stream().sorted(Comparator.comparingDouble(k -> k.getBounds2D().getCenterX())).collect(Collectors.toList());
        List<Node> nodes = new ArrayList<>((int) Math.ceil(r / (double) n));
        for (List<K> slice : Lists.partition(shapes, (int) Math.ceil(Math.sqrt(Math.ceil(r / (double) n))))) {
            slice.sort(Comparator.comparingDouble(k -> k.getBounds2D().getCenterY()));

            for (List<K> cell : Lists.partition(slice, n)) {
                Node node = new Node();
                for (K shape : cell) {
                    node.insert(shape, values.get(shape));
                }
                nodes.add(node);
            }
        }

        while (nodes.size() > 1) {
            List<Node> newNodes = new ArrayList<>((int) Math.ceil(nodes.size() / (double) n));
            for (List<Node> children : Lists.partition(nodes, n)) {
                Node parent = new Node();
                for (Node child : children) {
                    parent.insert(child);
                }
                newNodes.add(parent);
            }
            nodes = newNodes;
        }

        if (nodes.size() < 1) {
            throw new IllegalArgumentException("Bulk insertion must contain at least one element");
        }
        this.root = nodes.get(0);
    }

    public class Node extends Rectangle2D.Double {
        private Node parent;
        private List<Node> children;
        private Map<K, V> values;

        Node() {
            super(java.lang.Double.NaN, 0,0,0);
            values = new HashMap<>();
        }

        Node(Node parent) {
            this(); // call default constructor
            this.parent = parent;
        }

        @Override
        public boolean intersects(double x, double y, double w, double h) {
            double x0 = getX();
            double y0 = getY();
            return (x + w >= x0 &&
                    y + h >= y0 &&
                    x <= x0 + getWidth() &&
                    y <= y0 + getHeight());
        }

        boolean isLeaf() {
            return children == null;
        }

        boolean hasContents() {
            return !isNaN(getX());
        }

        void insert(K elem, V value) {
            if (isLeaf()) {
                values.put(elem, value);
                if (values.size() > bucketSize) {
                    split();
                    return;
                }
            } else {
                decideOverlap(children, elem).insert(elem, value);
            }

            // grow the bounding box
            if (hasContents()) {
                this.add(elem.getBounds2D());
            } else {
                this.setRect(elem.getBounds2D());
            }
        }

        void insert(Node n) {
            if (children == null) {
                children = new ArrayList<>();
            }

            children.add(n);
            if (children.size() > bucketSize) {
                split();
            } else {
                // override parent
                n.parent = this;
                // update boundingbox
                if (hasContents()) {
                    this.add(n);
                } else {
                    this.setRect(n);
                }
            }
        }

        void insert(Node n, Node m) {
            if (children == null) {
                children = new ArrayList<>();
            }

            children.add(n);
            children.add(m);
            if (children.size() > bucketSize) {
                split();
            } else {
                // override parent
                n.parent = this;
                m.parent = this;
                // update boundingbox
                if (hasContents()) {
                    this.add(n);
                } else {
                    this.setRect(n);
                }
                this.add(m);
            }
        }

        void remove(Node n) {
            if (children != null) {
                children.remove(n);
            }
        }

        void findInWindow(Rectangle2D bounds, Set<V> resultSet) {
            if (isLeaf()) {
                for (Map.Entry<K,V> value : values.entrySet()) {
                    if (value.getKey().intersects(bounds)) {
                        resultSet.add(value.getValue());
                    }
                }
            } else {
                for (Node child : children) {
                    if (child.intersects(bounds)) {
                        child.findInWindow(bounds, resultSet);
                    }
                }
            }
        }

        /**
         * R-tree implementation of the heuristic, minimizing the growth of the bounding box.
         */
        Node decide(Collection<Node> nodes, Shape elem) {
            double min = java.lang.Double.MAX_VALUE;
            Node minNode = null;
            Rectangle2D bounds = elem.getBounds();
            for (Node n : nodes) {
                Rectangle2D u = bounds.createUnion(n);
                double area = u.getWidth() * u.getHeight();
                if (area < min || (area == min && n.width*n.height < minNode.width*minNode.height)) {
                    min = area;
                    minNode = n;
                }
            }
            return minNode;
        }

        Node decideOverlap(Collection<Node> nodes, Shape elem) {
            double min = java.lang.Double.MAX_VALUE;
            Node minNode = null;
            Rectangle2D bounds = elem.getBounds();
            for (Node n : nodes) {
                double overlap = overlap(nodes, n, elem);
                if (overlap < min || (overlap == min && n.width*n.height < minNode.width*minNode.height)) {
                    min = overlap;
                    minNode = n;
                }
            }
            return minNode;
        }

        double overlap(Collection<Node> nodes, Node n, Shape elem) {
            Rectangle2D union = n.createUnion(elem.getBounds());
            double sum = 0;
            for (Node m : nodes) {
                if (n == m) continue;
                Rectangle2D intersect = union.createIntersection(m);
                sum += intersect.getWidth() * intersect.getHeight();
            }
            return sum;
        }

        void split() {
            Node parent;
            if (this.parent == null) {
                // we can assume 'this' is the root, so we create a new root
                parent = new Node();
                root = parent;
            } else {
                parent = this.parent;
                parent.remove(this);
            }
            Node left = new Node(), right = new Node();
            List<Node> newNodes = Arrays.asList(left, right);

            if (isLeaf()) {
                Pair<K, K> groups = group(values.keySet());
                left.insert(groups.getFirst(), values.get(groups.getFirst()));
                right.insert(groups.getSecond(), values.get(groups.getSecond()));

                for (Map.Entry<K,V> value : values.entrySet()) {
                    if (value.getKey() == groups.getFirst() || value.getKey() == groups.getSecond()) continue;

                    decide(newNodes, value.getKey()).insert(value.getKey(), value.getValue());
                }
            } else {
                Pair<Node, Node> groups = group(children);
                left.insert(groups.getFirst());
                right.insert(groups.getSecond());

                for (Node value : children) {
                    if (value == groups.getFirst() || value == groups.getSecond()) continue;

                    decideOverlap(newNodes, value).insert(value);
                }
            }
            parent.insert(left, right);
        }

        private <T extends Shape> Pair<T,T> group(Collection<T> values) {
            double maxDist = -1d;
            Pair<T,T> maxPair = null;
            for (T s : values) {
                for (T t : values) {
                    if (s == t) continue;

                    double distance = distance(s, t);
                    if (distance > maxDist) {
                        maxDist = distance;
                        maxPair = new Pair<>(s, t);
                    }
                }
            }
            return maxPair;
        }

        double distance(Shape k1, Shape k2) {
            Rectangle2D b1 = k1.getBounds2D();
            Rectangle2D b2 = k2.getBounds2D();

            double xDist = Math.max(0, Math.max(b2.getMinX() - b1.getMaxX(), b1.getMinX() - b2.getMaxX()));
            double yDist = Math.max(0, Math.max(b2.getMinY() - b1.getMaxY(), b1.getMinY() - b2.getMaxY()));

            // return distance squared to save a sqrt()
            return xDist*xDist + yDist*yDist;
        }

        int depth() {
            if (parent == null) return 0;

            return parent.depth() + 1;
        }

        int size() {
            if (isLeaf()) {
                return values.size();
            } else {
                return children.size();
            }
        }

        List<Rectangle2D> bounds() {
            if (isLeaf()) {
                return values.keySet().stream().map(K::getBounds).collect(Collectors.toList());
            } else {
                return children.stream().map(Node::getBounds2D).collect(Collectors.toList());
            }
        }

        int nestedSize() {
            if (isLeaf()) {
                return 1;
            } else {
                return children.stream().mapToInt(Node::nestedSize).sum();
            }
        }
    }

    @Override
    public String toString() {
        return "RTree(" + root.nestedSize() + ")";
    }
}
