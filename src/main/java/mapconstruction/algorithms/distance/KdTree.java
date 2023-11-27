package mapconstruction.algorithms.distance;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

public class KdTree<T> {

    private final int k;
    private Node root;
    private int size = 0;
    private BiFunction<double[], double[], Double> distance;

    public KdTree(int dimensions) {
        this.k = dimensions;
        this.root = new Node(0);
        this.distance = (c1, c2) -> {
            double s = 0;
            for (int i = 0; i < c1.length && i < c2.length; i++) {
                s += Math.pow(c1[i] - c2[i], 2);
            }
            return Math.sqrt(s);
        };
    }

    public KdTree(int dimensions, BiFunction<double[], double[], Double> d) {
        this.k = dimensions;
        this.root = new Node(0);
        this.distance = d;
    }

    public void insert(T value, double... coordinates) {
        root.insert(value, coordinates);
        size ++;
    }

    public void insertAll(Collection<T> values, Function<T, List<Double>> m) {
        for (T value : values) {
            insert(value, m.apply(value).stream().mapToDouble(Number::doubleValue).toArray());
        }
    }

    public void softDelete(T value, double... coordinates) {
        for (int i = 0; i < coordinates.length / k; i++) {
            root.removeValue(value, Arrays.copyOfRange(coordinates, i, i + k));
        }
    }

    public int size() {
        return size;
    }

    public Set<T> values() {
        Set<T> values = new HashSet<>();
        root.get(values);
        return values;
    }

    public Set<T> rangeQuery(double dist, double... coordinates) {
        Set<T> resultSet = new HashSet<>();
        for (int i = 0; i < coordinates.length / k; i++) {
            root.getInRange(dist, Arrays.copyOfRange(coordinates, i, i + k), resultSet);
        }
        return resultSet;
    }

    public Set<T> boxQuery(double dist, double... coordinates) {
        Set<T> resultSet = new HashSet<>();
        for (int i = 0; i < coordinates.length / (2*k); i++) {
            root.getInBox(dist, Arrays.copyOfRange(coordinates, i, i + 2*k), resultSet);
        }
        return resultSet;
    }

    public Set<T> boxQuery(Set<T> resultSet, double dist, double... coordinates) {
        for (int i = 0; i < coordinates.length / (2*k); i++) {
            root.getInBox(dist, Arrays.copyOfRange(coordinates, i, i + 2*k), resultSet);
        }
        return resultSet;
    }

    private class Node {
        Node c1, c2;
        int depth;
        double[] pos = null;
        T value;

        Node(int depth) {
            this.depth = depth;
        }

        void insert(T value, double... coordinates) {
            if (pos == null) {
                this.value = value;
                this.pos = coordinates;
                c1 = new Node(depth + 1);
                c2 = new Node(depth + 1);
            } else {
                int axis = depth % k;
                if (coordinates[axis] < pos[axis]) {
                    c1.insert(value, coordinates);
                } else {
                    c2.insert(value, coordinates);
                }
            }
        }

        void get(Set<T> resultSet) {
            if (value == null) return;
            resultSet.add(value);
            c1.get(resultSet);
            c2.get(resultSet);
        }

        void getInRange(double dist, double[] coordinate, Set<T> resultSet) {
            if (pos == null) return;

            int axis = depth % k;
            double min = coordinate[axis] - dist;
            double max = coordinate[axis] + dist;
            if (pos[axis] >= min) {
                c1.getInRange(dist, coordinate, resultSet);
            }
            if (pos[axis] <= max) {
                c2.getInRange(dist, coordinate, resultSet);
            }
            // more accurately measure distance
            if (value != null && distance.apply(coordinate, pos) <= dist) {
                resultSet.add(value);
            }
        }

        void getInBox(double dist, double[] coordinates, Set<T> resultSet) {
            if (pos == null) return;

            int axis = depth % k;
            double min = coordinates[axis] - dist;
            double max = coordinates[k + axis] + dist;
            if (pos[axis] >= min) {
                c1.getInBox(dist, coordinates, resultSet);
            }
            if (pos[axis] <= max) {
                c2.getInBox(dist, coordinates, resultSet);
            }

            if (value == null) return;
            for (int d = 0; d < k; d++) { // check if pos is inside the box
                if (pos[d] < coordinates[d] - dist || pos[d] > coordinates[k + d] + dist) return;
            }
            resultSet.add(value);
        }

        void removeValue(T value, double[] coordinates) {
            if (pos == null) return;

            if (value.equals(this.value)) {
                this.value = null;
            }

            int axis = depth % k;
            if (pos[axis] >= coordinates[axis]) {
                c1.removeValue(value, coordinates);
            }
            if (pos[axis] <= coordinates[axis]) {
                c2.removeValue(value, coordinates);
            }
        }
    }


}
