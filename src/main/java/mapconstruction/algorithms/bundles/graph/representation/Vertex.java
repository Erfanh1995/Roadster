package mapconstruction.algorithms.bundles.graph.representation;

import java.util.Comparator;
import java.util.Objects;

public class Vertex implements Comparable<Vertex> {

    private final int x;
    private final int y;

    public Vertex(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public int x() {
        return x;
    }

    public int y() {
        return y;
    }

    public Vertex toSubtrajectoryStart() {
        // also known as (low,low) in Graph representation.
        return new Vertex(x / 2, y / 2);
    }

    public Vertex toSubtrajectoryEnd() {
        // also known as (low,high) in Graph representation.
        return new Vertex( x / 2, (y + 1) / 2);
    }

    @Override
    public String toString() {
        return "Vertex{" + "x=" + x + ", y=" + y + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vertex vertex = (Vertex) o;
        return x == vertex.x &&
                y == vertex.y;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public int compareTo(Vertex o) {
        if (this.x == o.x) {
            return Comparator.comparingInt(Vertex::y).compare(this, o);
        }
        return Comparator.comparingInt(Vertex::x).compare(this, o);
    }
}
