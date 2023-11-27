package mapconstruction.algorithms.bundles.graph.representation;

import java.util.Comparator;

public class Event implements Comparable<Event> {

    public Vertex s;
    public Vertex t;
    private boolean start;

    public Event(Vertex s, Vertex t, boolean start) {
        this.s = s;
        this.t = t;
        this.start = start;
    }

    public int getHeight() {
        return s.y();
    }

    public boolean isStart() {
        return start;
    }

    @Override
    public String toString() {
        return "Event (" + s + ", " + t + ") " + (start ? "start" : "end");
    }

    @Override
    public int compareTo(Event o) {
        if (o == null) return 1;

        return Comparator.comparingInt(Event::getPrimaryIndex)
                .thenComparing(Event::isStart)
//                .thenComparing(Comparator.comparingInt(Event::getSecondaryIndex).reversed())
                .thenComparingInt(Event::getSecondaryIndex)
                .thenComparingInt(Event::getEndHeight)
                .thenComparingInt(Event::getStartHeight)
                .compare(this, o);
    }

    public int getPrimaryIndex() {
        return this.start ? s.x() : t.x();
    }

    public int getSecondaryIndex() {
        return this.start ? t.x() : s.x();
    }

    public int getStartHeight() {
        return s.y();
    }

    public int getEndHeight() {
        return t.y();
    }
}
