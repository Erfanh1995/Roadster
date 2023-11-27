package mapconstruction.algorithms.bundles.graph.representation;

import java.util.Map;

/**
 * Helper class representing a n edge in the labelled graph.
 */
public class LabelledEdge {

    private final Vertex source;
    private final Vertex target;
    private final int label;
    private final Vertex origin;

    public LabelledEdge(Vertex source, Vertex target, int label, Vertex origin) {
        this.source = source;
        this.target = target;
        this.label = label;
        this.origin = origin;
    }

    public LabelledEdge(Vertex source, Vertex target, int label) {
        this(source, target, label, null);
    }

    public LabelledEdge(int si, int sj, int ti, int tj, int label) {
        this(new Vertex(si, sj), new Vertex(ti, tj), label);
    }

    public Vertex getSource() {
        return source;
    }

    public Vertex getTarget() {
        return target;
    }

    public int getLabel() {
        return label;
    }

    public Vertex getOrigin() {
        return origin;
    }

    @Override
    public String toString() {
        return "LabelledEdge{" + "source=" + source + ", target=" + target + ", label=" + label + ", origin=" + origin + '}';
    }

}
