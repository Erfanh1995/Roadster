package mapconstruction.algorithms.bundles.graph;

import java.util.SortedSet;

public interface EventGenerator<T> {

    SortedSet<T> collectEvents();

}
