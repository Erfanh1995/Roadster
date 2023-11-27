package mapconstruction.GUI.listeners;

import mapconstruction.GUI.filter.TriPredicate;
import mapconstruction.algorithms.diagram.EvolutionDiagram;

/**
 * Event with information about a chaning fileter
 *
 * @author Roel
 */
public class FilterChangeEvent {

    /**
     * Object causing the event.
     */
    private final Object source;

    private final TriPredicate<EvolutionDiagram, Integer, Double> predicate;

    public FilterChangeEvent(Object source, TriPredicate<EvolutionDiagram, Integer, Double> predicate) {
        this.source = source;
        this.predicate = predicate;
    }

    public Object getSource() {
        return source;
    }

    public TriPredicate<EvolutionDiagram, Integer, Double> getPredicate() {
        return predicate;
    }


}
