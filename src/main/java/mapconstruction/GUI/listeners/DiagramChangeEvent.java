package mapconstruction.GUI.listeners;

import mapconstruction.algorithms.diagram.EvolutionDiagram;

/**
 * Envent containing information about changed diagrams.
 *
 * @author Roel
 */
public class DiagramChangeEvent {

    /**
     * Source of the event.
     */
    private final Object source;

    /**
     * Old evolution diagram
     */
    private final EvolutionDiagram oldDiagram;

    /**
     * New evolution diagram.
     */
    private final EvolutionDiagram newDiagram;

    public DiagramChangeEvent(Object source, EvolutionDiagram oldDiagram, EvolutionDiagram newDiagram) {
        this.source = source;
        this.oldDiagram = oldDiagram;
        this.newDiagram = newDiagram;
    }

    /**
     * @return the source of the event
     */
    public Object getSource() {
        return source;
    }

    /**
     * @return the olde diagram.
     */
    public EvolutionDiagram getOldDiagram() {
        return oldDiagram;
    }

    /**
     * @return the new diagram.
     */
    public EvolutionDiagram getNewDiagram() {
        return newDiagram;
    }


}
