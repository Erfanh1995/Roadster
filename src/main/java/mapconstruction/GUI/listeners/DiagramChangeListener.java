package mapconstruction.GUI.listeners;

/**
 * Interface for listeners to changes in the evolution diagram of the data
 * storage.
 *
 * @author Roel
 */
public interface DiagramChangeListener {

    /**
     * Signals listener that the diagram has changed.
     *
     * @param evt
     */
    void diagramChanged(DiagramChangeEvent evt);
}
