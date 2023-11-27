package mapconstruction.attributes;

import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.util.ToDoubleTriFunction;

/**
 * Function computing the atrribute value of a bundle class
 *
 * @author Roel
 */
public abstract class BundleClassAttribute implements ToDoubleTriFunction<EvolutionDiagram, Integer, Double> {

    private final String name;

    public BundleClassAttribute(String name) {
        this.name = name;
    }


    /**
     * Computes the attribute value for the given bundle class at the given epsilon in the given evolution diagram.
     *
     * @param diagram
     * @param bundleClass
     * @param epsilon
     * @return
     */
    @Override
    public abstract double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon);


    public String name() {
        return name;
    }


}
