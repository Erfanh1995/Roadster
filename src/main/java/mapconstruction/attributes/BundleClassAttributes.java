/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.attributes;

import mapconstruction.algorithms.diagram.EvolutionDiagram;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Class containing different attributes that can be computed for bundle
 * classes.
 *
 * @author Roel
 */
public final class BundleClassAttributes {

    /**
     * Map mapping the name of the attribute to the function computing it.
     */
    private static final Map<String, BundleClassAttribute> ATTRIBUTES;

    static {
        ATTRIBUTES = new LinkedHashMap<>();

        // add all simple bundle attributes.
        for (BundleAttribute attr : BundleAttribute.values()) {
            ATTRIBUTES.put(attr.name(), new BundleClassAttribute(attr.name()) {
                @Override
                public double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon) {
                    return attr.applyAsDouble(diagram.getBundleUpToLevel(bundleClass, epsilon));
                }
            });
        }

        // birth moment of bundle class
        BundleClassAttribute birth = new BundleClassAttribute("Birth") {
            @Override
            public double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon) {
                return diagram == null ? Double.NaN : diagram.getBirthMoment(bundleClass);
            }
        };
        ATTRIBUTES.put(birth.name(), birth);

        // merge moment of bundle class
        BundleClassAttribute merge = new BundleClassAttribute("Merge") {
            @Override
            public double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon) {
                return diagram == null ? Double.NaN : diagram.getMergeMoment(bundleClass);
            }
        };
        ATTRIBUTES.put(merge.name(), merge);

        // Life span of bundle class
        BundleClassAttribute life = new BundleClassAttribute("LifeSpan") {
            @Override
            public double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon) {
                return diagram == null ? Double.NaN : diagram.getLifeSpan(bundleClass);
            }
        };
        ATTRIBUTES.put(life.name(), life);

        // relative lifespan of bundle class
        // This is the procentual increase of eps during its lifetime.
        BundleClassAttribute relative = new BundleClassAttribute("RelativeLifeSpan") {
            @Override
            public double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon) {
                return life.applyAsDouble(diagram, bundleClass, epsilon) / birth.applyAsDouble(diagram, bundleClass, epsilon);
            }
        };
        ATTRIBUTES.put(relative.name(), relative);

        // relative lifespan of bundle class
        // This is the procentual increase of eps during its lifetime.
        BundleClassAttribute besteps = new BundleClassAttribute("BestEps") {
            @Override
            public double applyAsDouble(EvolutionDiagram diagram, Integer bundleClass, Double epsilon) {
                return diagram.getBestEpsilon(bundleClass);
            }
        };
        ATTRIBUTES.put(besteps.name(), besteps);


    }


    /**
     * Gets the computer for the bundle class attributes with the given name.
     *
     * @param name
     * @return
     */
    public static BundleClassAttribute get(String name) {
        return ATTRIBUTES.get(name);
    }

    /**
     * Returns the list of attributes names.
     *
     * @return
     */
    public static List<String> names() {
        return new ArrayList<>(ATTRIBUTES.keySet());
    }
}
