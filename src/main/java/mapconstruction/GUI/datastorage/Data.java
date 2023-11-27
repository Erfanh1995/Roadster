package mapconstruction.GUI.datastorage;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;
import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class Data implements Serializable {
    /**
     *
     */
    private static final long serialVersionUID = -5082034454087406151L;
    /**
     * List of loaded trajectories.
     */
    public final List<Trajectory> trajectories;
    /**
     * All currently generated bundles
     */
    public BiMap<Bundle, Integer> allBundlesWithClasses;

    /**
     * All currently generated bundles
     */
    public BiMap<Bundle, Integer> allBundlesWithClassesUnfiltered;
    /**
     * All currently generated bundles
     */
    public BiMap<Bundle, Integer> displayedBundlesWithClasses;

    /**
     * Evolution diagram.
     */
    public EvolutionDiagram evolutionDiagram;


    public Data() {
        trajectories = new ArrayList<>();
        allBundlesWithClasses = HashBiMap.create();
        allBundlesWithClassesUnfiltered = HashBiMap.create();
        displayedBundlesWithClasses = HashBiMap.create();
    }

}
