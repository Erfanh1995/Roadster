package mapconstruction.workers;


import mapconstruction.algorithms.AbortableAlgorithm;

import javax.swing.*;

public abstract class AbortableAlgorithmWorker<T, V> extends SwingWorker<T, V> {

    protected AbortableAlgorithm algo;

    public void abortAlgo() {
        algo.abort();
    }

}