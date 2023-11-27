package mapconstruction.algorithms.bundles;

import mapconstruction.algorithms.listeners.AlgorithmProgressListener;

/**
 * @author Roel
 */
public abstract class BundleAlgorithmDecorator extends BundleGenerationAlgorithm implements AlgorithmProgressListener {

    /**
     * Decorated bundle algorithm.
     */
    protected final BundleGenerationAlgorithm algorithm;

    public BundleAlgorithmDecorator(BundleGenerationAlgorithm algorithm) {
        super(algorithm.isIgnoreDirection());
        this.algorithm = algorithm;
        // Listen for the progress
        algorithm.addListener(this);
    }

    @Override
    public void progressChanged() {
        setProgress(algorithm.getProgress());
    }

    @Override
    public void abort() {
        super.abort();
        algorithm.abort();
    }

}
