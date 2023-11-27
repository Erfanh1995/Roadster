/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.algorithms.preprocessing;

import mapconstruction.trajectories.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Preprcessor that processes a list of preprocessors in sequence.
 *
 * @author Roel
 */
public class CompositePreprocessor extends Preprocessor {

    private final List<Preprocessor> preprocessors;

    public CompositePreprocessor() {
        preprocessors = new ArrayList<>();
    }


    @Override
    protected List<Trajectory> runAlgorithm(List<Trajectory> trajectories) {
        for (Preprocessor p : preprocessors) {
            checkAbort();
            trajectories = p.run(trajectories);
        }
        return trajectories;
    }

    public int numOfPreprocessors() {
        return preprocessors.size();
    }

    public boolean add(Preprocessor e) {
        return preprocessors.add(e);
    }

    public boolean remove(Preprocessor o) {
        return preprocessors.remove(o);
    }

    public Preprocessor set(int index, Preprocessor element) {
        return preprocessors.set(index, element);
    }

    public void add(int index, Preprocessor element) {
        preprocessors.add(index, element);
    }

    public Preprocessor remove(int index) {
        return preprocessors.remove(index);
    }

    @Override
    public void abort() {
        super.abort(); //To change body of generated methods, choose Tools | Templates.
        preprocessors.forEach(Preprocessor::abort);
    }


}
