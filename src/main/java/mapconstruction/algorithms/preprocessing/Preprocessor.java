/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.algorithms.preprocessing;

import mapconstruction.algorithms.AbstractTrajectoryAlgorithm;
import mapconstruction.trajectories.Trajectory;

import java.util.List;

/**
 * Abstract class for preprocessors on trajectories.
 * <p>
 * Preprocessors take a collection of trajectories and
 * transform it into a new collection of trajectories by manipulating the
 * trajectories.
 *
 * @author Roel
 */
public abstract class Preprocessor extends AbstractTrajectoryAlgorithm<List<Trajectory>> {


}
