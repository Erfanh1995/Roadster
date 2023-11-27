/**
 * This file is the layer between the table and the actual drawing.
 * In this file everything is related to: trajectories.
 *
 * @author Jorrick Sleijster
 * @since 23/10/2018
 */




/**
 * If a specific trace is selected, draw it with this function
 * @param traceindices = index of
 */
function drawTableSelectedTrajectories(traceindices) {
    if (enableTable) {
        for (let index of traceindices) {
            let trajectory = trajectoriesDataset['data']['filtered'][index];
            let result;
            if ('parent' in trajectory){
                result = drawASubTrajectory(trajectory, StyleClass.TRAJECTORY, trajectory['label']);
            } else {
                result = drawATrajectory(trajectory['points'], trajectory['label'], StyleClass.TRAJECTORY, Layer.TRAJECTORY, true);
            }

            removeSpecificTableDrawnTrajectory(index);
            drawnTableTrajectories[index] = result;
        }
    }
}


/**
 * Removes all table-chosen drawn trajectories
 */
function removeTableDrawnTrajectories() {
    for (let index in drawnTableTrajectories) {
        if (drawnTableTrajectories.hasOwnProperty(index) && drawnTableTrajectories[index] !== undefined) {
            Layer.getLayerSource(Layer.TRAJECTORY).removeFeature(drawnTableTrajectories[index]);
        }
    }
    drawnTableTrajectories = [];
}



/**
 * Remove a specific table-chosen drawn trajectory
 * @param index, the index in trajectoriesDataset of the trajectory to remove
 */
function removeSpecificTableDrawnTrajectory(index) {
    if (typeof drawnTableTrajectories[index] !== 'undefined' && drawnTableTrajectories[index] !== '') {
        vectorLayer.getSource().removeFeature(drawnTableTrajectories[index]);
    }
}


/**
 * On hovering a specific trajectory from the table, it is drawn.
 * @param traceIndex, the index in trajectoriesDataset of the trajectory to draw
 */
function drawHoverTrajectory(traceIndex) {
    let step = 'filtered';
    let trajectory = trajectoriesDataset['data'][step][traceIndex];
    removeHover();

    if ('parent' in trajectory){
        drawnHoverTrajectory = drawASubTrajectory(trajectory, propTableTrajectory, trajectory['label']);
    } else {
        drawnHoverTrajectory = drawATrajectory(trajectory['points'], trajectory['label'], StyleClass.TRAJECTORY, Layer.TRAJECTORY, true);
    }
}

/**
 * Remove the drawn hover trajectory
 */
function removeHoverTrajectory(){
    if (drawnHoverTrajectory !== null) {
        Layer.getLayerSource(Layer.TRAJECTORY).removeFeature(drawnHoverTrajectory);
    }
    drawnHoverTrajectory = null;
}

