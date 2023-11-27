/**
 * This file is the layer between the table and the actual drawing.
 * In this file everything is related to: connections
 * These are the bundle parts between two intersections.
 *
 * @author Jorrick Sleijster
 * @since 10/12/2018
 */

/**
 * If a certain connection is selected, draw it with this function.
 * @param indices = index of the intersections in the original query.
 */
function drawTableSelectedConnections(indices) {
    if (enableTable){
        for (let index of indices){
            let connection = trajectoriesDataset['data']['network']['intersectionConnections'][index];

            let result = drawAConnection(connection);
            removeSpecificTableDrawnConnections(index);
            drawnTableConnections[index] = result;
        }
    }
}


/**
 * Removes all table-chosen drawn intersections
 */
function removeTableDrawnConnections() {
    for (let item in drawnTableConnections) {
        for (let drawnObject in drawnTableConnections[item]) {
            Layer.removeFeature(drawnTableConnections[item][drawnObject]);
        }
    }
    drawnTableConnections = {};
}

/**
 * Remove a specific table-chosen drawn intersection
 * @param index, the index in trajectoriesDataset of the intersection to remove
 */
function removeSpecificTableDrawnConnections(index) {
    if (typeof drawnTableConnections[index] !== 'undefined' && drawnTableConnections[index] !== '') {
        for (let drawnObject in drawnTableConnections[index]) {
            Layer.removeFeature(drawnTableConnections[index][drawnObject]);
        }
        drawnTableConnections[index] = [];
    }
}

/**
 * On hovering a specific trajectory from the table, it is drawn.
 * @param traceIndex, the index in trajectoriesDataset of the trajectory to draw
 */
function drawHoverConnection(traceIndex) {
    let step = 'network';
    let object = trajectoriesDataset['data'][step]['intersectionConnections'][traceIndex];
    removeHover();

    drawnHoverConnection = drawAConnection(object);
}

/**
 * Remove the drawn hover intersection
 */
function removeHoverConnection(){
    if (drawnHoverConnection !== null && drawnHoverConnection !== undefined) {
        for (let drawnObject in drawnHoverConnection) {
            Layer.removeFeature(drawnHoverConnection[drawnObject]);
        }
    }
    drawnHoverConnection = null;
}