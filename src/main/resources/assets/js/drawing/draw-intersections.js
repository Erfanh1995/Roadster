/**
 * This file is the layer between the table and the actual drawing.
 * In this file everything is related to: intersections.
 *
 * @author Jorrick Sleijster
 * @since 23/10/2018
 */

/**
 * If a certain intersection is selected, draw it with this function
 * @param indices = index of the intersections in the original query.
 */
function drawTableSelectedIntersections(indices) {
    let step = 'network';
    if (enableTable) {
        for (let index of indices) {
            let intersection = trajectoriesDataset['data'][step]['intersections'][index];

            let result = drawAnIntersection(intersection, index);
            removeSpecificTableDrawnIntersections(index);
            drawnTableIntersections[index] = result;
        }
    }
}

/**
 * Removes all table-chosen drawn intersections
 */
function removeTableDrawnIntersections() {
    for (let item in drawnTableIntersections) {
        for (let drawnObject in drawnTableIntersections[item]) {
            Layer.removeFeature(drawnTableIntersections[item][drawnObject]);
        }
    }
    drawnTableIntersections = {};
}

/**
 * Remove a specific table-chosen drawn intersection
 * @param index, the index in trajectoriesDataset of the intersection to remove
 */
function removeSpecificTableDrawnIntersections(index) {
    if (typeof drawnTableIntersections[index] !== 'undefined' && drawnTableIntersections[index] !== '') {
        for (let drawnObject in drawnTableIntersections[index]) {
            Layer.removeFeature(drawnTableIntersections[index][drawnObject])
        }
    }
}

/**
 * On hovering a specific trajectory from the table, it is drawn.
 * @param traceIndex, the index in trajectoriesDataset of the trajectory to draw
 */
function drawHoverIntersection(traceIndex) {
    let step = 'network';
    let object = trajectoriesDataset['data'][step]['intersections'][traceIndex];
    removeHover();

    drawnHoverIntersection = drawAnIntersection(object, traceIndex);
}

/**
 * Remove the drawn hover intersection
 */
function removeHoverIntersection(){
    if (drawnHoverIntersection !== null && drawnHoverIntersection !== undefined) {
        for (let drawnObject in drawnHoverIntersection) {
            Layer.removeFeature(drawnHoverIntersection[drawnObject]);
        }
    }
    drawnHoverIntersection = null;
}