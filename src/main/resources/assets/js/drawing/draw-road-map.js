/**
 * This file is the layer between the table and the actual drawing.
 * In this file everything is related to: road-map. (The final road map, and thus drawing the roads.)
 *
 * @author Jorrick Sleijster
 * @since 23/10/2018
 */

/**
 * If a certain road is selected, draw it with this function
 * @param roadindices = index of roads in the original query.
 */
function drawTableSelectedRoads(roadindices) {
    if (enableTable) {
        for (let index of roadindices) {
            let road = trajectoriesDataset['data']['network']['roadMap']['roadSections'][index];

            let result = drawARoadSection(road,  StyleClass.ROAD_EDGE);

            removeSpecificTableDrawnRoad(index);
            drawnTableRoads[index] = result;
        }
    }
}

/**
 * Removes all table-chosen drawn roads
 */
function removeTableDrawnRoads() {
    for (let item in drawnTableRoads) {
        for (let subtrajectory in drawnTableRoads[item]) {
            let feature = drawnTableRoads[item][subtrajectory]
            Layer.removeFeature(feature);
            // if (feature.values_.hasOwnProperty("layer")) {
            //     feature.values_.layer.removeFeature(feature);
            // } else {
            //     console.log(feature);
            // }
        }
    }
    drawnTableRoads = [];
}

/**
 * Remove a specific table-chosen drawn road
 * @param index, the index in trajectoriesDataset of the road to remove
 */
function removeSpecificTableDrawnRoad(index) {
    if (typeof drawnTableRoads[index] !== 'undefined' && drawnTableRoads[index] !== '') {
        let roadSection = drawnTableRoads[index];
        for (let subtrajectory in roadSection) {
            let feature = drawnTableRoads[index][subtrajectory]
            Layer.removeFeature(feature);
        }
        drawnTableRoads[index] = [];
    }
}

/**
 * On hovering a specific road from the table, it is drawn.
 * @param traceIndex, the index in trajectoriesDataset of the road to draw
 */
function drawHoverRoad(traceIndex) {
    let road = trajectoriesDataset['data']['network']['roadMap']['roadSections'][traceIndex];
    removeHover();

    drawnHoverRoad = drawARoadSection(road, StyleClass.ROAD_EDGE);

    let markers = [road['startVertex']['location'], road['endVertex']['location']];
    let allDrawnMarkers = drawMarkers(markers, 5, StyleClass.MARKER('$red'), Layer.MARKER);
    allDrawnMarkers.forEach(function (element) {
        drawnHoverConnectionCVs[drawnHoverConnectionCVs.length] = element;
    });
}

/**
 * Remove the drawn hover road map
 */
function removeHoverRoad(){
    let source = Layer.getLayerSource(Layer.ROADMAP);
    if (drawnHoverRoad !== null) {
        for (let subtrajectory in drawnHoverRoad) {
            source.removeFeature(drawnHoverRoad[subtrajectory]);
        }
    }
    drawnHoverRoad = null;

    source = Layer.getLayerSource(Layer.MARKER);
    for (let index in drawnHoverConnectionCVs){
        source.removeFeature(drawnHoverConnectionCVs[index]);
    }
    drawnHoverConnectionCVs = [];
}