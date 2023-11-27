/**
 * This file contains the main function for the google maps integration.
 * Most functions are created for the different type of object we have:
 * - Standard trajectories are the once that are right out of the dataset(or filtered).
 * - Table trajectories come from the table
 * - Hover trajectories come from hovering a specific bundle or trace in the table.
 * - Intersections are specific for drawing of intersections
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

// Contains the data loaded in from the back-end.
let trajectoriesDataset;
let groundTruthDataset;
// Which geographic system is used
let geographicSystem = '';
// The infoWindow that is popped up on google maps when we hover a trajectory.
let infoWindow;

/**
 * Contains a list of all specific traces or (traces of) bundles drawn.
 */
let standardDrawnTraces = [];
let drawnGroundTruths = [];
let drawnTableTrajectories = {};
let drawnTableBundles = {};
let drawnTableRoads = {};
let drawnTableIntersections = {};
let drawnTableConnections = {};
let drawnConnectionVertices = [];

let drawnHoverTrajectory = null;
let drawnHoverBundle = null;
let drawnHoverRoad = null;
let drawnHoverIntersection = null;
let drawnHoverConnection = null;
let drawnHoverConnectionCVs = [];


/**
 * Sets the trajectories dataset and updates all required things.
 * @param dataset
 */
function setTrajectoryDataset(dataset) {
    trajectoriesDataset = dataset;
    // Beginning with drawing the trajectories and setting the map on the right place.
    setGeographicSettings();
    centerMap();
    drawAllStandardTrajectories();

    // Preventing previous drawn traces from staying on the map forever.
    removeTableDraws();
    // Creating the menu tables.
    createTables();
    // Play done sound
    playDoneSound();
    // Hiding the loader
    hideLoader();
}

/**
 * Sets the ground truth dataset at the right location.
 * @param groundTruth
 */
function setGroundTruthDataset(groundTruth) {
    groundTruthDataset = groundTruth;
}

/**
 * This sets the constants for the dataset and shows the dataset folder.
 *
 * Without these constants we would be unable to relate this data to google maps.
 */
function setGeographicSettings() {
    setZoneAndSouthHemi(
        trajectoriesDataset['data']['settings']['zone'],
        (trajectoriesDataset['data']['settings']['hemisphere'] === "S"));
    geographicSystem = trajectoriesDataset['data']['settings']['system'];
    $('.dataset-folder').text(trajectoriesDataset['data']['settings']['path']);
    if (trajectoriesDataset['data']['settings']['walkingDataset']){
        $('.walking-data-enabled').show();
    } else {
        $('.walking-data-disabled').show();
    }
    $('.dataset-folder').text(trajectoriesDataset['data']['settings']['path']);
}

/**
 * Get's a specific dataset. Centers it to this dataset and draws all polylines.
 */
function centerMap() {
    focusOnTrajectories();
    // create infowindow
    let $tooltip = $('<span class="tooltip"> {0} </span>');
    $('document').append($tooltip);
    infoWindow = new ol.Overlay({
        element: $tooltip.get(0),
        // put tooltip right above cursor so it doesn't interfere
        positioning: 'bottom-center',
        offset: [0, -1],
    });

    map.addOverlay(infoWindow);
    map.on('pointermove', function(e) {
        if (e.dragging) return;

        let features = map.getFeaturesAtPixel(e.pixel);
        for (let feature of features) {
            if (feature.values_.tooltip) {
                infoWindow.setPosition(e.coordinate);
                infoWindow.getElement().innerHTML = htmlEntities(feature.values_.tooltip);
                infoWindow.getElement().style.opacity = "1";
                return;
            }
        }
        infoWindow.getElement().style.opacity = "0";
    });
    // new google.maps.InfoWindow();
}

function htmlEntities(str) {
    return String(str).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}

/**
 * Function that focuses on the trajectory.
 * @param trajectories list<Trajectory>, list of trajectories.
 */
function focusOnTrajectories() {
    let bounds = getTrajectoryBounds();
    map.getView().fit(
        bounds,
        map.getSize()
    );
}

/**
 * Function to get the bounds of our trajectories
 */
function getTrajectoryBounds() {
    let bounds = new ol.extent.createEmpty();
    for (let trajectory of trajectoriesDataset['data']['original']) {
        // let coordinates = trajectoriesDataset['data']['original'][index]['points'];
        let coordinates = trajectory.points.map(p => ol.proj.fromLonLat(convertUTMtoLongLat(p)));
        ol.extent.extend(bounds, ol.extent.boundingExtent(coordinates));
    }
    // let bounds = new google.maps.LatLngBounds();
    // for (let index in trajectoriesDataset['data']['original']) {
    //     let coordinates = trajectoriesDataset['data']['original'][index]['points'];
    //     for (let indexCoordinates in coordinates) {
    //         let UTMCoordinate = coordinates[indexCoordinates];
    //         bounds.extend(convertUTMtoLatLong(UTMCoordinate));
    //     }
    // }
    return bounds;
}

/**
 * Function that centers on the subtrajectories
 * @param subTrajectories list<SubTrajectory>, list of subtrajectory objects which we should focus on.
 */
function focusOnSubTrajectories(subTrajectories) {
    let bounds = new google.maps.LatLngBounds();
    for (let index in subTrajectories) {
        console.log(subTrajectories, index, subTrajectories[index]);
        let coordinates = subTrajectories[index]['parent']['points'];
        let beginPoint = subTrajectories[index]['fromIndex'];
        let endPoint = subTrajectories[index]['toIndex'];

        let mapsCoordinates = [];
        for (let indexOfCoordinate in coordinates) {
            // Allows for both increasing and decreasing trajectories. Required for when direction is ignored.
            if ((indexOfCoordinate >= beginPoint && indexOfCoordinate <= endPoint) ||
                (indexOfCoordinate <= beginPoint && indexOfCoordinate >= endPoint)) {
                let coordinate = coordinates[indexOfCoordinate];
                mapsCoordinates.push(convertUTMtoLatLong(coordinate));
            }
        }

        for (let indexCoordinates in mapsCoordinates) {
            bounds.extend(mapsCoordinates[indexCoordinates]);
        }
    }
    map.fitBounds(bounds);
}

/**
 * Function that centers on the points provided
 * @param points list<Point2D>, list of points which we should focus on.
 */
function focusOnPoints(points) {
    let bounds = new google.maps.LatLngBounds();
    for (let point of points) {
        bounds.extend(convertUTMtoLatLong(point));
    }
    map.fitBounds(bounds);
}

/**
 * Draw all the polylines of a specific step
 */
function drawAllStandardTrajectories() {
    step = 'filtered';
    if (!(step === 'original' || step === 'filtered')) {
        console.error('Had to draw GPS traces for step "' + step + '" which is not possible.')
    }

    if (standardDrawnTraces.length > 0) return;
    removeStandardDrawnTrajectories();

    for (let trajectory of trajectoriesDataset['data'][step]) {
        let identifier = trajectory['label'];
        if ("parent" in trajectory){
            standardDrawnTraces[identifier] =
                drawASubTrajectory(trajectory, propStandardTrajectory, trajectory['label']);
        } else {
            standardDrawnTraces[identifier] =
                drawATrajectory(trajectory['points'], identifier, StyleClass.TRAJECTORY, Layer.TRAJECTORY);
        }
    }
}

/**
 * Remove all drawn trajectories that are not part of the hovering or the table
 */
function removeStandardDrawnTrajectories() {
    let source = Layer.getLayerSource(Layer.TRAJECTORY);
    for (let item in standardDrawnTraces) {
        source.removeFeature(standardDrawnTraces[item]);
    }
    standardDrawnTraces = [];
}

/**
 * Draw the whole ground truth map
 */
function drawGroundTruthMap(hideInvalid = false) {
    removeDrawnGroundTruthMap();

    let edges = groundTruthDataset['data']['edges'];
    let valid = groundTruthDataset['data']['validEdges'];
    for (let edge of edges) {
        let edge_list = [edge['v1'], edge['v2']];
        let is_valid = !valid || valid.includes(edge['id']);

        if (!hideInvalid || is_valid) {
            let style = is_valid ? StyleClass.GROUND_TRUTH : StyleClass.GROUND_TRUTH_INVALID;
            drawnGroundTruths.push(drawATrajectory(edge_list, '', style, Layer.GROUND_TRUTH, false));
            // drawnGroundTruths[drawnGroundTruths.length] = ;
            // drawnGroundTruths[drawnGroundTruths.length] = drawATrajectory(edge_list, propGroundTruthBorder, false);
        }
    }
}

/**
 * Removes the drawn ground truth map
 */
function removeDrawnGroundTruthMap() {
    for (let item of drawnGroundTruths) {
        Layer.removeFeature(item);
    }
    drawnGroundTruths = [];
}

/**
 * Draw all Connection vertices
 */
function drawAllConnectionVertices(){
    if (trajectoriesDataset['data']['network']['roadMap'] == null){
        alert("Please compute the final RoadMap first. There is no RoadMap ATM.");
        return;
    }
    for (let index in trajectoriesDataset['data']['network']['roadMap']['connectionVertices']) {
        let locationPoint = trajectoriesDataset['data']['network']['roadMap']['connectionVertices'][index]['location'];
        let drawnMarkers = drawMarkers([locationPoint], 6, StyleClass.MARKER('$blue'), Layer.MARKER, false);
        drawnMarkers.forEach(function (element) {
            drawnConnectionVertices[drawnConnectionVertices.length] = element;
        });
    }
}

/**
 * Remove all Connection vertices
 */
function removeAllConnectionVertices(){
    for (let index in drawnConnectionVertices){
        Layer.removeFeature(drawnConnectionVertices[index]);
    }
    drawnConnectionVertices = [];
}


let simpleMap = {};

/**
 * Function for finding a bundle and it's properties by it's class
 * @param bundleClass, the class of the bundle we want to find
 * @returns null if not found or the property view of a bundle if found. (hence pv['Bundle'] is the actual bundle)
 */
function findBundlePropsByClass(bundleClass){
    if (Object.keys(simpleMap).length === 0){
        simpleMap = {};
        const allBundles = trajectoriesDataset['data']['bundles'];
        for (let bundleIndex in allBundles){
            let BundleProp = allBundles[bundleIndex];
            simpleMap[BundleProp['BundleClass']] = bundleIndex;
        }
    }

    if (simpleMap.hasOwnProperty(bundleClass)){
        return trajectoriesDataset['data']['bundles'][simpleMap[bundleClass]];
    } else {
        console.trace("Error! BundleClass " + bundleClass + " not found in simpleMap");
        console.log(simpleMap);
        return null;
    }
}

/**
 * Executes all table draws
 */
function executeTableDraws(){
    drawAllTrajectoriesFromTheTable();
    drawAllBundlesFromTheTable();
    drawAllIntersectionsFromTheTable();
    drawAllConnectionsFromTheTable();
    drawAllRoadsFromTheTable();
}

/**
 * Removes all table draws.
 */
function removeTableDraws(){
    removeTableDrawnTrajectories();
    removeTableDrawnBundles();
    removeTableDrawnIntersections();
    removeTableDrawnConnections();
    removeTableDrawnRoads();
}

/**
 * We remove the traces drawn because of the hovering action.
 */
function removeHover() {
    removeHoverTrajectory();
    removeHoverBundle();
    removeHoverIntersection();
    removeHoverConnection();
    removeHoverRoad();
}

/**
 * Creates all menu tables
 */
function createTables(){
    createTrajectoryTable('filtered');
    createBundleTable();
    createIntersectionTable();
    createConnectionTable();
    createRoadMapTable();

    initializeShiftSelectForAllCheckboxes();
}

/**
 * Gives a little beep when the loading is done
 *
 * In chrome this requires
 * chrome://flags/#autoplay-policy to be set to false
 */
function playDoneSound() {
    var audio = new Audio('sound/beep.mp3');
    audio.play();
}