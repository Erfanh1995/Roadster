/**
 * This file is responsible for the actual drawing of:
 * - The Markers
 * - The Trajectories
 * - The Subtrajectories
 * - The Bundles
 *
 * @author Jorrick Sleijster
 * @since 23/10/2018
 */


// import {InfoWindow as infoWindow} from "googlemaps";

/**
 * This functions draws marker
 * @param locations, the locations the markers should be
 * @param size, the size of the markers
 * @param styleClass
 * @param layer
 * @param showInfoWindow, whether to show an info windows or not
 * @param identifier, the text to show in the infowindow
 * @returns {Array} with drawn marker instances.
 */
function drawMarkers(locations, size = 6, styleClass=StyleClass.MARKER(), layer = Layer.MARKER, showInfoWindow = true, identifier = '') {
    let features = [];
    let source = Layer.getLayerSource(Layer.MARKER);
    for (let location of locations) {
        let geom = new ol.geom.Circle(ol.proj.fromLonLat(convertUTMtoLongLat(location)), size);
        let options = {
            name: 'Marker ' + identifier,
            geometry: geom,
            styleclass: styleClass,
            layer: layer,
            size: size,
        };
        if (showInfoWindow) options.tooltip = identifier;
        let feature = new ol.Feature(options);

        source.addFeature(feature);
        features.push(feature);
    }
    return features;

    // let colors = ['green', 'yellow', 'purple', 'blue', 'red'];
    // if (colors.indexOf(color) < 0) {
    //     console.log("Error maps-draw.drawAMarker. Color ", color, " not supported");
    //     return [];
    // } else {
    //     let allMarkers = [];
    //
    //     let image = {
    //         url: 'img/dot-' + color + ".png",
    //         size: new google.maps.Size(size, size),
    //         // The origin for this image is (0, 0).
    //         origin: new google.maps.Point(0, 0),
    //         // The anchor for this image is the base of our point.
    //         // At which point we attach the image to the location.
    //         anchor: new google.maps.Point(size / 2, size / 2),
    //         // Changing the scale of the image
    //         scaledSize: new google.maps.Size(size, size)
    //     };
    //     for (let i = 0; i < locations.length; i++) {
    //         let location = locations[i];
    //         let marker = new google.maps.Marker({
    //             position: convertUTMtoLatLong(location),
    //             map: map,
    //             label: {
    //                 text: identifier,
    //                 fontSize: '15px',
    //                 fontWeight: '900',
    //                 color: 'red',
    //             },
    //             icon: image,
    //             zIndex: zIndex
    //         });
    //
    //         if (showInfoWindow) {
    //             google.maps.event.addListener(marker, 'mouseover', function (e) {
    //                 infoWindow.setPosition(e.latLng);
    //                 let xy = convertLatLongtoUTM(e.latLng);
    //                 infoWindow.setContent("You are at " + e.latLng + "<br>" + "[" + xy[0] + ", " + xy[1] + "]<br>" + identifier);
    //                 infoWindow.open(map);
    //             });
    //
    //             // Close the InfoWindow on mouseout:
    //             google.maps.event.addListener(marker, 'mouseout', function () {
    //                 infoWindow.close();
    //             });
    //         }
    //
    //         allMarkers.push(marker);
    //     }
    //     return allMarkers;
    // }
}


/**
 * Draws a trajectory given by it's coordinates
 * @param coordinates Array[coordinate]. Coordinate list.
 * @param identifier str. Identifier for the subtrajectory.
 * @param styleClass The class used for drawing the trajectory
 * @param layer The layer in which to draw the trajectory
 * @param showInfoWindow. Whether we should get info of the location if we hover over it.
 * @param movable bool. Whether the trajectory should add movable trajectories.
 * @returns Polyline. The drawn object.
 */
function drawATrajectory(coordinates, identifier = '', styleClass = StyleClass.DEFAULT, layer = Layer.DEFAULT, showInfoWindow = true, movable = false) {
    let mapsCoordinates = coordinates.map(c => ol.proj.fromLonLat(convertUTMtoLongLat(c)));

    let polyline = new ol.geom.LineString(mapsCoordinates);
    let options = {
        name: 'Trajectory ' + identifier,
        geometry: polyline,
        styleclass: styleClass,
        layer: layer,
    };
    if (showInfoWindow) options.tooltip = identifier;
    let feature = new ol.Feature(options);

    Layer.getLayerSource(layer).addFeature(feature);

    // let polyline = new google.maps.Polyline({
    //     path: mapsCoordinates,
    //     editable: movable,
    //     geodesic: true,
    //     strokeColor: properties.color,
    //     strokeOpacity: 1.0,
    //     strokeWeight: properties.width,
    //     zIndex: properties.zIndex
    // });
    //
    // polyline.setMap(map);
    //
    // if (showInfoWindow) {
    //     google.maps.event.addListener(polyline, 'mouseover', function (e) {
    //         infoWindow.setPosition(e.latLng);
    //         let xy = convertLatLongtoUTM(e.latLng);
    //         infoWindow.setContent("You are at " + e.latLng + "<br>" + "[" + xy[0] + ", " + xy[1] + "]<br>" + identifier);
    //         infoWindow.open(map);
    //     });
    //
    //     // Close the InfoWindow on mouseout:
    //     google.maps.event.addListener(polyline, 'mouseout', function () {
    //         infoWindow.close();
    //     });
    // }

    return feature;
}

/**
 * For a specific subtrajectory object it draws the trajectory within the subtrajectory ranges
 * @param subTrajectory object. Contains all the properties of a subTrajectory.
 * @param styleClass A dictionary containing the color, zIndex and width property.
 * @param layer
 * @param identifier str. Identifier for the subtrajectory.
 * @returns Polyline. The drawn object.
 */
function drawASubTrajectory(subTrajectory, styleClass = StyleClass.DEFAULT, layer = Layer.SUBTRAJECTORY, identifier = '') {
    let xyCoordinates = getPointListOfSubtrajectory(subTrajectory);
    return drawATrajectory(xyCoordinates, identifier, styleClass, layer, identifier.length);
}

/**
 * Get points from subTrajectory. If subtrajectory has a parent and the parent has a parent, we recursively get the
 * pointList.
 * @param subTrajectory object. Contains all the properties of a subTrajectory.
 * @return the points covered by this subtrajectory.
 */
function getPointListOfSubtrajectory(subTrajectory) {
    let parent = subTrajectory['parent'];
    let beginPoint = subTrajectory['fromIndex'];
    let endPoint = subTrajectory['toIndex'];

    let coordinates;
    if ("parent" in parent){
        coordinates = getPointListOfSubtrajectory(parent);
    }  else {
        coordinates = parent['points'];
    }

    let xyCoordinates = [];
    for (let indexOfCoordinate in coordinates) {
        if (parseInt(indexOfCoordinate) === Math.floor(beginPoint)) {
            xyCoordinates.push(subTrajectory['firstPoint']);
        } else if (parseInt(indexOfCoordinate) === Math.ceil(endPoint)) {
            xyCoordinates.push(subTrajectory['lastPoint']);
        } else if ((indexOfCoordinate >= beginPoint && indexOfCoordinate <= endPoint) ||
            (indexOfCoordinate <= beginPoint && indexOfCoordinate >= endPoint)) {
            let coordinate = coordinates[indexOfCoordinate];
            xyCoordinates.push(coordinate);
        }
    }
    return xyCoordinates;
}

/**
 * For a trajectory label_id and start and end point, it draws the trajectory within the subtrajectory ranges.
 * @param trajectoryLabel, the label of the Trajectory.
 * @param beginIndex, starting index where we start drawing for on the Trajectory.
 * @param endIndex, ending index where we stop drawing for on the Trajectory.
 * @param properties A dictionary containing the color, zIndex and width property.
 * @param identifier str. Identifier for the subtrajectory.
 * @returns Polyline. The drawn object.
 */
function drawASubtrajectoryByTrajectoryLabel(trajectoryLabel, beginIndex, endIndex, properties, identifier = '') {
    let parent = null;
    for (let trajectory of trajectoriesDataset['data']['filtered']) {
        if (trajectory['label'] === trajectoryLabel) {
            parent = trajectory;
            break;
        }
    }
    if (parent == null) {
        console.trace("Error. TrajectoryLabel could not be found in trajectoriesDataset");
    }

    let coordinates = parent['points'];
    return drawSpecialSubTrajectory(coordinates, beginIndex, endIndex, properties, identifier);
}

/**
 * For a bundleClass and start and end point, it draws the bundles representative within the subtrajectory ranges.
 * @param bundleClass, the class of the bundle.
 * @param beginIndex, starting index where we start drawing for on the Trajectory.
 * @param endIndex, ending index where we stop drawing for on the Trajectory.
 * @param styleClass A dictionary containing the color, zIndex and width property.
 * @param identifier str. Identifier for the subtrajectory.
 * @returns Polyline. The drawn object.
 */
function drawASubtrajectoryByBundleClass(bundleClass, beginIndex, endIndex, styleClass, identifier = '') {
    let bundleProps = findBundlePropsByClass(bundleClass);
    if (bundleProps == null) {
        console.trace("Error. TrajectoryLabel could not be found in trajectoriesDataset");
    }

    let bundle = bundleProps['Bundle'];
    let representative = bundle['representative'];
    let coordinates = representative['points'];

    if (endIndex === -1){
        endIndex = representative['points'].length - 1;
    }

    return drawSpecialSubTrajectory(coordinates, beginIndex, endIndex, properties, identifier);
}

/**
 *
 * Given a bunch of coordinates, a start index and an ending index,
 * it draws the bundles representative within the subtrajectory ranges.
 * @param coordinates, the coordinates.
 * @param beginIndex, starting index where we start drawing for on the Trajectory.
 * @param endIndex, ending index where we stop drawing for on the Trajectory.
 * @param properties A dictionary containing the color, zIndex and width property.
 * @param identifier str. Identifier for the subtrajectory.
 * @returns Polyline. The drawn object.
 */
function drawSpecialSubTrajectory(coordinates, beginIndex, endIndex, properties, identifier = '') {
    let xyCoordinates = [];
    for (let indexOfCoordinate in coordinates) {
        if (parseInt(indexOfCoordinate) === Math.floor(beginIndex)) {
            const point = getPointOnLine(coordinates[Math.floor(beginIndex)], coordinates[Math.ceil(beginIndex)], beginIndex % 1.0);
            xyCoordinates.push(point);
        } else if (parseInt(indexOfCoordinate) === Math.ceil(endIndex)) {
            const point = getPointOnLine(coordinates[Math.floor(endIndex)], coordinates[Math.ceil(endIndex)], endIndex % 1.0);
            xyCoordinates.push(point);
        } else if ((indexOfCoordinate >= beginIndex && indexOfCoordinate <= endIndex) ||
            (indexOfCoordinate <= beginIndex && indexOfCoordinate >= endIndex)) {
            let coordinate = coordinates[indexOfCoordinate];
            xyCoordinates.push(coordinate);
        }
    }
    return drawATrajectory(xyCoordinates, properties, true, identifier);
}


let drawFinalAsForce = false;
/**
 * This function is called to draw a bundle.
 * @param bundle Bundle, the bundle object.
 * @param bundleClass The class of this bundle.
 * @returns Array[Feature]. Contains the drawn features.
 */
function drawABundle(bundle, bundleClass = null) {
    let drawn_objects = [];

    if (settingsBundleDrawing['drawBundleSubtrajectories']['value']) {
        for (let index in bundle['subtrajectories']) {
            let subTrajectory = bundle['subtrajectories'][index];
            drawn_objects[drawn_objects.length] = drawASubTrajectory(subTrajectory, StyleClass.SUBTRAJECTORY, Layer.BUNDLE);
        }
    }

    if (settingsBundleDrawing['drawOriginalRepresentative']['value']) {
        drawn_objects[drawn_objects.length] = drawASubTrajectory(bundle['originalRepresentative'], StyleClass.REPRESENTATIVE, Layer.BUNDLE);
    }

    if (settingsBundleDrawing['drawForceRepresentative']['value']) {
        if (bundle.hasOwnProperty('forcesRepresentativeJSON')) {
            drawn_objects[drawn_objects.length] = drawATrajectory(bundle['forcesRepresentativeJSON'], StyleClass.FORCE_PATH, Layer.BUNDLE);
        }
    }

    if (settingsBundleDrawing['drawFinalRepresentative']['value']) {
        if (bundle.hasOwnProperty('mergedRepresentativeJSON')) {
            let style = StyleClass.BUNDLE(bundle.representative.parentBundleClass); // ._color
            if (drawFinalAsForce){
                style = StyleClass.FORCE_PATH;
            }
            drawn_objects[drawn_objects.length] = drawATrajectory(bundle['mergedRepresentativeJSON'], "Bundle " + bundleClass, style, Layer.BUNDLE);
        }
    }

    if (settingsBundleDrawing['drawForcePoints']['value']) {
        if (bundle.hasOwnProperty('forceStepsJSON')) {
            for (let array_of_points of bundle['forceStepsJSON']) {
                let allDrawnMarkers = drawMarkers(array_of_points, 3, StyleClass.MARKER('$yellow'), Layer.BUNDLE);
                allDrawnMarkers.forEach(function (element) {
                    drawn_objects[drawn_objects.length] = element;
                });
            }
        }
    }

    if (settingsBundleDrawing['drawForceOrthogonals']['value']) {
        if (bundle.hasOwnProperty('forcePerpendicularLinesJSON')) {
            let points = bundle['forcePerpendicularLinesJSON'];
            for (let iString in bundle['forcePerpendicularLinesJSON']) {
                let i = parseInt(iString);
                if (i % 2 === 1) {
                    drawn_objects[drawn_objects.length] = drawATrajectory([points[i - 1], points[i]], '', StyleClass.FORCE_DIAGONAL, Layer.BUNDLE, false);
                }
            }
        }
    }

    if (settingsBundleDrawing['drawForceAcLines']['value']) {
        if (bundle.hasOwnProperty('forceACLinesJSON')) {
            let points = bundle['forceACLinesJSON'];
            for (let iString in bundle['forceACLinesJSON']) {
                let i = parseInt(iString);
                if (i % 2 === 1) {
                    drawn_objects[drawn_objects.length] = drawATrajectory([points[i - 1], points[i]], '', StyleClass.FORCE_AC, Layer.BUNDLE, false);
                }
            }
        }
    }


    if (settingsBundleDrawing['drawForceLinearRegression']['value']) {
        if (bundle.hasOwnProperty('forceRepresentativeRegressionLineJSON')) {
            let points = bundle['forceRepresentativeRegressionLineJSON']['Line'];
            drawn_objects[drawn_objects.length] = drawATrajectory(points, '', StyleClass.FORCE_REGRESSION, Layer.BUNDLE, false);
        }
    }

    if (settingsBundleDrawing['drawSinglePointTurns']['value']) {
        if (bundle.hasOwnProperty('singlePointSharpTurnsJSON') && bundle['singlePointSharpTurnsJSON'] != null) {
            let allDrawnMarkers = drawMarkers(bundle['singlePointSharpTurnsJSON'], 4, StyleClass.TURN_POINT, Layer.BUNDLE, false);
            allDrawnMarkers.forEach(function (element) {
                drawn_objects[drawn_objects.length] = element;
            });
        }
    }

    if (settingsBundleDrawing['drawMultiEdgesTurns']['value']) {
        if (bundle.hasOwnProperty('multiEdgesSharpTurnsJSON') && bundle['multiEdgesSharpTurnsJSON'] != null) {
            for (let Subtrajectory of bundle['multiEdgesSharpTurnsJSON']) {
                for (let turn of Subtrajectory) {
                    let points = turn['points'];
                    drawn_objects[drawn_objects.length] = drawATrajectory(points, '', StyleClass.TURNS, Layer.BUNDLE, false);

                    let allDrawnMarkers = drawMarkers(points, 4, StyleClass.TURN_POINT, Layer.BUNDLE, false);
                    allDrawnMarkers.forEach(function (element) {
                        drawn_objects[drawn_objects.length] = element;
                    });
                }
            }
        }
    }

    if (bundle.hasOwnProperty('allTurnsJSON') && bundle['allTurnsJSON'] != null) {
        console.warn("Bundle All Turns: not yet implemented");

        for (let turn of bundle['allTurnsJSON']) {
            break;

            let color = '#' + Math.floor(Math.random() * 16777215).toString(16) + '75';
            let props = {
                'color': color,
                'zIndex': 20,
                'width': 8
            };

            if (turn.hasOwnProperty('subtrajectoryTurnParts') && settingsBundleDrawing['drawTurns']['value']) {
                for (let points of turn['subtrajectoryTurnParts']) {
                    drawn_objects[drawn_objects.length] = drawATrajectory(points, props);
                }

                let allDrawnMarkers = drawMarkers([turn['averagePoint']], 'red', 15, 10, true);
                allDrawnMarkers.forEach(function (element) {
                    drawn_objects[drawn_objects.length] = element;
                });
            }

            if (turn.hasOwnProperty('subBeforeTurnParts') && settingsBundleDrawing['drawBeforeTurn']['value']) {
                for (let pointsOfSub of turn['subBeforeTurnParts']) {
                    drawn_objects[drawn_objects.length] = drawATrajectory(pointsOfSub, props);
                }
            }
            if (turn.hasOwnProperty('subAfterTurnParts') && settingsBundleDrawing['drawAfterTurn']['value']) {
                for (let pointsOfSub of turn['subAfterTurnParts']) {
                    drawn_objects[drawn_objects.length] = drawATrajectory(pointsOfSub, props);
                }
            }

            if (turn.hasOwnProperty('linearRegressionLines') && settingsBundleDrawing['drawLinearRegression']['value']) {
                let points = turn['linearRegressionLines'];
                props.color = String(props.color).slice(0, -2);
                drawn_objects[drawn_objects.length] = drawATrajectory(points.slice(0, 2), props);
                drawn_objects[drawn_objects.length] = drawATrajectory(points.slice(2, 4), props);
                let allDrawnMarkers = drawMarkers([points[0], points[2]], 'yellow', 15, 30, true);
                allDrawnMarkers.forEach(function (element) {
                    drawn_objects[drawn_objects.length] = element;
                });
            }
        }
    }

    if (settingsMapConstructionDrawing['drawRoadPointsOnBundle']['value']) {
        if (bundle.hasOwnProperty('bundleEndsAreRoadPointsJSON') && bundle['bundleEndsAreRoadPointsJSON'] != null) {
            let numberOfEnds = bundle['bundleEndsAreRoadPointsJSON'];
            let points = [];
            let rep = bundle['mergedRepresentativeJSON'];
            if (numberOfEnds === 1 || numberOfEnds === 3) {
                points.push(rep[0]);
            }
            if (numberOfEnds === 2 || numberOfEnds === 3) {
                points.push(rep[rep.length - 1]);
            }

            let allDrawnMarkers = drawMarkers(points, 3, StyleClass.ROAD_POINT, Layer.BUNDLE, false);
            allDrawnMarkers.forEach(function (element) {
                drawn_objects[drawn_objects.length] = element;
            });
        }
    }

    return drawn_objects;
}


let disableIntersectionText = true;
/**
 * Function to draw an intersection
 */
function drawAnIntersection(intersection, indexOfIntersection) {
    let drawn_objects = [];

    if (settingsMapConstructionDrawing['drawIntersectionLocation']['value']) {
        if (intersection['location'] == null) {
            console.log('Error! Location of the drawIntersectionLocation is null');
        } else {
            let textLabel = indexOfIntersection;
            if (disableIntersectionText){
                textLabel = ' ';
            }
            let drawnMarkers = drawMarkers([intersection['location']], 5, StyleClass.MARKER('$blue'), Layer.MARKER, true, textLabel);
            drawnMarkers.forEach(function (element) {
                drawn_objects[drawn_objects.length] = element;
            });
        }
    }

    if (settingsMapConstructionDrawing['drawIntersectionApproximateLocation']['value']) {
        let drawnMarkers = drawMarkers([intersection['approximateLocation']], 5, StyleClass.MARKER('$green'), Layer.MARKER);
        drawnMarkers.forEach(function (element) {
            drawn_objects[drawn_objects.length] = element;
        });
    }

    let allIntersectionClusters = intersection['allIntersectionClusters'];
    allIntersectionClusters.forEach(function (intersectionCluster) {
        let allDrawnIntersectionClusterObjects = drawAnIntersectionCluster(intersectionCluster);
        allDrawnIntersectionClusterObjects.forEach(function (element) {
            drawn_objects[drawn_objects.length] = element;
        });
    });

    let allIntersectionPoints = intersection['allIntersectionPoints'];
    allIntersectionPoints.forEach(function (intersectionPoint) {
        let allDrawnIntersectionPointObjects = drawAsIntersectionPoint(intersectionPoint);
        allDrawnIntersectionPointObjects.forEach(function (element) {
            drawn_objects[drawn_objects.length] = element;
        });
    });
    // if (settingsMapConstructionDrawing['drawLinearRegressionUsedParts']['value']) {
    //     intersection['bundleRepsPartsBeforeAfterIntersection'].forEach(function (subTrajectory, index) {
    //         drawn_objects[drawn_objects.length] = drawASubTrajectory(subTrajectory, propTableTrajectory, "");
    //     });
    // }

    if (settingsMapConstructionDrawing['drawIntersectionStraightestBundle']['value']
        && intersection.hasOwnProperty('straightestBundleClass') && intersection['straightestBundleClass'] !== -1) {
        drawMergedRep(drawn_objects, intersection['straightestBundleClass'], StyleClass.INT_END_BUNDLE);
    }

    if (settingsMapConstructionDrawing['drawIntersectionBundleParts']['value'] &&
        intersection.hasOwnProperty('bundlePartsBundledJSON')) {
        intersection['bundlePartsBundledJSON'].forEach(function (intersectionPart) {
            // TODO take a consistent key instead of random()
            let style = StyleClass.INTERSECTION_PART(Math.random());

            intersectionPart.forEach(function (subtrajectory) {
                drawn_objects[drawn_objects.length] = drawASubTrajectory(subtrajectory, style, Layer.INTERSECTION);
            });
        });
    }

    if (settingsMapConstructionDrawing['drawIntersectionShapeFittingLines']['value'] &&
        intersection.hasOwnProperty('shapeFittingLinesJSON')) {
        intersection['shapeFittingLinesJSON'].forEach(function (fittingLine) {

            drawn_objects[drawn_objects.length] = drawATrajectory(fittingLine, '', StyleClass.SHAPE_FITTING_LINES, Layer.INTERSECTION);
        });
    }

    if (settingsMapConstructionDrawing['drawAllBundlesConnectedToIntersection']['value'] &&
        intersection.hasOwnProperty('bundlesClassesConnectedToIntersection')) {
        intersection['bundlesClassesConnectedToIntersection'].forEach(function (bundleClass) {
            drawMergedRep(drawn_objects, bundleClass, StyleClass.INTERSECTION_BUNDLE(bundleClass));
        });
    }

    return drawn_objects;
}

/**
 * Function to draw an intersectionCluster
 */
function drawAnIntersectionCluster(intersectionCluster) {
    let drawn_objects = [];

    if (settingsMapConstructionDrawing['drawIntersectionClusters']['value']) {
        let allDrawnMarkers = drawMarkers([intersectionCluster['location']], 3, StyleClass.MARKER('$green'), Layer.MARKER);
        allDrawnMarkers.forEach(function (element) {
            drawn_objects[drawn_objects.length] = element;
        });
    }

    return drawn_objects;
}

/**
 * Function to draw an intersectionPoint
 */
function drawAsIntersectionPoint(intersectionPoint) {
    let drawn_objects = [];

    if (settingsMapConstructionDrawing['drawIntersectionPoints']['value']) {
        let identifierOfIP = 'turn';
        if (intersectionPoint['endBundleClass'] != null) {
            identifierOfIP = 'endBundle = ' + intersectionPoint['endBundleClass']
        }
        if (disableIntersectionText){
            identifierOfIP = ' ';
        }
        let allDrawnMarkers = drawMarkers([intersectionPoint['location']], 3, StyleClass.MARKER('$green'), Layer.MARKER, true, identifierOfIP);
        allDrawnMarkers.forEach(function (element) {
            drawn_objects[drawn_objects.length] = element;
        });
    }

    if (settingsMapConstructionDrawing['drawIntersectionPointEndingBundleRep']['value']
        && intersectionPoint.hasOwnProperty('endBundleClass')) {
        drawMergedRep(drawn_objects, intersectionPoint['endBundleClass'], StyleClass.INT_END_BUNDLE);
    }

    if (settingsMapConstructionDrawing['drawIntersectionPointBundlePairRep']['value']) {
        drawMergedRep(drawn_objects, intersectionPoint['longBundle1Class'], StyleClass.INT_0TH_BUNDLE_1);
        drawMergedRep(drawn_objects, intersectionPoint['longBundle2Class'], StyleClass.INT_0TH_BUNDLE_2);
    }

    if (settingsMapConstructionDrawing['drawIntersectionPointOverlappingBundleRep']['value']) {
        drawMergedRep(drawn_objects, intersectionPoint['overlappingBundleClass'], StyleClass.INT_0TH_BUNDLE_3);
    }

    // if (settingsMapConstructionDrawing['drawPointsWithOffsetFromRoadPoint']['value']) {
    //     let allDrawnMarkers = drawMarkers(intersectionPoint['pointsPastLocation'], 'purple', 12, 20, true,
    //         'endBundle = ' + intersectionPoint['endBundleClass']);
    //     allDrawnMarkers.forEach(function (element) {
    //         drawn_objects[drawn_objects.length] = element;
    //     });
    // }
    return drawn_objects;
}

/**
 * Function to draw a merged representative
 * @param bundleClass, the class of the bundle
 * @param propItem, which properties we should apply to it
 */
function drawMergedRep(drawn_objects, bundleClass, propItem) {
    const bundleProps = findBundlePropsByClass(bundleClass);
    const bundle = bundleProps['Bundle'];
    if (bundle != null && bundle.hasOwnProperty('mergedRepresentativeJSON')) {
        drawn_objects[drawn_objects.length] = drawATrajectory(bundle['mergedRepresentativeJSON'], '', propItem, Layer.MERGED_REP);
    }
}


/**
 * Function to draw merged turns
 */
let drawn_merged_turns = [];

function drawMergedTurns(startI, endI) {
    for (let index in drawn_merged_turns) {
        drawn_merged_turns[index].setMap(null);
    }

    let turnClusters = trajectoriesDataset['data']['network']['turnClusters'];
    for (let index in turnClusters) {
        if (index < startI || index > endI) {
            continue;
        }
        let turnCluster = turnClusters[index];
        for (let indexTurn in turnCluster) {
            let turn = turnCluster[indexTurn];

            let allDrawnMarkers = drawMarkers([turn['turnLocation']], 'purple', 12, 20, true,
                'turnIndex = ' + 0);
            allDrawnMarkers.forEach(function (element) {
                drawn_merged_turns[drawn_merged_turns.length] = element;
            });

            let allDrawnBundles = drawABundle(turn['bundle'], propRepresentativeBundle, propStandardTrajectory);
            allDrawnBundles.forEach(function (element) {
                drawn_merged_turns[drawn_merged_turns.length] = element;
            });
        }
    }
}

/**
 * Draw a single merged turn
 * @param index, index of the merged turn
 */
function drawAMergedTurn(index) {
    drawMergedTurns(index, index);
}

/**
 * Draw a connection between intersections
 */
function drawAConnection(connection) {
    let drawn_objects = [];

    if (connection.hasOwnProperty('mainBundleStreet')){
        let mainBundleStreet = connection['mainBundleStreet'];
        if (mainBundleStreet.hasOwnProperty('representativeSubtrajectory')) {
            drawn_objects[drawn_objects.length] = drawASubTrajectory(mainBundleStreet['representativeSubtrajectory'], StyleClass.CONNECTION, Layer.CONNECTION);
        }
    }

    return drawn_objects;
}

/**
 * Draw a Road Map RoadSection
 */
function drawARoadSection(roadSection, styling) {
    let drawn_objects = [];

    if (settingsMapConstructionDrawing['drawAllSubTrajectoriesOfRoadSection']['value']) {
        roadSection['subtrajectories'].forEach(function (sub) {
            drawn_objects[drawn_objects.length] = drawASubTrajectory(sub, StyleClass.SUBTRAJECTORY, Layer.ROADMAP, "Road Segment " + sub['label']);
        });
    }

    if (settingsMapConstructionDrawing['drawAllBundlesOfRoadSection']['value']) {
        // roadSection['bundleStreetsClasses'].forEach(function (bundleClass) {
        //     let color = '#' + Math.floor(Math.random() * 16777215).toString(16) + '75';
        //     let props = {
        //         'color': color,
        //         'zIndex': 20,
        //         'width': 8
        //     };
        //     drawn_objects[drawn_objects.length] = drawASubtrajectoryByBundleClass(bundleClass, 0, -1, props, bundleClass)
        // });
    }

    // As last, so it draws over all subtrajectories
    if (settingsMapConstructionDrawing['drawRoadMapRoadSections']['value']) {
        drawn_objects[drawn_objects.length] = drawATrajectory(roadSection['pointList'], "Road Segment " + roadSection['uid'], styling, Layer.ROADMAP);
    }
    return drawn_objects;
}