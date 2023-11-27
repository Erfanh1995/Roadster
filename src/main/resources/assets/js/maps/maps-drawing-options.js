/**
 * This file contains all the different drawing styles and variables.
 * @author Jorrick Sleijster
 * @since  16/10/2018
 */

// Whether we should draw hovering items or table selected items.
let enableHover = true;
let enableTable = true;

/**
 * All the settings related to bundle drawing.
 *
 * During experiments there are a lot of variables and parts of bundles we want to draw.
 * To be able to quickly add parts that we can draw we only need to add a new item here and implement the actual
 * drawing inside the drawABundle function. The form item will automatically be created by menu-ui and update the value
 * in this list, as you change it there.
 */
let settingsBundleDrawing = {
    'drawBundleSubtrajectories': {
        'type': 'checkbox',
        'value': false
    },
    'drawOriginalRepresentative': {
        'type': 'checkbox',
        'value': false
    },
    'drawForceRepresentative': {
        'type': 'checkbox',
        'value': false
    },
    'drawFinalRepresentative': {
        'type': 'checkbox',
        'value': true
    },
    'drawForcePoints': {
        'type': 'checkbox',
        'value': false
    },
    'drawForceAcLines': {
        'type': 'checkbox',
        'value': false
    },
    'drawForceOrthogonals': {
        'type': 'checkbox',
        'value': false
    },
    'drawForceLinearRegression': {
        'type': 'checkbox',
        'value': false
    },
    'drawSinglePointTurns': {
        'type': 'checkbox',
        'value': false
    },
    'drawMultiEdgesTurns': {
        'type': 'checkbox',
        'value': false
    },
    'drawTurns': {
        'type': 'checkbox',
        'value': false
    },
    'drawBeforeTurn': {
        'type': 'checkbox',
        'value': false
    },
    'drawAfterTurn': {
        'type': 'checkbox',
        'value': false
    },
    'drawLinearRegression': {
        'type': 'checkbox',
        'value': false
    }
};

let settingsMapConstructionDrawing = {
    'drawIntersectionLocation': {
        'type': 'checkbox',
        'value': true
    },
    'drawIntersectionApproximateLocation': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionClusters': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionPoints': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionPointEndingBundleRep': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionPointBundlePairRep': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionPointOverlappingBundleRep': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionStraightestBundle':{
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionBundleParts': {
        'type': 'checkbox',
        'value': false
    },
    'drawIntersectionShapeFittingLines': {
        'type': 'checkbox',
        'value': false
    },
    'drawAllBundlesConnectedToIntersection': {
        'type': 'checkbox',
        'value': false
    },
    'drawRoadPointsOnBundle': {
        'type': 'checkbox',
        'value': false
    },
    'drawRoadMapRoadSections': {
        'type': 'checkbox',
        'value': true
    },
    'drawAllSubTrajectoriesOfRoadSection': {
        'type': 'checkbox',
        'value': false
    },
    'drawAllBundlesOfRoadSection': {
        'type': 'checkbox',
        'value': false
    }
};

function VectorStyleManager(feature, resolution) {
    let style = feature.values_.styleclass;
    if (style && style instanceof ol.style.Style) {
        return feature.values_.styleclass;
    } else if (typeof style === "function") {
        return feature.values_.styleclass(resolution, feature);
    }
    return StyleClass.DEFAULT;
}

let StyleClass = {
    TRAJECTORY: stroke('#3498db'),
    SUBTRAJECTORY: stroke('#9b59b685', 2),
    REPRESENTATIVE: stroke('#ff1400a1', 6),
    BUNDLE: (k) => stroke(randomColor(k), 6),
    GROUND_TRUTH: function(r) {
        // resolution steps determining width of the stroke
        let w = r < 0.5 ? 10 : r < 2 ? 6 : r < 10 ? 3 : 1;
        return stroke('#d4d4d2', w);
    },
    GROUND_TRUTH_INVALID: function(r) {
        // resolution steps determining width of the stroke
        let w = r < 0.5 ? 5 : r < 2 ? 3 : r < 10 ? 1 : 0.5;
        return stroke('#d4d4d2', w);
    },
    MARKER: (c = 'yellow') => {
        return function(r, f) {
            let circle = f.getGeometry();
            // console.log(f, circle, f.values_.size * (r < 0.5 ? 1 : r < 2 ? 2 : r < 10 ? 5 : 10));
            if (circle !== undefined && circle.setRadius !== undefined) {
                circle.setRadius(f.values_.size * (r < 0.5 ? 1 : r < 2 ? 2 : r < 10 ? 5 : 10));
            }
            return strokeFill(c, 2)
        }

    },
    ROAD: stroke('#FF00FF', 6),
    ROAD_EDGE: stroke('$red', 6),
    ROAD_EDGE_HOVER: stroke('#00ccffb3', 9),
    ROAD_POINT: strokeFill('#ff0000', 2),
    FORCE_PATH: stroke('#ffc000', 6),
    FORCE_AC: stroke('#FF000071', 4),
    FORCE_DIAGONAL: stroke('#ffff0071', 4),
    FORCE_REGRESSION: stroke('#b100ff94', 6),
    TURN_POINT: strokeFill('#f104047a', 8),
    TURNS: stroke('#f104047a', 8),
    MERGED_REPRESENTATIVE: stroke('#fb9e3a', 6),
    INT_END_BUNDLE: stroke('#ff0000af', 6),
    INT_0TH_BUNDLE_1: stroke('#ffaa1d99', 5),
    INT_0TH_BUNDLE_2: stroke('#29961799', 5),
    INT_0TH_BUNDLE_3: stroke('#ff547099', 5),
    INTERSECTION_PART: (k) => stroke(randomColor(k), 8),
    INTERSECTION_BUNDLE: (k) => stroke(randomColor(k), 8),
    SHAPE_FITTING_LINES: stroke('#0066CC', 8),
    CONNECTION: stroke('#fb9e3a', 6),

    DEFAULT: stroke('#ecf0f1'),
};

/**
 * Get a color object {primary: <>, secondary: <>} from a string, array or object representing color. This function also
 * supports several defaults, accessed by using a string prefixed with $. E.g. colorGetDefault('$blue');
 * @param color The color to return
 * @returns {{primary: *, secondary: *}}
 */
function colorGetDefault(color) {
    if (color.startsWith && color.startsWith("$")) {
        return {
            blue: colorGetDefault(['#3498db', '#2980b9']),
            green: colorGetDefault(['#2ecc71', '#27ae60']),
            purple: colorGetDefault(['#9b59b6', '#9b59b6']),
            red: colorGetDefault(['#e74c3c', '#c0392b']),
            yellow: colorGetDefault(['#f1c40f', '#f39c12']),
        }[color.substring(1)];
    } else if (color.hasOwnProperty("primary")) {
        if (color.hasOwnProperty("secondary")) {
            return color;
        } else {
            return {
                primary: color.primary,
                secondary: color.primary,
            }
        }
    } else if (Array.isArray(color)) {
        return {
            primary: color[0],
            secondary: color[1],
        }
    } else {
        return {
            primary: color,
            secondary: color
        }
    }
}

function randomColor(key, alpha = '75') {
    if (this.hashtable === undefined) {
        this.hashtable = {}
    }

    if (this.hashtable.hasOwnProperty(key)) {
        return this.hashtable[key];
    }
    this.hashtable[key] = '#' + Math.floor(Math.random() * 16777215).toString(16) + alpha;
    return this.hashtable[key];
}

/**
 * Convert HEX color to RGB object
 * @param hex The hex color
 * @returns RGB object
 */
function hexToRgb(hex) {
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}

/**
 * Helper function for creating strokes
 * @param color The stroke color
 * @param width The stroke width
 */
function stroke(color, width = 1) {
    color = colorGetDefault(color);
    return new ol.style.Style({
        stroke: new ol.style.Stroke({
            color: color.primary,
            width: width
        })
    });
}

/**
 * Helper function for creating shapes with stroke and fill
 * @param color One (or two) colors to use for fill (primary) and stroke (secondary)
 * @param width The stroke width
 */
function strokeFill(color, width = 1) {
    color = colorGetDefault(color);

    return new ol.style.Style({
        fill: new ol.style.Fill({
            color: color.primary,
        }),
        stroke: new ol.style.Stroke({
            color: color.secondary,
            width: width,
        })
    });
}

/**
 * The different drawing styles. Defining them here allows for quick and easy changes.
 * @type {{color: string, zIndex: number, width: number}}
 */
let propStandardTrajectory = {
    'color': '#0000002f',
    'zIndex': 0,
    'width': 2
};
let propTableTrajectory = {
    'color': '#00ccff85',
    'zIndex': 3,
    'width': 5
};
let propHoverTrajectory = {
    'color': '#00ccff61',
    'zIndex': 5,
    'width': 8
};
let propRepresentativeBundle = {
    'color': '#ff1400a1',
    'zIndex': 6,
    'width': 6
};
let propRoad = {
    'color': '#FF00FF',
    'zIndex': 4,
    'width': 6
};
let propForcePath = {
    // 'color': '#fb9e3abf',
    'color': '#ffc000',
    'zIndex': 11,
    'width': 6
};
let propForceAC = {
    'color': '#FF000071',
    'zIndex': 11,
    'width': 4
};
let propForceDiagonal = {
    'color': '#ffff0071',
    'zIndex': 11,
    'width': 4
};
let propForceLinearRegressionPath = {
    'color': '#b100ff94',
    'zIndex': 15,
    'width': 6
};
let propGroundTruthBorder = {
    'color': '#7d7d7d5b',
    'zIndex': 0,
    'width': 10
};
let propGroundTruth = {
    'color': '#ffffffff',
    'zIndex': 1,
    'width': 8
};
let propTurns = {
    'color': '#f104047a',
    'zIndex': 20,
    'width': 8
};
let propMergedRepr = {
    // 'color': '#ff0000af',
    'color': '#fb9e3a',
    'zIndex': 11,
    'width': 6
};
let propRoadEdge = {
    'color': '#ff0000',
    'zIndex': 3,
    'width': 6
};
let propRoadEdgeHover = {
    'color': '#00ccffb3',
    'zIndex': 3,
    'width': 9
};


/**
 * Related to intersections drawing
 */
let propIntEndBundle = {
    'color': '#ff0000af',
    'zIndex': 11,
    'width': 6
};
let propIntOthBundle1 = {
    'color': '#ffaa1d99',
    'zIndex': 10,
    'width': 5
};
let propIntOthBundle2 = {
    'color': '#29961799',
    'zIndex': 9,
    'width': 5
};
let propIntOthBundle3 = {
    'color': '#ff547099',
    'zIndex': 8,
    'width': 5
};

function setZoomedIn(){
    propMergedRepr['width'] = 6;
    propRoadEdge['width'] = 6;
    propStandardTrajectory.width = 2;
}

function setZoomedOut(){
    propMergedRepr['width'] = 3;
    propRoadEdge['width'] = 3;
    propStandardTrajectory.width = 1;
    propStandardTrajectory.color = '#00000066';
    console.log('Changed property standard trajectory color to #00000066 from #0000002f');
}

function setImageStat(){
    disableMapTypes();
    propMergedRepr['width'] = 4;
    propRepresentativeBundle['width'] = 4;
    propRoadEdge['width'] = 5;
    propStandardTrajectory.width = 1;
    propStandardTrajectory.color = '#00000066';
    drawAllStandardTrajectories();
}

function disableImageStat(){
    enableMapTypes();
    propMergedRepr['width'] = 6;
    propRepresentativeBundle['width'] = 6;
    propRoadEdge['width'] = 6;
    propStandardTrajectory.width = 2;
    propStandardTrajectory.color = '#0000002f';
    drawAllStandardTrajectories();
}


function setSatView() {
    propStandardTrajectory = {
        'color': '#0000FF',
        'zIndex': 2,
        'width': 2
    };
}

function delSatView() {
    propStandardTrajectory = {
        'color': '#0000002f',
        'zIndex': 2,
        'width': 2
    };
}

function mapsHelp() {
    console.log('Preferred method for images is setting the map style to Grey');
    console.log('You have the following options to improve the final map:');
    console.log('setZoomedIn();     Sets the RoadEdges to a size of 6 (default)');
    console.log('setZoomedOut();    Sets the RoadEdges to a size of 3, mainly for bigger views.');
    console.log('setSatView();      Changes the color of the standard trajectories to blue');
    console.log('delSatView();      Changes the color of the standard trajectories to black again');;
}