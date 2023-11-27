/**
 * This file is responsible for showing a table with all connections.
 *
 * When hovering over an item of the table, the specific connection between intersections will be drawn.
 * Furthermore you can select items from the table, which are also drawn in that case.
 *
 * Note: The actual implementation of drawing these trajectories is implemented in maps-draw.js
 *
 * As one can see, there are a couple of menu-files.
 *
 * @author Jorrick Sleijster
 * @since  10/12/2018
 */

let intersectionNotFound = false;

/**
 * This creates a table of all the bundles.
 *
 * All items are made hoverable and selectable.
 */
function createConnectionTable() {
    let step = 'network';
    let tableClassifier = 'connections-selector-table';

    $tableHead = $('.' + tableClassifier + ' thead');
    $tableHead.empty();
    $tableBody = $('.' + tableClassifier + ' tbody');
    $tableBody.empty();
    let listOfProperties = ['id', '#int', '#int', '#b', '#bWSI', 'pbAT', 'avgBE', 'avgBES', 'avgBESAT', 'bc', 'cl'];
    let listOfPropertiesIndexes = ["intersection1Index", 'intersection2Index', 'noBundleStreets', 'noBundleStreetsWithSameBothIntersection',
        'percentageOfBundlesAboveThreshold', 'averageBundleEpsilon', 'averageBundleEpsilonSquared', 'averageBundleEpsilonSquaredWithThreshold', 'mainBundleStreetClass'];
    let listOfReversedPropertiesIndexes = ['intersection2Index', "intersection1Index", 'noBundleStreets', 'noBundleStreetsWithSameBothIntersection',
        'percentageOfBundlesAboveThreshold', 'averageBundleEpsilon', 'averageBundleEpsilonSquared', 'averageBundleEpsilonSquaredWithThreshold', 'mainBundleStreetClass'];

    $tr = $('<tr>').appendTo($tableHead);
    $td = $('<th>').text('').appendTo($tr);
    for (let key in listOfProperties) {
        let property = listOfProperties[key];
        $('<th>').text(property)
            .addClass('no-overflow-but-ellipsis')
            .appendTo($tr);
    }

    if (trajectoriesDataset['data'][step] !== null) {
        for (let index in trajectoriesDataset['data'][step]['intersectionConnections']) {
            // let properties = ['founds_eps', 'k', 'max_eps', 'min_eps'];

            let connection = trajectoriesDataset['data'][step]['intersectionConnections'][index];

            if (intersectionNotFound){
                console.log(connection['intersection1Index'], connection['intersection2Index']);
            }
            if (intersectionNotFound &&
                (connection['intersection1Index'] == "-1" || connection['intersection2Index'] == "-1")){
                continue;
            }

            $tr = $('<tr onmouseover="drawHoveredConnection($(this));" onmouseout="removeHover();"' +
                ' onclick="checkIfTableConnectionShouldBeShown($(this));">')
                .attr('connections_index', index)
                .appendTo($tableBody);
            $tr.dblclick(function () {
                focusOnConnection($(this));
            });
            $td = $('<td>').appendTo($tr);
            $bundleCheckboxDiv = $('<div>').addClass('ui checkbox simple-checkbox').appendTo($td);
            $bundleCheckboxInput = $('<input type="checkbox" onclick="checkIfTableConnectionShouldBeShown($(this));">')
                .attr('connections_index', index)
                .appendTo($bundleCheckboxDiv);
            $bundleCheckboxLabel = $('<label>').text(' ').appendTo($bundleCheckboxDiv);

            $('<td>').text(index).appendTo($tr);

            for (let key of listOfPropertiesIndexes){
                $('<td>').text(humanize(connection[key])).appendTo($tr);
            }
            $('<td>').text(humanize(connection['mainBundleStreet']['continuousLength'])).appendTo($tr);



            $tr = $('<tr onmouseover="drawHoveredConnection($(this));" onmouseout="removeHover();"' +
                ' onclick="checkIfTableConnectionShouldBeShown($(this));">')
                .attr('connections_index', index)
                .appendTo($tableBody);
            $tr.dblclick(function () {
                focusOnConnection($(this));
            });
            $td = $('<td>').appendTo($tr);
            $bundleCheckboxDiv = $('<div>').addClass('ui checkbox simple-checkbox').appendTo($td);
            $bundleCheckboxInput = $('<input type="checkbox" onclick="checkIfTableConnectionShouldBeShown($(this));">')
                .attr('connections_index', index)
                .appendTo($bundleCheckboxDiv);
            $bundleCheckboxLabel = $('<label>').text(' ').appendTo($bundleCheckboxDiv);

            $('<td>').text(-index).appendTo($tr);

            for (let key of listOfReversedPropertiesIndexes){
                $('<td>').text(humanize(connection[key])).appendTo($tr);
            }
            $('<td>').text(humanize(connection['mainBundleStreet']['continuousLength'])).appendTo($tr);

        }
        $('.' + tableClassifier + ' .ui.checkbox').checkbox();
        $('#' + tableClassifier).tablesorter();
    }
}

/**
 * Sets the center on the specific bundle.
 * Function that is called when a double click is executed on the table.
 * @param tr
 */
function focusOnConnection(tr) {
    let connections_index = tr.attr('connections_index');
    let connection = trajectoriesDataset['data']['network']['intersectionConnections'][connections_index];

    // Yet to implement
    console.log('focusOnConnections nog te implementeren...')
    // focusOnPoints([intersection['location']]);
}

/**
 * Draws all connections that are selected in the table.
 */
function drawAllConnectionsFromTheTable() {
    if (enableTable) {
        let indices = [];
        $('#sidebar-connections .tiny-table .ui.checkbox.checked').each(function () {
            indices.push($($(this).children()[0]).attr('connections_index'));
        });
        removeTableDrawnConnections();
        drawTableSelectedConnections(indices);
    }
}

/**
 * Draws the hovered connection.
 * @param tr, the tr table item that is hovered.
 */
function drawHoveredConnection(tr) {
    if (enableHover) {
        drawHoverConnection(tr.attr('connections_index'));
    }
}

/**
 * Checks if the clicked connection in the table should be drawn, if so draws it and otherwise removes it.
 * @param tableDiv, the table item that it clicked.
 */
function checkIfTableConnectionShouldBeShown(tableDiv) {
    $child = $(tableDiv.find('.ui.checkbox')[0]);
    let attribute = $($child.find('input')[0]).attr('connections_index');
    if ($child.hasClass('checked')) {
        drawTableSelectedConnections([attribute]);
    } else {
        removeSpecificTableDrawnConnections(attribute);
    }
}

function humanize(x){
    return x.toFixed(2).replace(/\.?0*$/,'');
}
