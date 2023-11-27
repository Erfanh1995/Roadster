/**
 * This file is responsible for showing a table with all roads from the final constructed road map.
 *
 * When hovering over an item of the table, the specific road will be drawn.
 * Furthermore you can select items from the table, which are also drawn in that case.
 *
 * Note: The actual implementation of drawing these trajectories is implemented in maps-draw.js
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

/**
 * This creates a table of all the roads.
 *
 * All items are made hoverable and selectable.
 */
function createRoadMapTable() {
    let tableClassifier = 'road-edges-selector-table';
    let listOfProperties = ['BundleClass', '# points', 'Length', 'uid', 'between', 'removed'];

    $tableHead = $('.' + tableClassifier + ' thead');
    $tableHead.empty();
    $tableBody = $('.' + tableClassifier + ' tbody');
    $tableBody.empty();
    $tr = $('<tr>').appendTo($tableHead);
    $td = $('<th>').text('').appendTo($tr);
    for (let key in listOfProperties){
        let property = listOfProperties[key];
        $('<th>').text(property)
            .addClass('no-overflow-but-ellipsis')
            .appendTo($tr);
    }
    if (trajectoriesDataset['data']['network'] !== null) {
        if (trajectoriesDataset['data']['network']['roadMap'] == null){
            return;
        }
        for (let index in trajectoriesDataset['data']['network']['roadMap']['roadSections']) {
            // let properties = ['founds_eps', 'k', 'max_eps', 'min_eps'];

            let edge = trajectoriesDataset['data']['network']['roadMap']['roadSections'][index];

            $tr = $('<tr onmouseover="drawHoveredRoad($(this));" onmouseout="removeHover();"' +
                ' onclick="checkIfTableRoadShouldBeShown($(this));">')
                .attr('map_road_index', index)
                .appendTo($tableBody);
            $tr.dblclick(function () {
                focusOnRoad($(this));
            });
            $td = $('<td>').appendTo($tr);
            $bundleCheckboxDiv = $('<div>').addClass('ui checkbox simple-checkbox').appendTo($td);
            $bundleCheckboxInput = $('<input type="checkbox" onclick="checkIfTableRoadShouldBeShown($(this));">')
                .attr('map_step', step)
                .attr('map_road_index', index)
                .appendTo($bundleCheckboxDiv);
            $bundleCheckboxLabel = $('<label>').text(' ').appendTo($bundleCheckboxDiv);

            $('<td>').text(edge['drawnBundleStreetClass']).appendTo($tr);
            $('<td>').text(edge['pointList'].length).appendTo($tr);
            $('<td>').text(edge['continuousLength']).appendTo($tr);
            $('<td>').text(edge['uid']).appendTo($tr);
            $('<td>').text(edge['connectionBetweenMergeAreas']).appendTo($tr);
            $('<td>').text(edge['removed']).appendTo($tr);
        }
        $('.' + tableClassifier + ' .ui.checkbox').checkbox();
        $('#' + tableClassifier).tablesorter();
    }
}

/**
 * Sets the center on the specific road.
 * Function that is called when a double click is executed on the table.
 * @param tr
 */
function focusOnRoad(tr){
    let edge_index = tr.attr('map_road_index');
    let edge = trajectoriesDataset['data']['network']['roadMap']['roadSections'][edge_index];
    focusOnPoints(edge['pointList']);
}

/**
 * Draws all roads that are selected in the table.
 */
function drawAllRoadsFromTheTable() {
    if (enableTable) {
        let roadindices = [];
        $('#sidebar-roadmap .tiny-table .ui.checkbox.checked').each(function () {
            roadindices.push($($(this).children()[0]).attr('map_road_index'));
        });
        removeTableDrawnRoads();
        drawTableSelectedRoads(roadindices);
    }
}

/**
 * Draws the hovered road.
 * @param tr, the tr table item that is hovered.
 */
function drawHoveredRoad(tr) {
    if (enableHover){
        drawHoverRoad(tr.attr('map_road_index'));
    }
}

/**
 * Checks if the clicked road in the table should be drawn, if so draws it and otherwise removes it.
 * @param tableDiv, the table item that it clicked.
 */
function checkIfTableRoadShouldBeShown(tableDiv) {
    $child = $(tableDiv.find('.ui.checkbox')[0]);
    let attribute = $($child.find('input')[0]).attr('map_road_index');
    if ($child.hasClass('checked')) {
        drawTableSelectedRoads([attribute]);
    } else {
        removeSpecificTableDrawnRoad(attribute);
    }
}

$(document).ready(function () {
    // Toggling to draw all connection vertices
    $('.ui.checkbox.enable-all-connection-vertices-drawing').checkbox({
        onChecked: function () {
            drawAllConnectionVertices();
        },
        onUnchecked: function () {
            removeAllConnectionVertices();
        }
    });
});