/**
 * This file is responsible for showing a table with all trajectories.
 *
 * When hovering over an item of the table, the specific trajectory will be drawn.
 * Furthermore you can select items from the table, which are also drawn in that case.
 *
 * Note: The actual implementation of drawing these trajectories is implemented in maps-draw.js
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

/**
 * This creates a table of all the trajectories.
 *
 * All items are made hoverable and selectable.
 */
function createTrajectoryTable() {
    let step = 'filtered';
    let tableClassifier = 'trajectories-selector-table';


    trajectoriesDataset['data'][step].sort(function (a, b){
        return compareStrings(a['label'], b['label']);
    });
    $tableBody = $('.' + tableClassifier + ' tbody');

    for (let index in trajectoriesDataset['data'][step]) {
        let trajectory = trajectoriesDataset['data'][step][index];

        $tr = $('<tr><td><form class="ui form"><div class="inline field"' +
            ' onmouseover="drawHoveredTrajectory($(this));" onmouseout="removeHover();"' +
            ' onclick="checkIfTableTrajectoryShouldBeShown($(this))">' +
            '<div class="ui checkbox"></div></div></form></td></tr>')
            .attr('map_trajectory_index', index)
            .appendTo($tableBody);
        $tr.dblclick(function(){
            focusOnTrajectory($(this));
        });
        while ($tr.children().length > 0) {
            $tr = $tr.children();
        }
        $input = $('<input type="checkbox" tabindex="0" class="hidden">')
            .attr('map_step', step)
            .attr('map_trajectory_index', index)
            .appendTo($tr);

        // Here we define how we can find back this specific trajectory
        let properties = ['id', 'numPoints', 'label', 'reverse'];
        for (let indexProperties in properties) {
            $input.attr('map_' + properties[indexProperties], trajectory[properties[indexProperties]]);
        }

        $label = $('<label>')
            .text(trajectory['label'])
            .appendTo($tr);
    }

    $('.' + tableClassifier + ' .ui.checkbox').checkbox();
}

/**
 * Sets the center on the specific trajectory.
 * Function that is called when a double click is executed on the table.
 * @param tr
 */
function focusOnTrajectory(tr){
    let trajectory_index = tr.attr('map_trajectory_index');
    let trajectory = trajectoriesDataset['data']['filtered'][trajectory_index];

    if ("parent" in trajectory){
        focusOnSubTrajectories([trajectory]);
    } else {
        focusOnTrajectories([trajectory]);
    }
}

/**
 * Draws all trajectories that are selected in the table.
 */
function drawAllTrajectoriesFromTheTable() {
    if (enableTable) {
        let traceNames = [];
        $('#sidebar-trajectories .tiny-table .ui.checkbox.checked').each(function () {
            traceNames.push($($(this).children()[0]).attr('map_trajectory_index'));
        });
        removeTableDrawnTrajectories();
        drawTableSelectedTrajectories(traceNames);
    }
}

/**
 * Draw the hovered trajectory.
 * @param tableDiv, the table item that is hovered.
 */
function drawHoveredTrajectory(tableDiv) {
    if (enableHover) {
        $input = tableDiv.find('input');
        drawHoverTrajectory($input.attr('map_trajectory_index'))
    }
}

/**
 * Checks if the clicked trajectory in the table should be drawn, if so draws it and otherwise removes it.
 * @param tableDiv, the table item that it clicked.
 */
function checkIfTableTrajectoryShouldBeShown(tableDiv){
    $child = $(tableDiv.children().first());
    let attribute = $($child.children()[0]).attr('map_trajectory_index');
    if ($child.hasClass('checked')){
        drawTableSelectedTrajectories([attribute]);
    } else{
        removeSpecificTableDrawnTrajectory(attribute);
    }
}