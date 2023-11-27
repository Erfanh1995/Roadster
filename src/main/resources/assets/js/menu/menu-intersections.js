/**
 * This file is responsible for showing a table with all intersections.
 *
 * When hovering over an item of the table, the specific intersections will be drawn.
 * Furthermore you can select items from the table, which are also drawn in that case.
 *
 * Note: The actual implementation of drawing these trajectories is implemented in maps-draw.js
 *
 * As one can see, there are a couple of menu- files.
 *
 * @author Jorrick Sleijster
 * @since  12/11/2018
 */


let allowIntersectionIds = [];


/**
 * This creates a table of all the bundles.
 *
 * All items are made hoverable and selectable.
 */
function createIntersectionTable() {
    let step = 'network';
    let tableClassifier = 'intersection-selector-table';

    $tableHead = $('.' + tableClassifier + ' thead');
    $tableHead.empty();
    $tableBody = $('.' + tableClassifier + ' tbody');
    $tableBody.empty();
    let listOfProperties = ['id', 'n.o. Clusters', 'n.o. IPs'];

    $tr = $('<tr>').appendTo($tableHead);
    $td = $('<th>').text('').appendTo($tr);
    for (let key in listOfProperties) {
        let property = listOfProperties[key];
        $('<th>').text(property)
            .addClass('no-overflow-but-ellipsis')
            .appendTo($tr);
    }

    if (trajectoriesDataset['data'][step] !== null) {

        for (let index in trajectoriesDataset['data'][step]['intersections']) {
            // let properties = ['founds_eps', 'k', 'max_eps', 'min_eps'];

            let intersection = trajectoriesDataset['data'][step]['intersections'][index];

            if (allowIntersectionIds.length > 0 && allowIntersectionIds.indexOf(parseInt(index)) < 0){
                continue;
            }

            $tr = $('<tr onmouseover="drawHoveredIntersection($(this));" onmouseout="removeHover();"' +
                ' onclick="checkIfTableIntersectionShouldBeShown($(this));">')
                .attr('intersection_index', index)
                .appendTo($tableBody);
            $tr.dblclick(function () {
                focusOnIntersection($(this));
            });
            $td = $('<td>').appendTo($tr);
            $bundleCheckboxDiv = $('<div>').addClass('ui checkbox simple-checkbox').appendTo($td);
            $bundleCheckboxInput = $('<input type="checkbox" onclick="checkIfTableIntersectionShouldBeShown($(this));">')
                .attr('intersection_index', index)
                .appendTo($bundleCheckboxDiv);
            $bundleCheckboxLabel = $('<label>').text(' ').appendTo($bundleCheckboxDiv);

            $('<td>').text(index).appendTo($tr);
            $('<td>').text(intersection['allIntersectionClusters'].length).appendTo($tr);
            $('<td>').text(intersection['allIntersectionPoints'].length).appendTo($tr);

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
function focusOnIntersection(tr) {
    let intersection_index = tr.attr('intersection_index');
    let intersection = trajectoriesDataset['data']['network']['intersections'][intersection_index];

    focusOnPoints([intersection['location']]);
}

/**
 * Draws all bundles that are selected in the table.
 */
function drawAllIntersectionsFromTheTable() {
    if (enableTable) {
        let indices = [];
        $('#sidebar-intersections .tiny-table .ui.checkbox.checked').each(function () {
            indices.push($($(this).children()[0]).attr('intersection_index'));
        });
        removeTableDrawnIntersections();
        drawTableSelectedIntersections(indices);
    }
}

/**
 * Draws the hovered bundle.
 * @param tr, the tr table item that is hovered.
 */
function drawHoveredIntersection(tr) {
    if (enableHover) {
        drawHoverIntersection(tr.attr('intersection_index'));
    }
}

/**
 * Checks if the clicked bundle in the table should be drawn, if so draws it and otherwise removes it.
 * @param tableDiv, the table item that it clicked.
 */
function checkIfTableIntersectionShouldBeShown(tableDiv) {
    $child = $(tableDiv.find('.ui.checkbox')[0]);
    let attribute = $($child.find('input')[0]).attr('intersection_index');
    if ($child.hasClass('checked')) {
        drawTableSelectedIntersections([attribute]);
    } else {
        removeSpecificTableDrawnIntersections(attribute);
    }
}

/**
 * Get's a string which identifies a bundle
 * @param intersection, the intersection with the info of the bundle
 * @param bClass, the identifier of the bClass of the bundle class
 * @returns {string}
 */
function getBundleIdentifier(intersection, bClass){
    if (intersection.hasOwnProperty(bClass)){
        const bundleClass = intersection[bClass];
        const bundleProps = findBundlePropsByClass(bundleClass);
        let text = "(" + bundleProps['Size'] + ") ";
        text += bundleClass;
        return text;
    }
    return '-1';
}


function setAllowedIntsIDs(list){
    allowIntersectionIds = list;
    createIntersectionTable();
}