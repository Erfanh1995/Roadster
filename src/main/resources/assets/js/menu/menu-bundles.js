/**
 * This file is responsible for showing a table with all bundles.
 *
 * When hovering over an item of the table, the specific bundle will be drawn.
 * Furthermore you can select items from the table, which are also drawn in that case.
 *
 * Note: The actual implementation of drawing these trajectories is implemented in maps-draw.js
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

let filterAllowedEps = [];
let filterMinK = 0;
let filterMaxK = 999;
let allowedBundleClasses = [];

/**
 * This creates a table of all the bundles.
 *
 * All items are made hoverable and selectable.
 */
function createBundleTable() {
    let step = 'bundles';
    let tableClassifier = 'bundles-selector-table';

    $tableHead = $('.' + tableClassifier + ' thead');
    $tableHead.empty();
    $tableBody = $('.' + tableClassifier + ' tbody');
    $tableBody.empty();
    let listOfProperties = ['BundleClass', 'Size', 'DiscreteLength', 'ContinuousLength', 'Birth', 'Merge', 'LifeSpan', 'RelativeLifeSpan', 'BestEps'];
    let listOfNames = ['id', 'k', 'dL', 'cL', 'Birth', 'Merge', 'Life', 'RelativeLifeSpan', 'BestEps'];
    let listOfRoundings = [0, 0, 0, 0, 2, 2, 2, 2, 2];
    $tr = $('<tr>').appendTo($tableHead);
    $td = $('<th>').text('').appendTo($tr);
    for (let key in listOfProperties) {
        let property = listOfProperties[key];
        $('<th>')
            .html("<span class=\"header vertical\"> " + property + " </span>")
            // .addClass('no-overflow-but-ellipsis')
            .appendTo($tr);
    }

    for (let index in trajectoriesDataset['data'][step]) {
        // let properties = ['founds_eps', 'k', 'max_eps', 'min_eps'];

        let bundle = trajectoriesDataset['data'][step][index];
        // Filter on k
        if (bundle['Size'] < filterMinK || bundle['Size'] > filterMaxK) {
            continue;
        }

        // Filter on epsilon
        let found = false;
        for (let eps in filterAllowedEps) {
            if (parseInt(filterAllowedEps[eps]) >= parseInt(bundle['Birth']) &&
                parseInt(filterAllowedEps[eps]) <= parseInt(bundle['Merge'])) {
                found = true;
            }
        }
        if (!found && filterAllowedEps.length !== 0) {
            continue;
        }

        // Special hidden filter on bundleClass.
        if (allowedBundleClasses.length > 0 && allowedBundleClasses.indexOf(parseInt(bundle['BundleClass'])) < 0){
            continue;
        }

        // Hidden filters on bundle values. Can be anything :)
        if (!bundleCompliesWithFilters(bundle)) {
            continue;
        }


        $tr = $('<tr onmouseover="drawHoveredBundle($(this));" onmouseout="removeHover();"' +
            ' onclick="checkIfTableBundleShouldBeShown($(this));">')
            .attr('map_bundle_index', index)
            .appendTo($tableBody);
        $tr.dblclick(function () {
            focusOnBundle($(this));
        });
        $td = $('<td>').appendTo($tr);
        $bundleCheckboxDiv = $('<div>').addClass('ui checkbox simple-checkbox').appendTo($td);
        $bundleCheckboxInput = $('<input type="checkbox" onclick="checkIfTableBundleShouldBeShown($(this));">')
            .attr('map_step', step)
            .attr('map_bundle_index', index)
            .appendTo($bundleCheckboxDiv);
        $bundleCheckboxLabel = $('<label>').text(' ').appendTo($bundleCheckboxDiv);

        for (let key in listOfProperties) {
            let property = listOfProperties[key];
            // First we convert it to 4 digits, then we round it to at most 2 decimals.
            let roundToDigits = listOfRoundings[key];
            let number = parseFloat(bundle[property]).toFixed(roundToDigits);
            // Ugly fix for showing the number in the table.
            let final_number = 1.0 + parseFloat(number) - 1.0;
            final_number = Math.round(final_number * 10000) / 10000;
            $('<td>').text(final_number)
                .appendTo($tr);
        }

    }

    $('.' + tableClassifier + '  .ui.checkbox').checkbox();
    $('.' + tableClassifier).tablesorter();
}

/**
 * Sets the center on the specific bundle.
 * Function that is called when a double click is executed on the table.
 * @param tr
 */
function focusOnBundle(tr) {
    let bundle_index = tr.attr('map_bundle_index');
    let bundle = trajectoriesDataset['data']['bundles'][bundle_index];
    bundle = bundle['Bundle'];
    focusOnPoints(bundle['mergedRepresentativeJSON']);
}

/**
 * Draws all bundles that are selected in the table.
 */
function drawAllBundlesFromTheTable() {
    if (enableTable) {
        let bundlesNames = [];
        $('#sidebar-bundles .tiny-table .ui.checkbox.checked').each(function () {
            bundlesNames.push($($(this).children()[0]).attr('map_bundle_index'));
        });
        removeTableDrawnBundles();
        drawTableSelectedBundles(bundlesNames);
    }
}

/**
 * Draws the hovered bundle.
 * @param tr, the tr table item that is hovered.
 */
function drawHoveredBundle(tr) {
    if (enableHover) {
        drawHoverBundle(tr.attr('map_bundle_index'));
    }
}

/**
 * Checks if the clicked bundle in the table should be drawn, if so draws it and otherwise removes it.
 * @param tableDiv, the table item that it clicked.
 */
function checkIfTableBundleShouldBeShown(tableDiv) {
    $child = $(tableDiv.find('.ui.checkbox')[0]);
    let attribute = $($child.find('input')[0]).attr('map_bundle_index');
    if ($child.hasClass('checked')) {
        drawTableSelectedBundles([attribute]);
    } else {
        removeSpecificTableDrawnBundle(attribute);
    }
}

function setBundleFilters() {
    $epsilons = $('#epsilon-selector');
    $minK = $('#bundle-min-k');
    $maxK = $('#bundle-max-k');

    filterAllowedEps = $epsilons.dropdown('get value');
    filterMinK = $minK.val();
    filterMaxK = parseInt($maxK.val()) >= 0 ? $maxK.val() : 200;

    removeTableDrawnBundles();

    createBundleTable();
    initializeShiftSelectForAllCheckboxes();
}

function clearBundleFilters() {
    $epsilons = $('#epsilon-selector');
    $minK = $('#bundle-min-k');
    $maxK = $('#bundle-max-k');

    $epsilons.dropdown('clear');
    $minK.val('');
    $maxK.val('');

    filterAllowedEps = [];
    filterMinK = 0;
    filterMaxK = 999;
}

function bundleCompliesWithFilters(bundle) {
    for (let key in specialBundleFilters) {
        let depth = specialBundleFilters[key]['depth'];
        depth = depth.replace(/(\.)/g, ' $1').replace('.', '').split(' ');

        let minValue = specialBundleFilters[key]['min'];
        let maxValue = specialBundleFilters[key]['max'];
        let valueContainer = bundle;
        let value;
        // We don't need the base case of depth0 === "" as that is already fixed.
        if (depth[0] !== "") {
            if (!valueContainer.hasOwnProperty(depth[0])) {
                console.log("Error in menu-bundles.bundleCompliesWithFilters. Filter depth[0]: ",
                    depth[0], " is incorrect");
            }
            valueContainer = valueContainer[depth[0]];
        } else {
            console.log("Error in menu-bundles.bundleCompliesWithFilters. Depth[0] is incorrect", bundle, depth);
        }
        if (depth.length === 2) {
            if (!valueContainer.hasOwnProperty(depth[1])) {
                console.log("Error in menu-bundles.bundleCompliesWithFilters. Filter depth[1]: ",
                    depth[1], " is incorrect with original depth: ", specialBundleFilters[key]['depth']);
            }
            valueContainer = valueContainer[depth[1]];
        }

        if (!valueContainer.hasOwnProperty(key)) {
            console.log("Error in menu-bundles.bundleCompliesWithFilters. Filter key: ",
                key, " is incorrect with given depth: ", specialBundleFilters[key]['depth'], ".");
            console.log("ValueContainer at the time was: ", valueContainer);
        }

        value = valueContainer[key];

        if (!(minValue <= value && value <= maxValue)) {
            return false;
        }
    }
    return true;
}

function setAllowedBundleClasses(list){
    allowedBundleClasses = list;
    createBundleTable();
}

let specialBundleFilters = {};

// These are here to set for yourself in the console.
let specialBundleStraightNessFilters = {
    'R^2': {
        'depth': 'Bundle.forceRepresentativeRegressionLineJSON',
        'min': 0.5,
        'max': 1.0,
        'enabled': true
    },
    'FullLineLength': {
        'depth': 'Bundle.forceRepresentativeStraightMetricJSON',
        'min': 0.999,
        'max': 1.000
    }
};

let specialBundleAlmostStraighNessFilter = {
    'R^2': {
        'depth': 'Bundle.forceRepresentativeRegressionLineJSON',
        'min': 0.25,
        'max': 1.0,
        'enabled': true
    },
    'FullLineLength': {
        'depth': 'Bundle.forceRepresentativeStraightMetricJSON',
        'min': 0.975,
        'max': 1.000
    }
};

let specialBundleCutStraightNessFilters = {
    'R^2': {
        'depth': 'Bundle.forceRepresentativeRegressionLineJSON',
        'min': 0.3,
        'max': 1.0,
        'enabled': true
    },
    'CutLineLength': {
        'depth': 'Bundle.forceRepresentativeStraightMetricJSON',
        'min': 0.999,
        'max': 1.000
    }
};

