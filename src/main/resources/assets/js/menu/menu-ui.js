/**
 * This file contains all the logic of the UI page in the menu.
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

/**
 * All these functions are executed when the HTML dom is finished.
 */
$(document).ready(function () {
    // Dropdown and toggle maps background
    $('.selection.dropdown.map-type').dropdown({
        onChange: function (value, text, $selectedItem) {
            changeMapType(value);
        }
    });
    $('.selection.dropdown.map-style').dropdown({
        values: dropdownMapStyleValues,
        onChange: function (value, text, $selectedItem) {
            setMapTheme(value);
        }
    });
    $('.selection.dropdown.map-poi').dropdown({
        values: dropdownMapPOIValues,
        onChange: function (value, text, $selectedItem) {
            setMapPoi(value);
        }
    });
    $('.ui.checkbox.draw-maps-background').checkbox({
        onChecked: function () {
            enableMapTypes();
        },
        onUnchecked: function () {
            disableMapTypes();
        }
    });

    $('.ui.checkbox.make-image-ready-div').checkbox({
        onChecked: function () {
            setImageStat();
        },
        onUnchecked: function () {
            disableImageStat();
        }
    });

    $('.ui.checkbox.draw-ground-truth-background').checkbox({
        onChecked: function () {
            drawGroundTruthMap($('#hide-ground-truth-checkbox').is(':checked'));
        },
        onUnchecked: function () {
            removeDrawnGroundTruthMap();
        }
    });

    $('.ui.checkbox.hide-ground-truth-invalid').checkbox({
        onChecked: function () {
            if ($('#draw-ground-truth-checkbox').is(':checked')) {
                drawGroundTruthMap(true);
            }
        },
        onUnchecked: function() {
            if ($('#draw-ground-truth-checkbox').is(':checked')) {
                drawGroundTruthMap(false);
            }
        }
    });

    // Setting all required things for the zoom to function.
    $('#map-zoom-range').range({
        min: 12,
        max: 22,
        start: 15,
        input: '#map-zoom-input',
        onChange: function (value, extraParams) {
            if (extraParams['triggeredByUser']){
                updateZoom(value);
            }
        }
    });
    $('#map-zoom-input').on("change", function () {
        updateZoom($(this).val());
    });

    // Toggling which trajectories to draw. First all regular, then only table.
    $('.ui.checkbox.enable-all-trajectories-drawing').checkbox({
        onChecked: function () {
            drawAllStandardTrajectories();
        },
        onUnchecked: function () {
            removeStandardDrawnTrajectories();
        }
    });
    $('.ui.checkbox.enable-table-drawing').checkbox({
        onChecked: function () {
            enableTable = true;
            executeTableDraws();
        },
        onUnchecked: function () {
            enableTable = false;
            removeTableDraws();
        }
    });
    $('.ui.checkbox.enable-hover-drawing').checkbox({
        onChange: function () {
            enableHover = $(this).is(':checked');
        }
    });

    /**
     * Instead of having to create a checkbox for every option, we simply created a large dict (settingsBundleDrawing)
     * containing all the options, and here we just build all checkboxes related to them.
     * Allows for faster development when testing.
     */
    $bundleDrawingOptionsContainer = $('.bundle-drawing-options');
    createAllUIOptionsForDictionary($bundleDrawingOptionsContainer, settingsBundleDrawing);

    $roadMapDrawingOptionsContainer = $('.road-map-drawing-options');
    createAllUIOptionsForDictionary($roadMapDrawingOptionsContainer, settingsMapConstructionDrawing);


    // Selecting the color
    mapBackgroundColor = new Picker({
        parent: document.querySelector('#background-color-opener'),
        color: 'F7F7F7'
    });
    mapBackgroundColor.onDone = function (color) {
        $('#background-color-opener').parent().css('background-color', color.rgbaString);
        changeMapBackgroundColor(color.rgbaString);
    };

    standardTrajectoryColorPicker = new Picker(document.querySelector('#standard-color-opener'));
    standardTrajectoryColorPicker.onDone = function (color) {
        $('#standard-color-opener').parent().css('background-color', color.rgbaString);
        changeStandardTrajectoryColor(color.rgbaString);
    };

    tableTrajectoryColorPicker = new Picker(document.querySelector('#table-color-opener'));
    tableTrajectoryColorPicker.onDone = function (color) {
        $('#table-color-opener').parent().css('background-color', color.rgbaString);
        changeTableTrajectoryColor(color.rgbaString);

    };

    hoverTrajectoryColorPicker = new Picker(document.querySelector('#hover-color-opener'));
    hoverTrajectoryColorPicker.onDone = function (color) {
        $('#hover-color-opener').parent().css('background-color', color.rgbaString);
        changeHoverTrajectoryColor(color.rgbaString);
    };
});

/**
 * Instead of having to create a checkbox for every option, we simply created a large dict (settingsBundleDrawing)
 * containing all the options, and here we just build all checkboxes related to them.
 * Allows for faster development when testing.
 */
function createAllUIOptionsForDictionary(container, uiOptionsDict){
    for (let index in uiOptionsDict) {
        // First we create a checkbox somewhere.
        $mainDiv = $('<div>')
            .addClass('ui checkbox extra-margin')
            .appendTo(container);
        if (uiOptionsDict[index]['value']) {
            $mainDiv.addClass('checked');
        }

        $input = $('<input>')
            .attr('type', uiOptionsDict[index]['type'])
            .attr('checked', '')
            .appendTo($mainDiv);

        $label = $('<label>')
            .text(convertCamelCaseToWords(index))
            .appendTo($mainDiv);

        $mainDiv.checkbox(
            {
                onChecked: function () {
                    uiOptionsDict[index]['value'] = true;
                    drawAllBundlesFromTheTable();
                    drawAllIntersectionsFromTheTable();

                },
                onUnchecked: function () {
                    uiOptionsDict[index]['value'] = false;
                    drawAllBundlesFromTheTable();
                    drawAllIntersectionsFromTheTable();
                }
            });
        if (!uiOptionsDict[index]['value']) {
            $mainDiv.checkbox('set unchecked');
        }
        // Then we initialize the checkbox.
        // On change event added.
    }
}

// Keeps track of previous known maps zoom level. This is to prevent updates being called when not necessary.
let previousKnownMapsZoom = 0;
/**
 * A function to update the zoom.
 * Note: it assumes we can not get an infinite loop of events this way.
 * @param zoomValue
 */
function updateZoom(zoomValue) {
    zoomValue = Math.ceil(zoomValue);
    if (previousKnownMapsZoom !== zoomValue) {
        previousKnownMapsZoom = zoomValue;

        $('#map-zoom-input').val(zoomValue);
        if (map !== undefined && map !== null){
            map.getView().setZoom(zoomValue);
        }
        $('#map-zoom-range').range('set value', zoomValue);
    }
}

/**
 * We convert strings of camelCase to a sentence/words (camelCase -> Camel case).
 * @param string, the camelCase string
 * @returns words with the first word being capital.
 */
function convertCamelCaseToWords(string) {
    string = string
    // insert a space before all caps
        .replace(/([A-Z])/g, ' $1');
    return string.charAt(0).toUpperCase() + string.slice(1);
}


/**
 * Values of the Map Style dropdown.
 */
const dropdownMapStyleValues = [
    {
        name: 'Normal',
        value: 'normal',
        selected: true
    },
    {
        name: 'Silver',
        value: 'silver'
    },
    {
        name: 'Grey',
        value: 'grey'
    },
    {
        name: 'White',
        value: 'white'
    },
    {
        name: 'Ski',
        value: 'ski'
    },
    {
        name: 'Retro',
        value: 'retro'
    },
    {
        name: 'Dark',
        value: 'dark'
    },
    {
        name: 'Night',
        value: 'night'
    },
    {
        name: 'Aubergine',
        value: 'aubergine'
    },
];

/**
 * Values of the POI dropdown.
 */
const dropdownMapPOIValues = [
    {
        name: 'Normal',
        value: 'normal'
    },
    {
        name: 'Hide landmarks',
        value: 'hideLandsmarks'
    },
    {
        name: 'Hide all',
        value: 'hideAll',
        selected: true
    }
];