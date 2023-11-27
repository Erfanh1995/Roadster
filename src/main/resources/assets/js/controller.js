/**
 * This file contains all of the logic behind the dataset selection screen and the switching between screens.
 * Note: This file does focus on working with a sever as background. Without server is shown in serverless.js.
 * Therefore we assume from here on out that we are talking with the use of a server.
 *
 * The dataset selection screen first has to load all available datasets and saved states from the webserver.
 * Then display these datasets where you can pick one. Then when picked it needs to react accordingly.
 * If there is one already loaded into the webserver, the selection screen will be skipped and the dataset will be
 * shown directly.
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

/**
 * String   currentUrl      contains the URL we are using at the moment.
 * String   mapsApiKey      contains the API key for the google maps windows. Doesn't work without it.
 */
// Hardcoded to allow for local testing...
let currentUrl = location.protocol + '//' + location.hostname + ':9000' + '/api/api';
let mapsApiKey = '';
let debugJsonOutput = false;
let chosenASavedStateToLoad = false;

// let currentUrl = location.protocol + '//' + location.hostname + (location.port ? ':' + location.port : '') + '/api/api';
$.urlParam = function(name){
    let results = new RegExp('[\?&]' + name + '=([^&#]*)').exec(window.location.href);
    if (results==null) {
        return null;
    }
    return decodeURI(results[1]) || 0;
};


/**
 * Loads the settings and setup the window to select the dataset or saved state.
 * If a saved state or dataset was already chosen, we go to the maps screen instantly.
 *
 * When the page is loaded, we get the settings. It first logs them. Then we set the mapsApiKey.
 * It can create the google maps interface now that we know the google maps api key.
 *
 * Note: The google maps interface is still hidden at this moment.
 */
$(document).ready(function () {
    // NProgress.start();

    if ($.urlParam('headless') !== null){
        headless = true;
        initWithoutServer();
        console.log('Initializing without server');
    } else {
        headless = false;
        initWithServer();
    }
});

function initWithServer(){
    // Radio checkboxes for selecting whether you want to pick a saved state or start with a data directory.
    $('.ui.radio.checkbox.check-data-or-state')
        .checkbox({
            onChange: function () {
                $chosenCheckboxValue = $('.ui.radio.checkbox.check-data-or-state.checked');
                if ($('.check-data-or-state-dataset').checkbox('is checked')) {
                    showAllChoosableDatasets();
                    hideOrShowWalkingDatasetOption('show');
                }
                if ($('.check-data-or-state-state').checkbox('is checked')){
                    showAllChoosableSavedStates();
                    hideOrShowWalkingDatasetOption('hide');
                }
            }
        });

    $('.check-data-or-state-state').checkbox('set checked');

    // Getting the required constants and then checking whether the dataset is set.
    // If the dataset is set, we open the maps screen, otherwise we show the different datasets and the checkboxes.
    getConstants();
}

function hideOrShowWalkingDatasetOption(string) {
    if (string === 'show'){
        $('.walkingDatasetCheckBox').show();
        $('.SavedStateCheckboxCalculateRoadMap').hide();
    } else if (string === 'hide'){
        $('.walkingDatasetCheckBox').hide();
        $('.SavedStateCheckboxCalculateRoadMap').show();
    } else {
        console.log("hideOrShowWalkingDatasetOption() called with incorrect parameter: ", string);
    }
}

/**
 * Shows all dataset entries.
 */
function showAllChoosableDatasets() {
    $('.content.a-saved-state-name').hide();
    $('.content.a-dataset-name').show();
}

/**
 * Shows all saved states entries.
 */
function showAllChoosableSavedStates() {
    $('.content.a-saved-state-name').show();
    $('.content.a-dataset-name').hide();
}

/**
 * Getting the required constants and then checking whether the dataset is set.
 * If the dataset is set, we open the maps screen, otherwise we show the different datasets and the checkboxes.
 *
 * If the webserver is not ready yet we get a net::ERR_CONNECTION_REFUSED.
 * In the case of an error we simply try this function again until it works.
 */

async function getConstants() {
    await $.getJSON(currentUrl + '/get_constants')
        .done(function (return_data) {
            if (debugJsonOutput) {
                console.log(return_data);
            }
            mapsApiKey = return_data['data']['googleMapsApiKey'];
        })
        .fail(function () {
            setTimeout(getConstants, 1000);
        });
    // Loads in the google maps script after which a function(initMap) is called which creates the map.
    await $.getScript("https://maps.googleapis.com/maps/api/js?v=weekly&key=" + mapsApiKey); // + "&callback=initMap");
    initMap();

    await $.getJSON(currentUrl + '/is_dataset_set')
        .then(function (return_data) {
            if (return_data['data'] === true){
                makeMapsScreenReady();
                loadAllObjects();
                getGroundTruth();
            } else {
                loadAllChoosableDatasets();
                loadAllChoosableSavedStates();
            }
        });
}

/**
 * Loads the dataset and makes the screen ready.
 *
 * It loads all possible datasets and creates a table like view of them after which they can be clicked.
 * Furthermore, it disables the loader and shows the actual screen.
 */
function loadAllChoosableDatasets() {
    $.getJSON(currentUrl + '/get_all_datasets')
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            let datasets = json['data'];
            datasets.sort(function (a, b){
                return compareStrings(a, b);
            });

            $datasetCollector = $('.path-card-collector');

            for (datasetCounter in datasets) {
                let dataset = datasets[datasetCounter];

                $divContainer = $('<div>')
                    .addClass('content a-dataset-name')
                    .text(dataset)
                    .on("click", function () {
                        chosenADataset($(this).text());
                    });
                $datasetCollector.append($divContainer);

            }
            showAllChoosableSavedStates();
            hideOrShowWalkingDatasetOption('hide');
            // $('#path-selector').show();
            // hideLoader();
        });
}

/**
 * Loads all saved states and create the elements.
 *
 * The actual elements are only shows when when the item is clicked
 */
function loadAllChoosableSavedStates() {
    $.getJSON(currentUrl + '/get_all_saved_states')
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            let datasets = json['data'];
            datasets.sort(function (a, b){
                return compareStrings(a, b);
            });

            $datasetCollector = $('.path-card-collector');

            for (let savedState in datasets) {
                let dataset = datasets[savedState];

                $divContainer = $('<div>')
                    .addClass('content a-saved-state-name')
                    .text(dataset)
                    .on("click", function () {
                        chosenASavedState($(this).text());
                    });
                $datasetCollector.append($divContainer);

            }
            showAllChoosableSavedStates();
            hideOrShowWalkingDatasetOption('hide');
            $('#path-selector').show();
            hideLoader();
        }).fail(e => console.error(e));
}

/**
 * Compares the two strings, used for sorting
 * @param a first string
 * @param b second string
 * @returns -1 if a should appear before b, 0 if the same, 1 if a should appear after b.
 */
function compareStrings(a, b) {
    a = a.toLowerCase();
    b = b.toLowerCase();
    return (a < b) ? -1 : (a > b) ? 1 : 0;
}

/**
 * Function to open a dataset in the back-end and open the loader on the screen.
 * @param dataset   String(path)    Path to the chosen dataset.
 */
function chosenADataset(dataset) {
    let url = currentUrl + '/choose_dataset?dataset_dir=' + dataset;

    let isWalking = $('.ui.checkbox.walkingDatasetCheckBox').checkbox('is checked');
    if (isWalking){
        url = url + '&enable_walking=true';
    } else {
        url = url + '&enable_walking=false';
    }

    chooseDatasetOrSavedStateRequest(url);
}

/**
 * Function which sets back a saved state in the back-end and open the loader on the screen.
 * @param saved_state the filename of the saved state.
 */
function chosenASavedState(saved_state) {
    let url = currentUrl + '/choose_saved_state?saved_state=' + saved_state;
    chooseDatasetOrSavedStateRequest(url);
    chosenASavedStateToLoad = true;
}

/**
 * This function does the actual setting of the dataset or the saved state.
 *
 * After it is set, we load the dataset.
 * @param url, the exact url to set the right dataset or state.
 */
function chooseDatasetOrSavedStateRequest(url) {
    makeMapsScreenReady();

    $.getJSON(url)
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            if (json['error'] === true) {
                if (!debugJsonOutput) {
                    console.log(json);
                }
                console.log('Choosing dataset failed with error message: ', json['errorMessage']);
            }

            loadAllObjects();
            getGroundTruth();
        });
}

/**
 * Loads all data present in the controller.
 *
 * Gets the dataset from the java webserver and executes a function to set the dataset and draw all the trajectories.
 */
function loadAllObjects() {
    if (chosenASavedStateToLoad && $('.SavedStateCheckboxCalculateRoadMap').checkbox('is checked')){
        computeRoadNetwork();
        chosenASavedStateToLoad = false;
        return;
    }
    $.getJSON(currentUrl + '/get_all_objects')
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            setTrajectoryDataset(json);
        });
}

/**
 * Loads the ground truth data of the current dataset, if available.
 */
function getGroundTruth() {
    $.getJSON(currentUrl + '/get_ground_truth')
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            setGroundTruthDataset(json);
        });
}

/**
 * Reset the application
 */
function resetApplication(){
    $.getJSON(currentUrl + '/reset_to_start')
        .done(function (json) {
            console.log(json);
            location.reload(true);
        });
}

/**
 * Enables the maps screen and opens the dataset.
 *
 * This function is executed after the dataset was selected.
 * - It changes the screen to the right container.
 * - Finally, loads the right dataset into the map view.
 */
function makeMapsScreenReady() {
    showLoader();

    $('#path-selector').hide();
    $('#container').show();
    $('#map > div').css('background-color', '#F7F7F7');

    // Enable the menu
    initializeMenu();
}


/**
 * Variable keeping track whether the loader is hidden.
 */
var __HIDDENLOADER__ = true;

/**
 * Hiding the loader by an animation of 0,4 seconds.
 */
function hideLoader(){
    if(__HIDDENLOADER__){
        $('#pre-loader').fadeOut(400, function(){
            $('#pre-loader').hide();
        });
        __HIDDENLOADER__ = false;
    }
}

/**
 * Shows the loader with an animation.
 */
function showLoader() {
    if (!__HIDDENLOADER__) {
        $('.loader').show();
        $('#pre-loader').show();
        __HIDDENLOADER__ = true;
    }
}
