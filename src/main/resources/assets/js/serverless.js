/**
 * This file contains the logic for working without the server and using the data in the datasets folder.
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */


let fileUrls = window.location.href.substring(0, window.location.href.lastIndexOf('/')) + "/datasets/";

async function initWithoutServer(){
    // Getting stuff like what your google maps api key is.
    console.log('Getting ', fileUrls + 'constants.json');
    await $.getJSON(fileUrls + 'constants.json')
        .done(function (return_data) {
            if (debugJsonOutput) {
                console.log(return_data);
            }
            mapsApiKey = return_data['data']['googleMapsApiKey'];
        });

    // Loads in the google maps script after which a function(initMap) is called which creates the map.
    console.log('Getting maps');
    await $.getScript("https://maps.googleapis.com/maps/api/js?key=" + mapsApiKey + "&callback=initMap");

    if ($.urlParam('dataset') != null){
        chosenAServerLessDataset($.urlParam('dataset'));
    } else {
        // Hiding all form options
        $('#firstPageForm').hide();
        $('.walkingDatasetCheckBox').hide();
        $('.SavedStateCheckboxCalculateRoadMap').hide();

        // Modify the text on the first page
        $('#firstPageHeader').text("Welcome! Please select which dataset you which to open.");
        $('#firstPageMeta').text("Note: all these datasets were pre-computed.");


        // Show the page
        loadAllServerlessChoosableDatasets();
    }
}

function loadAllServerlessChoosableDatasets(){
    console.log('Getting ', fileUrls + 'datasets.json');
    $.getJSON(fileUrls + 'datasets.json')
        .done(function (return_data) {
            $datasetCollector = $('.path-card-collector');

            let datasets = return_data['datasets'];
            for (let datasetIndex in datasets) {
                let dataset = datasets[datasetIndex];

                $divContainer = $('<div>')
                    .addClass('content a-dataset-name')
                    .text(dataset)
                    .attr('datasetID', dataset)
                    .on("dblclick", function () {
                        window.onbeforeunload = null;
                        console.log("Set window.location.href to :" + String( window.location.href ).replace( /#/, "" ) + '&dataset=' + $(this).attr('datasetID'));
                        window.location.href = String( window.location.href ).replace( /#/, "" ) + '&dataset=' + $(this).attr('datasetID');
                        // location.reload();
                        // chosenAServerLessDataset($(this).attr('datasetID'));
                    })
                    .on("click", function () {
                        var win = window.open(String( window.location.href ).replace( /#/, "" ) + '&dataset=' + $(this).attr('datasetID'), '_blank');
                    });
                $datasetCollector.append($divContainer);

            }
            $('#path-selector').show();
            hideLoader();
        });
}

function chosenAServerLessDataset(url){
    makeMapsScreenReady();
    loadAllServerlessObjects(url);
    getSeverlessGroundTruth(url);
}


/**
 * Loads all data present in the controller.
 *
 * Gets the dataset from the java webserver and executes a function to set the dataset and draw all the trajectories.
 */
function loadAllServerlessObjects(url) {
    $.getJSON(fileUrls + 'objects/'+url+'.json')
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            setTrajectoryDataset(json);
            postStatisticsAtComputeOption();
        });
}

/**
 * Loads the ground truth data of the current dataset, if available.
 */
function getSeverlessGroundTruth(url) {
    $.getJSON(fileUrls + 'groundTruth/'+url+'.json')
        .done(function (json) {
            if (debugJsonOutput) {
                console.log(json);
            }
            setGroundTruthDataset(json);
        });
}

/**
 * If we use the serverless interface, we automatically embed all the information of the trajectories on the compute tab
 */
function postStatisticsAtComputeOption() {
    $('#sidebar-compute').empty();
    addStatisticsToDiv($('#sidebar-compute'));
}