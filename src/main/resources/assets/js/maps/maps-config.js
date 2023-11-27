/**
 * This file contains the functionality of making changes to things on or around the map.
 * This includes toggling the google maps background, the color,
 *
 * Note: Changing actual properties of the map itself, making the map silver, is done in maps-styles.
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */

/**
 * Disables the OSM tiles and shows a uniform background color.
 */
function disableMapTypes(){
    tileLayer.setVisible(false);
    // $('#map').addClass('disable-tiles');
    $('#map > div').css('background', 'white');
}

/**
 * Enables the OSM tiles back again.
 */
function enableMapTypes(){
    tileLayer.setVisible(true);
    // $('#map').removeClass('disable-tiles');
}

/**
 * Set the background color.
 *
 * When the Google Maps tiles are hidden, the background color is shown.
 * @param color, a css interpretable color
 */
function changeMapBackgroundColor(color) {
    $('#map > div').css('background-color', color);
}

/**
 * Changes the color of all the standard drawn trajectories.
 * @param color, a css interpretable color
 */
function changeStandardTrajectoryColor(color) {
    propStandardTrajectory.color = color;
    drawAllStandardTrajectories();
}

/**
 * Changes the color of the trajectories select in the table.
 * @param color, a css interpretable color
 */
function changeTableTrajectoryColor(color) {
    propTableTrajectory.color = color;
    executeTableDraws();
}

/**
 * Changes the color of the trajectories select in the table.
 * @param color, a css interpretable color
 */
function changeHoverTrajectoryColor(color) {
    propHoverTrajectory.color = color;
}


