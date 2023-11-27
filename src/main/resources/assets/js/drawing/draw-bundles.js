/**
 * This file is the layer between the table and the actual drawing.
 * In this file everything is related to: bundles.
 *
 * @author Jorrick Sleijster
 * @since 23/10/2018
 */

/**
 * If a certain bundle is selected, draw it with this function
 * @param bundleindices = index of bundles in the original query.
 */
function drawTableSelectedBundles(bundleindices) {
    if (enableTable) {
        for (let index of bundleindices) {
            let bundle = trajectoriesDataset['data']['bundles'][index];
            // console.log(trajectoriesDataset['data']['bundles'][index]);
            // console.log(bundle);

            let result = drawABundle(bundle['Bundle'], bundle['BundleClass']);
            removeSpecificTableDrawnBundle(index);
            drawnTableBundles[index] = result;
        }
    }
}

/**
 * Removes all table-chosen drawn bundles
 */
function removeTableDrawnBundles() {
    for (let item in drawnTableBundles) {
        for (let feature of drawnTableBundles[item]) {
            Layer.removeFeature(feature);
        }
    }
    drawnTableBundles = [];
}

/**
 * Remove a specific table-chosen drawn bundle
 * @param index, the index in trajectoriesDataset of the bundle to remove
 */
function removeSpecificTableDrawnBundle(index) {
    if (typeof drawnTableBundles[index] !== 'undefined' && drawnTableBundles[index] !== '') {
        let bundle = drawnTableBundles[index];
        for (let subtrajectory in bundle) {
            Layer.removeFeature(drawnTableBundles[index][subtrajectory]);
        }
        drawnTableBundles[index] = [];
    }
}

/**
 * On hovering a specific bundle from the table, it is drawn.
 * @param bundleIndex, the index in trajectoriesDataset of the bundle to draw
 */
function drawHoverBundle(bundleIndex) {
    let bundle = trajectoriesDataset['data']['bundles'][bundleIndex]['Bundle'];
    removeHover();
    drawnHoverBundle = drawABundle(bundle);
}

/**
 * Remove the drawn hover road map
 */
function removeHoverBundle() {
    if (drawnHoverBundle !== null) {
        for (let subtrajectory in drawnHoverBundle) {
            Layer.removeFeature(drawnHoverBundle[subtrajectory]);
        }
    }
    drawnHoverBundle = null;
}
