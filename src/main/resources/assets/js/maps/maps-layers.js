/**
 * This file is responsible for managing the different drawing layers of the map
 * @author Jorren Hendriks
 * @since  24/04/2020
 */

let _layers = {};

let Layer = {
    DEFAULT: 5,
    TRAJECTORY: 10,
    SUBTRAJECTORY: 5,
    BUNDLE: 20,
    ROADMAP: 30,
    MERGED_REP: 35,
    INTERSECTION: 40,
    CONNECTION: 50,
    MARKER: 100,
    GROUND_TRUTH: 0,

    /**
     * Get the appropriate layer for the given index or create a new layer if it does not exist.
     * @param index The index of the returned layer.
     */
    getLayerSource: function(index) {
        if (!_layers.hasOwnProperty(index)) {
            _layers[index] = new ol.layer.Vector({
                source: new ol.source.Vector(),
                style: VectorStyleManager,
                zIndex: index
            });
            map.addLayer(_layers[index]);
        }

        return _layers[index].getSource();
    },
    /**
     * Remove a feature from its layer. If the feature has a 'layer' value, this layer is used for removal. Otherwise if
     * the layer parameter is present, the corresponding layer will be used for removal. If neither of these two is
     * supplied, a warning is shown.
     * @param feature The feature to remove
     * @param layer (optional) The layer to remove it from
     */
    removeFeature: function(feature, layer = null) {
        if (feature.hasOwnProperty("values_") && feature.values_.hasOwnProperty("layer")) {
            this.getLayerSource(feature.values_.layer).removeFeature(feature);
        } else if (layer !== null) {
            this.getLayerSource(layer).removeFeature(feature);
        } else {
            console.warn("Tried to remove a feature without known layer: " + feature);
        }
    }
};