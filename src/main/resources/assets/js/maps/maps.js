var map;

function initMap() {
    map = new google.maps.Map(document.getElementById('map'), {
        center: {lat: -34.397, lng: 150.644},
        zoom: 8,
        zoomControl: false,
        mapTypeControl: false,
        scaleControl: true,
        streetViewControl: false,
        rotateControl: true,
        fullscreenControl: true
    });

    // Make sure the zoom updates with UI.
    map.addListener('zoom_changed', function() {
        updateZoom(map.getZoom(), true, false);
    });

    // We update the map settings
    updateMap();
    initRectangle();
}

window.onbeforeunload = function() {
    return "Dude, are you sure you want to leave? This might take you forever!";
};