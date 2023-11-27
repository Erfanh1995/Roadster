let rectangle = null;


function initRectangle() {
    if (rectangle == null){
        rectangle = new google.maps.Rectangle({
            bounds: getTrajectoryBounds(),
            editable: true
        });
    }
    rectangle.setMap(map);
}

function showRectangle() {
    initRectangle();
}

function removeRectangle() {
    if (rectangle != null){
        rectangle.setMap(null);
    }
}

function resetRectanglePosition() {
    if (rectangle != null) {
        rectangle.setBounds(getTrajectoryBounds());
    }
}

function showRectanglePositions() {
    if (rectangle != null) {
        let bounds = rectangle.getBounds();
        const sw = bounds.getSouthWest();
        const ne = bounds.getNorthEast();
        let text = "The rectangle is at \n";
        let xy = convertLatLongtoUTM(sw);
        text += "SW: x:" + xy[0] + " y: " + xy[1] + "\n";
        xy = convertLatLongtoUTM(ne);
        text += "NE: x:" + xy[0] + " y: " + xy[1] + "\n \n";
        xy = convertLatLongtoUTM(sw);
        text += xy[0] + " " + xy[1] + " ";
        xy = convertLatLongtoUTM(ne);
        text += xy[0] + " " + xy[1] + " \n";
        console.log(text);
        alert(text);
    }
}

