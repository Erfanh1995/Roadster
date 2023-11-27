
let allKMLLayers = [];

let KMLLayersURL = window.location.href.substring(0, window.location.href.lastIndexOf('/')) + "/resultsRelatedWork/";
let remoteURL = 'https://fellenoord.nl/random/jorrick/';
function addLocalKMLLayer(localUrl) {
    addKMLLayer(KMLLayersURL + localUrl)
}

function addRemoteKMLLayer(remoteUrl) {
    addKMLLayer(remoteURL + remoteUrl)
}

function addKMLLayer(url) {
    var kmlLayer = new google.maps.KmlLayer(url, {
        suppressInfoWindows: true,
        preserveViewport: true,
        map: map,
        zindex: 20,
        clickable : false
    });
    allKMLLayers[allKMLLayers.length] = kmlLayer;
}

function removeKMLLayers() {
    for (let index in allKMLLayers){
        allKMLLayers[index].setMap(null);
    }
    allKMLLayers = [];
}


function getKMLString() {
    var xw = new XMLWriter('UTF-8');
    xw.formatting = 'indented'; //add indentation and newlines
    xw.indentChar = ' '; //indent with spaces
    xw.indentation = 2; //add 2 spaces per level

    xw.writeStartDocument();
    xw.writeStartElement('kml');
    xw.writeAttributeString("xmlns", "http://www.opengis.net/kml/2.2");
    xw.writeStartElement('Document');
    xw.writeStartElement('Folder');
    xw.writeStartElement('Placemark');
    xw.writeStartElement('MultiGeometry');

    // xw.writeStartElement('Placemark');
    // xw.writeStartElement('name');
    // xw.writeCDATA("Buchin-" + trajectoriesDataset['data']['settings']['path']);
    // xw.writeEndElement();
    // xw.writeStartElement('description');
    // xw.writeCDATA("Created by Jorrick Sleijster");
    // xw.writeEndElement();
    // xw.writeEndElement(); // END PlaceMarker

    for (let i in drawnTableRoads) {
        for (let j in drawnTableRoads[i]) {
            // console.log(drawnTableRoads[i][j].getPath());

            xw.writeStartElement('LineString');
            xw.writeElementString('tessellate', '1');
            xw.writeElementString('altitudeMode', 'clampToGround');
            xw.writeStartElement("coordinates");
            for (var k = 0; k < drawnTableRoads[i][j].getPath()['j'].length; k++) {
                xw.writeString(drawnTableRoads[i][j].getPath()['j'][k].lng() + "," + drawnTableRoads[i][j].getPath()['j'][k].lat() + ",0");
            }
            xw.writeEndElement();
            xw.writeEndElement();

        }
    }

    xw.writeEndElement();
    xw.writeEndElement();
    xw.writeEndElement();
    xw.writeEndElement();
    xw.writeEndElement();
    xw.writeEndDocument();

    var xml = xw.flush(); //generate the xml string
    xw.close(); //clean the writer
    xw = undefined; //don't let visitors use it, it's closed
    //set the xml
    console.log(xml);

}

//
// Add this to the beginning!!!!
//
// <?xml version="1.0" encoding="utf-8" standalone="yes"?>
// <kml xmlns="http://www.opengis.net/kml/2.2">
//     <Document>
//         <name><![CDATA[athens_small_ahmed]]></name>
//         <visibility>1</visibility>
//         <open>1</open>
//         <Snippet><![CDATA[created using <a href="http://www.gpsvisualizer.com/?ref=ge&time=20190606083540">GPS Visualizer</a>]]></Snippet>
//         <Style id="gv_track">
//             <LineStyle>
//                 <color>ff0000e6</color>
//                 <width>4</width>
//             </LineStyle>
//             <BalloonStyle>
//                 <text><![CDATA[<div style="font-family:Arial,sans-serif; min-width:200px;"><h3>$[name]</h3> <div style="margin-top:8px;">$[description]</div></div>]]></text>
//             </BalloonStyle>
//         </Style>
//         <Folder id="Tracks">
//             <name>Tracks</name>
//             <visibility>1</visibility>
//             <open>0</open>
//             <Placemark>
//                 <name><![CDATA[Graph]]></name>
//                 <Snippet></Snippet>
//                 <description><![CDATA[&nbsp;]]></description>
//                 <styleUrl>#gv_track</styleUrl>
//                 <Style>
//                     <LineStyle>
//                         <color>ff0000ff</color>
//                         <width>5</width>
//                     </LineStyle>
//                 </Style>
//                 <MultiGeometry>