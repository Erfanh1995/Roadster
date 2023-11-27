function getNOTrajectories(){
    return _getTotalTrajectories(trajectoriesDataset['data']['original']);
}

function getNOFilteredTrajectories() {
    return _getTotalTrajectories(trajectoriesDataset['data']['filtered']);
}

function getNOVertices() {
    return _getTotalVertices(trajectoriesDataset['data']['original']);
}

function getNOFilteredVertices() {
    return _getTotalVertices(trajectoriesDataset['data']['filtered']);
}

function _getTotalTrajectories(dataset) {
    return dataset.length;
}

function _getTotalVertices(dataset) {
    let counter = 0;
    for (let index in dataset){
        let trajectory = dataset[index];
        if ('parent' in trajectory){
            let coordinates = parent['points'];
            for (let indexOfCoordinate in coordinates) {
                if (parseInt(indexOfCoordinate) === Math.floor(beginPoint)) {
                    counter++;
                } else if (parseInt(indexOfCoordinate) === Math.ceil(endPoint)) {
                    counter++;
                } else if ((indexOfCoordinate >= beginPoint && indexOfCoordinate <= endPoint) ||
                    (indexOfCoordinate <= beginPoint && indexOfCoordinate >= endPoint)) {
                    counter++;
                }
            }
        } else { 
            counter += trajectory['points'].length;
        }
    }
    return counter;
}

function statHelp() {
    console.log("Type getData() to get trajectory information of the dataset.")
}

function getData() {
    console.log("NO original trajectories: ", getNOTrajectories());
    console.log("NO filtered trajectories: ", getNOFilteredTrajectories());
    console.log("NO original vertices: ", getNOVertices());
    console.log("NO filtered vertices: ", getNOFilteredVertices());
    console.log(getNOTrajectories(), getNOFilteredTrajectories(), getNOVertices(), getNOFilteredVertices());
}

function addStatisticsToDiv(div) {
    div.append("NO original trajectories: " + getNOTrajectories() + "<br>");
    div.append("NO filtered trajectories: " + getNOFilteredTrajectories() + "<br>");
    div.append("NO original vertices: " + getNOVertices() + "<br>");
    div.append("NO filtered vertices: " + getNOFilteredVertices() + "<br>");
    div.append(getNOTrajectories() + " " + getNOFilteredTrajectories() + " " + getNOVertices() + " " + getNOFilteredVertices());
}