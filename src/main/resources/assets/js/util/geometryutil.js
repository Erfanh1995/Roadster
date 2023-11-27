/**
 * Get's the points on a line
 * @param firstPoint, the starting point of the line
 * @param secondPoint, the ending point of the line
 * @param indexOnLine, the index on the line. 0 is all the way to firstPoint, 1 is all the way to secondPoint.
 * @returns {{x: number, y: number}}, a new coordinate.
 */
function getPointOnLine(firstPoint, secondPoint, indexOnLine) {
    let x = firstPoint['x'] * (1 - indexOnLine) + secondPoint['x'] * indexOnLine;
    let y = firstPoint['y'] * (1 - indexOnLine) + secondPoint['y'] * indexOnLine;
    return {'x': x,  'y': y}
}