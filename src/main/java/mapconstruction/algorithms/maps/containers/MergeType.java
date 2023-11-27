package mapconstruction.algorithms.maps.containers;

public enum MergeType {
    // DoubleIntersection is for BundleStreets in between two intersections
    // SingleIntersection is for BundleStreets connected to one intersection
    // Loner is for BundleStreets not associated with any intersections.
    DoubleIntersection, SingleIntersection, Loner
}
