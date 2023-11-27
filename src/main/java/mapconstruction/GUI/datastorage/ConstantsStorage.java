package mapconstruction.GUI.datastorage;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * Class for keeping track of constants throughout the program.
 * Note that some constants are kept to prevent breaking the source code, even though those pieces of code are not used
 *
 * @author Jorrick Sleijster
 */
public enum ConstantsStorage {
    ALGOCONSTANTS;

    /* Bundling algorithm constants */
    /**
     * Number of threads we use to run our program
     */
    private int numThreads;

    /* Cut ending of representative constants */
    /**
     * Enable or disable cutting of the representative
     */
    private boolean enableCutOff;

    /**
     * Maximum orthogonal line length
     */
    private double cutEndOrthogonalLineLength;

    /* Regular force representative constants */
    /**
     * When there is no epsilon defined for the given bundle, we take this as maximum epsilon.
     */
    private double forceMaxEps;
    /**
     * Minimum distance between two points on the representative to be included in the force representative
     */
    private double forceMinDistancePoints;
    /**
     * Maximum allowed heading difference for another trajactory to apply force.
     */
    private double forceHeadingDifference;
    /**
     * Calculation of the orthogonals(through B) are done on AC. Minimum distance of A from B.
     */
    private double forceMinDistanceAB;

    /* Turn constants */
    /**
     * Turn minimum angle
     */
    private double turnMinAngle;
    /**
     * Maximum distance from sharpest angle to a turn to be considered part of the turn.
     */
    private double turnMaxDistanceSharpestAngle;
    /**
     * Maximum heading difference in the ending parts of the edges(that were considered part of a turn).
     */
    private double turnMaxDifferenceInEndHeadingAngles;
    /**
     * Scale difference between differenceEndingHeadingAngle and maxDistanceSharpestAngle.
     */
    private double turnScaleHeadingAngleVSDistance;
    /**
     * Minimum factor of the number of all subtrajectories that must be present in a turn for it to be a valid turn
     */
    private double turnMinRepresentedFactor;
    /**
     * Minimum absolute number of subtrajectories that must be present in a turn for it to be a valid turn
     */
    private double turnMinRepresentedAbsolute;
    /**
     * When we are lengthening the parts before and after the turn, what is the maximum angle that the heading angle
     * may differ.
     */
    private double turnLengtenMaxAngle;
    /**
     * When we are lengthening the parts before and after the turn, what is the maximum distance from the turn
     */
    private double turnLengtenMaxDistance;

    /* Map construction constants */
    /**
     * Distance for RoadSections with intersections on two endings to be considered mergeable with a road map.
     */
    private double smallMergeDistance;
    /**
     * Distance for RoadSections with intersections on one endings to be considered mergeable with a road map.
     */
    private double midMergeDistance;
    /**
     * Distance for RoadSections with intersections on none ending to be considered mergeable with a road map.
     */
    private double largeMergeDistance;
    /**
     * Distance for RoadSections with one point exception for one or two intersections on the roadSection ending.
     * // midMaxSinglePointSplitOffDistanceToReadEdge
     */
    private double midMaxSinglePointDistance;
    /**
     * Distance for RoadSections with one point exception for zero intersection on the roadSection endings.
     * // midMaxSinglePointSplitOffDistanceToReadEdge
     */
    private double largeMaxSinglePointDistance;
    /**
     * Max Heading direction difference in the RoadNetwork
     */
    private double maxMergingHeadingDirectionDifference;
    /**
     * Merge RoadSection (or Connection part) with intersection. Max distance we backtrack on trajectory to merge.
     * (Meaning, we look at the closest point and from there we can traverse at most x meters back and set that as last
     * point to connect to the intersection the next point)
     */
    private double maxRoadSectionMergeWithIntDistance;
    /**
     * The minimum distance of the other end of the roadSection of which we are not allowed to 'steal' points. Meaning,
     * that part will always remain untouched by the merging.
     */
    private double minRoadSectionMergeRemainingOppositeLength;
    /**
     * The maximum error or (Ramer-Douglas-Peucker distance) allowed in between the newly created edge and all points
     * that we propose to skip by taking a later point. (That later point with the intersection creates the edge).
     */
    private double maxRoadSectionMergeErrorDistanceToNewlyCreatedEdge;
    /**
     * Maximum merging angle difference when comparing the extended(infinite) edge we are currently at compared to the
     * edge from the endpoint to our intersection. (Simply said, the error angle we get by extending this edge to the
     * intersection)
     */
    private double maxRoadSectionMergeAngleDifference;
    /**
     * Minimum bundle street length to be considered valid
     */
    private double minBundleStreetLength;

    /**
     * Whether all map construction related constants were set correctly.
     */
    private boolean allMapConstructionConstantsAreSet;


    ConstantsStorage() {
        // Bundling algorithm constants
        numThreads = Runtime.getRuntime().availableProcessors();

        // Cut off constant
        enableCutOff = true;

        // Force related constants
        forceMaxEps = 200;
        forceMinDistancePoints = 10.0;
        forceHeadingDifference = 60.0;
        forceMinDistanceAB = 25;

        // Turn related constants
        turnMinAngle = 45.0;
        turnMaxDistanceSharpestAngle = 100; // To fit both very bad as precise datasets.
        turnMaxDifferenceInEndHeadingAngles = 60; // To allow 30 degrees on both sides.
        turnScaleHeadingAngleVSDistance = 1.0;
        turnMinRepresentedFactor = 0.5; // At least half should be in a turn.
        turnMinRepresentedAbsolute = 3.0; // At least 3 seems to be a correct number to filter out noise.
        turnLengtenMaxAngle = 25;
        turnLengtenMaxDistance = 100;

        // Cut ending constants
        // Map related constants
        allMapConstructionConstantsAreSet = false;
        setMapConstructionRelatedConstants();
    }

    private void setMapConstructionRelatedConstants(){
        if (allMapConstructionConstantsAreSet || STORAGE.getDatasetConfig() == null){
            return;
        }

        if (STORAGE.getDatasetConfig().isWalkingDataset()){
            // Cut end constants
            cutEndOrthogonalLineLength = 40;

            // Walking dataset properties
            // Map construction related
            minBundleStreetLength = 15;
//
            smallMergeDistance = 20.0;
            midMergeDistance = 30.0;
            largeMergeDistance = 40.0;

            midMaxSinglePointDistance = 50.0;
            largeMaxSinglePointDistance = 60.0;

            // Testing
//            smallMergeDistance = 30.0;
//            midMergeDistance = 40.0;
//            largeMergeDistance = 50.0;
//
//            midMaxSinglePointDistance = 50.0;
//            largeMaxSinglePointDistance = 70.0;
//
            // Merging intersection and RoadSection related
            maxRoadSectionMergeWithIntDistance = 30;
            minRoadSectionMergeRemainingOppositeLength = 15;
            maxRoadSectionMergeErrorDistanceToNewlyCreatedEdge = 15;
            maxRoadSectionMergeAngleDifference = 50;

        } else {
            // Cut end constants
            cutEndOrthogonalLineLength = 100;

            // Car dataset properties
            // Map construction related
            minBundleStreetLength = 50;

            smallMergeDistance = 30.0;
            midMergeDistance = 50.0;
            largeMergeDistance = 75.0;

            midMaxSinglePointDistance = 60.0;
            largeMaxSinglePointDistance = 85.0;

            // Merging intersection and RoadSection related
            maxRoadSectionMergeWithIntDistance = 75;
            minRoadSectionMergeRemainingOppositeLength = 50;
            maxRoadSectionMergeErrorDistanceToNewlyCreatedEdge = 50;
            maxRoadSectionMergeAngleDifference = 25;
        }
        maxMergingHeadingDirectionDifference = 35.0;

        // They are set.
        allMapConstructionConstantsAreSet = true;
    }
    
    public int getNumThreads() {
        return numThreads;
    }

    public void setNumThreads(int numThreads) {
        this.numThreads = numThreads;
    }
    
    public double getForceMaxEps() {
        return forceMaxEps;
    }

    public double getForceMinDistancePoints() {
        return forceMinDistancePoints;
    }

    public double getForceHeadingDifference() {
        return forceHeadingDifference;
    }

    public double getForceMinDistanceAB() {
        return forceMinDistanceAB;
    }
    public double getTurnMinAngle() {
        return turnMinAngle;
    }

    public double getTurnMaxDistanceSharpestAngle() {
        return turnMaxDistanceSharpestAngle;
    }

    public double getTurnMaxDifferenceInEndHeadingAngles() {
        return turnMaxDifferenceInEndHeadingAngles;
    }

    public double getTurnScaleHeadingAngleVSDistance() {
        return turnScaleHeadingAngleVSDistance;
    }

    public double getTurnMinRepresentedFactor() {
        return turnMinRepresentedFactor;
    }

    public double getTurnMinRepresentedAbsolute() {
        return turnMinRepresentedAbsolute;
    }

    public double getTurnLengtenMaxAngle() {
        return turnLengtenMaxAngle;
    }

    public double getTurnLengtenMaxDistance() {
        return turnLengtenMaxDistance;
    }

    public double getSmallMergeDistance() {
        setMapConstructionRelatedConstants();
        return smallMergeDistance;
    }

    public double getMidMergeDistance() {
        setMapConstructionRelatedConstants();
        return midMergeDistance;
    }

    public double getLargeMergeDistance() {
        setMapConstructionRelatedConstants();
        return largeMergeDistance;
    }

    public double getMidMaxSinglePointDistance() {
        setMapConstructionRelatedConstants();
        return midMaxSinglePointDistance;
    }

    public double getLargeMaxSinglePointDistance() {
        setMapConstructionRelatedConstants();
        return largeMaxSinglePointDistance;
    }

    public double getMaxMergingHeadingDirectionDifference() {
        setMapConstructionRelatedConstants();
        return maxMergingHeadingDirectionDifference;
    }

    public double getMaxRoadSectionMergeWithIntDistance() {
        setMapConstructionRelatedConstants();
        return maxRoadSectionMergeWithIntDistance;
    }

    public double getMinRoadSectionMergeRemainingOppositeLength() {
        setMapConstructionRelatedConstants();
        return minRoadSectionMergeRemainingOppositeLength;
    }

    public double getMaxRoadSectionMergeErrorDistanceToNewlyCreatedEdge() {
        setMapConstructionRelatedConstants();
        return maxRoadSectionMergeErrorDistanceToNewlyCreatedEdge;
    }

    public double getMaxRoadSectionMergeAngleDifference() {
        setMapConstructionRelatedConstants();
        return maxRoadSectionMergeAngleDifference;
    }

    public double getCutEndOrthogonalLineLength() {
        setMapConstructionRelatedConstants();
        return cutEndOrthogonalLineLength;
    }

    public double getMinBundleStreetLength() {
        setMapConstructionRelatedConstants();
        return minBundleStreetLength;
    }

    public boolean isEnableCutOff() {
        return enableCutOff;
    }

    public void setEnableCutOff(boolean enableCutOff) {
        this.enableCutOff = enableCutOff;
    }
}
