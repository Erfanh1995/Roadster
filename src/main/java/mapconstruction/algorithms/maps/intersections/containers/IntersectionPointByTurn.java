package mapconstruction.algorithms.maps.intersections.containers;

import com.fasterxml.jackson.annotation.JsonIgnore;
import mapconstruction.algorithms.representative.containers.Turn;
import mapconstruction.trajectories.Bundle;
import mapconstruction.util.GeometryUtil;

import java.awt.geom.Point2D;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * DataClass for an IntersectionPoint created by a turn.
 *
 * @author Jorrick
 * @since 06/11/2018
 */

public class IntersectionPointByTurn extends IntersectionPoint {

    /**
     * All the turns involved
     */
    private List<Turn> turns;

    /**
     * The little part just before and after a turn.
     */
    private Set<Bundle> beforeTurn;
    private Set<Bundle> afterTurn;

    /**
     * Initializes the IntersectionPointByTurn
     * @param longBundle1, the first bundle of the pair
     * @param longBundle2, the second bundle of the pair
     * @param overlappingBundle, an overlapping bundle of the pair (or at least a try to get one..)
     * @param turns, the turns that are included in this intersectionPointByTurn.
     */
    public IntersectionPointByTurn(Bundle longBundle1, Bundle longBundle2, Bundle overlappingBundle, List<Turn> turns) {
        super(longBundle1, longBundle2, overlappingBundle);
        this.turns = turns;
    }

    /**
     * We calculate the average location of this intersectionPoint.
     * @return the location
     */
    @JsonIgnore
    Point2D calculateLocation() {
        List<Point2D> turnLocations = turns.stream().map(Turn::getTurnLocation).collect(Collectors.toList());
        return GeometryUtil.getAverage(turnLocations);
    }

    /**
     * Get's all bundles connected to the IntersectionPoint
     * @return all bundles around the IntersectionPoint
     */
    @JsonIgnore
    public Set<Bundle> getAllBundlesConnectedToIntersectionPoint(){
        Set<Bundle> bundles = new HashSet<>();
        bundles.add(getLongBundle1());
        bundles.add(getLongBundle2());
        bundles.add(getOverlappingBundle());

        for (Turn turn: turns){
            bundles.add(turn.getBundle());
        }

        return bundles;
    }

    /**
     * Add the part just before and after the turn to this point.
     * @param beforeTurn, the part just before a turn
     * @param afterTurn, the part just after a turn
     */
    public void setBundlesBeforeAndAfterTurn(Set<Bundle> beforeTurn, Set<Bundle> afterTurn) {
        this.beforeTurn = beforeTurn;
        this.afterTurn = afterTurn;
    }

    /**
     * Get the number of turns that are part of this IntersectionPoint
     * @return the number of turns part of this IntersectionPoint
     */
    public int getNumberOfTurnsIncluded(){
        return turns.size();
    }

    /**
     * Gets all turns that are part of this IntersectionPoint.
     * @return all the turns included
     */
    @JsonIgnore
    public List<Turn> getAllTurns(){
        return turns;
    }
}
