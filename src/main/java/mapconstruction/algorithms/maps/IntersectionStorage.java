package mapconstruction.algorithms.maps;

import mapconstruction.algorithms.maps.intersections.containers.Intersection;

import java.util.ArrayList;
import java.util.List;

/**
 * Singleton instance storing the relevant data for the intersection calculations
 *
 * @author Jorrick Sleijster
 */
public enum IntersectionStorage {
    INTERSECTION_STORAGE;

    /**
     * List of all intersections
     */
    private List<Intersection> intersections;


    IntersectionStorage(){
        intersections = new ArrayList<>();
    }


    public List<Intersection> getIntersections() {
        return intersections;
    }

    public void setIntersections(List<Intersection> intersections) {
        this.intersections = new ArrayList<>(intersections);
    }

    public int getIntersectionClassByIntersection(Intersection intersection){
        return this.intersections.indexOf(intersection);
    }

    public Intersection getIntersectionByClass(int intersectionClass){
        return this.intersections.get(intersectionClass);
    }
}
