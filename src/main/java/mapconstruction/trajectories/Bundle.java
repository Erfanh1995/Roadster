package mapconstruction.trajectories;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.common.base.Preconditions;
import mapconstruction.algorithms.representative.*;
import mapconstruction.algorithms.representative.containers.OrthogonalIntersection;
import mapconstruction.algorithms.representative.containers.Turn;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.LinearRegression;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;

import static mapconstruction.GUI.datastorage.DataStorage.STORAGE;

/**
 * Class representing a bundle.
 * <p>
 * A bundle is a set of subtrajectories.
 * <p>
 * A representativeSubtrajectory can be assigned, but it not considered for equality.
 *
 * @author Roel
 */
public class Bundle implements Serializable {

    private static final long serialVersionUID = 1L;
    /**
     * Set of subtrajectories in this bundle
     */
    private final Set<Subtrajectory> trajectories;

    /**
     * Mapping of subtrajectories by their id
     */
    private final Map<String, Subtrajectory> namedTrajectories;
    /**
     * Remember hashcode.
     */
    private final int hashcode;
    /**
     * Whether we want to report all the force related bundle properties to the JSON or only the required once.
     * By setting this to False, we decrease the size of the JSON, which is of course preferred.
     */
    private final boolean debugBundleForceProperties;
    /**
     * Whether we want to report all the turn related bundle properties to the JSON or only the required once.
     * By setting this to False, we decrease the size of the JSON, which is of course preferred.
     */
    private final boolean debugBundleTurnProperties;
    /**
     * Representative trajectory. Null if no representativeSubtrajectory is specified.
     */
    protected Subtrajectory representativeSubtrajectory;
    /**
     * At a certain point we cut off the bundle of the bundle representative, here we keep track if that was already done.
     */
    private boolean bundleRepCutOff;
    private Subtrajectory median;
    /**
     * These variables keep track of everything related to the unmerged force representativeSubtrajectory, UFR.
     */
    private List<Point2D> listOfACLines;  // Keeps track of which AC lines were used to create the perpendicular line on.       # storing intermediate results
    private List<Point2D> listOfPerpendicularLines;  // Keeps track of on which line the force steps were executed              # storing intermediate results
    private List<List<Point2D>> listOfForceSteps;  // Keeps track of the actual force steps executed (steps of 1 meter).        # storing intermediate results
    private List<List<OrthogonalIntersection>> listOfIntersectionsWithAngles;  // List of intersections with subtrajectories    # storing intermediate results
    private List<Point2D> unmergedForceRepresentative;  // The force representativeSubtrajectory without merged turns.          # this is used in turn detection
    private Map<String, Object> unmergedForceLinearRegressionDict;  // Contains all information about the LR of the UFR.        # storing statistics
    private Map<String, Object> unmergedForceStraightnessDict;  // Contains information of how straight the UFR is.             # storing statistics
    /**
     * These variables keep track of everything related to turns and turn finding.
     */
    private List<Point2D> singlePointSharpTurns;  // Single points sharp turns                  # intermediate variable
    private List<List<Map<String, Object>>> multiPointSharpTurns;  // Multi point sharp turns   # intermediate variable
    private List<Turn> turns;  // Actual turns                                                  # required for computation using getAllTurns()
    private List<Turn> sortedTurns;  // Sorted the turns based on occurrence close to the UFR.  # value of JSON representation
    private List<Point2D> mergedForceRepresentative;  // Merged the UFR with turns.             # used in getRepresentative()

    /**
     * The newly created representative
     */
    private Representative representative;

    /**
     * These variables are used for checking whether a bundle's end is considered a roadPoint.
     */
    private Integer bundleEndsAreRoadPoints;

    /**
     * Creates a bundle with the given collection of subtrajectories.
     *
     * @param trajectories trajectories in the bundle.
     * @throws NullPointerException if {@code trajectories == null}
     */
    protected Bundle(Collection<Subtrajectory> trajectories) {
        this(trajectories, null);
    }

    /**
     * Creates a bundle with the given collection of subtrajectories and the
     * given representativeSubtrajectory.
     *
     * @param trajectories                trajectories in the bundle.
     * @param representativeSubtrajectory representativeSubtrajectory trajectory.
     * @throws NullPointerException if {@code trajectories == null}
     */
    protected Bundle(Collection<Subtrajectory> trajectories, Subtrajectory representativeSubtrajectory) {
        Preconditions.checkNotNull(trajectories, "trajectories == null");
        this.trajectories = new HashSet<>(trajectories);
        // assume each representative can only occur once in a bundle
        this.namedTrajectories = new HashMap<>();
        for (Subtrajectory t : trajectories) {
            namedTrajectories.put(t.getParent().getUndirectionalLabel(), t);
        }

        this.representativeSubtrajectory = representativeSubtrajectory;
        hashcode = computeHashCode();

        this.debugBundleForceProperties = false;
        this.debugBundleTurnProperties = false;

        this.bundleRepCutOff = false;
    }

    /**
     * Creates a bundle with the given collection of subtrajectories.
     *
     * @param trajectories trajectories in the bundle.
     * @return
     * @throws NullPointerException if {@code trajectories == null}
     */
    public static Bundle create(Collection<Subtrajectory> trajectories) {
        return new Bundle(trajectories);
    }

    /**
     * Creates a bundle with the given collection of subtrajectories and the
     * given representativeSubtrajectory.
     *
     * @param trajectories   trajectories in the bundle.
     * @param representative representativeSubtrajectory trajectory.
     * @return
     * @throws NullPointerException if {@code trajectories == null}
     */
    public static Bundle create(Collection<Subtrajectory> trajectories, Subtrajectory representative) {
        return new Bundle(trajectories, representative);
    }

    public Bundle newInstance(Collection<Subtrajectory> trajectories, Subtrajectory representative) {
        return create(trajectories, representative);
    }

    /**
     * Returns a unmodifiable view of the subtrajectories in this bundles
     *
     * @return all subtrajectories in a unmodifiable view
     */
    @JsonProperty
    public Set<Subtrajectory> getSubtrajectories() {
        return Collections.unmodifiableSet(this.trajectories);
    }

    /**
     * Returns a set of all non-reversed subtrajectories in this bundle.
     */
    @JsonIgnore
    public Set<Subtrajectory> getNonReverseSubtrajectories() {
        HashSet<Subtrajectory> subs = new HashSet<>();
        this.trajectories.forEach(
                t -> {
                    if (t.isReverse()) {
                        t = t.reverse();
                    }
                    subs.add(t);
                }
        );
        return subs;
    }

    /**
     * Returns a set of all subtrajectories in this bundle within the range of the subtrajectory of the representative
     *
     * @param subRep a subtrajectry of the representative, given a range of what length the subs should have.
     * @return a set of all subtrajectories with their new ranges.
     */
    @JsonIgnore
    public Set<Subtrajectory> getSubtrajectoriesForRange(Subtrajectory subRep) {
        HashSet<Subtrajectory> subs = new HashSet<>();
        this.trajectories.forEach(
                t -> subs.add(GeometryUtil.cutOffSubtrajectoryByRepresentativeRange(subRep, t))
        );
        return subs;
    }

    /**
     * Returns a set of all non-reverse subtrajectories in this bundle within the range of the subtrajectory of the
     * representative
     *
     * @param subRep a subtrajectry of the representative, given a range of what length the subs should have.
     * @return a set of all non-reverse subtrajectories with their new ranges.
     */
    @JsonIgnore
    public Set<Subtrajectory> getNonReverseSubTrajectoriesForRange(Subtrajectory subRep) {
        Set<Subtrajectory> subs = getSubtrajectoriesForRange(subRep);
        return GeometryUtil.convertSubsIntoNonReverseSubs(subs);
    }

    /**
     * Returns a set of all full trajectories that contain a subtrajectory in this bundle.
     *
     * @return the parent trajectories
     */
    @JsonIgnore
    public Set<Trajectory> getParentTrajectories() {
        HashSet<Trajectory> full = new HashSet<>();
        trajectories.forEach(t -> full.add(t.getParent()));
        return full;
    }

    /**
     * Returns a set of all full non-reversed trajectories that contain a subtrajectory in this bundle.
     *
     * @return the parent trajectories
     */
    @JsonIgnore
    public Set<Trajectory> getParentNonReverseTrajectories() {
        HashSet<Trajectory> full = new HashSet<>();
        trajectories.forEach(
                t -> {
                    if (t.isReverse()) {
                        t = t.reverse();
                        full.add(t.getParent());
                    } else {
                        full.add(t.getParent());
                    }
                }
        );
        return full;
    }

    /**
     * Here we edit the representative subtrajectory. This is done because the
     *
     * @param sub the same trajectory but with tighter bounds.
     */
    public void setNewRepresentativeSubtrajectory(Subtrajectory sub) {
        if (!sub.getParent().equals(representativeSubtrajectory.getParent())) {
            System.out.println("Error Bundle.setNewRepresentativeSubtrajectory. Setting different sub!?!");
            return;
        }
        trajectories.remove(representativeSubtrajectory);
        trajectories.add(sub);
        representativeSubtrajectory = sub;
    }

    /**
     * Size of the bundle.
     *
     * @return Number of sub trajectories in this bundle.
     */
    @JsonProperty
    public int size() {
        return trajectories.size();
    }

    /**
     * Discrete length of the bundle. That is, the maximum number of vertices on
     * a trajectory in this bundle.
     *
     * @return the maximum number of vertices on a trajectory in this bundle.
     */
    @JsonProperty
    public int discreteLength() {
        return trajectories.stream()
                .mapToInt(Subtrajectory::numPoints) // Get  of points for each trajectory
                .max() // get maxiumum
                .orElse(0); // 0 if the bundle is empty
    }

    /**
     * Continuous length of the bundle
     *
     * @return The euclidean length of the longest trajectory in the bundle.
     */
    @JsonProperty
    public double continuousLength() {
        return trajectories.stream()
                .mapToDouble(Subtrajectory::euclideanLength) // Get length for each trajectory
                .max() // get maxiumum
                .orElse(0); // 0 if the bundle is empty
    }

    /**
     * Determines whether this bundle covers the given bundle
     *
     * @param other     bundle to compare to
     * @param checkSize whether to enforce that the size of the given bundle
     *                  must be at most the size of this bundle.
     * @return {@code true} if {@code this} covers {@code other}, {@code false}
     * otherwise
     * @throws NullPointerException if {@code other == null}.
     */
    public boolean covers(Bundle other, boolean checkSize) {
        Preconditions.checkNotNull(other, "other == null");
        // This bundle must be at least the size as other
        if (checkSize && this.size() < other.size()) {
            return false;
        }

        // For all of the trajectories T1 in other
        // we have to find a matching T2 trajectory in this
        // meaning that
        // T1 is a subtrajectory of T2
        return other.getSubtrajectories().stream()
                .allMatch(t1 // forall T1
                        -> this.trajectories.stream()
                        .anyMatch(t2 // Exists T2
                                -> trajectoryHasAsSubtrajectory(t2, t1))
                );
    }

    /**
     * Determines whether this bundle covers the given bundle.
     * <p>
     * Enforces that the size of the given bundle must be at most the size of
     * this bundle.
     *
     * @param other bundle to compare to
     * @return {@code true} if {@code this} covers {@code other}, {@code false}
     * otherwise
     * @throws NullPointerException if {@code other == null}.
     */
    public boolean covers(Bundle other) {
        return covers(other, true);
    }

    /**
     * Whether the trajectory sup has sub as subtrajectory
     *
     * @param sup
     * @param sub
     * @return
     * @throws NullPointerException if either subtrajectory is {@code null}.
     */
    protected boolean trajectoryHasAsSubtrajectory(Subtrajectory sup, Subtrajectory sub) {
        Preconditions.checkNotNull(sup);
        Preconditions.checkNotNull(sub);
        return sup.hasAsSubtrajectory(sub);
    }

    /**
     * Determine whether this bundle has the given bundle as (weak) subbundle.
     *
     * @param other
     * @return {@code true} if {@code this} has {@code other} as subbundle.
     * @throws NullPointerException if {@code other == null}.
     */
    public boolean hasAsSubBundle(Bundle other) {
        return this.covers(other);
    }

    /**
     * Get a subtrajectory which matches with the given name.
     *
     * @param name
     * @return {@code Subtrajectory} or {@code null} if not present.
     */
    public Subtrajectory getTrajectoryByName(String name) {
        return namedTrajectories.get(name);
    }

    /**
     * This is the original representativeSubtrajectory.
     *
     * @Deprecated kept to avoid breakage with future code. Remove when mapping algorithm is implemented.
     */
    @JsonProperty
    public Subtrajectory getOriginalRepresentative() {
        return representativeSubtrajectory;
    }

    /**
     * Gets the median of a trajectory.
     * Old code of Roel. Never used because it showed bad results.
     *
     * @return a subtrajectory
     */
    @JsonIgnore
    public Subtrajectory getMapRepresentative() {
        if (median == null)
            median = Median.representativeTrajectory(trajectories, representativeSubtrajectory.getFirstPoint(), representativeSubtrajectory.getLastPoint());
        return median;
    }

    @JsonIgnore
    public List<Point2D> getRepresentativePolyline() {
        return mergedForceRepresentative;
    }

    @JsonProperty
    public Representative getRepresentative() {
        if (representative == null) {
            if (STORAGE.getDatasetConfig().isWalkingDataset()){
                // No turns in our representative
                representative = new Representative(unmergedForceRepresentative, this);
            } else {
                // Representative with merged forces in there.
                representative = new Representative(mergedForceRepresentative, this);
            }
        }
        return representative;
    }

    /**
     * Calculates all the properties required.
     */
    @JsonIgnore
    public void calculateForceProperties() {
        if (unmergedForceRepresentative == null || unmergedForceRepresentative.size() == 0) {
            calculateForceRelatedProperties();
        }
    }
    @JsonIgnore
    public void calculateTurnProperties(){
        if (mergedForceRepresentative == null || mergedForceRepresentative.size() == 0) {
            calculateTurnRelatedProperties();
        }
    }

    @JsonIgnore
    private synchronized void calculateForceRelatedProperties() {
        if (debugBundleForceProperties) {
            listOfForceSteps = new ArrayList<>();
            listOfACLines = new ArrayList<>();
            listOfPerpendicularLines = new ArrayList<>();
            listOfIntersectionsWithAngles = new ArrayList<>();
        }

        int bundleClass = STORAGE.getClassFromBundle(this);

        /* Calculate the unmerged force representativeSubtrajectory */
        if (bundleClass != -1) {
            double bestEps = STORAGE.getEvolutionDiagram().getBestEpsilon(bundleClass);
            unmergedForceRepresentative = Forces.representativeTrajectory(trajectories, representativeSubtrajectory, bestEps,
                    listOfForceSteps, listOfPerpendicularLines, listOfACLines, listOfIntersectionsWithAngles);
        } else {
            unmergedForceRepresentative = new ArrayList<>();
        }

        if (debugBundleForceProperties) {
            unmergedForceLinearRegressionDict = new HashMap<>();
            unmergedForceStraightnessDict = new HashMap<>();

            /* Calculate the straightness properties for the unmerged force representativeSubtrajectory */
            if (unmergedForceRepresentative.size() > 0) {
                unmergedForceLinearRegressionDict.put(
                        "Line",
                        LinearRegression.createLinearRegressionLineForRepresentative(
                                unmergedForceRepresentative, unmergedForceLinearRegressionDict)
                );
                Straightener.calculateStraightness(unmergedForceRepresentative, unmergedForceStraightnessDict);
            }
        }
    }

    @JsonProperty
    public List<Point2D> getForcesRepresentativeJSON() {
        return unmergedForceRepresentative;
    }

    @JsonIgnore
    public List<List<Point2D>> getForceStepsJSON() {
        if (debugBundleForceProperties) {
            return listOfForceSteps;
        }
        return null;
    }

    @JsonProperty
    public List<Point2D> getForcePerpendicularLinesJSON() {
        return listOfPerpendicularLines;
    }

    @JsonProperty
    // This is only two points extra per representativeSubtrajectory point. Always interesting to see.
    public List<Point2D> getForceACLinesJSON() {
        if (debugBundleForceProperties) {
            return listOfACLines;
        }
        return null;
    }

    // Updated to prevent final JSON of becoming to large.
    @JsonProperty
    public List<List<OrthogonalIntersection>> getForceIntersectionsWithAnglesJSON() {
        if (debugBundleForceProperties) {
            return listOfIntersectionsWithAngles;
        }
        return null;
    }

    @JsonProperty
    public Map<String, Object> getForceRepresentativeStraightMetricJSON() {
        return unmergedForceStraightnessDict;
    }

    @JsonProperty
    public Map<String, Object> getForceRepresentativeRegressionLineJSON() {
        return unmergedForceLinearRegressionDict;
    }

    @JsonIgnore
    private synchronized void calculateTurnRelatedProperties() {
        List<List<Map<String, Object>>> multiPointSharpTurns = TurnDecider.getMultiEdgesSharpTurnParts(this.trajectories);
        if (debugBundleTurnProperties) {
            this.singlePointSharpTurns = TurnDecider.getAllSharpTurns(this.trajectories);
            this.multiPointSharpTurns = multiPointSharpTurns;
        }
        turns = TurnDecider.clusterIntoTurns(multiPointSharpTurns, this);

        if (unmergedForceRepresentative.size() == 0) {
            sortedTurns = new ArrayList<>();
            mergedForceRepresentative = new ArrayList<>();
        } else {
            if (debugBundleTurnProperties)
                sortedTurns = ForceTurnMerger.sortTurnsBasedOnRep(turns, unmergedForceRepresentative);
            // compute mergedForceRepresentative because it is always used.
            mergedForceRepresentative = ForceTurnMerger.MergeTurnsAndForceRepresentative(
                    turns, unmergedForceRepresentative
            );
        }
    }

    @JsonProperty
    public List<Point2D> getSinglePointSharpTurnsJSON() {
        if (debugBundleTurnProperties) {
            return singlePointSharpTurns;
        }
        return null;
    }

    @JsonProperty
    public List<List<Map<String, Object>>> getMultiEdgesSharpTurnsJSON() {
        if (debugBundleTurnProperties) {
            return multiPointSharpTurns;
        }
        return null;
    }

    @JsonProperty
    public List<Turn> getAllTurnsJSON() {
        if (debugBundleTurnProperties) {
            return turns;
        }
        return null;
    }

    @JsonIgnore
    public List<Turn> getAllTurns() {
        return turns;
    }

    @JsonProperty
    public List<Turn> getSortedTurnsJSON() {
        if (debugBundleTurnProperties) {
            return sortedTurns;
        }
        return null;
    }

    @JsonProperty
    public List<Point2D> getMergedRepresentativeJSON() {
        if (STORAGE.getDatasetConfig().isWalkingDataset()){
            return unmergedForceRepresentative;
        } else {
            return mergedForceRepresentative;
        }
    }

    public void setBundleEndsAreRoadPoints(int bundleRoadPointsInt) {
        bundleEndsAreRoadPoints = bundleRoadPointsInt;
    }

    @JsonProperty
    public int getBundleEndsAreRoadPointsJSON() {
        if (bundleEndsAreRoadPoints == null) {
            return 404;
        }
        return bundleEndsAreRoadPoints;
    }

    /**
     * Determine whether this bundle has the given bundle as lambda-subbundle.
     *
     * @param other     Bundle to check
     * @param lambda    Error (>= 0)
     * @param checkSize Whether to enforce that the size of the given bundle
     *                  must be at most the size of this bundle.
     * @return {@code true} if {@code this} has {@code other} as
     * lambda-subbundle.
     * @throws IllegalArgumentException {@code if lambda < 0}
     * @throws NullPointerException     if {@code other == null}.
     */
    public boolean hasAsLambdaSubBundle(Bundle other, double lambda, boolean checkSize) {
        if (lambda < 0) {
            throw new IllegalStateException(this.getClass() + "::hasAsLambdaSubBundle. lambda < 0");
        }
        Preconditions.checkNotNull(other, "other == null");

        if (checkSize && this.size() < other.size()) {
            return false;
        }

        // For all of the trajectories T1 in other
        // we have to find a matching T2 trajectory in this
        // meaning that
        // T1 is a lambda subtrajectory of T2
        return other.getSubtrajectories().stream()
                .allMatch(t1 -> {
                    Subtrajectory t2 = this.getTrajectoryByName(t1.getParent().getUndirectionalLabel());
                    return t2 != null && trajectoryHasAsLambdaSubtrajectory(t2, t1, lambda);
                });

//        return other.getSubtrajectories().stream()
//                .allMatch(t1 // forall T1
//                        -> this.trajectories.stream()
//                        .anyMatch(t2 // Exists T2
//                                -> trajectoryHasAsLambdaSubtrajectory(t2, t1, lambda))
//                );
    }

    /**
     * Determine whether this bundle has the given bundle as lambda-subbundle.
     * <p>
     * Enforces that the size of the given bundle must be at most the size of
     * this bundle.
     *
     * @param other  Bundle to check
     * @param lambda Error (>= 0)
     * @return {@code true} if {@code this} has {@code other} as
     * lambda-subbundle.
     * @throws IllegalArgumentException {@code if lambda < 0}
     * @throws NullPointerException     if {@code other == null}.
     */
    public boolean hasAsLambdaSubBundle(Bundle other, double lambda) {
        return this.hasAsLambdaSubBundle(other, lambda, true);
    }

    /**
     * Whether the trajectory sup has sub as subtrajectory
     *
     * @param sup
     * @param sub
     * @param lambda
     * @return
     * @throws NullPointerException if either subtrajectory is {@code null}.
     */
    protected boolean trajectoryHasAsLambdaSubtrajectory(Subtrajectory sup, Subtrajectory sub, double lambda) {
        Preconditions.checkNotNull(sup);
        Preconditions.checkNotNull(sub);
        return sup.hasAsLambdaSubtrajectory(sub, lambda);
    }

    @Override
    public String toString() {
        return toLabelString();
    }

    public String toLabelString() {
        int bundleClass = STORAGE.getClassFromBundle(this);
        String label = this.getClass().getSimpleName() + "{" + trajectories.stream().map(Trajectory::getLabel).collect(Collectors.toList()) + ((representativeSubtrajectory != null) ? ", rep=" + representativeSubtrajectory.getLabel() : "") + '}';
        if (bundleClass == -1) {
            return label;
        } else {
            return "BundleClass : " + Integer.toString(bundleClass) + ", " + label;
        }
    }

    public String toPointsString() {
        return this.getClass().getSimpleName() + "{" + trajectories.stream().map(Trajectory::toPointsString).collect(Collectors.toList()) + ((representativeSubtrajectory != null) ? ", rep=" + representativeSubtrajectory : "") + '}';
    }

    @Override
    public int hashCode() {
        return hashcode;
    }

    private int computeHashCode() {
        int hash = 3;
        hash = 97 * hash + Objects.hashCode(this.trajectories);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Bundle other = (Bundle) obj;
        if (!Objects.equals(this.trajectories, other.trajectories)) {
            return false;
        }
        return true;
    }

    /**
     * Creates a copy of the subtrajectories in this bundle in such that they
     * are not in reversed form.
     *
     * @return
     */
    public Set<Subtrajectory> createNonInversed() {
        Set<Subtrajectory> nonInversed = this.getSubtrajectories().stream()
                .map(t -> (t.isReverse()) ? t.reverse() : t)
                .collect(Collectors.toSet());
        return nonInversed;
    }

    /**
     * Creates a copy of the bundle in such that the subtrajectories. are not in
     * reversed form.
     *
     * @return
     */
    public Bundle createNonInversedBundle() {
        // create reverse of rep
        Subtrajectory rep = null;
        if (this.representativeSubtrajectory != null) {
            rep = (representativeSubtrajectory.isReverse()) ? representativeSubtrajectory.reverse() : representativeSubtrajectory;
        }
        return newInstance(createNonInversed(), rep);
    }

    /**
     * Trims the start of the trajectories in the bundle by the given length.
     *
     * @param length length to trim the start of the trajectories with.
     * @return new Bundle in which the start of the trajectories are trimmed by
     * the given length
     * @deprecated since 18/10/2018. Changed representativeSubtrajectory structure which is not updated anymore. (set to private)
     */
    private Bundle trimStart(double length) {
        return performTrim(t -> t.trimStart(length));
    }

    /**
     * Trims the end of the trajectories in the bundle by the given length.
     *
     * @param length length to trim the start of the trajectories with.
     * @return new Bundle in which the end of the trajectories are trimmed by
     * the given length
     * @deprecated since 18/10/2018. Changed representativeSubtrajectory structure which is not updated anymore. (set to private)
     */
    private Bundle trimEnd(double length) {
        return performTrim(t -> t.trimEnd(length));
    }

    /**
     * Trims both ends of the trajectories in the bundle by the given length.
     *
     * @param length length to trim the ends of the trajectories with.
     * @return new Bundle in which both ends of the trajectories are trimmed by
     * the given length
     * @deprecated since 18/10/2018. Changed representativeSubtrajectory structure which is not updated anymore. (set to private)
     */
    private Bundle trim(double length) {
        return performTrim(t -> t.trim(length));
    }

    private Bundle performTrim(UnaryOperator<Subtrajectory> trimFunction) {
        Set<Subtrajectory> trimmed = new HashSet<>();
        Subtrajectory trimRep = null;
        for (Subtrajectory t : trajectories) {
            Subtrajectory trim = trimFunction.apply(t);
            trimmed.add(trim);
            if (representativeSubtrajectory == t) {
                trimRep = trim;
            }
        }
        return newInstance(trimmed, trimRep);

    }

    /**
     * Trims both ends of the trajectories in the bundle by the given proportion
     * of the length of the subtrajecctory
     *
     * @param prop proportion to trim the ends of the trajectories with.
     * @return new Bundle in which both ends of the trajectories are trimmed by
     * the given propotion of the length
     */
    public Bundle trimProportional(double prop) {
        return performTrim(t -> t.trim(prop * t.euclideanLength()));
    }


    /**
     * For all trajectories in the bundle, we get the index closest to getLocation()
     * (Works perfectly atm ;))
     *
     * @return a hashmap with all trajectories of the bundle and their index.
     */
    public Map<Trajectory, Double> getParentTrajectoryIndexes(Point2D location, boolean noReverseTrajectories) {
        Map<Trajectory, Double> trajectoryIndexes = new HashMap<>();

        Set<Subtrajectory> allSubtrajectories = getSubtrajectories();
        if (noReverseTrajectories) {
            allSubtrajectories = getNonReverseSubtrajectories();
        }
        for (Subtrajectory sub : allSubtrajectories) {
            double bestIndexSub = GeometryUtil.getIndexOfTrajectoryClosestToPoint(sub, location);
            if (!(bestIndexSub >= 0 && bestIndexSub <= sub.numPoints() - 1) || Double.isNaN(bestIndexSub)) {
                System.out.println("Error RoadPoint.getTrajectoryIndexes. Invalid value");
                System.out.println("Value bestIndexSub: " + bestIndexSub + " and value numPoints: " + sub.numPoints());
            }
            double bestIndexTrajectory = GeometryUtil.convertSubIndexToTrajectoryIndex(sub, bestIndexSub);

            if (bestIndexTrajectory < 0 || bestIndexTrajectory > sub.getParent().numPoints() - 1
                    || Double.isNaN(bestIndexTrajectory)) {
                System.out.println("Error RoadPoint.getTrajectoryIndexes. Invalid value");
                System.out.printf("bestIndexTrajectory: %f, parent.NumPoints: %d%n", bestIndexTrajectory, sub.getParent().numPoints());
            }
            trajectoryIndexes.put(sub.getParent(), bestIndexTrajectory);
        }


        return trajectoryIndexes;
    }

    public boolean isBundleRepCutOff() {
        return bundleRepCutOff;
    }

    public void setBundleRepCutOff(boolean bundleRepCutOff) {
        this.bundleRepCutOff = bundleRepCutOff;
    }

    public double euclideanLength() {
        return representativeSubtrajectory.euclideanLength();
    }
}
