package mapconstruction.trajectories;

import com.google.common.base.Preconditions;

import java.io.Serializable;
import java.util.Collection;
import java.util.Objects;
import java.util.Set;

/**
 * Special type of bundle that ignores the direction of its trajectories when
 * performing equality comparison and checking subbundle relations.
 *
 * @author Roel
 */
public class UndirectionalBundle extends Bundle implements Serializable {

    private static final long serialVersionUID = 1L;

    /**
     * Remember hashcode.
     */
    private final int hashcode;

    private UndirectionalBundle(Collection<Subtrajectory> trajectories) {
        super(trajectories);
        hashcode = computeHashCode();
    }

    public UndirectionalBundle(Collection<Subtrajectory> trajectories, Subtrajectory representative) {
        super(trajectories, representative);
        hashcode = computeHashCode();
    }

    public static UndirectionalBundle create(Collection<Subtrajectory> trajectories) {
        return new UndirectionalBundle(trajectories);
    }

    public static UndirectionalBundle create(Collection<Subtrajectory> trajectories, Subtrajectory representative) {
        return new UndirectionalBundle(trajectories, representative);
    }

    @Override
    public UndirectionalBundle newInstance(Collection<Subtrajectory> trajectories, Subtrajectory representative) {
        return create(trajectories, representative);
    }

    @Override
    protected boolean trajectoryHasAsLambdaSubtrajectory(Subtrajectory sup, Subtrajectory sub, double lambda) {
        Preconditions.checkNotNull(sup);
        Preconditions.checkNotNull(sub);
        return sup.hasAsLambdaSubtrajectory(sub, lambda) || sup.hasAsLambdaSubtrajectory(sub.reverse(), lambda);
    }

    @Override
    protected boolean trajectoryHasAsSubtrajectory(Subtrajectory sup, Subtrajectory sub) {
        Preconditions.checkNotNull(sup);
        Preconditions.checkNotNull(sub);
        return sup.hasAsSubtrajectory(sub) || sup.hasAsSubtrajectory(sub.reverse());
    }

    @Override
    public int hashCode() {
        return hashcode;
    }

    private int computeHashCode() {
        // The hashcode is based on the non-reversed versions of the trajectories.
        Set<Subtrajectory> nonInversed = createNonInversed();

        int hash = 3;
        hash = 97 * hash + Objects.hashCode(nonInversed);
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

        // bundles must have the same size
        if (this.size() != other.size()) {
            return false;
        }

        // For each trajectory in this bundle we have to check if
        // itself or its reverse is present in the other bundle
        for (Subtrajectory t : this.getSubtrajectories()) {
            if (!other.getSubtrajectories().contains(t) && !other.getSubtrajectories().contains(t.reverse())) {
                return false;
            }
        }

        return true;
    }
}
