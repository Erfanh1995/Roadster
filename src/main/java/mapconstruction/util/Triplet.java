package mapconstruction.util;

import java.io.Serializable;
import java.util.Objects;

/**
 * Class representing a Triplet of two values.
 *
 * @param <X> Type of the first value
 * @param <Y> Type of the second value
 * @param <Z> Type of the third value
 * @author Jorrick Sleijster
 */
public class Triplet<X, Y, Z> implements Serializable {
    private final X first;
    private final Y second;
    private final Z third;

    public Triplet(X first, Y second, Z third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    public X getFirst() {
        return first;
    }

    public Y getSecond() {
        return second;
    }

    public Z getThird() {
        return third;
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 67 * hash + Objects.hashCode(this.first);
        hash = 67 * hash + Objects.hashCode(this.second);
        hash = 67 * hash + Objects.hashCode(this.third);
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
        final Triplet<?, ?, ?> other = (Triplet<?, ?, ?>) obj;
        if (!Objects.equals(this.first, other.first)) {
            return false;
        }
        if (!Objects.equals(this.second, other.second)) {
            return false;
        }
        if (!Objects.equals(this.third, other.third)) {
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return "Triplet{" + "first=" + first + ", second=" + second + ", third=" + third + '}';
    }
}
