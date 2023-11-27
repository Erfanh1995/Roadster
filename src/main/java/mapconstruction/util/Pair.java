package mapconstruction.util;

import java.io.Serializable;
import java.util.Objects;

/**
 * Class representing a pair of two values.
 *
 * @param <X> Type of the first value
 * @param <Y> Type of the second value
 * @author Roel
 */
public class Pair<X, Y> implements Serializable {
    private final X first;
    private final Y second;

    public Pair(X first, Y second) {
        this.first = first;
        this.second = second;
    }

    public X getFirst() {
        return first;
    }

    public Y getSecond() {
        return second;
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 67 * hash + Objects.hashCode(this.first);
        hash = 67 * hash + Objects.hashCode(this.second);
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
        final Pair<?, ?> other = (Pair<?, ?>) obj;
        if (!Objects.equals(this.first, other.first)) {
            return false;
        }
        if (!Objects.equals(this.second, other.second)) {
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return "Pair{" + "first=" + first + ", second=" + second + '}';
    }


}
