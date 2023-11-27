package mapconstruction.GUI.filter;

import java.io.Serializable;

/**
 * Predicate used on three arguments
 *
 * @author Roel
 */
public interface TriPredicate<T, U, V> extends Serializable {

    boolean test(T t, U u, V v);
}
