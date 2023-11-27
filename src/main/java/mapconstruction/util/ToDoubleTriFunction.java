package mapconstruction.util;

import java.io.Serializable;

/**
 * Represents a function that produces a double-valued result, based on three arguments.
 *
 * @param <T> the type of the first argument to the function
 * @param <U> the type of the second argument to the function
 * @param <V> the type of the third argument to the function
 * @author Roel
 */
public interface ToDoubleTriFunction<T, U, V> extends Serializable {

    double applyAsDouble(T t, U u, V v);

}
