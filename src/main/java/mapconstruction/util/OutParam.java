package mapconstruction.util;

/**
 * Utility class model output parameters.
 * <p>
 * An OutParam as a generic settable field.
 * By passing an empty OutParam to a method,
 * allows the method to set a value which can then be retrieved.
 *
 * @param <T> Type of the parameter
 * @author Roel
 */
public class OutParam<T> {

    private T value;

    /**
     * Constructs an empty OutputParam.
     * Has default value Null.
     */
    public OutParam() {
        value = null;
    }

    /**
     * Constructs an output parameter with the
     * given default value;
     *
     * @param value
     */
    public OutParam(T value) {
        this.value = value;
    }


    public T get() {
        return value;
    }

    public void set(T value) {
        this.value = value;
    }


}
