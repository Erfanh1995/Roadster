package mapconstruction.exceptions;

/**
 * Exception thrown if an algorithm is aborted.
 *
 * @author Roel
 */
public class AlgorithmAbortedException extends RuntimeException {

    public AlgorithmAbortedException() {
    }

    public AlgorithmAbortedException(String message) {
        super(message);
    }

    public AlgorithmAbortedException(String message, Throwable cause) {
        super(message, cause);
    }

    public AlgorithmAbortedException(Throwable cause) {
        super(cause);
    }

    public AlgorithmAbortedException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }


}
