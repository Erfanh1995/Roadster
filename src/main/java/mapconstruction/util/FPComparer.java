package mapconstruction.util;

/**
 * Helper class with comparison functions for floating points.
 * Allows to present an acceptable error in the computations.
 *
 * @author Roel
 */
public class FPComparer {

    public static final double DEFAULT_DELTA = 1E-6;

    public static boolean eq(double d1, double d2, double delta) {
        return Math.abs(d1 - d2) < delta;
    }

    public static boolean eq(double d1, double d2) {
        return eq(d1, d2, DEFAULT_DELTA);
    }

    public static boolean lt(double d1, double d2, double delta) {
        return d1 < d2 + delta;
    }

    public static boolean lt(double d1, double d2) {
        return lt(d1, d2, DEFAULT_DELTA);
    }

    public static boolean leq(double d1, double d2, double delta) {
        return lt(d1, d2, delta) || eq(d1, d2, delta);
    }

    public static boolean leq(double d1, double d2) {
        return leq(d1, d2, DEFAULT_DELTA);
    }

    public static boolean gt(double d1, double d2, double delta) {
        return !leq(d1, d2, delta);
    }

    public static boolean gt(double d1, double d2) {
        return gt(d1, d2, DEFAULT_DELTA);
    }

    public static boolean geq(double d1, double d2, double delta) {
        return !lt(d1, d2, delta);
    }

    public static boolean geq(double d1, double d2) {
        return geq(d1, d2, DEFAULT_DELTA);
    }


}
