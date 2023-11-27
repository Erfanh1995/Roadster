/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.filter;

/**
 * @author Roel
 */
public final class BooleanTriPredicates {

    public static <T, U, V> TriPredicate<T, U, V> alwaysTrue() {
        return new AlwaysTrue<>();
    }

    public static <T, U, V> TriPredicate<T, U, V> alwaysFalse() {
        return new AlwaysFalse<>();
    }

    private static class AlwaysTrue<T, U, V> implements TriPredicate<T, U, V> {

        @Override
        public boolean test(T t, U u, V v) {
            return true;
        }

        @Override
        public String toString() {
            return "<TRUE>";
        }


    }

    private static class AlwaysFalse<T, U, V> implements TriPredicate<T, U, V> {

        @Override
        public boolean test(T t, U u, V v) {
            return false;
        }

        @Override
        public String toString() {
            return "<FALSE>";
        }


    }

}
