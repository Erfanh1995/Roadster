/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.filter;

import java.util.List;

/**
 * Interface for predicates consiting of one or more sub predicates
 *
 * @author Roel
 */
public interface CompositeTriPredicate<T, U, V> extends TriPredicate<T, U, V> {

    /**
     * Gets the predicate at the given index
     *
     * @param index
     * @return
     */
    TriPredicate<T, U, V> get(int index);

    /**
     * Gets the list of predicates in this predicate.
     *
     * @return
     */
    List<TriPredicate<T, U, V>> getPredicates();

    /**
     * Index of the give predicate.
     *
     * @param p predicate
     * @return Index of the give predicate. -1 if this predicate does not contain the given predicate.
     */
    int indexOf(TriPredicate<T, U, V> p);


    /**
     * Sets the predicate at the given position, replacing the one already there.
     *
     * @param index   position to set the predicate
     * @param element predicate to put at the given position
     * @return Predicate that was replaced.
     */
    TriPredicate<T, U, V> set(int index, TriPredicate<T, U, V> element);

    /**
     * Gets the number of predicates in this AndPredicate.
     *
     * @return number of predicates in this AndPredicate.
     */
    int size();

}
