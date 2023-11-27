/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.filter;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Composite predicate of arbitrary size.
 *
 * @param <T>
 * @author Roel
 */
public abstract class DynamicSizeCompositeTriPredicate<T, U, V> implements CompositeTriPredicate<T, U, V> {
    /**
     * List of predicates in composition
     */
    protected final List<TriPredicate<T, U, V>> predicates;

    public DynamicSizeCompositeTriPredicate() {
        this.predicates = new ArrayList<>();
    }

    public DynamicSizeCompositeTriPredicate(List<TriPredicate<T, U, V>> predicates) {
        this();
        this.predicates.addAll(predicates);
    }

    /**
     * Gets the number of predicates in this AndPredicate.
     *
     * @return number of predicates in this AndPredicate.
     */
    @Override
    public int size() {
        return predicates.size();
    }

    /**
     * Adds a predicate to this AndPredicate
     *
     * @param p Predicate to add
     */
    public void add(TriPredicate<T, U, V> p) {
        p = Objects.requireNonNull(p, "Predicate may not be null");
        predicates.add(p);
    }

    /**
     * Removes a predicate from this AndPredicate
     *
     * @param p Predicate to remove
     */
    public void remove(TriPredicate<T, U, V> p) {
        predicates.remove(p);
    }

    /**
     * Removes the predicate at the given position
     *
     * @param index index of the predicate to remove.
     * @return The predicate that was removed.
     */
    public TriPredicate<T, U, V> remove(int index) {
        return predicates.remove(index);
    }

    /**
     * Gets the list of predicates in this predicate.
     *
     * @return
     */
    @Override
    public List<TriPredicate<T, U, V>> getPredicates() {
        return predicates;
    }

    public void add(int index, TriPredicate<T, U, V> element) {
        predicates.add(index, element);
    }

    @Override
    public TriPredicate<T, U, V> get(int index) {
        return predicates.get(index);
    }

    @Override
    public int indexOf(TriPredicate<T, U, V> p) {
        return predicates.indexOf(p);
    }

    @Override
    public TriPredicate<T, U, V> set(int index, TriPredicate<T, U, V> element) {
        return predicates.set(index, element);
    }


}
