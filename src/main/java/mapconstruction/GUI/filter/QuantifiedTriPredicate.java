/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.filter;

import com.google.common.base.Joiner;

import java.util.List;

/**
 * Predicate that quantifies over a list of predicates
 *
 * @param <T>
 * @author Roel
 */
public class QuantifiedTriPredicate<T, U, V> extends DynamicSizeCompositeTriPredicate<T, U, V> {

    private Quantifier quantifier;

    public QuantifiedTriPredicate(Quantifier quantifier) {
        this.quantifier = quantifier;
    }

    public QuantifiedTriPredicate(Quantifier quantifier, List<TriPredicate<T, U, V>> predicates) {
        super(predicates);
        this.quantifier = quantifier;
    }

    public Quantifier quantifier() {
        return quantifier;
    }

    public void setQuantifier(Quantifier quantifier) {
        this.quantifier = quantifier;
    }

    @Override
    public boolean test(T t, U u, V v) {
        switch (quantifier) {
            case All: {
                boolean b = predicates.stream().allMatch(p -> p.test(t, u, v));
                return b;
            }

            case Any:
                return predicates.stream().anyMatch(p -> p.test(t, u, v));
            case NotAll:
                return !predicates.stream().allMatch(p -> p.test(t, u, v));
            case None:
                return predicates.stream().noneMatch(p -> p.test(t, u, v));
            default:
                throw new IllegalStateException("Unknown connective: " + quantifier);
        }
    }

    @Override
    public String toString() {
        return quantifier.name() + "(" + Joiner.on(", ").join(predicates) + ')';
    }

    public enum Quantifier {
        All,
        Any,
        NotAll,
        None;


        public TriPredicate<?, ?, ?> getUnitElement() {
            switch (this) {
                case All:
                    return BooleanTriPredicates.alwaysTrue();
                case Any:
                    return BooleanTriPredicates.alwaysFalse();
                case NotAll:
                    return BooleanTriPredicates.alwaysTrue();
                case None:
                    return BooleanTriPredicates.alwaysFalse();
                default:
                    throw new IllegalStateException();
            }
        }


    }


}
