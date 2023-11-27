/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.filter;

import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.attributes.BundleClassAttribute;
import mapconstruction.util.ToDoubleTriFunction;

/**
 * Predicate on the attribute value of the tested bundle class.
 *
 * @author Roel
 */
public class AttributeTriPredicate implements TriPredicate<EvolutionDiagram, Integer, Double> {

    /**
     * Attribute that should be tested
     */
    private BundleClassAttribute attribute;

    /**
     * Value that the attribute is tested against.
     */
    private ToDoubleTriFunction testValue;

    private Operator operator;

    public AttributeTriPredicate(BundleClassAttribute attribute, ToDoubleTriFunction testValue, Operator op) {
        this.attribute = attribute;
        this.testValue = testValue;
        this.operator = op;
    }

    @Override
    public boolean test(EvolutionDiagram d, Integer c, Double e) {
        switch (operator) {
            case NotEqual:
                return attribute.applyAsDouble(d, c, e) != testValue.applyAsDouble(d, c, e);
            case Equal:
                return attribute.applyAsDouble(d, c, e) == testValue.applyAsDouble(d, c, e);
            case GreaterThan:
                return attribute.applyAsDouble(d, c, e) > testValue.applyAsDouble(d, c, e);
            case GreaterThanOrEqual:
                return attribute.applyAsDouble(d, c, e) >= testValue.applyAsDouble(d, c, e);
            case LessThan:
                return attribute.applyAsDouble(d, c, e) < testValue.applyAsDouble(d, c, e);
            case LessThanOrEqual:
                return attribute.applyAsDouble(d, c, e) <= testValue.applyAsDouble(d, c, e);
            default:
                throw new IllegalStateException();
        }
    }

    @Override
    public String toString() {
        return attribute.name() + " " + operator.toString() + " " + testValue;
    }

    public BundleClassAttribute getAttribute() {
        return attribute;
    }

    public void setAttribute(BundleClassAttribute attribute) {
        this.attribute = attribute;
    }

    public ToDoubleTriFunction getTestValue() {
        return testValue;
    }

    public void setTestValue(ToDoubleTriFunction testValue) {
        this.testValue = testValue;
    }

    public Operator getOperator() {
        return operator;
    }

    public void setOperator(Operator operator) {
        this.operator = operator;
    }

    public enum Operator {
        LessThan("<"),
        LessThanOrEqual("<="),
        GreaterThan(">"),
        GreaterThanOrEqual(">="),
        Equal("=="),
        NotEqual("!=");

        private final String symbol;

        private Operator(String symbol) {
            this.symbol = symbol;
        }

        @Override
        public String toString() {
            return symbol;
        }


    }


}
