package mapconstruction.GUI.spinner;

import javax.swing.*;
import java.util.NavigableSet;
import java.util.TreeSet;

/**
 * SpinnerModel for selecting levels of the evolution idagram
 *
 * @author Roel
 */
public class LevelSpinnerModel extends AbstractSpinnerModel {

    // sorted set containing the values
    private NavigableSet<Double> epsilons;

    // current value
    private Double value;

    public LevelSpinnerModel(NavigableSet<Double> epsilons) {
        this.epsilons = new TreeSet<>(epsilons);
        if (this.epsilons.isEmpty()) {
            value = null;
        } else {
            value = epsilons.first();
        }
    }

    public LevelSpinnerModel() {
        this(new TreeSet<>());
    }

    public NavigableSet<Double> getEpsilons() {
        return epsilons;
    }

    public void setEpsilons(NavigableSet<Double> epsilons) {
        this.epsilons = new TreeSet<>(epsilons);
        if (this.epsilons.isEmpty()) {
            value = null;
        } else {
            value = epsilons.first();
        }
        fireStateChanged();
    }


    @Override
    public Object getValue() {
        return value;
    }

    /**
     * Sets the value of the spinner.
     * <p>
     * If the value is not in the model, it takes the closest possible value.
     *
     * @param value
     */
    @Override
    public void setValue(Object value) {
        if (!(value instanceof Number)) {
            throw new IllegalArgumentException();
        } else {
            double dValue = ((Number) value).doubleValue();
            Double up = epsilons.ceiling(dValue);
            Double down = epsilons.floor(dValue);
            up = up == null ? Double.POSITIVE_INFINITY : up;
            down = down == null ? Double.NEGATIVE_INFINITY : down;

            // pick closest
            double diffUp = up - dValue;
            double diffDown = dValue - down;
            if (diffUp < diffDown) {
                this.value = up;
            } else {
                this.value = down;
            }
        }
        fireStateChanged();
    }

    @Override
    public Object getNextValue() {
        return epsilons.higher(value);
    }

    @Override
    public Object getPreviousValue() {
        return epsilons.lower(value);
    }

}
