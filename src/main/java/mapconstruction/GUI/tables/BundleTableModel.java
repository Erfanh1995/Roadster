package mapconstruction.GUI.tables;

import com.google.common.collect.Lists;
import mapconstruction.algorithms.diagram.EvolutionDiagram;
import mapconstruction.attributes.BundleClassAttributes;

import javax.swing.event.TableModelEvent;
import javax.swing.table.AbstractTableModel;
import java.util.*;

/**
 * @author Roel
 */
public class BundleTableModel extends AbstractTableModel {

    /**
     * Column index of the bundles
     */
    public static final int BUNDLECOL = 1;

    /**
     * Column index of the visible flag.
     */
    public static final int VISIBLECOL = 0;

    /**
     * Columns that should be visible in the table by default.
     */
    private static final String[] DEFAULT_VIS = new String[]{
            "Size", "DiscreteLength", "ContinuousLength",
            "Birth", "Merge", "LifeSpan", "RelativeLifeSpan", "BestEps"
    };
    /**
     * Info for each column.
     */
    private final ArrayList<ColumnInfo> columns;
    /**
     * The data.
     */
    private Object[][] data;
    /**
     * Map indicicated which classes have the visibility flag set.
     * This is needed to ensure that the visibility is untouched upon
     * changes in the data.
     */
    private Map<Integer, Boolean> visibleClasses;

    public BundleTableModel() {
        columns = new ArrayList<>();
        initColumns();
        data = new Object[0][0];
        visibleClasses = new HashMap<>();
    }

    private void initColumns() {
        /**
         * Visibility
         */
        columns.add(new ColumnInfo("", Boolean.class, true, true));
        /**
         * Identifier for bundle
         */
        columns.add(new ColumnInfo("Bundle", Integer.class, true, false));

        /**
         * Add columns for each attribute
         */
        BundleClassAttributes.names()
                .forEach(name -> {
                    boolean isDefaultVisible = Arrays.stream(DEFAULT_VIS).anyMatch(a -> name.equals(a));

                    columns.add(new ColumnInfo(name, Double.class, isDefaultVisible, false));
                });
    }

    @Override
    public int getRowCount() {
        return data.length;
    }

    @Override
    public int getColumnCount() {
        return columns.size();
    }

    @Override
    public Object getValueAt(int rowIndex, int columnIndex) {
        return data[rowIndex][columnIndex];
    }

    @Override
    public void setValueAt(Object aValue, int rowIndex, int columnIndex) {
        switch (columnIndex) {
            case VISIBLECOL:
                boolean vis = (boolean) aValue;
                setVisible(rowIndex, vis);
                break;
            default:
                throw new UnsupportedOperationException(getColumnName(columnIndex) + " cannot be set");

        }
        fireTableCellUpdated(rowIndex, columnIndex);
    }

    private void setVisible(int row, boolean value) {
        data[row][VISIBLECOL] = value;
        visibleClasses.put((int) data[row][BUNDLECOL], value);
    }

    @Override
    public boolean isCellEditable(int rowIndex, int columnIndex) {
        // Only visibility flag is editable
        return columns.get(columnIndex).isEditable();
    }

    @Override
    public Class<?> getColumnClass(int columnIndex) {
        return columns.get(columnIndex).getColumnClass();
    }

    @Override
    public String getColumnName(int column) {
        return columns.get(column).getName();
    }

    public boolean isColumnDefaultVisible(int col) {
        return columns.get(col).isDefaultVisible();
    }

    /**
     * Sets the data for this table. The list bundle classes should indicate all
     * generated bundle, the given diagram is used to compute all the
     * attributes.
     * <p>
     * Attributes are computes for the best level
     *
     * @param d
     * @param bundleClasses
     */
    public void generateData(EvolutionDiagram d, Collection<Integer> bundleClasses) {
        this.data = new Object[bundleClasses.size()][columns.size()];
        // sort the classes
        ArrayList<Integer> sortedClasses = Lists.newArrayList(bundleClasses);
        sortedClasses.sort(Comparator.naturalOrder());

        for (int row = 0; row < sortedClasses.size(); row++) {
            int bundleClass = sortedClasses.get(row);
            double bestEps = d.getBestEpsilon(bundleClass);
            for (int col = 0; col < columns.size(); col++) {

                switch (col) {
                    case VISIBLECOL:
                        data[row][col] = visibleClasses.getOrDefault(bundleClass, true);
                        visibleClasses.put(bundleClass, (boolean) data[row][col]);
                        break;
                    case BUNDLECOL:
                        data[row][col] = bundleClass;
                        break;
                    default:
                        data[row][col] = BundleClassAttributes.get(columns.get(col).getName()).applyAsDouble(d, bundleClass, bestEps);
                        break;
                }
            }
        }

        fireTableDataChanged();

    }

    public void setAllVisibility(boolean b) {
        for (int r = 0; r < getRowCount(); r++) {
            setVisible(r, b);
        }
        fireTableChanged(new TableModelEvent(this, 0, getRowCount() - 1, VISIBLECOL, TableModelEvent.UPDATE));
    }

    public void resetAllVisibility(boolean b) {
        visibleClasses.clear();
        for (int r = 0; r < getRowCount(); r++) {
            setVisible(r, b);
        }
        fireTableChanged(new TableModelEvent(this, 0, getRowCount() - 1, VISIBLECOL, TableModelEvent.UPDATE));
    }

    /**
     * Tuple storing info about a particuar column
     */
    private static class ColumnInfo {

        private final String name;

        private final Class<?> columnClass;

        private final boolean isDefaultVisible;

        private final boolean isEditable;

        public ColumnInfo(String name, Class<?> columnClass, boolean isDefaultVisible, boolean isEditable) {
            this.name = name;
            this.columnClass = columnClass;
            this.isDefaultVisible = isDefaultVisible;
            this.isEditable = isEditable;
        }

        public String getName() {
            return name;
        }

        public Class<?> getColumnClass() {
            return columnClass;
        }

        public boolean isDefaultVisible() {
            return isDefaultVisible;
        }

        public boolean isEditable() {
            return isEditable;
        }

        @Override
        public String toString() {
            return "ColumnInfo{" + "name=" + name + ", columnClass=" + columnClass + ", isDefaultVisible=" + isDefaultVisible + ", isEditable=" + isEditable + '}';
        }

    }

}
