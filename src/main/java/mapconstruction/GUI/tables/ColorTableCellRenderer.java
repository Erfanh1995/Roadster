package mapconstruction.GUI.tables;

import javax.swing.*;
import javax.swing.table.TableCellRenderer;
import java.awt.*;

/**
 * Renderer for table cells representing a color
 *
 * @author Roel
 */
public class ColorTableCellRenderer extends JLabel implements TableCellRenderer {

    public ColorTableCellRenderer() {
        setOpaque(true);
    }

    @Override
    public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column) {
        Color c = (Color) value;
        this.setBackground(c);
        return this;
    }


}
