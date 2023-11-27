package mapconstruction.GUI.tables;

import mapconstruction.util.ColorScheme;

import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;
import java.awt.*;

/**
 * Renderer of a table cell representing a bundle.
 * <p>
 * Simply prints "Bundle" followed by the row index.
 *
 * @author Roel
 */
public class BundleTableCellRenderer extends DefaultTableCellRenderer {

    @Override
    public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column) {
        JLabel created = (JLabel) super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
        created.setHorizontalAlignment(SwingConstants.RIGHT);

        Font font = created.getFont();
        // same font but bold
        Font boldFont = new Font(font.getFontName(), Font.BOLD, font.getSize());
        created.setFont(boldFont);
        Color bg = ColorScheme.getDefaultColorCyclic((int) value);
        created.setBackground(bg);

        //float[] hsb = Color.RGBtoHSB(bg.getRed(), bg.getGreen(), bg.getBlue(), null);
        if (0.2126 * bg.getRed() + 0.7152 * bg.getGreen() + 0.0722 * bg.getBlue() < 128) {
            created.setForeground(Color.WHITE);
        } else {
            created.setForeground(Color.BLACK);
        }

        return created;
    }

}
