package mapconstruction.GUI.lists;

import mapconstruction.log.LogEntry;
import mapconstruction.log.LogLevel;
import mapconstruction.log.LogUser;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * JList for diplaying th log
 *
 * @author Roel
 */
public class LogList extends JList<LogEntry> implements LogUser {

    private LogListModel model;

    private boolean showTimestamp;

    public LogList() {
        super();
        model = new LogListModel();
        this.setModel(model);
        this.setCellRenderer(new LogEntryCellRenderer());
    }


    @Override
    public void process(List<LogEntry> entries) {
        for (LogEntry entry : entries) {
            model.addEntry(entry);
        }
    }

    public void setLevelVisible(LogLevel level, boolean isVisible) {
        model.setLevelVisible(level, isVisible);
    }

    public void setShowTimestamp(boolean showTimestamp) {
        this.showTimestamp = showTimestamp;
        this.repaint();
    }

    private static class LogListModel extends AbstractListModel<LogEntry> {

        private final ArrayList<LogEntry> data;

        List<LogEntry> displayedData;

        private Set<LogLevel> displayedLevels;

        public LogListModel() {
            super();
            data = new ArrayList(1024);
            displayedData = data;
            displayedLevels = new HashSet<>();
        }


        @Override
        public int getSize() {
            return displayedData.size();
        }

        @Override
        public LogEntry getElementAt(int index) {
            return displayedData.get(index);
        }

        public void addEntry(LogEntry entry) {
            data.add(entry);
            if (shouldDisplay(entry)) {
                displayedData.add(entry);
                fireIntervalAdded(this, getSize() - 1, getSize() - 1);
            }
        }

        private boolean shouldDisplay(LogEntry entry) {
            return displayedLevels.contains(entry.getLevel());
        }

        public void setLevelVisible(LogLevel level, boolean isVisible) {
            // Set the appropriate levels visible
            if (isVisible) {
                displayedLevels.add(level);
            } else {
                displayedLevels.remove(level);
            }

            // Filter data
            displayedData = data.stream().filter(entry -> shouldDisplay(entry)).collect(Collectors.toList());
            fireContentsChanged(this, 0, getSize() - 1);
        }

    }

    private class LogEntryCellRenderer extends DefaultListCellRenderer {

        @Override
        public Component getListCellRendererComponent(JList<?> list, Object value, int index, boolean isSelected, boolean cellHasFocus) {
            super.getListCellRendererComponent(list, value, index, isSelected, cellHasFocus);
            LogEntry entry = (LogEntry) value;

            String text = "";

            if (showTimestamp) {
                text += String.format("[%tT] ", entry.getTimestamp());
            }

            text += String.format("[%s] %s", entry.getTag(), entry.getMsg());

            // set text
            this.setText(text);

            // set color
            Color c;
            switch (entry.getLevel()) {
                case ERROR:
                    c = new Color(139, 0, 0);
                    break;
                case INFO:
                    c = new Color(0, 100, 0);
                    break;
                case STATUS:
                    c = Color.DARK_GRAY;
                    break;
                case WARNING:
                    c = new Color(184, 134, 11);
                    break;
                default:
                    c = Color.BLACK;
            }

            this.setForeground(c);

            return this;
        }

    }

}
