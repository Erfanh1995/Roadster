package mapconstruction.log;

import java.util.Calendar;
import java.util.Date;

/**
 * Class representing an entry in the log.
 * <p>
 * Contains message, level and timestamp.
 *
 * @author Roel
 */
public class LogEntry {

    /**
     * Log message format.
     */
    private final String formatMsg;

    /**
     * Log tag.
     */
    private final String tag;

    /**
     * Log level.
     */
    private final LogLevel level;

    /**
     * Timestamp of date.
     */
    private final Date timestamp;

    /**
     * Arguments for formatted string.
     */
    private final Object[] formatArgs;

    private String msg;

    public LogEntry(LogLevel level, String tag, String formatMsg, Object... formatArgs) {
        this.formatMsg = formatMsg;
        this.level = level;
        this.formatArgs = formatArgs;
        this.timestamp = Calendar.getInstance().getTime();
        this.msg = null;
        this.tag = tag;
    }

    public String getMsg() {
        if (msg == null) {
            msg = String.format(formatMsg, formatArgs);
        }

        return msg;
    }

    public LogLevel getLevel() {
        return level;
    }

    public Date getTimestamp() {
        return timestamp;
    }

    public String getTag() {
        return tag;
    }


    @Override
    public String toString() {
        return String.format("%1$tF %1$tT.%1$tL - [%2$s] [%3$s] %4$s", timestamp, level.name(), tag, getMsg());
    }


}
