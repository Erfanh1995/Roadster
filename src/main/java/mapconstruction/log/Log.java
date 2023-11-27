package mapconstruction.log;

import javax.swing.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Singleton class keeping track of a log. Is a Swingworker that can periodically
 * publish results to log displayers
 *
 * @author Roel
 */
public class Log extends SwingWorker<Void, LogEntry> {

    /**
     * Singleton instance of the log.
     */
    private static Log instance;

    /**
     * Queue containing the log entries.
     */
    private final LinkedBlockingQueue<LogEntry> log;


    /**
     * Classes using the log.
     */
    private final List<LogUser> users;

    private Log() {
        log = new LinkedBlockingQueue<>();
        users = Collections.synchronizedList(new ArrayList<>());
    }

    /**
     * Returns the singleton log instance.
     *
     * @return
     */
    public static Log instance() {
        if (instance == null) {
            instance = new Log();
        }
        return instance;
    }

    /**
     * Logs the given message at the given level.
     *
     * @param level
     * @param tag
     * @param msg
     */
    public static void log(LogLevel level, String tag, String msg) {
        log(level, tag, msg, "");
    }

    /**
     * Logs the given formatted string at the given level.
     *
     * @param level
     * @param tag
     * @param format
     * @param args
     */
    public static void log(LogLevel level, String tag, String format, Object... args) {
        try {
            instance().log.put(new LogEntry(level, tag, format, args));
        } catch (InterruptedException ex) {
            Logger.getLogger(Log.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public static void addLogUser(LogUser u) {
        instance().users.add(u);
    }

    @Override
    protected Void doInBackground() throws Exception {
        while (true) {
            publish(log.take());
        }
    }

    @Override
    protected void process(List<LogEntry> chunks) {
        synchronized (users) {
            users.forEach(u -> u.process(chunks));
        }
    }

}
