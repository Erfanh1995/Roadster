package mapconstruction.log;

import java.util.List;

/**
 * Interface for classes that use messages added to the log.
 *
 * @author Roel
 */
public interface LogUser {


    void process(List<LogEntry> entries);
}
