package mapconstruction.GUI.io;

import com.google.common.io.Files;
import mapconstruction.log.Log;
import mapconstruction.log.LogEntry;
import mapconstruction.log.LogLevel;
import mapconstruction.log.LogUser;

import java.io.*;
import java.nio.file.Paths;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Class for logging to file.
 *
 * @author Roel
 */
public class FileLogger implements LogUser {

    private static final String OUTDIR = "./log";
    private File output;

    public FileLogger() {
        Date timestamp = Calendar.getInstance().getTime();
        String filename = String.format("%1$tF-%1$tH.%1$tM.%1$tS.log", timestamp);
        output = Paths.get(OUTDIR, filename).toFile();

        try {
            // create output directory if it does not exist
            Files.createParentDirs(output);
        } catch (IOException ex) {
            Logger.getLogger(FileLogger.class.getName()).log(Level.SEVERE, null, ex);
        }

        Log.log(LogLevel.INFO, "FileLog", "File Log started @ " + timestamp);

    }

    @Override
    public void process(List<LogEntry> entries) {
        try (BufferedWriter writer = new BufferedWriter(new PrintWriter(new FileOutputStream(output, true)))) {
            for (LogEntry entry : entries) {
                writer.append(entry.toString());
                writer.newLine();
            }
        } catch (IOException ex) {
            Logger.getLogger(FileLogger.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
