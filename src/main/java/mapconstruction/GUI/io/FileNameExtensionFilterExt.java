package mapconstruction.GUI.io;

import com.google.common.base.Joiner;

import javax.swing.filechooser.FileFilter;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;

/**
 * Wrapper for FileNameExtensionFilter that shows the extensions in the description.
 *
 * @author Roel
 */
public class FileNameExtensionFilterExt extends FileFilter {
    FileNameExtensionFilter filter;

    public FileNameExtensionFilterExt(FileNameExtensionFilter filter) {
        this.filter = filter;
    }

    @Override
    public boolean accept(File f) {
        return filter.accept(f);
    }

    public String[] getExtensions() {
        return filter.getExtensions();
    }

    @Override
    public String toString() {
        return filter.toString();
    }

    @Override
    public String getDescription() {
        StringBuilder builder = new StringBuilder(filter.getDescription());
        builder.append(" ");
        builder.append('(');
        builder.append('.');
        Joiner.on(", .").appendTo(builder, filter.getExtensions());
        builder.append(')');
        return builder.toString();
    }


}
