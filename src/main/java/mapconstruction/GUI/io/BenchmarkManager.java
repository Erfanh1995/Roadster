package mapconstruction.GUI.io;

import mapconstruction.benchmark.BundleRenderer;
import mapconstruction.benchmark.Timing;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.Yaml;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;

public class BenchmarkManager {
    private String path;

    private final static DateFormat DATES = new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss");

    public BenchmarkManager(String path) {
        this.path = path;
    }

    public void saveStats(String name, Date date, Timing timings) {
        this.saveStats(name, date, timings, null);
    }

    /**
     * Given the timings and bundle results of an algorithm, create a statistical summary and save it to a file.
     */
    public void saveStats(String name, Date date, Timing timings, Map<String, Set<Bundle>> results) {
        String subPath = name + '/' + DATES.format(date);
        createDirectoryIfAbsent(subPath);

        Map<String, Object> stats = makeStats(timings, results);

        DumperOptions options = new DumperOptions();
        options.setDefaultFlowStyle(DumperOptions.FlowStyle.BLOCK);
        options.setPrettyFlow(true);

        Yaml yaml = new Yaml(options);
        StringWriter writer = new StringWriter();
        yaml.dump(stats, writer);

        writeToFile(subPath, "report.yml", writer.toString().getBytes());
    }

    /**
     * Given a set of result bundles, create a PNG snapshot of the bundles.
     */
    public void saveSnapshot(String name, Date date, String bundleName, Set<Bundle> result) {
        String subPath = name + '/' + DATES.format(date);
        createDirectoryIfAbsent(subPath);
        double minX = Double.MAX_VALUE, maxX = Double.MIN_VALUE, minY = Double.MAX_VALUE, maxY = Double.MIN_VALUE;
        for (Bundle b : result) {
            for (Trajectory s : b.getSubtrajectories()) {
                minX = Math.min(minX, s.points().stream().mapToDouble(Point2D::getX).min().orElse(Double.MAX_VALUE));
                maxX = Math.max(maxX, s.points().stream().mapToDouble(Point2D::getX).max().orElse(Double.MIN_VALUE));
                minY = Math.min(minY, s.points().stream().mapToDouble(Point2D::getY).min().orElse(Double.MAX_VALUE));
                maxY = Math.max(maxY, s.points().stream().mapToDouble(Point2D::getY).max().orElse(Double.MIN_VALUE));
            }
        }
        BundleRenderer renderer = new BundleRenderer(
                new Rectangle((int) minX, (int) minY, (int) (maxX - minX), (int) (maxY - minY)));

        result.stream().sorted(Comparator.comparingInt(Bundle::size).reversed()).forEach(renderer::draw);

        BufferedImage image = renderer.make();
        writeImageToFile(subPath, String.format("image_%s.png", bundleName.replaceAll("\\s", "_").toLowerCase()), image);
    }

    /**
     * Write a bytearray to file
     */
    private void writeToFile(String subdirectory, String filename, byte[] contents) {
        File output = new File(this.path + '/' + subdirectory + '/' + filename).getAbsoluteFile();
        try (FileOutputStream os = new FileOutputStream(output)) {
            os.write(contents);
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Write a BufferedImage to file
     */
    private void writeImageToFile(String subdirectory, String filename, BufferedImage image) {
        try {
            File output = new File(this.path + '/' + subdirectory + '/' + filename).getAbsoluteFile();
            ImageIO.write(image, "png", output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void createDirectoryIfAbsent(String subdirectory) {
        File dir = new File(this.path + '/' + subdirectory + '/');
        boolean mkdirs = dir.mkdirs();
        if (mkdirs) {
            Log.log(LogLevel.INFO, "BenchmarkManager", "Created benchmark directory: %s", dir.toString());
        }
    }

    /**
     * Preprocessor for YAML creation for the result statistics.
     */
    private Map<String, Object> makeStats(Timing timings, Map<String, Set<Bundle>> results) {
        Map<String, Object> t = new TreeMap<>();
        t.put("timings", makeTimings(timings));
        if (results != null) {
            List<Object> b = results.entrySet().stream().map(e -> {
                Map<String, Object> c = new TreeMap<>();
                c.put(e.getKey(), e.getValue().size());
                return c;
            }).collect(Collectors.toList());
            t.put("bundles", b);
        }
        return t;
    }

    /**
     * Preprocessor for YAML creation for the timings.
     */
    private Map<String, Object> makeTimings(Timing timings) {
        Map<String,Object> t = new TreeMap<>((t1, t2) -> (t1.equals("splits") ? 1 : 0) + (t2.equals("splits") ? -1 : 0));
        long duration = timings.getDuration();
        long minutes = duration / 60000;
        long seconds = (duration % 60000) / 1000;
        long millis = duration % 1000;
        t.put(timings.getTitle(), String.format("%d:%02d.%d", minutes, seconds, millis));
        if (!timings.getSubtimings().isEmpty()) {
            List<Object> subtimings = new ArrayList<>();
            timings.getSubtimings().forEach(s -> subtimings.add(makeTimings(s)));
            t.put("splits", subtimings);
        }
        return t;
    }

}
