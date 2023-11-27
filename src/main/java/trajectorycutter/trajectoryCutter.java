package trajectorycutter;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.*;
import java.nio.file.Files;
import java.util.*;
import java.util.List;

/**
 * A class to cut all trajectories in a dataset into pieces such that only subtrajectories within a bounding box remain.
 * Using the rectangle function in the ui (at the bottom), we can produce a new subset of the dataset particularly fast.
 * This allows us to test specific cases and improve our general algorithm.
 *
 * @author jorricks
 */

public class trajectoryCutter {

    public static void main(String[] args) {
        File datasetFolder = getDatasetFolder();

        while (true) {
            System.out.println("Your chosen dataset folder : " + datasetFolder.getAbsolutePath());

            String newFolderPath = getNewFolder(datasetFolder).getAbsolutePath();
            Rectangle2D theCut = getFastTheCut();
            System.out.println("Cut is " + theCut.getMinX() + " " + theCut.getMinY() + " " + theCut.getMaxX() + " " + theCut.getMaxY());

            for (File file : getAllFilesInDataset(datasetFolder)) {
                List<Point2D> point2DList = new ArrayList<>();

                read(file, point2DList);
                List<List<Point2D>> newTrajectories = new ArrayList<>();

                cut(theCut, point2DList, newTrajectories);

                for (int i = 0; i < newTrajectories.size(); i++) {
                    List<Point2D> newTrajectory = newTrajectories.get(i);
                    File newFile = new File(newFolderPath + '/' +
                            file.getName().replaceAll(".txt", "") + "_" + i + ".txt");
                    write(newTrajectory, newFile);
                }
            }

            // Copying our config file.
            try {
                Files.copy(new File(datasetFolder.getAbsolutePath() + "/dataset-config.yml").toPath(),
                        new File(newFolderPath + "/dataset-config.yml").toPath());
            } catch (IOException io) {
                System.out.println(io.toString());
            }

            System.out.println("Done! Your new folder is located at " + newFolderPath + "\n\n");
        }
    }

    private static File getDatasetFolder() {
        File file = null;
        while (file == null || !file.isDirectory()) {
            Scanner keyboard = new Scanner(System.in);
            System.out.println("Type in the dataset folder");
            String folderName = keyboard.nextLine();
            file = new File(folderName);
        }
        return file;
    }

    private static File getNewFolder(File originalDatasetFolder) {
        String currentPath = originalDatasetFolder.getParentFile().getAbsolutePath();

        Scanner keyboard = new Scanner(System.in);
        System.out.println("Type in the new dataset folder");
        String newDatasetFolder = keyboard.nextLine();
        File directory = new File(currentPath + "/" + newDatasetFolder);
        if (!directory.exists()) {
            if (!directory.mkdir()) {
                System.out.println("Error creating dataset folder at: " + directory.getAbsolutePath());
            }
        }
        return directory;
    }

    private static Rectangle2D getFastTheCut() {
        System.out.println("Enter coordinates as 'left-x, bottom-y, right-x, top-y' on the same line:");
        Scanner input = new Scanner(System.in);
        List<Double> values = new ArrayList<>();
        while (values.size() != 4) {
            values.clear();
            while (input.hasNext()) {
                if (input.hasNextDouble()) {
                    values.add(input.nextDouble());
//                    System.out.println(values.get(values.size() - 1));
                    if (values.size() == 4) {
                        break;
                    }
                } else {
                    input.next();
                }
            }
        }
        return makeRectangleOfValues(values);
    }

    private static Rectangle2D getTheCut() {
        List<String> infoStrings = new ArrayList<>(Arrays.asList("Enter Left-Bottom X coordinate", "Enter Left-Bottom Y coordinate", "Enter Right-Top X coordinate", "Enter Right-Top Y coordinate"));
        List<Double> values = new ArrayList<>();

        for (int i = 0; i < infoStrings.size(); i++) {
            while (true) {
                try {
                    System.out.println(infoStrings.get(i));
                    Scanner input = new Scanner(System.in);
                    double choice = input.nextDouble();
                    values.add(choice);
                    break;
                } catch (Exception exc) {
                    System.out.println(exc.toString());
                }
            }
        }
        return makeRectangleOfValues(values);
    }

    private static Rectangle2D makeRectangleOfValues(List<Double> values) {
        double width = values.get(2) - values.get(0);
        double height = values.get(3) - values.get(1);
        if (width < 0 || height < 0) {
            System.out.println("Error! Width: " + width + "    Height: " + height);
        }
        return new Rectangle2D.Double(values.get(0), values.get(1), width, height);
    }

    private static File[] getAllFilesInDataset(File datasetFolder) {
        return datasetFolder.listFiles(file -> file.getName().toLowerCase().endsWith(".txt"));
    }

    private static void read(File f, List<Point2D> points) {
        try (BufferedReader bf = new BufferedReader(new FileReader(f))) {
            // init list of points
            String line;
            while ((line = bf.readLine()) != null) {
                // Split on whitespace
                String[] numbers = line.split("\\s+");

                double x = Double.parseDouble(numbers[0]);
                double y = Double.parseDouble(numbers[1]);
                int z = (int) Double.parseDouble(numbers[2]);

                Point2D point = new Point2D.Double(x, y);
                if (points.size() == 0 || !point.equals(points.get(points.size() - 1))) {
                    points.add(point);
                }

            }
        } catch (IOException | NumberFormatException ex) {
            System.out.println(ex.toString());
        }
    }

    private static void write(List<Point2D> pointList, File file) {
        if (pointList.size() < 3) {
            return;
        }

        try (PrintWriter w = new PrintWriter(file)) {
            for (int i = 0; i < pointList.size(); i++) {
                Point2D p = pointList.get(i);
                w.format(Locale.forLanguageTag("en-US"), "%f %f %f \n", p.getX(), p.getY(), (double) i);
            }
        } catch (IOException ex) {
            System.out.println("Error!");
            System.out.println(ex.toString());
        }
    }

    private static void cut(Rectangle2D theCut, List<Point2D> originalPointList, List<List<Point2D>> newTrajectories) {

        List<Point2D> currentPointList = null;
        for (Point2D currentPoint : originalPointList) {

            if (theCut.contains(currentPoint)) {
                if (currentPointList == null) {
                    currentPointList = new ArrayList<>();
                }
                currentPointList.add(currentPoint);
            } else {
                if (currentPointList != null) {
                    newTrajectories.add(currentPointList);
                    currentPointList = null;
                }
            }
        }
        if (currentPointList != null) {
            newTrajectories.add(currentPointList);
        }
    }
}