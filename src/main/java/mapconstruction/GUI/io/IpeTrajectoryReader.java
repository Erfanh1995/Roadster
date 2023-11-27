/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.io;

import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Trajectory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Collectors;

/**
 * Reads trajectories from an Ipe file.
 *
 * @author Roel
 */
public class IpeTrajectoryReader implements TrajectoryReader {

    private DocumentBuilder dBuilder;

    public IpeTrajectoryReader() {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setFeature("http://apache.org/xml/features/nonvalidating/load-external-dtd", false);
            dBuilder = dbFactory.newDocumentBuilder();
        } catch (ParserConfigurationException ex) {
            Logger.getLogger(IpeTrajectoryReader.class.getName()).log(Level.SEVERE, null, ex);
            Log.log(LogLevel.ERROR, "IperParser", "Exception thrown on creation: " + ex.getMessage());
        }
    }

    @Override
    public List<? extends Trajectory> parse(File f) {
        List<Trajectory> trajectories = new ArrayList<>();
        try {

            Document doc = dBuilder.parse(f);

            // normalize document
            doc.getDocumentElement().normalize();

            NodeList pages = doc.getElementsByTagName("page");

            // loop through pages, and collect all paths on all pages
            for (int i = 0; i < pages.getLength(); i++) {
                Node page = pages.item(i);
                if (page.getNodeType() == Node.ELEMENT_NODE) {
                    // get the element of the page node
                    Element ePage = (Element) page;

                    // get active layers
                    Set<String> active = new HashSet<>(Arrays.asList(ePage.getElementsByTagName("view").item(0).getAttributes().getNamedItem("layers").getNodeValue().split("\\s")));


                    // get all paths on the page
                    NodeList paths = ePage.getElementsByTagName("path");

                    // Parse all paths
                    // whether the current layer is active.
                    boolean isActive = false;
                    for (int j = 0; j < paths.getLength(); j++) {
                        Node path = paths.item(j);

                        // change isActive flag;
                        Node layerAttribute = path.getAttributes().getNamedItem("layer");
                        if (layerAttribute != null) {
                            isActive = active.contains(layerAttribute.getNodeValue());
                        }

                        if (!isActive) {
                            continue;
                        }


                        String pathString = path.getTextContent();

                        FullTrajectory traj = parsePathString(pathString);
                        traj.setLabel(String.format("%s(%d)", f.getName(), trajectories.size()));
                        trajectories.add(traj);
                    }

                }
            }

        } catch (SAXException | IOException ex) {
            Logger.getLogger(IpeTrajectoryReader.class.getName()).log(Level.SEVERE, null, ex);
            Log.log(LogLevel.ERROR, "IperParser", "Exception thrown while parsing file: " + ex.getMessage());
        }
        return trajectories;
    }

    /**
     * Converts the path string to a trajectory.
     * <p>
     * The string is formatted as a repetition of the following pattern:
     * <p>
     * {@code <x-coord> <y-coord> <[ml]>}
     * <p>
     * each indicating a point (we ignore the m and l)
     *
     * @param s
     * @return
     */
    private FullTrajectory parsePathString(String s) {
        String[] split = s.split("[ml]"); // split on the character m or l

        // Create a list of points
        List<Point2D> points = Arrays.stream(split)
                .map(String::trim) // trim of whitespace
                .map(str -> str.split("\\s")) // split on whitespace
                .filter(arr -> arr.length == 2)
                .map(stringCoords -> new Point2D.Double(Double.parseDouble(stringCoords[0]), Double.parseDouble(stringCoords[1]))) // convert to point
                .collect(Collectors.toList());

        return new FullTrajectory(points);

    }

}
