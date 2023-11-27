package mapconstruction.benchmark;

import javafx.scene.shape.StrokeType;
import mapconstruction.trajectories.Bundle;
import mapconstruction.trajectories.Trajectory;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;

public class BundleRenderer {

    private Color background;

    private BufferedImage c;
    private Graphics2D g;
    private Rectangle bb;

    /**
     *  Colors borrowed from <a href="https://www.materialpalette.com/colors"> here </a>
     */
    private final static Color[] colors = new Color[]{
            new Color(244, 67, 54),     // RED
            new Color(233, 30, 99),     // PINK
            new Color(156, 39, 176),    // PURPLE
            new Color(103, 58, 183),    // DEEP PURPLE
            new Color(63, 81, 181),     // INDIGO
            new Color(33, 150, 243),    // BLUE
            new Color(3, 169, 244),     // LIGHT BLUE
            new Color(0, 188, 212),     // CYAN
            new Color(0, 150, 136),     // TEAL
            new Color(76, 175, 80),     // GREEN
            new Color(139, 195, 74),    // LIGHT GREEN
            new Color(205, 220, 57),    // LIME
            new Color(255, 235, 59),    // YELLOW
            new Color(255, 193, 7),     // AMBER
            new Color(255, 152, 0),     // ORANGE
            new Color(255, 87, 34),     // DEEP ORANGE
            new Color(121, 85, 72),     // BROWN
            new Color(158, 158, 158),   // GRAY
            new Color(96, 125, 139),    // BLUE GRAY
    };

    public BundleRenderer(Rectangle boundingBox) {
        this(boundingBox, Color.WHITE);
    }

    public BundleRenderer(Rectangle boundingBox, Color background) {
        this(500, 500, boundingBox, background);
    }

    public BundleRenderer(int width, int height, Rectangle boundingBox, Color background) {
        this.background = background;

        c = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        g = getGraphics(c);
        bb = boundingBox;
    }

    /**
     * Get the Graphics2D instance from a buffered image and set the required options
     */
    private Graphics2D getGraphics(BufferedImage source) {
        Graphics2D graphics = source.createGraphics();
        graphics.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        graphics.setBackground(this.background);
        graphics.clearRect(0, 0, source.getWidth(), source.getHeight());
        return graphics;
    }

    /**
     * Draw a bundle onto the current canvas
     */
    public void draw(Bundle b) {
        Trajectory t = b.getOriginalRepresentative();
        Point2D p1 = t.getPoint(0); Point2D p2;
        for(int i = 0; i < t.numPoints() - 1; i++) {
            p2 = t.getPoint(i+1);
            int p1x = (int) (c.getWidth() * (p1.getX() - bb.x) / (double) bb.width);
            int p1y = (int) (c.getHeight() * (p1.getY() - bb.y) / (double) bb.height);
            int p2x = (int) (c.getWidth() * (p2.getX() - bb.x) / (double) bb.width);
            int p2y = (int) (c.getHeight() * (p2.getY() - bb.y) / (double) bb.height);
            g.setColor(colors[b.size() % colors.length]);
            g.setStroke(new BasicStroke(b.size()));
            g.drawLine(p1x, p1y, p2x, p2y);
            p1 = p2;
        }
    }

    /**
     * Export the current canvas and create a new one
     */
    public BufferedImage make() {
        BufferedImage result = c;
        c = new BufferedImage(result.getWidth(), result.getHeight(), result.getType());
        g = getGraphics(c);
        return result;
    }

}
