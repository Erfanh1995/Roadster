package mapconstruction.GUI.listeners;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Listener to allow zooming and panning of a drawing on a pannel.
 * <p>
 * panning is done by click+drag; zooming is done by mousewheel.
 *
 * @author Roel
 */
public class ZoomAndPanListener implements MouseListener, MouseMotionListener, MouseWheelListener {

    private static final double ZOOM_MULTIPLIER = 1.1;

    /**
     * Component this listener is attached to.
     */
    private Component target;

    /**
     * Point where the dragging starts
     */
    private Point2D dragStart;

    /**
     * Transformation matrix on which the zooming and pannign works.
     */
    private AffineTransform transform;

    /**
     * Creates the zoom and pan listener on the given target.
     *
     * @param target
     */
    public ZoomAndPanListener(Component target) {
        this(target, new AffineTransform());
    }

    /**
     * Creates the zoom and pan listener on the given target and the gicen inital transform.
     *
     * @param target
     * @param initialTransform
     */
    public ZoomAndPanListener(Component target, AffineTransform initialTransform) {
        this.target = target;
        this.transform = initialTransform;
    }

    public AffineTransform getTransform() {
        return transform;
    }

    public void setTransform(AffineTransform transform) {
        this.transform = transform;
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        if (SwingUtilities.isLeftMouseButton(e)) {
            moveCamera(e);
        }

    }

    @Override
    public void mouseMoved(MouseEvent e) {
        // noop
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        // Noop
    }

    @Override
    public void mousePressed(MouseEvent e) {

        if (SwingUtilities.isLeftMouseButton(e)) {
            dragStart = e.getPoint();
        }

    }

    @Override
    public void mouseReleased(MouseEvent e) {
        // Noop
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        // Noop
    }

    @Override
    public void mouseExited(MouseEvent e) {
        // Noop
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        zoomCamera(e);
    }

    /**
     * Apply correct transformations to simulate the panning.
     *
     * @param e
     */
    private void moveCamera(MouseEvent e) {
        try {
            Point2D start = transform.inverseTransform(dragStart, null);
            Point2D end = transform.inverseTransform(e.getPoint(), null);
            dragStart = e.getPoint();
            double dx = end.getX() - start.getX();
            double dy = end.getY() - start.getY();
            transform.translate(dx, dy);
            target.repaint();

        } catch (NoninvertibleTransformException ex) {
            Logger.getLogger(ZoomAndPanListener.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

    /**
     * Apply correct transformations to simulate zooming
     *
     * @param e
     */
    private void zoomCamera(MouseWheelEvent e) {
        try {
            // In case of negative rotation (up), zoom in, otherwise zoom out
            double scaler = (e.getPreciseWheelRotation() < 0)
                    ? -e.getPreciseWheelRotation() * ZOOM_MULTIPLIER
                    : 1 / (e.getPreciseWheelRotation() * ZOOM_MULTIPLIER);

            Point2D before = transform.inverseTransform(e.getPoint(), null);
            transform.scale(scaler, scaler);
            Point2D after = transform.inverseTransform(e.getPoint(), null);

            double dx = after.getX() - before.getX();
            double dy = after.getY() - before.getY();

            transform.translate(dx, dy);
            target.repaint();
        } catch (NoninvertibleTransformException ex) {
            Logger.getLogger(ZoomAndPanListener.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

}
