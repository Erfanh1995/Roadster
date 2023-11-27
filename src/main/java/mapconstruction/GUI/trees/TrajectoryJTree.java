/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.trees;

import mapconstruction.trajectories.Trajectory;

import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;
import javax.swing.tree.TreeModel;
import javax.swing.tree.TreeNode;
import java.awt.*;
import java.util.Hashtable;
import java.util.Vector;

/**
 * @author Roel
 */
public class TrajectoryJTree extends ExpandedJTree {

    public TrajectoryJTree() {
        init();
    }

    public TrajectoryJTree(Object[] value) {
        super(value);
        init();
    }

    public TrajectoryJTree(Vector<?> value) {
        super(value);
        init();
    }

    public TrajectoryJTree(Hashtable<?, ?> value) {
        super(value);
        init();
    }

    public TrajectoryJTree(TreeNode root) {
        super(root);
        init();
    }

    public TrajectoryJTree(TreeNode root, boolean asksAllowsChildren) {
        super(root, asksAllowsChildren);
        init();
    }

    public TrajectoryJTree(TreeModel newModel) {
        super(newModel);
        init();
    }

    private void init() {
        setRootVisible(false);
        DefaultTreeCellRenderer renderer = new DefaultTreeCellRenderer();
        renderer.setLeafIcon(null);
        renderer.setOpenIcon(null);
        renderer.setClosedIcon(null);

        setCellRenderer(renderer);

        setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
    }

    @Override
    public String convertValueToText(Object value, boolean selected, boolean expanded, boolean leaf, int row, boolean hasFocus) {
        if (value instanceof DefaultMutableTreeNode) {
            DefaultMutableTreeNode node = (DefaultMutableTreeNode) value;

            Object nodeVal = node.getUserObject();
            if (nodeVal instanceof Trajectory) {
                Trajectory t = (Trajectory) nodeVal;
                return t.getLabel();
            }
        }

        return super.convertValueToText(value, selected, expanded, leaf, row, hasFocus); //To change body of generated methods, choose Tools | Templates.
    }

}
