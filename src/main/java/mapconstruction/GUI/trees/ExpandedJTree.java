/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.trees;

import javax.swing.*;
import javax.swing.event.TreeModelEvent;
import javax.swing.event.TreeModelListener;
import javax.swing.tree.TreeModel;
import javax.swing.tree.TreeNode;
import java.util.Hashtable;
import java.util.Vector;

/**
 * JTree that is always expanded and can not be collapsed.
 *
 * @author Roel
 */
public class ExpandedJTree extends JTree {

    public ExpandedJTree() {
        init();
    }

    public ExpandedJTree(Object[] value) {
        super(value);
        init();
    }

    public ExpandedJTree(Vector<?> value) {
        super(value);
        init();
    }

    public ExpandedJTree(Hashtable<?, ?> value) {
        super(value);
        init();
    }

    public ExpandedJTree(TreeNode root) {
        super(root);
        init();
    }

    public ExpandedJTree(TreeNode root, boolean asksAllowsChildren) {
        super(root, asksAllowsChildren);
        init();
    }

    public ExpandedJTree(TreeModel newModel) {
        super(newModel);
        init();
    }

    private void init() {
        this.setToggleClickCount(0);
    }


    /**
     * Creates model listener that makes sure the tree is always expanded.
     *
     * @return
     */
    @Override
    protected TreeModelListener createTreeModelListener() {
        return new JTree.TreeModelHandler() {
            @Override
            public void treeNodesRemoved(TreeModelEvent e) {
                super.treeNodesRemoved(e);
                expandAll();
            }

            @Override
            public void treeStructureChanged(TreeModelEvent e) {
                super.treeStructureChanged(e);
                expandAll();
            }

        };
    }

    private void expandAll() {
        for (int i = 0; i < this.getRowCount(); i++) {
            this.expandRow(i);
        }
    }
}
