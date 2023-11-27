/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mapconstruction.GUI.trees;

import mapconstruction.GUI.filter.*;
import mapconstruction.GUI.filter.AttributeTriPredicate.Operator;
import mapconstruction.GUI.filter.QuantifiedTriPredicate.Quantifier;
import mapconstruction.attributes.BundleClassAttribute;
import mapconstruction.util.ToDoubleTriFunction;

import javax.swing.event.TreeModelEvent;
import javax.swing.event.TreeModelListener;
import javax.swing.tree.TreeModel;
import javax.swing.tree.TreePath;
import java.util.ArrayList;
import java.util.List;


/**
 * Tree model to build a predicate to filter the bundles.
 *
 * @author Roel
 */
public class BundleFilterTreeModel implements TreeModel {

    /**
     * Listeners for this model.
     */
    private final List<TreeModelListener> listeners;
    /**
     * Root of the predicate hierarchy is a (possibly empty) composite
     * predicate.
     */
    private CompositeTriPredicate<?, ?, ?> root;

    public BundleFilterTreeModel(CompositeTriPredicate<?, ?, ?> root) {
        this.root = root;
        this.listeners = new ArrayList<>();
    }

    //////////////// Fire events //////////////////////////////////////////////

    /**
     * The only event raised by this model is TreeStructureChanged with the root
     * as path, i.e. the whole tree has changed.
     *
     * @param oldRoot
     */
    protected void fireTreeStructureChanged(CompositeTriPredicate<?, ?, ?> oldRoot) {
        TreeModelEvent e = new TreeModelEvent(this,
                new Object[]{oldRoot});
        listeners.stream().forEach((tml) -> {
            tml.treeStructureChanged(e);
        });
    }

    protected void firetreeNodesInserted(CompositeTriPredicate<?, ?, ?> parent, TriPredicate<?, ?, ?> child, int childIndex) {
        TreeModelEvent e = new TreeModelEvent(this, pathToNode(parent), new int[]{childIndex}, new Object[]{child});
        listeners.stream().forEach((tml) -> {
            tml.treeNodesInserted(e);
        });
    }

    protected void firetreeNodesRemoved(CompositeTriPredicate<?, ?, ?> parent, TriPredicate<?, ?, ?> child, int childIndex) {
        TreeModelEvent e = new TreeModelEvent(this, pathToNode(parent), new int[]{childIndex}, new Object[]{child});
        listeners.stream().forEach((tml) -> {
            tml.treeNodesRemoved(e);
        });
    }

    protected void firetreeNodesChanged(TriPredicate<?, ?, ?> changedNode) {
        TreePath parentPath = pathToNode(changedNode).getParentPath();
        boolean isRoot = parentPath == null;

        TreeModelEvent e = new TreeModelEvent(this,
                parentPath,
                isRoot ? null : new int[]{((CompositeTriPredicate) parentPath.getLastPathComponent()).indexOf(changedNode)},
                isRoot ? null : new Object[]{changedNode}
        );
        listeners.stream().forEach((tml) -> {
            tml.treeNodesChanged(e);
        });
    }

    /**
     * Fire all treeNodeChanged events for the nodes on the given path
     *
     * @param changedpath
     */
    protected void firetreeNodesChangedOnPath(TreePath changedpath) {

        TreePath current = changedpath;
        TreePath parentPath;
        while ((parentPath = current.getParentPath()) != null) {
            TriPredicate node = (TriPredicate) current.getLastPathComponent();
            TreeModelEvent e = new TreeModelEvent(this,
                    parentPath,
                    new int[]{((CompositeTriPredicate) parentPath.getLastPathComponent()).indexOf(node)},
                    new Object[]{node}
            );
            listeners.stream().forEach((tml) -> {
                tml.treeNodesChanged(e);
            });
            current = parentPath;
        }
        listeners.stream().forEach((tml) -> {
            Object[] oo = null;
            tml.treeNodesChanged(new TreeModelEvent(this, oo, null, null));
        });

    }

    //////////////// Interface implementation /////////////////////////////////
    @Override
    public Object getRoot() {
        return root;
    }

    /**
     * Sets the root.
     *
     * @param root
     */
    public void setRoot(CompositeTriPredicate<?, ?, ?> root) {
        CompositeTriPredicate<?, ?, ?> rt = root;
        this.root = root;
        fireTreeStructureChanged(rt);
    }

    @Override
    public Object getChild(Object parent, int index) {
        if (isLeaf(parent)) {
            return null;
        }

        CompositeTriPredicate composite = (CompositeTriPredicate) parent;
        if (index >= 0 && index < composite.size()) {
            return composite.get(index);
        } else {
            return null;
        }

    }

    @Override
    public int getChildCount(Object parent) {
        if (isLeaf(parent)) {
            return 0;
        } else {
            CompositeTriPredicate composite = (CompositeTriPredicate) parent;
            return composite.size();
        }
    }

    @Override
    public boolean isLeaf(Object node) {
        return !(node instanceof CompositeTriPredicate);
    }

    @Override
    public void valueForPathChanged(TreePath path, Object newValue) {
        // No op
    }

    @Override
    public int getIndexOfChild(Object parent, Object child) {
        if (parent == null || child == null) {
            return -1;
        }
        if (isLeaf(parent)) {
            return -1;
        }

        CompositeTriPredicate composite = (CompositeTriPredicate) parent;
        return composite.indexOf((TriPredicate) child);
    }

    @Override
    public void addTreeModelListener(TreeModelListener l) {
        listeners.add(l);
    }
    //////////////// Additional helper methods /////////////////////////////////

    @Override
    public void removeTreeModelListener(TreeModelListener l) {
        listeners.remove(l);
    }

    public <T, U, V> void addToParent(DynamicSizeCompositeTriPredicate<T, U, V> parent, TriPredicate<T, U, V> child) {
        parent.add(child);
        firetreeNodesInserted(parent, child, parent.size() - 1);
        firetreeNodesChanged(parent);
        firetreeNodesChangedOnPath(pathToNode(parent));
    }

    public <T, U, V> void removeFromParent(DynamicSizeCompositeTriPredicate<T, U, V> parent, TriPredicate<T, U, V> child) {
        int index = parent.indexOf(child);
        parent.remove(child);
        firetreeNodesRemoved(parent, child, index);
        firetreeNodesChangedOnPath(pathToNode(parent));
    }

    public <T, U, V> void setAtParent(CompositeTriPredicate<T, U, V> parent, TriPredicate<T, U, V> child, int index) {
        TriPredicate<T, U, V> oldChild = parent.set(index, child);
        firetreeNodesRemoved(parent, oldChild, index);
        firetreeNodesInserted(parent, child, index);
        firetreeNodesChangedOnPath(pathToNode(parent));
    }

    public <T, U, V> void setQuantifier(QuantifiedTriPredicate<T, U, V> pred, Quantifier c) {
        pred.setQuantifier(c);
        firetreeNodesChangedOnPath(pathToNode(pred));
    }

    public void setAttributePredicateProperties(AttributeTriPredicate pred, BundleClassAttribute attribute, Operator operator, ToDoubleTriFunction value) {
        pred.setAttribute(attribute);
        pred.setOperator(operator);
        pred.setTestValue(value);
        firetreeNodesChangedOnPath(pathToNode(pred));
    }


    private TreePath pathToNode(TriPredicate<?, ?, ?> goal) {
        List<TriPredicate<?, ?, ?>> path = new ArrayList<>();
        path.add(root);
        boolean result = pathToNode(goal, path);
        if (!result) {
            return null;
        } else {
            return new TreePath(path.toArray());
        }

    }

    private boolean pathToNode(TriPredicate<?, ?, ?> goal, List<TriPredicate<?, ?, ?>> path) {
        TriPredicate<?, ?, ?> last = path.get(path.size() - 1);
        if (last.equals(goal)) {
            return true;
        }
        if (last instanceof CompositeTriPredicate) {
            for (TriPredicate<?, ?, ?> pred : ((CompositeTriPredicate<?, ?, ?>) last).getPredicates()) {
                path.add(pred);
                if (pathToNode(goal, path)) {
                    return true;
                } else {
                    path.remove(path.size() - 1);
                }
            }
            return false;
        } else {
            return false;
        }
    }

}
