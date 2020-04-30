package graph;

/**
 * Functor for visiting a node.
 * 
 * An instance of this class is passed to BFS or DFS,
 * and the visit() method will be called on each node. 
 * 
 * @author jspacco
 *
 */
public interface NodeVisitor
{
    /**
     * Method for visiting a node. The exact behavior is undefined, and is
     * left to the implementing subclass (or abstract inner class).
     * 
     * @param node
     */
    public void visit(INode node);
}
