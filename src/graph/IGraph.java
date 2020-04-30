package graph;

import java.util.Collection;
import java.util.Map;

public interface IGraph
{
    /**
     * Return the {@link Node} with the given name.
     * 
     * If no {@link Node} with the given name exists, create
     * a new node with the given name and return it. Subsequent
     * calls to this method with the same name should
     * then return the node just created.
     * 
     * @param name
     * @return
     */
    INode getOrCreateNode(String name);
    
    /**
     * Return true if the graph contains a node with the given name,
     * and false otherwise.
     * 
     * @param name
     * @return
     */
    boolean containsNode(String name);
    
    /**
     * Return a collection of all of the nodes in the graph.
     * 
     * Changes to this collection <b>SHOULD NOT</b> change the graph.
     * 
     * @return
     */
    Collection<INode> getAllNodes();
    
    /**
     * Perform a breadth-first search on the graph, starting at the node
     * with the given name. The visit method of the {@link NodeVisitor} should
     * be called on each node the first time we visit the node.
     * 
     * <b>NOTE:</b> This method visit nodes in alphabetical order, using the name of the
     * node to sort alphabetically.
     * 
     * @param startNodeName
     * @param v
     */
    void breadthFirstSearch(String startNode, NodeVisitor v);
    
    /**
     * Perform a depth-first search on the graph, starting at the node
     * with the given name. The visit method of the {@link NodeVisitor} should
     * be called on each node the first time we visit the node.
     * 
     * <b>NOTE:</b> This method visit nodes in alphabetical order, using the name of the
     * node to sort alphabetically.
     * 
     * @param startNodeName
     * @param v
     */
    void depthFirstSearch(String startNode, NodeVisitor v);
    
    /**
     * Perform Dijkstra's algorithm for computing the cost of the shortest path
     * to every node in the graph starting at the node with the given name.
     * Return a mapping from every node in the graph to the total minimum cost of reaching
     * that node from the given start node.
     * 
     * <b>Hint:</b> Creating a helper class called Path, which stores a destination
     * (String) and a cost (Integer), and making it implement Comparable, can be
     * helpful. Well, either than or repeated linear scans.
     * 
     * @param startName
     * @return
     */
    Map<INode,Integer> dijkstra(String sourceNode);
    
    /**
     * Perform Prim-Jarnik's algorithm to compute a Minimum Spanning Tree (MST).
     * 
     * The MST is itself a graph containing the same nodes (or at least nodes
     * with the same names) and a subset of the edges from the original graph.
     * 
     * @return
     */
    IGraph primJarnik();
}
