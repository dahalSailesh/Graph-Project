package graph;

import java.util.Stack;

import graph.impl.Graph;

/**
 * Pretty cool class for creating "grid graphs", which are graphs with the nodes
 * arranged on a grid of rows and columns. Perfomring a DFS on this graph creates
 * a maze.
 * 
 * @author jspacco
 *
 */
public class GridGraph
{
    public static IGraph makeGridGraph(int rows, int cols) {
        IGraph graph = new Graph();
        // Each node is named something like r2c1, which means "row 2, column 1".
        // Nodes have edges to other nodes that are to their left or right, and
        // above or below.
        for (int r = 0; r < rows; r++){
            for (int c=0; c < cols; c++){
                if (r > 0) {
                    INode current = graph.getOrCreateNode(String.format("r%dc%d", r, c));
                    INode other = graph.getOrCreateNode(String.format("r%dc%d", r-1, c));
                    current.addUndirectedEdgeToNode(other, 1);
                }
                if (r < rows-1) {
                    INode current = graph.getOrCreateNode(String.format("r%dc%d", r, c));
                    INode other = graph.getOrCreateNode(String.format("r%dc%d", r+1, c));
                    current.addUndirectedEdgeToNode(other, 1);
                }
                if (c > 0) {
                    INode current = graph.getOrCreateNode(String.format("r%dc%d", r, c));
                    INode other = graph.getOrCreateNode(String.format("r%dc%d", r, c-1));
                    current.addUndirectedEdgeToNode(other, 1);
                }
                if (c < cols-1) {
                    INode current = graph.getOrCreateNode(String.format("r%dc%d", r, c));
                    INode other = graph.getOrCreateNode(String.format("r%dc%d", r, c+1));
                    current.addUndirectedEdgeToNode(other, 1);
                }
            }
        }
        return graph;
    }
    
    public static void main(String[] args){
        int rows = 5;
        int cols = 5;
        
        IGraph g = makeGridGraph(rows, cols);

        // This code generates a maze with the given number of rows and cols
        // using DFS, and prints it out in Graphviz format. 
        System.out.printf("graph gr {\n");
        
        
        g.depthFirstSearch("r0c0", new NodeVisitor() {
            Stack<INode> nodes = new Stack<INode>();
            @Override
            public void visit(INode node) {
                // Go through the stack of nodes we've already seen,
                // and connect to it.
                for (INode n : nodes){
                    if (node.hasEdge(n)){
                        System.out.printf("%s -- %s;\n", n.getName(), node.getName());
                        break;
                    }
                }
                nodes.push(node);
            }
        });
        
        // This creates a bunch of "rank=same" directives that tell graphviz to lay out
        // the graph a certain way.
        for (int r=0; r<rows; r++){
            String rank = "";
            for (int c=0; c<cols; c++){
                String label = String.format("r%dc%d", r, c);
                //System.out.printf("%s [\n\tlabel = %s\npos = \"%d,%d\" ];\n", label, label, r, c);
                rank += label + ", ";
            }
            // remove last comma
            rank = rank.substring(0, rank.length()-2);
            System.out.printf("{ rank=same; %s }\n", rank);
        }
        
        System.out.printf("}\n");
    }
    
}
