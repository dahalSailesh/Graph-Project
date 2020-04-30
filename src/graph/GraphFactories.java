package graph;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashSet;
import java.util.Scanner;
import java.util.Set;

import graph.impl.Graph;

/**
 * Static factory methods for creating graphs from different input file formats.
 * 
 * @author jspacco
 *
 */
public class GraphFactories
{
    /**
     * Static factory method for creating a graph from a list of connections
     * between nodes. The given InputStream will contain lines in the following format:
     * <pre>
     * A B
     * A C
     * B C
     * C D
     * </pre>
     * 
     * This would mean that there are edges from A to B, A to C, B to C, and C to D.
     * The graph is undirected and also unweighted, so the edge weights are not relevant.
     * 
     * @param in
     * @return
     * @throws IOException
     */
    public static IGraph createUndirectedGraphFromAdjacencyList(InputStream in)
    throws IOException
    {
        IGraph g = new Graph();
        Scanner scan = new Scanner(in);
        while (scan.hasNext()){
            INode src = g.getOrCreateNode(scan.next());
            INode dst = g.getOrCreateNode(scan.next());
            src.addUndirectedEdgeToNode(dst, 1);
        }
        scan.close();
        return g;
    }
    
    /**
     * Static factory that creates and returns a weighted, undirected
     * graph from list of edges and weights in the given InputStream.
     * The format will be one edge on each line:
     * <pre>
     * A B 3
     * B C 7
     * A D 8
     * </pre>
     * This means that there are edges between A and B with a cost of 3,
     * B and C with a cost of 7, and A and D with a cost of 8. All of these
     * edges are undirected, or directed in both directions.
     * 
     * @param in
     * @return
     * @throws IOException
     */
    public static IGraph createUndirectedWeightedGraphFromEdgeList(InputStream in)
    throws IOException
    {
        IGraph g = new Graph();
        Scanner scanner = new Scanner(in);
        while (scanner.hasNext()){
            String src=scanner.next();
            String dst=scanner.next();
            int cost=scanner.nextInt();
            INode srcNode=g.getOrCreateNode(src);
            INode dstNode=g.getOrCreateNode(dst);
            srcNode.addUndirectedEdgeToNode(dstNode, cost);
        }
        scanner.close();
        return g;
    }
    
    /**
     * Create a String representing the given graph in DOT format, suitable
     * for display with GraphViz. The graph is assumed to be
     * an <b>unweighted</b>, <b>undirected</b> graph, so the edges will not be labeled.  
     * The graph and its file will be named
     * according to the given graphname.
     * 
     * You can visualize DOT files on http://www.webgraphviz.com/
     * 
     * For example, this is a graph in DOT format:
     * <pre>
     * graph G {
     * A -- B
     * A -- C
     * B -- C
     * C -- D 
     * }
     * </pre>
     * 
     * Try this graph out at 
     * 
     * Note that edges should be listed only once, and should be given 
     * in alphabetical order (i.e. A -- B, not B -- A).
     * 
     * @param g
     * @param graphname 
     */
    public static String toUndirectedUnweightedDotFile(Graph g, String graphname)
    {
        StringBuilder buf = new StringBuilder();
        Set<String> set=new HashSet<String>();
        buf.append("graph "+graphname+" {\n");
        for (INode src : g.getAllNodes()) {
            for (INode dst : src.getNeighbors()) {
                String srcStr=src.getName();
                String dstStr=dst.getName();
                String key=srcStr+" "+dstStr;
                if (srcStr.compareTo(dstStr)>0) {
                    key=dstStr+" "+srcStr;
                }
                if (set.contains(key)){
                    continue;
                }
                set.add(key);
                buf.append(srcStr+" -- "+dstStr+"\n");
            }
        }
        buf.append("}\n");
        return buf.toString();
    }
    
    /**
     * Return a String containing the given graph in DOT format, suitable
     * for display with GraphViz. The graph is assumed to be
     * a <b>weighted</b>, <b>undirected</b> graph, so each edge will be labeled with
     * its weight. The graph and its file will be named
     * according to the given graphname.
     * 
     * You can visualize DOT files on http://www.webgraphviz.com/
     * <pre>
     * graph G {
     * A -- B [label=10];
     * A -- C [label=6];
     * B -- C [label=7];
     * C -- D [label=3];
     * }
     * </pre>
     * 
     * This would create a weighted undirected graph where there are edges
     * between A and B with weight 10, A and C with weight 6, B and C with weight 7,
     * and C and D with weight 3.
     * 
     * @param g the graph
     * @param graphname the name of the graph
     */
    public static String toUndirectedWeightedDotFile(IGraph g, String graphname)
    {
        StringBuilder buf = new StringBuilder();
        Set<String> set=new HashSet<String>();
        buf.append("graph "+graphname+" {\n");
        for (INode src : g.getAllNodes()) {
            for (INode dst : src.getNeighbors()) {
                String srcStr=src.getName();
                String dstStr=dst.getName();
                String key=srcStr+" "+dstStr;
                if (srcStr.compareTo(dstStr)>0){
                    key=dstStr+" "+srcStr;
                }
                if (set.contains(key)) {
                    continue;
                }
                set.add(key);
                int cost=src.getWeight(dst);
                buf.append(src.getName()+" -- "+dst.getName()+" [label="+cost+"];\n");
            }
        }
        buf.append("}\n");
        return buf.toString();
    }
    
    /**
     * Return a String containing the given graph in DOT format, suitable
     * for display with GraphViz. The graph is assumed to be
     * a weighted, directed graph, so each edge will be labeled with
     * its weight, as well as a direction. The graph and its file will be named
     * according to the given graphname.
     * 
     * You can visualize DOT files on http://www.webgraphviz.com/
     * <pre>
     * digraph G {
     * A -> B [label=10];
     * A -> C [label=6];
     * B -> C [label=7];
     * C -> D [label=3];
     * }
     * </pre>
     * 
     * This would create a weighted directed graph where there are edges
     * from A to B with weight 10, A to C with weight 6, B to C with weight 7,
     * and C to D with weight 3.
     * 
     * @param g the graph
     * @param graphname the name of the graph
     */
    public static String toDirectedWeightedDotFile(IGraph g, String graphname)
    {
        StringBuilder buf = new StringBuilder();
        Set<String> set=new HashSet<String>();
        buf.append("digraph "+graphname+" {\n");
        for (INode src : g.getAllNodes()) {
            for (INode dst : src.getNeighbors()) {
                String srcStr=src.getName();
                String dstStr=dst.getName();
                String key=srcStr+" "+dstStr;
                if (srcStr.compareTo(dstStr)>0){
                    key=dstStr+" "+srcStr;
                }
                if (set.contains(key)) {
                    continue;
                }
                set.add(key);
                int cost=src.getWeight(dst);
                buf.append(src.getName()+" -> "+dst.getName()+" [label="+cost+"];\n");
            }
        }
        buf.append("}\n");
        return buf.toString();
    }
    
    /**
     * Return a String containing the given graph in DOT format, suitable
     * for display with GraphViz. The graph is assumed to be
     * a <b>unweighted</b>, <b>directed</b> graph, so each edge will be labeled with
     * its weight, as well as a direction. The graph and its file will be named
     * according to the given graphname.
     * 
     * You can visualize DOT files on http://www.webgraphviz.com/
     * <pre>
     * digraph G {
     * A -> B;
     * A -> C;
     * B -> C;
     * C -> D;
     * }
     * </pre>
     * 
     * This would create an unweighted directed graph where there are edges
     * from A to B, A to C, B to C, and C to D.
     * 
     * @param g the graph
     * @param graphname the name of the graph
     */
    public static String toDirectedUnWeightedDotFile(IGraph g, String graphname)
    {
        StringBuilder buf = new StringBuilder();
        Set<String> set=new HashSet<String>();
        buf.append("digraph "+graphname+" {\n");
        for (INode src : g.getAllNodes()) {
            for (INode dst : src.getNeighbors()) {
                String srcStr=src.getName();
                String dstStr=dst.getName();
                String key=srcStr+" "+dstStr;
                if (srcStr.compareTo(dstStr)>0){
                    key=dstStr+" "+srcStr;
                }
                if (set.contains(key)) {
                    continue;
                }
                set.add(key);
                buf.append(src.getName()+" -> "+dst.getName()+";\n");
            }
        }
        buf.append("}\n");
        return buf.toString();
    }


    /**
     * Read a graph from a dotfile.
     * 
     * The graph is assumed to be weighted.
     * 
     * @param in
     * @return
     * @throws IOException
     */
    public static IGraph readFromDotFile(InputStream in)
    throws IOException
    {
        IGraph g = new Graph();
        Scanner scan=new Scanner(in);
        while (scan.hasNextLine()) {
            String line=scan.nextLine();
            line=line.replaceAll("--", "");
            line=line.replaceAll("\\];", "");
            line=line.replaceAll("\\[label=", "");
            if (line.equals("") || 
                    line.contains("{") ||
                    line.contains("}"))
            {
                continue;
            }
            System.out.println(line);
            Scanner lscan=new Scanner(line);
            String srcname=lscan.next();
            String dstname=lscan.next();
            int weight=lscan.nextInt();
            lscan.close();
            INode src=g.getOrCreateNode(srcname);
            INode dst=g.getOrCreateNode(dstname);
            src.addUndirectedEdgeToNode(dst, weight);
        }
        scan.close();
        return g;
    }
    
}
