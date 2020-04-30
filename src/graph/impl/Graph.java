package graph.impl;

import java.io.FileInputStream;
import java.util.Collection;
import java.util.Queue;
import java.util.LinkedList;
import java.util.Stack;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.HashSet;
import java.util.PriorityQueue;

import graph.GraphFactories;
import graph.IGraph;
import graph.INode;
import graph.NodeVisitor;

/**
 * A basic representation of a graph that can perform BFS, DFS, Dijkstra, and
 * Prim-Jarnik's algorithm for a minimum spanning tree.
 * 
 * @author jspacco
 *
 */
public class Graph implements IGraph {
	private Map<String, INode> nodes;

	public Graph() {
		this.nodes = new HashMap<String, INode>();
	}

	/**
	 * Return the {@link Node} with the given name.
	 * 
	 * If no {@link Node} with the given name exists, create a new node with the
	 * given name and return it. Subsequent calls to this method with the same name
	 * should then return the node just created.
	 * 
	 * @param name
	 * @return
	 */
	public INode getOrCreateNode(String name) {
		if (this.nodes.containsKey(name)) {
			return this.nodes.get(name);
		}

		INode newNode = new Node(name);
		this.nodes.put(name, newNode);
		return newNode;
	}

	/**
	 * Return true if the graph contains a node with the given name, and false
	 * otherwise.
	 * 
	 * @param name
	 * @return
	 */
	public boolean containsNode(String name) {
		return this.nodes.containsKey(name);

	}

	/**
	 * Return a collection of all of the nodes in the graph.
	 * 
	 * @return
	 */
	public Collection<INode> getAllNodes() {
		return this.nodes.values();
	}

	/**
	 * Perform a breadth-first search on the graph, starting at the node with the
	 * given name. The visit method of the {@link NodeVisitor} should be called on
	 * each node the first time we visit the node.
	 * 
	 * 
	 * @param startNodeName
	 * @param v
	 */
	public void breadthFirstSearch(String startNodeName, NodeVisitor v) {

		INode head = this.nodes.get(startNodeName);
		Queue<INode> queue = new LinkedList<INode>();
		Set<INode> visited = new HashSet<INode>();
		queue.add(head);

		while (!queue.isEmpty()) {
			INode curr = queue.poll();

			System.out.println(curr.getName());
			if (visited.contains(curr)) {
				continue;
			}
			v.visit(curr);
			visited.add(curr);
			Collection<INode> currNeighbors = curr.getNeighbors();
			for (INode neighbor : currNeighbors) {
				if (!visited.contains(neighbor)) {
					queue.add(neighbor);
				}
			}
		}
	}

	/**
	 * Perform a depth-first search on the graph, starting at the node with the
	 * given name. The visit method of the {@link NodeVisitor} should be called on
	 * each node the first time we visit the node.
	 * 
	 * 
	 * @param startNodeName
	 * @param v
	 */
	public void depthFirstSearch(String startNodeName, NodeVisitor v) {
		INode head = this.nodes.get(startNodeName);
		Stack<INode> stack = new Stack<INode>();
		Set<INode> visited = new HashSet<INode>();
		stack.push(head);

		while (!stack.isEmpty()) {
			INode curr = stack.pop();
			if (visited.contains(curr)) {
				continue;
			}
			v.visit(curr);
			visited.add(curr);
			Collection<INode> currNeighbors = curr.getNeighbors();
			for (INode neighbor : currNeighbors) {
				if (!visited.contains(neighbor)) {
					stack.push(neighbor);
				}
			}
		}
	}

	/**
	 * Perform Dijkstra's algorithm for computing the cost of the shortest path to
	 * every node in the graph starting at the node with the given name. Return a
	 * mapping from every node in the graph to the total minimum cost of reaching
	 * that node from the given start node.
	 * 
	 * <b>Hint:</b> Creating a helper class called Path, which stores a destination
	 * (String) and a cost (Integer), and making it implement Comparable, can be
	 * helpful. Well, either than or repeated linear scans.
	 * 
	 * @param startName
	 * @return
	 */

	private class Path implements Comparable<Path> {
		private String startName;
		private int cost;

		private Path(String startName, int cost) {
			this.startName = startName;
			this.cost = cost;
		}

		private INode getNode() {
			return nodes.get(this.startName);
		}

		private int getCost() {
			return this.cost;
		}

		@Override
		public int compareTo(Path o) {
			return this.cost - o.cost;

		}

	}

	public Map<INode, Integer> dijkstra(String startName) {
		INode head = this.nodes.get(startName);
		Map<INode, Integer> result = new HashMap<INode, Integer>();
		PriorityQueue<Path> pq = new PriorityQueue<Path>();
		pq.add(new Path(startName, 0));

		while (result.size() < this.nodes.size()) {
			Path nextPath = pq.poll();
			INode node = nextPath.getNode();
			if (result.containsKey(node)) {
				continue;
			}
			// the cost to get to the current Path
			int cost = nextPath.getCost();
			result.put(node, cost);

			for (INode n : node.getNeighbors()) {
				pq.add(new Path(n.getName(), cost + node.getWeight(n)));
			}

		}
		return result;

	}

	/**
	 * Perform Prim-Jarnik's algorithm to compute a Minimum Spanning Tree (MST).
	 * 
	 * The MST is itself a graph containing the same nodes and a subset of the edges
	 * from the original graph.
	 * 
	 * @return
	 */

	private class Edge implements Comparable<Edge> {
		private String source;
		private String dest;
		private int weight;

		private Edge(int weight, String source, String dest) {
			this.weight = weight;
			this.source = source;
			this.dest = dest;
		}

		private int getWeight() {
			return this.weight;
		}

		private String getSource() {
			return this.source;
		}

		private String getDest() {
			return this.dest;
		}

		@Override
		public int compareTo(Edge o) {
			return this.weight - o.weight;
		}

	}

	public IGraph primJarnik() {

		PriorityQueue<Edge> pq = new PriorityQueue<Edge>();
		HashSet<INode> visited = new HashSet<INode>();

		INode head = (INode) this.getAllNodes().toArray()[0];
		visited.add(head);

		// adding head's neighbors in the priority queue
		HashSet<Edge> mstEdges = new HashSet<Edge>();
		for (INode neighbor : head.getNeighbors()) {
			Edge newEdge = new Edge(head.getWeight(neighbor), head.getName(), neighbor.getName());
			pq.add(newEdge);
		}

		int graphSize = this.getAllNodes().size();
		while (visited.size() < graphSize) {
			Edge currEdge = pq.poll();

			INode currNode = this.nodes.get(currEdge.getDest());
			mstEdges.add(currEdge);
			visited.add(currNode);

			for (INode neighbor : currNode.getNeighbors()) {
				if (!visited.contains(neighbor)) {
					Edge newEdge = new Edge(currNode.getWeight(neighbor), currNode.getName(), neighbor.getName());
					pq.add(newEdge);
				}
			}

		}

		return constructGraph(mstEdges);

	}

	private IGraph constructGraph(HashSet<Edge> mstEdges) {
		IGraph mst = new Graph();

		for (Edge edge : mstEdges) {
			String source = edge.getSource();
			String dest = edge.getDest();

			INode srcNode = mst.getOrCreateNode(source);
			INode dstNode = mst.getOrCreateNode(dest);
			srcNode.addUndirectedEdgeToNode(dstNode, edge.getWeight());

		}
		System.out.println(mst.getAllNodes().size());
		return mst;
	}

}