package junit;

import static org.junit.Assert.*;

import java.util.LinkedList;
import java.util.List;

import org.junit.Test;

import graph.IGraph;
import graph.INode;
import graph.NodeVisitor;
import graph.impl.Graph;

public class TestXFS {
	static class OrderedNodeVisitor implements NodeVisitor {
		private List<String> nodes = new LinkedList<>();

		public List<String> getOrder() {
			return nodes;
		}

		@Override
		public void visit(INode node) {
			nodes.add(node.getName());
		}
	}

	@Test
	public void testBFS1() {
		/*
		 * Testing on the graph below. Plug into Graphviz at http://www.webgraphviz.com
		 * or https://dreampuf.github.io/GraphvizOnline/
		 * 
		 * graph bfs1 { A -- B A -- C A -- D D -- E }
		 * 
		 */
		IGraph g = new Graph();
		INode a = g.getOrCreateNode("A");
		INode b = g.getOrCreateNode("B");
		INode c = g.getOrCreateNode("C");
		INode d = g.getOrCreateNode("D");
		INode e = g.getOrCreateNode("E");
		a.addUndirectedEdgeToNode(b, 1);
		a.addUndirectedEdgeToNode(c, 1);
		a.addUndirectedEdgeToNode(d, 1);
		d.addUndirectedEdgeToNode(e, 1);

		OrderedNodeVisitor v = new OrderedNodeVisitor();
		g.breadthFirstSearch("A", v);

		List<String> order = v.getOrder();
		assertEquals("A", order.get(0));
		assertEquals("E", order.get(4));
	}

	@Test
	public void testBFS2() {
		/*
		 * Testing on the graph below. Plug into Graphviz at http://www.webgraphviz.com
		 * or https://dreampuf.github.io/GraphvizOnline/
		 * 
		 * graph bfs2 { A -- B A -- C A -- D B -- E C -- E D -- E E -- F F -- G }
		 * 
		 */
		IGraph g = new Graph();
		INode a = g.getOrCreateNode("A");
		INode b = g.getOrCreateNode("B");
		INode c = g.getOrCreateNode("C");
		INode d = g.getOrCreateNode("D");
		INode e = g.getOrCreateNode("E");
		INode f = g.getOrCreateNode("F");
		INode gn = g.getOrCreateNode("G");
		a.addUndirectedEdgeToNode(b, 1);
		a.addUndirectedEdgeToNode(c, 1);
		a.addUndirectedEdgeToNode(d, 1);
		b.addUndirectedEdgeToNode(e, 1);
		c.addUndirectedEdgeToNode(e, 1);
		d.addUndirectedEdgeToNode(e, 1);
		e.addUndirectedEdgeToNode(f, 1);
		f.addUndirectedEdgeToNode(gn, 1);

		OrderedNodeVisitor v = new OrderedNodeVisitor();
		g.breadthFirstSearch("A", v);

		List<String> order = v.getOrder();
		assertEquals("A", order.get(0));
		assertEquals("E", order.get(4));
		assertEquals("F", order.get(5));
		assertEquals("G", order.get(6));
	}

	@Test
	public void testDFS1() {
		/*
		 * Testing on this graph. Plug into Graphviz at http://www.webgraphviz.com or
		 * https://dreampuf.github.io/GraphvizOnline/
		 * 
		 * graph dfs1 { A -- B A -- C A -- D B -- E C -- E D -- E }
		 */
		IGraph g = new Graph();
		INode a = g.getOrCreateNode("A");
		INode b = g.getOrCreateNode("B");
		INode c = g.getOrCreateNode("C");
		INode d = g.getOrCreateNode("D");
		INode e = g.getOrCreateNode("E");
		a.addUndirectedEdgeToNode(b, 1);
		a.addUndirectedEdgeToNode(c, 1);
		a.addUndirectedEdgeToNode(d, 1);
		b.addUndirectedEdgeToNode(e, 1);
		c.addUndirectedEdgeToNode(e, 1);
		d.addUndirectedEdgeToNode(e, 1);

		OrderedNodeVisitor v = new OrderedNodeVisitor();
		g.depthFirstSearch("A", v);

		List<String> order = v.getOrder();
		System.out.println(order);
		assertEquals("A", order.get(0));
		assertEquals("E", order.get(2));
	}

}
