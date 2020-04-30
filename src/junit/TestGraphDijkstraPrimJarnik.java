package junit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.FileInputStream;
import java.util.Map;

import org.junit.Test;

import graph.GraphFactories;
import graph.IGraph;
import graph.INode;

public class TestGraphDijkstraPrimJarnik {

	/**
	 * tests/dijkstra1.txt contains the graph that we did in class. The file looks
	 * like this:
	 * 
	 * 
	 * A C 10 A G 25 A B 3 B E 5 E F 3 C F 2 C G 1
	 * 
	 * As graphviz file here:
	 * 
	 * graph dijkstra1 { A -- C [label=10]; A -- G [label=25]; A -- B [label=3]; A
	 * -- D [label=2]; B -- E [label=5]; E -- F [label=3]; C -- F [label=2]; C -- G
	 * [label=1]; }
	 * 
	 * Dijkstra cares about the weights, so note that we are using the
	 * createUndirectedWeightedGraphFromEdgeList method.
	 * 
	 * 
	 */
	@Test
	public void testDijkstra1() throws Exception {

		IGraph g = GraphFactories.createUndirectedWeightedGraphFromEdgeList(new FileInputStream("tests/dijkstra1.txt"));
		Map<INode, Integer> shortPaths = g.dijkstra("A");

		assertEquals(7, shortPaths.size());

		assertEquals(0, (int) shortPaths.get(g.getOrCreateNode("A")));
		assertEquals(2, (int) shortPaths.get(g.getOrCreateNode("D")));
		assertEquals(3, (int) shortPaths.get(g.getOrCreateNode("B")));
		assertEquals(8, (int) shortPaths.get(g.getOrCreateNode("E")));
		assertEquals(10, (int) shortPaths.get(g.getOrCreateNode("C")));
		assertEquals(11, (int) shortPaths.get(g.getOrCreateNode("F")));
		assertEquals(11, (int) shortPaths.get(g.getOrCreateNode("G")));
	}

	/**
	 * Prim-Jarnik cares about edge weights, so use the
	 * createUndirectedWeightedGraphFromEdgeList static factor method to create the
	 * graph.
	 *
	 * Uses the same graph as the test for dijkstra above.
	 * 
	 * Remember that Prim-Jarnik can have multiple correct solutions, so make sure
	 * your test cases have only one correct solution.
	 * 
	 */
	@Test
	public void testPrimJarnik1() throws Exception {
		IGraph graph = GraphFactories
				.createUndirectedWeightedGraphFromEdgeList(new FileInputStream("tests/dijkstra1.txt"));
		IGraph g2 = graph.primJarnik();

		// TODO: put some assertions here to make sure that g2 works
		INode a = g2.getOrCreateNode("A");
		INode b = g2.getOrCreateNode("B");
		INode c = g2.getOrCreateNode("C");
		INode d = g2.getOrCreateNode("D");
		INode e = g2.getOrCreateNode("E");
		INode f = g2.getOrCreateNode("F");
		INode g = g2.getOrCreateNode("G");

		assertEquals(1, c.getWeight(g));
		assertFalse(g.hasEdge(a));
		assertFalse(c.hasEdge(a));
		assertEquals(2, c.getWeight(f));
		assertEquals(3, f.getWeight(e));
		assertEquals(5, e.getWeight(b));
		assertEquals(3, b.getWeight(a));
		assertEquals(2, a.getWeight(d));
		assertFalse(d.hasEdge(e));
	}
}
