package junit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;

import org.junit.Test;

import graph.IGraph;
import graph.INode;
import graph.impl.Graph;
import graph.impl.Node;

public class TestGraphBasic {

	@Test
	public void testNodeConstructor() {
		INode n = new Node("A");
		assertEquals("A", n.getName());
		assertEquals(0, n.getNeighbors().size());
	}

	@Test
	public void testAddUndirectedEdges() {
		INode n1 = new Node("A");
		INode n2 = new Node("B");
		n1.addUndirectedEdgeToNode(n2, 5);
		assertEquals(5, n1.getWeight(n2));
		assertEquals(5, n2.getWeight(n1));
	}

	@Test
	public void testAddDirectedEdge() {
		INode n1 = new Node("A");
		INode n2 = new Node("B");
		n1.addDirectedEdgeToNode(n2, 6);
		assertEquals(6, n1.getWeight(n2));
		try {
			n2.getWeight(n1);
			fail("Should have thrown an exception");
		} catch (Exception e) {
			// do nothing; we are supposed to get an exception
		}
	}

	@Test
	public void testNeighbors() {
		INode n1 = new Node("A");
		INode n2 = new Node("B");
		INode n3 = new Node("C");
		INode n4 = new Node("D");
		INode n5 = new Node("E");

		n1.addUndirectedEdgeToNode(n2, 1);
		n1.addUndirectedEdgeToNode(n3, 1);
		n2.addUndirectedEdgeToNode(n4, 1);
		n5.addUndirectedEdgeToNode(n1, 2);

		String[] neighbors = new String[n1.getNeighbors().size()];
		int i = 0;
		for (INode n : n1.getNeighbors()) {
			neighbors[i] = n.getName();
			i++;
		}
		// arrays can be sorted
		Arrays.sort(neighbors);
		// this should be the correct result
		String[] correct = new String[] { "B", "C", "E" };
		// make sure we have the same neighbors we are expecting
		for (int j = 0; j < correct.length; j++) {
			assertEquals(correct[j], neighbors[j]);
		}
	}

	@Test
	public void testAddNode() throws Exception {
		IGraph g = new Graph();
		INode n1 = g.getOrCreateNode("A");
		assertTrue(g.containsNode("A"));
		INode n2 = g.getOrCreateNode("A");
		// should get back the same instance
		assertTrue(n1 == n2);
		assertEquals("A", n2.getName());
		assertEquals(0, n2.getNeighbors().size());
	}

	@Test
	public void testAddTwoConnectedNode() throws Exception {
		IGraph g = new Graph();
		INode n1 = g.getOrCreateNode("A");
		INode n2 = g.getOrCreateNode("B");
		n1.addUndirectedEdgeToNode(n2, 5);
		assertEquals(2, g.getAllNodes().size());
		assertEquals(1, n1.getNeighbors().size());
		assertEquals(1, n2.getNeighbors().size());
	}

	@Test
	public void testEdgesAreUpdated() {
		IGraph g = new Graph();
		INode n1 = g.getOrCreateNode("A");
		INode n2 = g.getOrCreateNode("B");
		n1.addUndirectedEdgeToNode(n2, 5);
		assertEquals(5, n1.getWeight(n2));
		assertEquals(5, n2.getWeight(n1));
		n1.addUndirectedEdgeToNode(n2, 8);
		assertEquals(8, n1.getWeight(n2));
		assertEquals(8, n2.getWeight(n1));
	}

	@Test
	public void testRemoveUndirectedEdges() {
		IGraph g = new Graph();
		INode n1 = g.getOrCreateNode("A");
		INode n2 = g.getOrCreateNode("B");
		n1.addUndirectedEdgeToNode(n2, 5);
		assertEquals(5, n1.getWeight(n2));
		assertEquals(5, n2.getWeight(n1));
		n1.removeUndirectedEdgeToNode(n2);
		assertFalse(n1.hasEdge(n2));
		assertFalse(n2.hasEdge(n1));
	}

	@Test
	public void testRemoveDirectedEdges() {
		IGraph g = new Graph();
		INode n1 = g.getOrCreateNode("A");
		INode n2 = g.getOrCreateNode("B");
		n1.addDirectedEdgeToNode(n2, 5);
		n2.addDirectedEdgeToNode(n1, 7);
		assertEquals(5, n1.getWeight(n2));
		assertEquals(7, n2.getWeight(n1));
		n1.removeDirectedEdgeToNode(n2);
		assertFalse(n1.hasEdge(n2));
		assertEquals(7, n2.getWeight(n1));
	}
}
