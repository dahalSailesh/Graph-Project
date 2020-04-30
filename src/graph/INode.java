package graph;

import java.util.Collection;

public interface INode
{
    String getName();
    
    Collection<INode> getNeighbors();
    
    void addDirectedEdgeToNode(INode neighbor, int weight);
    
    void addUndirectedEdgeToNode(INode neighbor, int weight);
    
    void removeDirectedEdgeToNode(INode neighbor);
    
    void removeUndirectedEdgeToNode(INode neightbor);
    
    boolean hasEdge(INode node);
    
    int getWeight(INode node);
}
