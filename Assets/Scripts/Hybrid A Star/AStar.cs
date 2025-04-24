using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AStar : MonoBehaviour
{
    
    public static float[,] CalculateCostToGoal(HybridAStar.TileTypeWalkable[,] walkableMap, Vector2Int startPosition)
    {
        // init costToGoal
        float[,] costToGoal = new float[walkableMap.GetLength(0), walkableMap.GetLength(1)];
        for (int x = 0; x < walkableMap.GetLength(0); x++)
        {
            for (int y = 0; y < walkableMap.GetLength(1); y++)
            {
                costToGoal[x, y] = float.MaxValue; // Initialize with a large value
            }
        }

        // init heap
        Heap<NodeAStar> openNodes = new Heap<NodeAStar>(200000);

        // Create the first node
        NodeAStar startNode = new NodeAStar(startPosition, 0);
        openNodes.Add(startNode);
        costToGoal[startPosition.x, startPosition.y] = 0;

        while (openNodes.Count > 0)
        {
            NodeAStar currentNode = openNodes.RemoveFirst();

            // Get neighbors and calculate costs
            foreach (Vector2Int neighbor in GetNeighbors(currentNode.position, walkableMap))
            {
                if (walkableMap[neighbor.x, neighbor.y] == HybridAStar.TileTypeWalkable.Walkable)
                {
                    float newCost = currentNode.cost + CalculateCost(currentNode.position, neighbor);
                    if (newCost < costToGoal[neighbor.x, neighbor.y])
                    {
                        costToGoal[neighbor.x, neighbor.y] = newCost;
                        NodeAStar neighborNode = new NodeAStar(neighbor, newCost);
                        openNodes.Add(neighborNode);
                    }
                }
            }
        }



        // Calculate the cost to the goal using the heuristic function
        // This is a placeholder implementation and should be replaced with a proper heuristic calculation
        return costToGoal;
    }

    private static Set GetNeighbors(Vector2Int position, HybridAStar.TileTypeWalkable[,] walkableMap)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();

        int width = walkableMap.GetLength(0);
        int height = walkableMap.GetLength(1);

        // Check the 4 possible directions (up, down, left, right)
        Vector2Int[] directions = { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };
        foreach (Vector2Int direction in directions)
        {
            Vector2Int neighborPos = position + direction;
            if (neighborPos.x >= 0 && neighborPos.x < width && neighborPos.y >= 0 && neighborPos.y < height)
            {
                Tuple.Create(neighborNode, costAdded);
                neighbors.Add(neighborPos);
            }
        }

        return neighbors.ToArray();
    }
}
