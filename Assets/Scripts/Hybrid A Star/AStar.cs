using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AStar
{
    // singleton
    private static AStar instance = null;
    public static AStar Instance
    {
        get
        {
            if (instance == null)
            {
                instance = new AStar();
            }
            return instance;
        }
    }

    public float[,] CalculateCostToGoal(MapBuilder.TileTypeWalkable[,] walkableMap, Vector2Int startPosition)
    {
        /* 
        accepts:
            walkableMap : 2D array of {Walkable, NotWalkable}
            startPosition : the starting position of the pathfinding algorithm

        returns:
            costToGoal : 2D array of the cost to each tile in the map
        */ 

        // init costToGoal
        float[,] costToGoal = new float[walkableMap.GetLength(0), walkableMap.GetLength(1)];
        bool[,] isVisitedMap = new bool[walkableMap.GetLength(0), walkableMap.GetLength(1)];
        for (int x = 0; x < walkableMap.GetLength(0); x++)
        {
            for (int y = 0; y < walkableMap.GetLength(1); y++)
            {
                costToGoal[x, y] = float.MaxValue; // Initialize with a large value
                isVisitedMap[x,y] = false;
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
            // pop from heap
            NodeAStar currentNode = openNodes.RemoveFirst();
            isVisitedMap[currentNode.position.x, currentNode.position.y] = true;

            // Get neighbors (must inside the map))
            foreach (var neighborTuple in GetNeighbors(currentNode.position, walkableMap))
            {
                Vector2Int neighbor = neighborTuple.Item1;
                
                // if the neighbor is walkable
                if (walkableMap[neighbor.x, neighbor.y] == MapBuilder.TileTypeWalkable.Walkable)
                {
                    // calculate the cost to the neighbor
                    float newCost = (float)(currentNode.cost + neighborTuple.Item2);

                    // if newCost is less than the current cost to the neighbor
                    if (!isVisitedMap[neighbor.x, neighbor.y] && newCost < costToGoal[neighbor.x, neighbor.y])
                    {
                        // update the cost to the neighbor and add it to the open nodes
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

    private List<Tuple<Vector2Int, double>> GetNeighbors(Vector2Int position, MapBuilder.TileTypeWalkable[,] walkableMap)
    {
        // returns List of tuple (neighbors position, cost)
        List<Tuple<Vector2Int, double>> neighbors = new List<Tuple<Vector2Int, double>>();

        int width = walkableMap.GetLength(0);
        int height = walkableMap.GetLength(1);

        // Check the 4 possible directions (up, down, left, right)
        Vector2Int[] directions = { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };
        foreach (Vector2Int direction in directions)
        {
            Vector2Int neighborPos = position + direction;
            if (neighborPos.x >= 0 && neighborPos.x < width && neighborPos.y >= 0 && neighborPos.y < height)
            {
                neighbors.Add(Tuple.Create(neighborPos, 1d));
            }
        }

        // for diagonal, sqrt(2)
        Vector2Int[] directionsDiagonal = { 
            Vector2Int.down + Vector2Int.left, 
            Vector2Int.down + Vector2Int.right,
            Vector2Int.up + Vector2Int.left, 
            Vector2Int.up + Vector2Int.right };

        foreach (Vector2Int direction in directionsDiagonal)
        {
            Vector2Int neighborPos = position + direction;
            if (neighborPos.x >= 0 && neighborPos.x < width && neighborPos.y >= 0 && neighborPos.y < height)
            {
                neighbors.Add(Tuple.Create(neighborPos, Math.Sqrt(2))); // Diagonal movement cost is sqrt(2)
            }
        }

        return neighbors;
    }
}
