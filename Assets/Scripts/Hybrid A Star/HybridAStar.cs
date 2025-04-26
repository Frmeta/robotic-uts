using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static MapBuilder;

//Hybrid A* pathfinding algorithm
public class HybridAStar
{
    //public MapBuilder mapBuilder;
    //The distance between each waypoint
    public static float stepDistance = 0.5f; // must be greater than cellSize
    //Used in the loop to easier include reversing
    private static float[] stepDistances = new float[] { stepDistance, -stepDistance};

    //The steering angles we are going to test
    private static float maxAngle = 30f;
    private static float[] steeringAngles = new float[] { -maxAngle * Mathf.Deg2Rad, 0f, maxAngle * Mathf.Deg2Rad };

    //The car will never reach the exact goal position, this is how accurate we want to be
    private const float posAccuracy = 0.5f;

    private bool[,,] isVisited;
    private int directionCount = 8; // for checking isVisited


    private MapBuilder mapBuilder;
    
    // singleton
    private static HybridAStar instance = null;
    public static HybridAStar Instance
    {
        get
        {
            if (instance == null)
            {
                instance = new HybridAStar();
            }
            return instance;
        }
    }

    //
    // Generate a path with Hybrid A*
    //
    public List<Node> GeneratePath(
        Vector2 startPosition,
        float startRotation,
        Vector2 endPosition,
        MapBuilder.TileTypeWalkable[,] walkableMap,
        float[,] aStarMap,
        float[,] wallConfidenceMap,
        bool isReversing,
        float wheelBase)
    {
        // get map width & height
        int mapWidth = walkableMap.GetLength(0);
        int mapHeight = walkableMap.GetLength(1);

        // init isVisited
        isVisited = new bool[mapWidth, mapHeight, directionCount];
        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapWidth; z++)
            {
                for (int i = 0; i < directionCount; i++)
                {
                    isVisited[x, z, i] = false;
                }
            }
        }
        
        //Open nodes - the parameter is how many items can fit in the heap
        //If we lower the heap size it will still find a path, which is more drunk
        Heap<Node> openNodes = new Heap<Node>(200000);
        

        //Create the first node
        Vector2Int startCellPosition = MapBuilder.instance.WorldToMap(startPosition);
        //int startRoundedRotation = Mathf.FloorToInt(startRotation / directionCount);
        Node startNode = new Node(null, startPosition, startRotation, false, 0, aStarMap[startCellPosition.x, startCellPosition.y]);
        openNodes.Add(startNode);

        // hybrid A*
        while (openNodes.Count > 0)
        {
            //Pop from heap
            Node currentNode = openNodes.RemoveFirst();

            //Mark this node as visited
            Vector2Int currentNodeCellPosition = MapBuilder.instance.WorldToMap(currentNode.worldPosition);
            int currentNodeRoundedRotation = Mathf.FloorToInt(currentNode.direction / directionCount);
            isVisited[currentNodeCellPosition.x, currentNodeCellPosition.y, currentNodeRoundedRotation] = true;

            //Check if we are at the goal position
            if (Vector2.Distance(currentNode.worldPosition, endPosition) < posAccuracy)
            {
                //We have reached the goal, return the path
                return GetFinalPath(currentNode);
            }

            //Get neighbors (must inside the map))
            foreach (Node neighborNode in GetNeighbors(currentNode, walkableMap, wheelBase, aStarMap))
            {
                // round the direction
                Vector2Int neighborCellPosition = MapBuilder.instance.WorldToMap(currentNode.worldPosition);
                int neighborRoundedDirection = Mathf.FloorToInt(neighborNode.direction / directionCount);

                //If the neighbor is walkable and not visited
                if (walkableMap[neighborCellPosition.x, neighborCellPosition.y] == MapBuilder.TileTypeWalkable.Walkable
                    && !isVisited[neighborCellPosition.x, neighborCellPosition.y, neighborRoundedDirection])
                {
                    //Create a new node and add it to the heap
                    openNodes.Add(neighborNode);
                }
            }
        }

       
        List<Node> finalPath = new List<Node>();
        return finalPath;
    }

    private List<Node> GetNeighbors(Node node, MapBuilder.TileTypeWalkable[,] walkableMap, float wheelBase, float[,] aStarMap)
    {
        // returns List of tuple (neighbors position, cost)

        // unbox node
        Vector2 worldPosition = node.worldPosition;
        float direction = node.direction;
        float costStartToNode = node.costStartToNode;
        bool isReversing = node.isReversing;

        // init empty list
        List<Node> neighbors = new List<Node>();

        int width = walkableMap.GetLength(0);
        int height = walkableMap.GetLength(1);


        // isReversed = false
        foreach (float steeringAngle in steeringAngles) {
            foreach (int majuMundur in new int[]{1, -1}){
                // 1=maju, -1=mundur
                float r = stepDistance/Mathf.Tan(steeringAngle);
                float delta_theta = majuMundur * stepDistance/r;

                float neighborX = worldPosition.x + majuMundur * r * (Mathf.Sin(direction + delta_theta) - Mathf.Sin(direction));
                float neighborY = worldPosition.y - majuMundur * r * (Mathf.Cos(direction + delta_theta) - Mathf.Cos(direction));
                float neighborDirection = (direction + delta_theta) % (2 * Mathf.PI);

                Vector2 neighborPosition = new Vector2(neighborX, neighborY);
                Vector2Int neighborCellPosition = MapBuilder.instance.WorldToMap(neighborPosition);

                neighbors.Add(
                    new Node(
                        node, 
                        neighborPosition,
                        neighborDirection,
                        majuMundur == -1,
                        costStartToNode 
                        + stepDistance/MapBuilder.instance.cellSize
                        + (isReversing ? 1f : 0) // give consequences if isReversing
                        + (isReversing == (majuMundur==1) ? 10f : 0), // give consequences if change maju/mundur
                        aStarMap[neighborCellPosition.x, neighborCellPosition.y]
                    )
                );
            }
            
        }

        // isReversed = true
        foreach (float steeringAngle in steeringAngles) {
            float r = stepDistance/Mathf.Tan(steeringAngle);
            float delta_theta = stepDistance/r;

            float neighborX = worldPosition.x + r * (Mathf.Sin(direction + delta_theta) - Mathf.Sin(direction));
            float neighborY = worldPosition.y + r * (Mathf.Cos(direction + delta_theta) - Mathf.Cos(direction));
            float neighborDirection = (direction + delta_theta) % (2 * Mathf.PI);

            Vector2 neighborPosition = new Vector2(neighborX, neighborY);
            Vector2Int neighborCellPosition = MapBuilder.instance.WorldToMap(neighborPosition);

            neighbors.Add(
                new Node(
                    node, 
                    neighborPosition,
                    neighborDirection,
                    false,
                    costStartToNode + stepDistance/MapBuilder.instance.cellSize + (isReversing ? 1 : 0),
                    aStarMap[neighborCellPosition.x, neighborCellPosition.y]
                )
            );
        }

        return neighbors;
    }

    //Get the final path from the start node to the goal node
    private static List<Node> GetFinalPath(Node goalNode)
    {
        List<Node> finalPath = new List<Node>();
        Node currentNode = goalNode;

        //Add the nodes to the path
        while (currentNode != null)
        {
            finalPath.Add(currentNode);
            currentNode = currentNode.previousNode;
        }

        //Reverse the path so it goes from start to goal
        finalPath.Reverse();
        return finalPath;
    }

    

}