using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using static MapBuilder;

//Hybrid A* pathfinding algorithm
public class HybridAStar : MonoBehaviour
{
    public Material lineMaterial;
    //public MapBuilder mapBuilder;
    //The distance between each waypoint
    public static float stepDistance = 0.4f; // must be greater than cellSize
    //Used in the loop to easier include reversing

    //The steering angles we are going to test
    public int noTurnAtBeginningCount = 2;
    public static float maxAngle = 15f;
    private float[] steeringAngles = new float[] {-maxAngle * Mathf.Deg2Rad, 0, maxAngle * Mathf.Deg2Rad}; //TODO:FIX KIRI

    //The car will never reach the exact goal position, this is how accurate we want to be
    private const float posAccuracy = 0.8f;

    private float[,,] lowestCost;
    public int directionCount = 8; // for checking direction
    private GameObject debugLine;

    
    // singleton
    public static HybridAStar instance = null;

    public void Awake()
    {
        if (instance == null){
            instance = this;
        }
        debugLine = new GameObject("HybridAStarLineDebug");
    }

    //
    // Generate a path with Hybrid A*
    //
    public IEnumerator GeneratePath(
        Vector3 startPosition,
        float startRotation,
        Vector3 endPosition,
        MapBuilder.TileTypeWalkable[,] walkableMap,
        float[,] aStarMap,
        float wheelBase,
        System.Action<List<Node>> callback)
    {
        // clear all debugLine children
        foreach (Transform transform in debugLine.transform)
        {
            Destroy(transform.gameObject);
        }

        // get map width & height
        int mapWidth = walkableMap.GetLength(0);
        int mapHeight = walkableMap.GetLength(1);

        // init lowestCost
        lowestCost = new float[mapWidth, mapHeight, directionCount];
        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapHeight; z++)
            {
                for (int i = 0; i < directionCount; i++)
                {
                    lowestCost[x, z, i] = float.MaxValue;
                }
            }
        }

        // clear minimap
         MapVisualizer.instance.ClearHybridMap();
        
        //Open nodes - the parameter is how many items can fit in the heap
        //If we lower the heap size it will still find a path, which is more drunk
        Heap<Node> openNodes = new Heap<Node>(200000);
        

        //Create the first node
        Vector2Int startCellPosition = MapBuilder.instance.WorldToMap(startPosition);

        Debug.Log("hybrid A* start at " + startPosition.ToString() + ", cell: " + startCellPosition.ToString());

        //int startRoundedRotation = Mathf.FloorToInt(startRotation/(2*Mathf.PI) *  directionCount)  % directionCount;
        Node startNode = new Node(null, startPosition, startRotation, false, 0, aStarMap[startCellPosition.x, startCellPosition.y], noTurnAtBeginningCount);
        openNodes.Add(startNode);

        // for starting backwards
        Node startNode2 = new Node(null, startPosition, startRotation, true, 0, aStarMap[startCellPosition.x, startCellPosition.y], noTurnAtBeginningCount);
        openNodes.Add(startNode2);

        yield return null;

        // hybrid A*
        while (openNodes.Count > 0)
        {
            //Pop from heap
            Node currentNode = openNodes.RemoveFirst();

            //Mark this node as visited
            Vector2Int currentNodeCellPosition = MapBuilder.instance.WorldToMap(currentNode.worldPosition);
            int currentNodeRoundedRotation = Rad2RoundedDirection(currentNode.direction);

            // buang yang tidak guna
            if (currentNode.costStartToNode > lowestCost[currentNodeCellPosition.x, currentNodeCellPosition.y, currentNodeRoundedRotation]){
                continue;
            }
            

            int j = 0;
            for (int i = 0; i < directionCount; i++){
                j += lowestCost[currentNodeCellPosition.x, currentNodeCellPosition.y, i] != float.MaxValue ? 1 : 0;
            }
            // Debug.Log("Out heap: " + currentNode.worldPosition.ToString() + 
            //     ", rotation: " + currentNode.direction.ToString() +
            //     ", cell: " + currentNodeCellPosition.ToString() +
            //     ", rounded rotation: " + currentNodeRoundedRotation.ToString());

            MapVisualizer.instance.VisitHybridMap(currentNodeCellPosition, (float)j/directionCount);

            // Check if we are at the goal position
            if (Vector2.Distance(
                new Vector2(currentNode.worldPosition.x, currentNode.worldPosition.z),
                new Vector2(endPosition.x, endPosition.z)) < posAccuracy
                || MapBuilder.instance.tileTypeMap[currentNodeCellPosition.x, currentNodeCellPosition.y] == TileType.Frontier)
            {
                //We have reached the goal, return the path
                Debug.Log("Hybrid done: reached goal/frontier");
                callback(GetFinalPath(currentNode));
                yield break;
            }

            yield return null;

            // Get neighbors (must inside the map))
            foreach (Node neighborNode in GetNeighbors(currentNode, walkableMap, wheelBase, aStarMap))
            {

                // round the direction
                Vector2Int neighborCellPosition = MapBuilder.instance.WorldToMap(neighborNode.worldPosition);
                int neighborRoundedDirection = Rad2RoundedDirection(neighborNode.direction);

                // should be never index out of bounds
                Assert.IsTrue(neighborCellPosition.x >= 0 && neighborCellPosition.x < mapWidth
                    && neighborCellPosition.y >= 0 && neighborCellPosition.y < mapHeight);
                Assert.IsTrue(neighborRoundedDirection>=0 && neighborRoundedDirection < directionCount);

                //If the neighbor is walkable and not visited
                if (walkableMap[neighborCellPosition.x, neighborCellPosition.y] == MapBuilder.TileTypeWalkable.Walkable
                    && neighborNode.costStartToNode < lowestCost[neighborCellPosition.x, neighborCellPosition.y, neighborRoundedDirection])
                {
                    lowestCost[currentNodeCellPosition.x, currentNodeCellPosition.y, currentNodeRoundedRotation] = neighborNode.costStartToNode;
                    //Create a new node and add it to the heap
                    openNodes.Add(neighborNode);
                }
            }
        }

        Debug.LogWarning("Hybrid done: everything has been explored, but no frontier");
        List<Node> finalPath = new List<Node>();
        callback(finalPath);
        yield break;
    }

    private List<Node> GetNeighbors(Node node, MapBuilder.TileTypeWalkable[,] walkableMap, float wheelBase, float[,] aStarMap)
    {
        // returns List of tuple (neighbors position, cost)

        // unbox node
        Vector3 worldPosition = node.worldPosition;
        float direction = node.direction;
        float costStartToNode = node.costStartToNode;
        bool isReversing = node.isReversing;

        // init empty list
        List<Node> neighbors = new List<Node>();

        int width = walkableMap.GetLength(0);
        int height = walkableMap.GetLength(1);


        // isReversed = false
        foreach (float steeringAngle in steeringAngles) {
            foreach (int majuMundur in new int[]{1, -1}){ // TODO: MUNDUR
                // 1=maju, -1=mundur

                bool isChangingDirection = isReversing == (majuMundur==1);
                float neighborX;
                float neighborZ;
                float neighborDirection;

                if (Mathf.Abs(steeringAngle) < 1e-3f){
                    // move forward/back
                    if (!isChangingDirection || (node.previousNode != null && node.direction == node.previousNode.direction)){
                        neighborX = worldPosition.x + majuMundur * Mathf.Cos(direction) * stepDistance;
                        neighborZ = worldPosition.z + majuMundur * Mathf.Sin(direction) * stepDistance;
                        neighborDirection = (direction) % (2 * Mathf.PI);
                    } else {
                        continue;
                    }
                    
                    
                } else {
                    if (node.countdownBeforeCanTurn <= 0 && !isChangingDirection){
                        // move left/right
                        //float r = wheelBase/Mathf.Tan(steeringAngle);
                        neighborDirection = direction + steeringAngle % (2 * Mathf.PI);
                        neighborX = worldPosition.x + majuMundur * Mathf.Cos(neighborDirection) * stepDistance;
                        neighborZ = worldPosition.z + majuMundur * Mathf.Sin(neighborDirection) * stepDistance;
                    } else {
                        continue;
                    }
                    
                }
                

                Vector3 neighborPosition = new Vector3(neighborX, worldPosition.y, neighborZ);
                Vector2Int neighborCellPosition = MapBuilder.instance.WorldToMap(neighborPosition);

                if (neighborCellPosition.x >= 0 && neighborCellPosition.x < width
                && neighborCellPosition.y >= 0 && neighborCellPosition.y < height
                && walkableMap[neighborCellPosition.x, neighborCellPosition.y] == TileTypeWalkable.Walkable){
                    neighbors.Add(
                        new Node(
                            node, 
                            neighborPosition,
                            neighborDirection,
                            majuMundur == -1,
                            costStartToNode // cost formula
                            + stepDistance/MapBuilder.instance.cellSize
                            + (isReversing ? 0.1f : 0) // give consequences if isReversing
                            + (isChangingDirection ? 3f : 0), // give consequences if change maju/mundur
                            aStarMap[neighborCellPosition.x, neighborCellPosition.y],
                            isReversing == (majuMundur==1) ? noTurnAtBeginningCount : Math.Max(0, node.countdownBeforeCanTurn-1)
                        )
                    );

                    // debug line
                    GameObject newLine = new GameObject("new line");

                    LineRenderer lr = newLine.AddComponent<LineRenderer>();

                    float rayWidth = 0.01f;
                    lr.startWidth = rayWidth;
                    lr.endWidth = rayWidth;
                    lr.positionCount = 2;
                    lr.material = lineMaterial;
                    lr.SetPositions(new Vector3[]{
                        node.worldPosition,
                        neighborPosition
                    });

                    newLine.transform.parent = debugLine.transform;
                }

               
            }
            
        }

        return neighbors;
    }

    private int Rad2RoundedDirection(float radians){
        int a = Mathf.FloorToInt(radians/(2*Mathf.PI) * directionCount) % directionCount;
        if (a < 0){
            a += directionCount;
        }
        return a;
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