using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static MapBuilder;

//Hybrid A* pathfinding algorithm
public class HybridAStar
{
    //public MapBuilder mapBuilder;
    //The distance between each waypoint
    public static float driveDistance = 0.5f; // must be greater than cellSize
    //Used in the loop to easier include reversing
    private static float[] driveDistances = new float[] { driveDistance, -driveDistance};

    //The steering angles we are going to test
    private static float maxAngle = 30f;
    private static float[] steeringAngles = new float[] { -maxAngle * Mathf.Deg2Rad, 0f, maxAngle * Mathf.Deg2Rad };

    //The car will never reach the exact goal position, this is how accurate we want to be
    private const float posAccuracy = 0.5f;

    private static bool[,,] isVisited;
    private static int directionCount = 8; // for checking isVisited


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
    public static List<Node> GeneratePath(Vector2 startPosition, float startRotation, Vector2 endPosition, MapBuilder.TileTypeWalkable[,] walkableMap, float[,] wallConfidenceMap)
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
        Node startNode = new Node(null, startPosition, startRotation, false, 0);

        // hybrid A*
        openNodes.Add(startNode);

       
        List<Node> finalPath = new List<Node>();
        return finalPath;
    }

    

}