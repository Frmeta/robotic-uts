using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Hybrid A* pathfinding algorithm
public static class HybridAStar
{
    //The distance between each waypoint
    public static float driveDistance = 0.5f; // must be greater than cellSize
    //Used in the loop to easier include reversing
    private static float[] driveDistances = new float[] { driveDistance, -driveDistance};

    //The steering angles we are going to test
    private static float maxAngle = 30f;
    private static float[] steeringAngles = new float[] { -maxAngle * Mathf.Deg2Rad, 0f, maxAngle * Mathf.Deg2Rad };

    //The car will never reach the exact goal position, this is how accurate we want to be
    private const float posAccuracy = 0.5f;

    private static int carRadiusInCell = 5;
    private static bool[,,] isVisited;
    private static int directionCount = 8; // for checking isVisited

    
    public enum TileTypeWalkable { Walkable, NotWalkable };
    
    //
    // Generate a path with Hybrid A*
    //
    public static List<Node> GeneratePath(Vector2 startPosition, float startRotation, Vector2 endPosition, MapBuilder.TileType[,] exploredMap, float[,] wallConfidenceMap)
    {
        // get map width & height
        int mapWidth = exploredMap.GetLength(0);
        int mapHeight = exploredMap.GetLength(1);

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

        // calculate minkowski sum of the map
        TileTypeWalkable[,] walkableMap = CalculateMinkowskiSum(exploredMap, wallConfidenceMap);
        
        //Open nodes - the parameter is how many items can fit in the heap
        //If we lower the heap size it will still find a path, which is more drunk
        Heap<Node> openNodes = new Heap<Node>(200000);
        

        //Create the first node
        Node startNode = new Node(null, startPosition, startRotation, false, 0, 0);
       
        List<Node> finalPath = new List<Node>();
        return finalPath;
    }

    private static TileTypeWalkable[,] CalculateMinkowskiSum(MapBuilder.TileType[,] map, float[,] wallConfidenceMap){
        //Calculate the Minkowski sum of the map
        int mapWidth = map.GetLength(0);
        int mapHeight = map.GetLength(1);
        TileTypeWalkable[,] minkowskiMap = new TileTypeWalkable[mapWidth, mapHeight];

        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapWidth; z++)
            {
                if (map[x, z] == MapBuilder.TileType.Explored)
                {
                    minkowskiMap[x, z] = TileTypeWalkable.Walkable;
                }
                else
                {
                    minkowskiMap[x, z] = TileTypeWalkable.NotWalkable;
                }
            }
        }

        //Calculate the Minkowski sum
        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapWidth; z++)
            {
                if (wallConfidenceMap[x, z] > 0.5f)
                {
                    // check surroundings in circle using carRadiusInCell
                    for (int i = -carRadiusInCell; i <= carRadiusInCell; i++)
                    {
                        for (int j = -carRadiusInCell; j <= carRadiusInCell; j++)
                        {
                            if (x + i >= 0 && x + i < mapWidth
                                && z + j >= 0 && z + j < mapHeight
                                && (i * i + j * j) <= (carRadiusInCell * carRadiusInCell)
                                )
                            {
                                if (minkowskiMap[x + i, z + j] == TileTypeWalkable.Walkable)
                                {
                                    minkowskiMap[x + i, z + j] = TileTypeWalkable.NotWalkable;
                                }
                            }
                        }
                    }
                }
            }
        }

        return minkowskiMap;
    }

}