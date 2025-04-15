using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Hybrid A* pathfinding algorithm
public static class HybridAStar
{
    //The distance between each waypoint
    //Should be greater than the hypotenuse of the cell width or node may end up in the same cell
    public static float driveDistance = Mathf.Sqrt(Parameters.cellWidth * Parameters.cellWidth * 2f) + 0.01f;
    //Used in the loop to easier include reversing
    private static float[] driveDistances = new float[] { driveDistance, -driveDistance};
    //The steering angles we are going to test
    private static float maxAngle = 30f;
    private static float[] steeringAngles = new float[] { -maxAngle * Mathf.Deg2Rad, 0f, maxAngle * Mathf.Deg2Rad };
    //The car will never reach the exact goal position, this is how accurate we want to be
    private const float posAccuracy = 1f;
    private const float headingAccuracy = 10f;
    //The heading resolution (Junior had 5) [degrees]
    private const float headingResolution = 15f;
    private const float headingResolutionTrailer = 15f;
    //To time the different parts of the algorithm 
    private static int timer_selectLowestCostNode;
    private static int timer_addNodeToHeap;
    private static int timer_findChildren;
    private static int timer_isCollidingWithObstacle;
    //At what distance to should we start expanding Reeds-Shepp nodes
    private static float maxReedsSheppDist = 15f;

    //
    // Generate a path with Hybrid A*
    //
    public static List<Node> GeneratePath(Vector2 startPosition, float startRotation, Vector2 endPosition, float[,] map, float[,] wallConfidenceMap)
    {
        //Reset timers
        timer_selectLowestCostNode = 0;
        timer_addNodeToHeap = 0;
        timer_findChildren = 0;
        timer_isCollidingWithObstacle = 0;
        //Other data we want to track


        //Init the data structure we need
        int mapWidth = map.GetLength(0);

        //Open nodes - the parameter is how many items can fit in the heap
        //If we lower the heap size it will still find a path, which is more drunk
        Heap<Node> openNodes = new Heap<Node>(200000);
        //int in the dictionaries below is the rounded heading used to enter a cell
        HashSet<int>[,] closedCells = new HashSet<int>[mapWidth, mapWidth];
        //The node in the cell with the lowest g-cost at a certain angle
        Dictionary<int, Node>[,] lowestCostNodes = new Dictionary<int, Node>[mapWidth, mapWidth];
        //Trailer
        //int in the dictionaries below is the rounded heading used to enter a cell
        HashSet<int>[,] closedCellsTrailer = new HashSet<int>[mapWidth, mapWidth];
        HashSet<int>[,] lowestCostNodesTrailer = new HashSet<int>[mapWidth, mapWidth];

        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapWidth; z++)
            {
                closedCells[x, z] = new HashSet<int>();
                lowestCostNodes[x, z] = new Dictionary<int, Node>();

                //Trailer
                closedCellsTrailer[x, z] = new HashSet<int>();
                lowestCostNodesTrailer[x, z] = new HashSet<int>();
            }
        }


        //Create the first node
       
        List<Node> finalPath = new List<Node>();
        return finalPath;
    }

}