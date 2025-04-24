using System;
using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Tilemaps;

public class MapBuilder : MonoBehaviour
{
    public Transform lidarTransform;
    public int numberOfRays = 32;
    public float rayLength = 5;
    public Material lineMaterial;
    public float rayWidth = 0.02f;
    public float cellSize = 0.2f;
    public enum TileType { Unexplored, Frontier, Explored };
    
    public Transform[] borders;

    public Tile whiteTile;
    public MapVisualizer mapVisualizer;
    public float lidarIntervalDistance = 1f;


    private LineRenderer[] lineRenderers;

    // Map settings
    [HideInInspector] public Vector2Int tilemapOffset;

    [HideInInspector] public TileType[,] map;
    [HideInInspector] public float[,] wallConfidenceMap;
    private int[,] lidarCountMap;

    private List<Vector2Int> frontierTiles = new List<Vector2Int>();

    private Vector2Int[] neighbors = new Vector2Int[]
            {
                new Vector2Int(1, 0),
                new Vector2Int(-1, 0),
                new Vector2Int(0, 1),
                new Vector2Int(0, -1)
            };

    private float lidarNeededDistance = 0;
    

    void Awake()
    {
        // init lidar visualizer
        lineRenderers = new LineRenderer[numberOfRays];
        for (int i = 0; i < numberOfRays; i++)
        {
            GameObject lineRendererObject = new GameObject("LineRenderer" + i);
            lineRenderers[i] = lineRendererObject.AddComponent<LineRenderer>();
            lineRenderers[i].startWidth = rayWidth;
            lineRenderers[i].endWidth = rayWidth;
            lineRenderers[i].positionCount = 2;
            lineRenderers[i].transform.parent = transform;
            lineRenderers[i].material = lineMaterial;
        }

        // init required size
        float borderMinX = Mathf.Min(borders[0].position.x, borders[1].position.x);
        float borderMaxX = Mathf.Max(borders[0].position.x, borders[1].position.x);
        float borderMinY = Mathf.Min(borders[0].position.z, borders[1].position.z);
        float borderMaxY = Mathf.Max(borders[0].position.z, borders[1].position.z);

        int borderMinXCell = Mathf.RoundToInt(borderMinX / cellSize);
        int borderMaxXCell = Mathf.RoundToInt(borderMaxX / cellSize);
        int borderMinYCell = Mathf.RoundToInt(borderMinY / cellSize);
        int borderMaxYCell = Mathf.RoundToInt(borderMaxY / cellSize);

        tilemapOffset = new Vector2Int(borderMinXCell, borderMinYCell);

        // init map
        map = new TileType[borderMaxXCell - borderMinXCell + 1, borderMaxYCell - borderMinYCell + 1];
        wallConfidenceMap = new float[borderMaxXCell - borderMinXCell + 1, borderMaxYCell - borderMinYCell + 1];
        lidarCountMap = new int[borderMaxXCell - borderMinXCell + 1, borderMaxYCell - borderMinYCell + 1];
        for (int x = 0; x < map.GetLength(0); x++)
        {
            for (int y = 0; y < map.GetLength(1); y++)
            {
                map[x, y] = TileType.Unexplored;
            }
        }
        Debug.Log("Map size: " + map.GetLength(0) + "x" + map.GetLength(1) + " cells, " + (map.GetLength(0) * map.GetLength(1)) + " cells total.");
    }

    void Update()
    {
        // shoot lidar after moved a certain distance
        lidarNeededDistance -= GetComponent<Rigidbody>().velocity.magnitude * Time.deltaTime;
        if (lidarNeededDistance <= 0)
        {
            lidarNeededDistance = lidarIntervalDistance;
            Lidar();
        }
    }

    void Lidar(){
        // lidar in action
        for (int i = 0; i < numberOfRays; i++)
        {
            // shoot ray
            float angle = (360f / numberOfRays) * i;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            Ray ray = new Ray(lidarTransform.position, direction);

            // lidar visualization
            lineRenderers[i].SetPosition(0, ray.origin);
            if (Physics.Raycast(ray, out RaycastHit hit, rayLength))
            {
                lineRenderers[i].SetPosition(1, hit.point);
                lineRenderers[i].startColor = Color.red;
                lineRenderers[i].endColor = Color.red;
            } else {
                lineRenderers[i].SetPosition(1, ray.origin + ray.direction * rayLength);
                lineRenderers[i].startColor = Color.green;
                lineRenderers[i].endColor = Color.green;
            }
            
            // get start & end
            Vector3 start = ray.origin;
            Vector3 end;
            if (hit.collider != null)
            {
                end = ray.origin + ray.direction * hit.distance;
            } else {
                end = ray.origin + ray.direction * rayLength;
            }

            // Bresenham's line algorithm to draw line
            Vector2Int startCell = WorldToMap(start);
            Vector2Int endCell = WorldToMap(end);
            int dx = Mathf.Abs(endCell.x - startCell.x);
            int dy = Mathf.Abs(endCell.y - startCell.y);
            int sx = (startCell.x < endCell.x) ? 1 : -1;
            int sy = (startCell.y < endCell.y) ? 1 : -1;
            int err = dx - dy;
            while (true)
            {
                Explore(startCell, false);

                if (startCell == endCell) break;
                int err2 = err * 2;
                if (err2 > -dy)
                {
                    err -= dy;
                    startCell.x += sx;
                }
                if (err2 < dx)
                {
                    err += dx;
                    startCell.y += sy;
                }
            }
            int wallThickness = 5;
            for (int j = 0; j < wallThickness && hit.collider != null; j++)
            {
                Explore(startCell, true);

                int err2 = err * 2;
                if (err2 > -dy)
                {
                    err -= dy;
                    startCell.x += sx;
                }
                if (err2 < dx)
                {
                    err += dx;
                    startCell.y += sy;
                }
            }
        }
    }

    private void Explore(Vector2Int mapPos, bool isWall)
    {
        // asert bounds
        if (mapPos.x < 0 || mapPos.x >= map.GetLength(0) || mapPos.y < 0 || mapPos.y >= map.GetLength(1)) {
            Debug.Log("Out of bounds: " + mapPos);
            return;
        }

        // if prev was frontier, remove it from list
        if (map[mapPos.x, mapPos.y] == TileType.Frontier)
        {
            frontierTiles.Remove(mapPos);
        }


        // count confidence
        lidarCountMap[mapPos.x, mapPos.y] += 1;
        if (lidarCountMap[mapPos.x, mapPos.y] == 1)
        {
            wallConfidenceMap[mapPos.x, mapPos.y] = ((isWall) ? 1f : 0f);
        } else {
            wallConfidenceMap[mapPos.x, mapPos.y] = (wallConfidenceMap[mapPos.x, mapPos.y]*(float)(lidarCountMap[mapPos.x, mapPos.y]-1) + ((isWall) ? 1f : 0f))/(float)lidarCountMap[mapPos.x, mapPos.y];
        }
        //Debug.Log("Wall confidence: " + wallConfidenceMap[mapPos.x, mapPos.y] + " at " + mapPos);
        SetTile(mapPos, TileType.Explored);


        if (wallConfidenceMap[mapPos.x, mapPos.y] < 0.05f) // if not wall
        {
            foreach (Vector2Int neighbor in neighbors)
            {
                Vector2Int neighborPosition = mapPos + neighbor;
                if (neighborPosition.x < 0 || neighborPosition.x >= map.GetLength(0) || neighborPosition.y < 0 || neighborPosition.y >= map.GetLength(1)) continue;
                if (map[neighborPosition.x, neighborPosition.y] == TileType.Unexplored)
                {
                    frontierTiles.Add(neighborPosition);
                    SetTile(neighborPosition, TileType.Frontier);
                }
            }
        } else {
            foreach (Vector2Int neighbor in neighbors)
            {
                Vector2Int neighborPosition = mapPos + neighbor;
                if (neighborPosition.x < 0 || neighborPosition.x >= map.GetLength(0) || neighborPosition.y < 0 || neighborPosition.y >= map.GetLength(1)) continue;
                if (map[neighborPosition.x, neighborPosition.y] == TileType.Frontier)
                {
                    frontierTiles.Remove(neighborPosition);
                    SetTile(neighborPosition, TileType.Unexplored);
                }
            }
        }
        
    }

    private void SetTile(Vector2Int mapPos, TileType tileType) // just update the tile & visualizer, no other check
    {
        // no need to update if same type
        if (map[mapPos.x, mapPos.y] == tileType) return; 

        // update map
        map[mapPos.x, mapPos.y] = tileType;
        
        // update visual
        mapVisualizer.SetTile(mapPos, tileType);
         
    }

    public Vector3 pathFindToBestFrontier(){
        // find clusters
        List<List<Vector2Int>> clusters = new List<List<Vector2Int>>();
        List<Vector2Int> visited = new List<Vector2Int>();
        foreach (Vector2Int tile in frontierTiles)
        {
            if (!visited.Contains(tile))
            {
                List<Vector2Int> cluster = new List<Vector2Int>();
                Queue<Vector2Int> queue = new Queue<Vector2Int>();
                queue.Enqueue(tile);
                while (queue.Count > 0)
                {
                    Vector2Int current = queue.Dequeue();
                    if (!visited.Contains(current))
                    {
                        visited.Add(current);
                        cluster.Add(current);
                        // check neighbors
                        Vector2Int[] neighbors = new Vector2Int[]
                        {
                            new Vector2Int(1, 0),
                            new Vector2Int(-1, 0),
                            new Vector2Int(0, 1),
                            new Vector2Int(0, -1)
                        };
                        foreach (Vector2Int neighbor in neighbors)
                        {
                            Vector2Int neighborPosition = current + neighbor;
                            if (frontierTiles.Contains(neighborPosition) && !visited.Contains(neighborPosition))
                            {
                                queue.Enqueue(neighborPosition);
                            }
                        }
                    }
                }
                clusters.Add(cluster);
            }
        }

        // find best cluster
        List<Vector2Int> bestCluster = null;
        float bestClusterScore = 0;
        foreach (List<Vector2Int> cluster in clusters)
        {
            Vector2Int center = new Vector2Int(0, 0);
            foreach (Vector2Int tile in cluster)
            {
                center += tile;
            }
            center /= cluster.Count;
            float distance = Vector2.Distance(center, transform.position);
            float score = cluster.Count*5 - distance; // score counting
            if (score > bestClusterScore)
            {
                bestClusterScore = score;
                bestCluster = cluster;
            }
        }

        // find the closest tile to the center of the best cluster
        Vector2Int bestCenter = new Vector2Int(0, 0);
        foreach (Vector2Int tile in bestCluster)
        {
            bestCenter += tile;
        }
        bestCenter /= bestCluster.Count;
        Vector2Int closestTile = bestCluster[0];
        float closestDistance = Vector2.Distance(bestCenter, bestCluster[0]);
        foreach (Vector2Int tile in bestCluster)
        {
            float distance = Vector2.Distance(bestCenter, tile);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestTile = tile;
            }
        }
        // return world pos
        return MapToWorld(closestTile);
    }

    public Vector2Int WorldToMap(Vector3 worldPos)
    {
        int x =  Mathf.RoundToInt(worldPos.x/cellSize) - tilemapOffset.x;
        int y =  Mathf.RoundToInt(worldPos.z/cellSize) - tilemapOffset.y;
        return new Vector2Int(x, y);
    }
    public Vector3 MapToWorld(Vector2Int mapPos)
    {
        Vector3 cellPos = new Vector3((mapPos.x + tilemapOffset.x) * cellSize, 0, (mapPos.y + tilemapOffset.y) * cellSize);
        return cellPos;
    }

}
