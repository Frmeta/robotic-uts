using System;
using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Events;
using UnityEngine.Tilemaps;

public class MapBuilder : MonoBehaviour
{
    // inputs

    [Header("Lidar settings")]
    public bool isDebugLidar;
    public Transform lidarTransform;
    public int numberOfRays = 32;
    public float rayLength = 5;
    public float lidarIntervalDistance = 1f;
    public Material lineMaterial;
    public float rayWidth = 0.02f;
    public float cellSize = 0.2f;

    
    [Header("Map settings")]
    public Transform[] borders;


    [Header("Car settings")]
    public int carRadiusInCell = 5; // in cells, not meters



    public GameObject bestTargetDebug;
    public TMP_Text bombText;


    // temporary variables
    
    public enum TileType { Unexplored, Frontier, Explored };
    public enum TileTypeWalkable { Walkable, NotWalkable };
    [HideInInspector] public TileType[,] tileTypeMap;
    [HideInInspector] public float[,] wallConfidenceMap;
    [HideInInspector] public TileTypeWalkable[,] walkableMap;
    [HideInInspector] public float[,] aStarPlayerMap;
    [HideInInspector] public float[,] aStarTargetMap;
    [HideInInspector] private int[,] lidarCountMap;
    [HideInInspector] public Vector2Int tilemapOffset;

    [HideInInspector] public List<Vector2Int> frontierTiles = new List<Vector2Int>();

    private Vector2Int[] neighbors = new Vector2Int[]
            {
                new Vector2Int(1, 0),
                new Vector2Int(-1, 0),
                new Vector2Int(0, 1),
                new Vector2Int(0, -1),
                new Vector2Int(1, 1),
                new Vector2Int(-1, 1),
                new Vector2Int(1, -1),
                new Vector2Int(-1, -1)
            };

    private float lidarNeededDistance = 0;
    private LineRenderer[] lineRenderers;
    public static MapBuilder instance = null;
    public List<Vector3> bombPositions = new List<Vector3>();
    public bool scanWholeLevelFirst;

    void Awake()
    {
        // singleton
        if (instance == null)
        {
            instance = this;
        }

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
            lineRenderers[i].gameObject.SetActive(isDebugLidar);
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
        tileTypeMap = new TileType[borderMaxXCell - borderMinXCell + 1, borderMaxYCell - borderMinYCell + 1];
        wallConfidenceMap = new float[borderMaxXCell - borderMinXCell + 1, borderMaxYCell - borderMinYCell + 1];
        lidarCountMap = new int[borderMaxXCell - borderMinXCell + 1, borderMaxYCell - borderMinYCell + 1];
        for (int x = 0; x < tileTypeMap.GetLength(0); x++)
        {
            for (int y = 0; y < tileTypeMap.GetLength(1); y++)
            {
                tileTypeMap[x, y] = TileType.Unexplored;
            }
        }
        Debug.Log("Map size: " + tileTypeMap.GetLength(0) + "x" + tileTypeMap.GetLength(1) + " cells, " + (tileTypeMap.GetLength(0) * tileTypeMap.GetLength(1)) + " cells total.");
        
    }

    void Start()
    {
        Ray ray = new Ray();
        ray.direction = Vector3.down;
        // joel method
        if (scanWholeLevelFirst){
            Debug.Log("joel metohd");
            for (int i = 0; i < tileTypeMap.GetLength(0); i++){
                for (int j = 0; j < tileTypeMap.GetLength(1); j++){
                    Vector2Int cellPos = new Vector2Int(i, j);
                    Vector3 worldPosition = MapToWorld(cellPos);
                    ray.origin = new Vector3(worldPosition.x, 20, worldPosition.z);

                    RaycastHit hit;
                    bool isHit = Physics.Raycast(ray, out hit, 20);

                    if (isHit && hit.collider.CompareTag("Bombs")){
                        // store its position
                        if (!bombPositions.Contains(hit.collider.transform.position)){
                            bombPositions.Add(hit.collider.transform.position);
                        }
                        Explore(cellPos, false);
                    } if (isHit && hit.point.y < 2.3f){
                        Explore(cellPos, false);
                    } else {
                        Explore(cellPos, true);
                    }
                }
            }

            // sort bomb closest to farthest
            UpdateWalkableMap();
            aStarPlayerMap = AStar.Instance.CalculateCostToGoal(walkableMap, WorldToMap(CarController.instance.approximatedPosition));

            foreach(Vector3 a in bombPositions){
                Vector2Int aPos = WorldToMap(a);

                // clear area
                int bombRadius = 4;
                for (int j = -bombRadius; j<=bombRadius; j++){
                    for (int k = -bombRadius; k<=bombRadius; k++){
                        Explore(aPos + new Vector2Int(j, k), false);
                    }
                }
            }

            Vector2 pPos = CarController.instance.approximatedPosition;
            bombPositions.Sort((a,b) => {
                Vector2Int aPos = WorldToMap(a);
                Vector2Int bPos = WorldToMap(b);
                int ab = Vector2.Distance(aPos, pPos).CompareTo(Vector2.Distance(bPos, pPos));
                return -ab;
            });
            UpdateBombText();
        }


        int times = 4;
        for (int i = 0; i < times; i++){
            Lidar((float) i/times);
        }
    }

    void Update()
    {
        // shoot lidar after moved a certain distance
        lidarNeededDistance -= GetComponent<Rigidbody>().velocity.magnitude * Time.deltaTime;
        if (lidarNeededDistance <= 0)
        {
            lidarNeededDistance = lidarIntervalDistance;
            Lidar(0);

            
            UpdateWalkableMap();
        }
    }

    void Lidar(float angleOffset){
        // lidar in action
        for (int i = 0; i < numberOfRays; i++)
        {
            // shoot ray
            float angle = (360f / numberOfRays) * (i + angleOffset);
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
            // if hit something
            if (hit.collider != null){
                if (hit.collider.CompareTag("Bombs")){
                    // hit bomb

                    // store its position
                    if (!bombPositions.Contains(hit.collider.transform.position)){
                        bombPositions.Add(hit.collider.transform.position);
                        UpdateBombText();
                    }

                    // clear area
                    int bombRadius = 4;
                    Vector2Int bombPos = WorldToMap(hit.collider.transform.position);
                    for (int j = -bombRadius; j<=bombRadius; j++){
                        for (int k = -bombRadius; k<=bombRadius; k++){
                            Explore(bombPos + new Vector2Int(j, k), false);
                        }
                    }

                } else {
                    // hit wall
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
            
        }
    }

    public void Explore(Vector2Int mapPos, bool isWall)
    {
        // asert bounds
        if (mapPos.x < 0 || mapPos.x >= tileTypeMap.GetLength(0) || mapPos.y < 0 || mapPos.y >= tileTypeMap.GetLength(1)) {
            return;
        }

        // if prev was frontier, remove it from list
        if (tileTypeMap[mapPos.x, mapPos.y] == TileType.Frontier)
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
        SetTile(mapPos, TileType.Explored);


        if (wallConfidenceMap[mapPos.x, mapPos.y] < 0.5f) // if not wall
        {
            foreach (Vector2Int neighbor in neighbors)
            {
                Vector2Int neighborPosition = mapPos + neighbor;
                if (neighborPosition.x < 0 || neighborPosition.x >= tileTypeMap.GetLength(0) || neighborPosition.y < 0 || neighborPosition.y >= tileTypeMap.GetLength(1)) continue;
                if (tileTypeMap[neighborPosition.x, neighborPosition.y] == TileType.Unexplored)
                {
                    frontierTiles.Add(neighborPosition);
                    SetTile(neighborPosition, TileType.Frontier);
                }
            }
        } else {
            foreach (Vector2Int neighbor in neighbors)
            {
                Vector2Int neighborPosition = mapPos + neighbor;
                if (neighborPosition.x < 0 || neighborPosition.x >= tileTypeMap.GetLength(0) || neighborPosition.y < 0 || neighborPosition.y >= tileTypeMap.GetLength(1)) continue;
                if (tileTypeMap[neighborPosition.x, neighborPosition.y] == TileType.Frontier)
                {
                    frontierTiles.Remove(neighborPosition);
                    SetTile(neighborPosition, TileType.Unexplored);
                }
            }
        }
        
    }

    private void SetTile(Vector2Int mapPos, TileType tileType) // just update the tile & visualizer, no other check
    {

        // update map
        tileTypeMap[mapPos.x, mapPos.y] = tileType;
        
        // update visual
        MapVisualizer.instance.SetTile(mapPos, tileType);
         
    }

    public IEnumerator getPathToBestTarget(System.Action<List<Node>> callback){


        UpdateWalkableMap();

        // calculate map
        Vector2Int bestTargetInCell;
        Vector3 bestTargetInWorld;

        if (CarController.instance.isBestTargetManual){
            // manual by player
            bestTargetInWorld = bestTargetDebug.transform.position;
            bestTargetInCell = WorldToMap(bestTargetInWorld);
        } else {
            // find frontier
            aStarPlayerMap = AStar.Instance.CalculateCostToGoal(walkableMap, WorldToMap(CarController.instance.approximatedPosition));
            Assert.IsNotNull(aStarPlayerMap);
            MapVisualizer.instance.UpdateAStarPlayerMap(aStarPlayerMap);

            
            /// get best target (frontier)
            if (bombPositions.Count == 0){
                bestTargetInCell = getBestTargetInCell();
            } else {
                bestTargetInCell = WorldToMap(bombPositions[0]);
            }
            
            if (bestTargetInCell == Vector2Int.zero){
                // entah kenapa best target jika fail
                Debug.Log("Best Target Finding Failed");
                callback(null);
                yield break;
            }
            bestTargetInWorld = MapToWorld(bestTargetInCell);
        }
        

        // Debug
        Debug.Log("Best target: " + bestTargetInCell.ToString());
        bestTargetDebug.transform.position = bestTargetInWorld;
        if (walkableMap[bestTargetInCell.x, bestTargetInCell.y] != TileTypeWalkable.Walkable){
            Debug.Log("Best Target Finding Failed: not walkable: " + bestTargetDebug.transform.position + ", cell: " + bestTargetInCell);
            callback(null);
            yield break;
        }

        // calculate A* from target
        aStarTargetMap = AStar.Instance.CalculateCostToGoal(walkableMap, bestTargetInCell);

        // update the map
        Assert.IsNotNull(aStarTargetMap);
        MapVisualizer.instance.UpdateAStarTargetMap(aStarTargetMap);


        StartCoroutine(HybridAStar.instance.GeneratePath(
            CarController.instance.approximatedPosition,
            Mathf.PI/2 - transform.eulerAngles.y * Mathf.Deg2Rad,
            bestTargetInWorld,
            walkableMap,
            aStarTargetMap,
            CarController.instance.wheelBase,
            callback
            ));
    }

    private void UpdateWalkableMap(){
        walkableMap = CalculateMinkowskiSum();
        MapVisualizer.instance.UpdateWalkableMap(walkableMap);
    }

    private Vector2Int getBestTargetInCell(){
        if (frontierTiles.Count == 0){
            return Vector2Int.zero;
        }

        // group per clusters
        List<List<Vector2Int>> clusters = new List<List<Vector2Int>>();
        List<Vector2Int> visited = new List<Vector2Int>();
        foreach (Vector2Int tile in frontierTiles)
        {
            if (!visited.Contains(tile) && walkableMap[tile.x, tile.y] == TileTypeWalkable.Walkable)
            {
                // apply BFS
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
                        foreach (Vector2Int neighbor in neighbors)
                        {
                            Vector2Int neighborPosition = current + neighbor;
                            if (frontierTiles.Contains(neighborPosition) && !visited.Contains(neighborPosition) && walkableMap[neighborPosition.x, neighborPosition.y] == TileTypeWalkable.Walkable)
                            {
                                queue.Enqueue(neighborPosition);
                            }
                        }
                    }
                }
                Assert.IsTrue(cluster.Count > 0);
                clusters.Add(cluster);
            }
        }
        Assert.IsTrue(clusters.Count > 0);

        // find 'best' cluster
        List<Vector2Int> bestCluster = clusters[0];
        float bestClusterScore = float.MinValue;
        foreach (List<Vector2Int> cluster in clusters)
        {
            // find representative point
            float highScore2 = float.MinValue;
            Vector2Int highScoreHolder2 = cluster[0];
            foreach (Vector2Int tile in cluster)
            {
                // score = A* distance + angle difference
                Assert.IsNotNull(aStarPlayerMap);
                float score2 = - aStarPlayerMap[tile.x, tile.y] - Mathf.Sin(Vector2.Angle(WorldToMap(CarController.instance.approximatedPosition), tile)) / 360f * 50f;
                if (score2 > highScore2){
                    highScore2 = score2;
                    highScoreHolder2 = tile;
                }
            }

            float clusterScore = cluster.Count*2 + highScore2; // score counting
            if (clusterScore > bestClusterScore)
            {
                bestClusterScore = clusterScore;
                bestCluster = cluster;
            } else {
                //Debug.Log(clusterScore + " is less than " + bestClusterScore);
            }
        }
        
        Assert.IsNotNull(bestCluster);
        Assert.IsTrue(bestCluster.Count > 0);

        // find the closest tile to player using aStar
        float highScore = float.MinValue;
        Vector2Int highScoreHolder = bestCluster[0];
        foreach (Vector2Int tile in bestCluster)
        {
            float score = - aStarPlayerMap[tile.x, tile.y] - Mathf.Sin(Vector2.Angle(WorldToMap(CarController.instance.approximatedPosition), tile)) / 360f * 50f;
            if (score > highScore) {
                highScore = score;
                highScoreHolder = tile;
            } 
        }
        // return world pos
        return highScoreHolder;
    }

    public Vector2Int WorldToMap(Vector3 worldPos)
    {
        int x =  Mathf.RoundToInt(worldPos.x/cellSize) - tilemapOffset.x;
        int y =  Mathf.RoundToInt(worldPos.z/cellSize) - tilemapOffset.y;
        return new Vector2Int(x, y);
    }
    public Vector3 MapToWorld(Vector2Int mapPos)
    {
        Vector3 cellPos = new Vector3((mapPos.x + tilemapOffset.x) * cellSize, CarController.instance.approximatedPosition.y, (mapPos.y + tilemapOffset.y) * cellSize);
        return cellPos;
    }

    private TileTypeWalkable[,] CalculateMinkowskiSum(){
        //Calculate the Minkowski sum of the map

        // get width & height
        int mapWidth = tileTypeMap.GetLength(0);
        int mapHeight = tileTypeMap.GetLength(1);

        // init the Minkowski map
        TileTypeWalkable[,] minkowskiMap = new TileTypeWalkable[mapWidth, mapHeight];
        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapHeight; z++)
            {
                // assign walkable if explored
                if (tileTypeMap[x, z] == MapBuilder.TileType.Explored || tileTypeMap[x, z] == MapBuilder.TileType.Frontier)
                {
                    minkowskiMap[x, z] = TileTypeWalkable.Walkable;
                }
                else
                {
                    minkowskiMap[x, z] = TileTypeWalkable.NotWalkable;
                }
            }
        }

        // convert walkable into not walkable if near wall
        for (int x = 0; x < mapWidth; x++)
        {
            for (int z = 0; z < mapHeight; z++)
            {
                // if wall
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
    public void BombDifused(){
        bombPositions.RemoveAt(0);
        UpdateBombText();
    }

    private void UpdateBombText(){
        bombText.text = "Bombs detected:";
        foreach (Vector3 bombPosition in bombPositions){
            bombText.text += "\n- " + bombPosition.ToString();
        }
    }

}
