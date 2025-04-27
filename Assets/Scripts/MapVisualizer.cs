using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Tilemaps;
using UnityEngine.UI;

public class MapVisualizer : MonoBehaviour
{
    
    [Header("Tilemap")]
    public Tilemap tilemap;
    public TileBase whiteTile;
    public MapBuilder mapBuilder;


    [Serializable]
    public class Minimap {
        public RawImage minimapRawImage;
        public RectTransform minimapScaler;
        [HideInInspector] public Texture2D minimapTexture;
    }
    

    [Header("Minimap")]
    public Minimap tileTypeMinimap;
    public Minimap aStarPlayerMinimap;
    public Minimap walkableMinimap;
    public Minimap hybridAStarMinimap;
    public Minimap aStarTargetMinimap;
    public GameObject objectToFollow;
    public float zoom = 1f;
    public float minZoom = 0.1f;
    public float maxZoom = 7f;
    public Scrollbar zoomScrollbar;


    // Temporary variables
    private bool isMaterialChanged = false;
    private int mapWidth;
    private int mapHeight;
    private Minimap[] minimaps;

    // singleton
    public static MapVisualizer instance = null;


    void Awake()
    {
        // singleton
        if (instance == null)
        {
            instance = this;
        }
    }

    void Start()
    {
        zoomScrollbar.onValueChanged.AddListener((value) => {
            zoom = Mathf.Lerp(minZoom, maxZoom, value);
        });

        // get map width & height
        mapWidth = mapBuilder.tileTypeMap.GetLength(0);
        mapHeight = mapBuilder.tileTypeMap.GetLength(1);

        // init minimap texture
        minimaps = new Minimap[] { tileTypeMinimap, walkableMinimap, aStarPlayerMinimap, hybridAStarMinimap, aStarTargetMinimap};
        foreach (Minimap minimap in minimaps)
        {
            InitMinimapTexture(minimap);
        }
    }
    

    void Update()
    {
        // follow object (in this case, the car)
        if (objectToFollow != null)
        {
            Vector3 diff = objectToFollow.transform.position -
                mapBuilder.MapToWorld( new Vector2Int(mapWidth / 2, mapHeight / 2));
            Vector2 diff2 = new Vector2(diff.x, diff.z);
            Vector3 newPos = new Vector3(-diff2.x, - diff2.y, 0);

            foreach (Minimap minimap in minimaps)
            {
                // update position, rotation, scale
                Assert.IsNotNull(minimap.minimapRawImage);
                minimap.minimapRawImage.rectTransform.anchoredPosition = newPos/mapBuilder.cellSize;
                minimap.minimapScaler.localScale = new Vector3(1, 1, 1) * zoom;
                minimap.minimapScaler.rotation = Quaternion.Euler(0, 0, objectToFollow.transform.rotation.y * Mathf.Rad2Deg + 45f);
            }
           
        }
    }

    void LateUpdate()
    {
        if (isMaterialChanged){
            tileTypeMinimap.minimapTexture.Apply();
            isMaterialChanged = false;
        }
    }

    public void SetTile(Vector2Int mapPos, MapBuilder.TileType tileType) // just update visualizer, no other check
    {
        // update visual
        Vector3Int cellPosForTilemap = new Vector3Int(mapPos.x + (int)mapBuilder.tilemapOffset.x, mapPos.y + (int)mapBuilder.tilemapOffset.y, 0);


        // colors
        Color unexploredColor = Color.clear;
        Color obstacleColor = Color.black;
        Color walkableColor = Color.white;
        Color frontierColor = Color.blue;

        tilemap.SetTileFlags(cellPosForTilemap, TileFlags.None);
        if (tileType == MapBuilder.TileType.Unexplored)
        {
            // unexplored tiles
            tilemap.SetTile(cellPosForTilemap, null);
            tileTypeMinimap.minimapTexture.SetPixel(cellPosForTilemap.x, cellPosForTilemap.y, unexploredColor);
        }
        else
        {
            tilemap.SetTile(cellPosForTilemap, whiteTile);
            if (tileType == MapBuilder.TileType.Explored)
            {
                // explored tiles
                // lerp between wallConfidence green to red
                Color lerpedColor = Color.Lerp(walkableColor, obstacleColor, mapBuilder.wallConfidenceMap[mapPos.x, mapPos.y]);
                tilemap.SetColor(cellPosForTilemap, lerpedColor);
                tileTypeMinimap.minimapTexture.SetPixel(mapPos.x, mapPos.y, lerpedColor);
                
            } else if (tileType == MapBuilder.TileType.Frontier)
            {
                // frontier tiles
                tilemap.SetColor(cellPosForTilemap, frontierColor);
                tileTypeMinimap.minimapTexture.SetPixel(mapPos.x, mapPos.y, frontierColor);
            }
        }
        isMaterialChanged = true;
    }

    public void UpdateWalkableMap(MapBuilder.TileTypeWalkable[,] walkableMap){
        for (int x = 0; x < mapWidth; x++)
        {
            for (int y = 0; y < mapHeight; y++)
            {
                // walkable minimap
                walkableMinimap.minimapTexture.SetPixel(x, y, Color.Lerp(Color.white, Color.black, walkableMap[x,y] == MapBuilder.TileTypeWalkable.Walkable ? 0 : 1));
            }
        }
        walkableMinimap.minimapTexture.Apply();
    }
    public void UpdateAStarPlayerMap(float[,] costMap)
    {
        for (int x = 0; x < mapWidth; x++)
        {
            for (int y = 0; y < mapHeight; y++)
            {
                // a* minimap
                Color color = Color.Lerp(Color.white, Color.black, costMap[x, y]/50f);
                aStarPlayerMinimap.minimapTexture.SetPixel(x, y, color);
            }
        }
        aStarPlayerMinimap.minimapTexture.Apply();
    }
    public void UpdateAStarTargetMap( float[,] costMap)
    {
        for (int x = 0; x < mapWidth; x++)
        {
            for (int y = 0; y < mapHeight; y++)
            {
                // a* minimap
                Color color = Color.Lerp(Color.white, Color.black, costMap[x, y]/50f);
                aStarTargetMinimap.minimapTexture.SetPixel(x, y, color);
            }
        }
        aStarTargetMinimap.minimapTexture.Apply();
    }


    public void ClearHybridMap(){
        
        InitMinimapTexture(hybridAStarMinimap);
        for (int x = 0; x < mapWidth; x++)
        {
            for (int y = 0; y < mapHeight; y++)
            {
                hybridAStarMinimap.minimapTexture.SetPixel(x, y, Color.black);
            }
        }
        hybridAStarMinimap.minimapTexture.Apply();
    }

    public void VisitHybridMap(Vector2Int tile, float value){
        hybridAStarMinimap.minimapTexture.SetPixel(tile.x, tile.y, Color.Lerp(Color.black, Color.white, value));
        hybridAStarMinimap.minimapTexture.Apply();
    }



    void InitMinimapTexture(Minimap minimap)
    {
        minimap.minimapTexture = new Texture2D(mapWidth, mapHeight, TextureFormat.RGBA32, false);
        minimap.minimapTexture.filterMode = FilterMode.Point;
        minimap.minimapTexture.wrapMode = TextureWrapMode.Repeat;
        minimap.minimapTexture.Apply();
        minimap.minimapRawImage.texture = minimap.minimapTexture;
        minimap.minimapRawImage.rectTransform.sizeDelta = new Vector2(mapWidth, mapHeight);
        
    }
}
