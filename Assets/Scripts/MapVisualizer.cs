using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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
    public Minimap aStarMinimap;
    public Minimap walkableMinimap;
    public Minimap hybridAStarMinimap;
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


    void Start()
    {
        // singleton
        if (instance == null)
        {
            instance = this;
        }

        // zoom scrollbar listener
        zoomScrollbar.onValueChanged.AddListener((value) => {
            zoom = Mathf.Lerp(minZoom, maxZoom, value);
        });

        // get map width & height
        mapWidth = mapBuilder.tileTypeMap.GetLength(0);
        mapHeight = mapBuilder.tileTypeMap.GetLength(1);

        // init minimap texture
        minimaps = new Minimap[] { tileTypeMinimap, walkableMinimap, aStarMinimap, hybridAStarMinimap };
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
                mapBuilder.MapToWorld( new Vector2Int(mapBuilder.tileTypeMap.GetLength(0) / 2, mapBuilder.tileTypeMap.GetLength(1) / 2));
            Vector2 diff2 = new Vector2(diff.x, diff.z);
            Vector3 newPos = new Vector3(-diff2.x, - diff2.y, 0);

            foreach (Minimap minimap in minimaps)
            {
                // update position, rotation, scale
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

    public void UpdateAStarMap(MapBuilder.TileTypeWalkable[,] walkableMap, float[,] costMap)
    {
        for (int x = 0; x < costMap.GetLength(0); x++)
        {
            for (int y = 0; y < costMap.GetLength(1); y++)
            {
                // walkable minimap
                walkableMinimap.minimapTexture.SetPixel(x, y, Color.Lerp(Color.white, Color.black, walkableMap[x,y] == MapBuilder.TileTypeWalkable.Walkable ? 0 : 1));

                // a* minimap
                Color color = Color.Lerp(Color.white, Color.black,costMap[x, y]/25f);
                aStarMinimap.minimapTexture.SetPixel(x, y, color);
            }
        }
        aStarMinimap.minimapTexture.Apply();
    }



    void InitMinimapTexture(Minimap minimap)
    {
        minimap.minimapTexture = new Texture2D(mapWidth, mapHeight, TextureFormat.RGBA32, false);
        minimap.minimapTexture.filterMode = FilterMode.Point;
        minimap.minimapTexture.wrapMode = TextureWrapMode.Repeat;
        minimap.minimapRawImage.texture = minimap.minimapTexture;
        minimap.minimapRawImage.rectTransform.sizeDelta = new Vector2(mapWidth, mapHeight);
    }
}
