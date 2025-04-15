using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;
using UnityEngine.UI;

public class MapVisualizer : MonoBehaviour
{
    public Tilemap tilemap;
    public TileBase whiteTile;
    public MapBuilder mapBuilder;
    [Header("Minimap")]
    public RawImage minimapRawImage;
    public RectTransform minimapScaler;
    public GameObject objectToFollow;
    public float zoom = 1f;


    private bool isMaterialChanged = false;
    

    private Texture2D minimapTexture;

    // Start is called before the first frame update
    void Start()
    {
        minimapTexture = new Texture2D(mapBuilder.map.GetLength(0), mapBuilder.map.GetLength(1), TextureFormat.RGBA32, false);
        minimapTexture.filterMode = FilterMode.Point;
        minimapTexture.wrapMode = TextureWrapMode.Repeat;
        minimapRawImage.texture = minimapTexture;
        minimapRawImage.rectTransform.sizeDelta = new Vector2(mapBuilder.map.GetLength(0), mapBuilder.map.GetLength(1));

    }

    // Update is called once per frame
    void Update()
    {
        // follow object
        if (objectToFollow != null)
        {
            Vector3 diff = objectToFollow.transform.position -
                mapBuilder.MapToWorld( new Vector2Int(mapBuilder.map.GetLength(0) / 2, mapBuilder.map.GetLength(1) / 2));
            Vector2 diff2 = new Vector2(diff.x, diff.z);
            Vector3 newPos = new Vector3(-diff2.x, - diff2.y, 0);
            minimapRawImage.rectTransform.anchoredPosition = newPos/mapBuilder.cellSize;

            minimapScaler.localScale = new Vector3(1, 1, 1) * zoom;
            minimapScaler.rotation = Quaternion.Euler(0, 0, objectToFollow.transform.rotation.y * Mathf.Rad2Deg + 45f);
        }
    }

    void LateUpdate()
    {
        if (isMaterialChanged){
            minimapTexture.Apply();
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
            tilemap.SetTile(cellPosForTilemap, null);
            minimapTexture.SetPixel(cellPosForTilemap.x, cellPosForTilemap.y, unexploredColor);
        }
        else
        {
            tilemap.SetTile(cellPosForTilemap, whiteTile);
            if (tileType == MapBuilder.TileType.Explored)
            {
                // lerp between wallConfidence green to red
                Color lerpedColor = Color.Lerp(walkableColor, obstacleColor, mapBuilder.wallConfidenceMap[mapPos.x, mapPos.y]);
                Debug.Log("color: " + lerpedColor + " wallConfidence: " + mapBuilder.wallConfidenceMap[mapPos.x, mapPos.y]);
                tilemap.SetColor(cellPosForTilemap, lerpedColor);
                minimapTexture.SetPixel(mapPos.x, mapPos.y, lerpedColor);
                
            } else if (tileType == MapBuilder.TileType.Frontier)
            {
                tilemap.SetColor(cellPosForTilemap, frontierColor);
                minimapTexture.SetPixel(mapPos.x, mapPos.y, frontierColor);
            }
        }
        isMaterialChanged = true;
         
    }
}
