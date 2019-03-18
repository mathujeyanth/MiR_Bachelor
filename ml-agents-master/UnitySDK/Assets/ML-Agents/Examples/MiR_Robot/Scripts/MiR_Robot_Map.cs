using System.Collections;
using System.Collections.Generic;
using UnityEngine;


//[ExecuteInEditMode]

public class MiR_Robot_Map : MonoBehaviour
{
    public enum TileType
    {
        Wall, Floor,
    }

    public GameObject wallTiles;                            // An array of wall tile prefabs.
    public GameObject Agents;

    public int GamesOnRows = 1;
    public int GamesOnColumns = 1;
    public int AgentsPerMap = 1;

    public Texture2D map;

    private GameObject mapObj;

    private int columns;
    private int rows;
    private TileType[][] tiles;
    //private CompositeCollider2D BoardHolder;

    // Start is called before the first frame update
    void Start()
    {
        rows = map.height;
        columns = map.width;

        mapObj = new GameObject("Maps");

        SetupTilesArray();

        MakeFloor();

        InstantiateTiles();

        spawnAgents();
    }


    void SetupTilesArray()
    {
        // Set the tiles jagged array to the correct width.
        tiles = new TileType[columns][];

        // Go through all the tile arrays...
        for (int i = 0; i < tiles.Length; i++)
        {
            // ... and set each tile array is the correct height.
            tiles[i] = new TileType[rows];
        }
    }

    void MakeFloor()
    {
        for (int i = 0; i < columns; i++)
        {
            for (int j = 0; j < rows; j++)
            {
                if (map.GetPixel(i, j) == Color.black)
                    tiles[i][j] = TileType.Wall;
                else
                    tiles[i][j] = TileType.Floor;
            }
        }
    }

    void InstantiateTiles()
    {
        // Go through all the tiles in the jagged array...
        for (int i = 0; i < tiles.Length; i++)
        {
            for (int j = 0; j < tiles[i].Length; j++)
            {
                // If the tile type is Wall...
                if (tiles[i][j] == TileType.Wall)
                {
                    // ... instantiate a wall over the top.
                    InstantiateFromArray(wallTiles, i, j);
                }

            }
        }
    }

    void InstantiateFromArray(GameObject prefabs, float xCoord, float yCoord)
    {
        for (int i = 0; i < GamesOnRows; i++)
        {
            for (int j = 0; j < GamesOnColumns; j++)
            {
                // The position to be instantiated at is based on the coordinates.
                Vector3 position = new Vector3((xCoord / 20) + (j * (columns * 1.1f)) / 20, (yCoord / 20) + (i * (rows * 1.1f)) / 20, 0f);

                // Create an instance of the prefab from the random index of the array.
                GameObject tileInstance = Instantiate(prefabs, position, Quaternion.identity) as GameObject;

                // Set the tile's parent to the board holder.
                tileInstance.transform.parent = mapObj.transform;
            }
        }
    }

    void spawnAgents()
    {
        for (int i = 0; i < GamesOnRows; i++)
        {
            for (int j = 0; j < GamesOnColumns; j++)
            {
                for (int k = 0;k<AgentsPerMap;k++)
                {
                    GameObject agent = new GameObject("Agent" + i + j + k);
                    Vector3 position = new Vector3((j * (columns * 1.1f) / 20), (i * (rows * 1.1f) / 20), 0f);
                    GameObject agents = Instantiate(Agents, position, Quaternion.identity) as GameObject;
                    agents.transform.parent = agent.transform;
                }
            }
        }

    }
}