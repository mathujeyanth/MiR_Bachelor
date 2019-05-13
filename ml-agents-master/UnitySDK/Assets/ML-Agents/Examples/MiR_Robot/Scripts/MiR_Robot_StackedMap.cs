using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//[ExecuteInEditMode]

public class MiR_Robot_StackedMap : MonoBehaviour {

    public enum TileType
    {
        Wall, Floor,
    }

    public GameObject wallTiles;                            // An array of wall tile prefabs.
    public GameObject Agents;

    public int[] AgentsPerMap;

    private int mapNumber = 0;

    public Texture2D[] maps;

    public TextAsset[] paths;

    public bool[] spawnStaticObs;

    private int addCol = 0;

    private GameObject mapObj;

    private int columns;
    private int rows;
    private TileType[][] tiles;
    //private CompositeCollider2D BoardHolder;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0;i<maps.Length;i++)
        {
            mapNumber = i;
            rows = maps[i].height;
            columns = maps[i].width;

            mapObj = new GameObject("Maps"+i);

            SetupTilesArray();

            MakeFloor(maps[i]);

            InstantiateTiles();

            spawnAgents();

            addCol += Mathf.FloorToInt((columns * 1.1f)/20);
        }
        
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

    void MakeFloor(Texture2D map)
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

        // The position to be instantiated at is based on the coordinates.
        Vector3 position = new Vector3((xCoord / 20) + addCol, (yCoord / 20));

        // Create an instance of the prefab from the random index of the array.
        GameObject tileInstance = Instantiate(prefabs, position, Quaternion.identity) as GameObject;

        // Set the tile's parent to the board holder.
        tileInstance.transform.parent = mapObj.transform;
    }

    void spawnAgents()
    {
        for (int k = 0; k < AgentsPerMap[mapNumber]; k++)
        {
            GameObject agent = new GameObject("Agent" + k);
            Vector3 position = new Vector3(addCol, 0);
            agent.transform.position = position;
            GameObject agents = Instantiate(Agents, position, Quaternion.identity) as GameObject;
            agents.transform.parent = agent.transform;
            agents.SendMessage("ReadCSVFile", paths[mapNumber]);

            if (spawnStaticObs[mapNumber])
                agents.SendMessage("SpawnStaticObsFunc");
        }
    }
}
