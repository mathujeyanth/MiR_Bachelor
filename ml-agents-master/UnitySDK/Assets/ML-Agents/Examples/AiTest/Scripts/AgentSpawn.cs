using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentSpawn : MonoBehaviour {

    public Texture2D Map;
    public GameObject Agents;

    private int columns;
    private int rows;

    public int GameCols = 1;
    public int GameRows = 1;

    void Start ()
    {
        rows = Map.height;
        columns = Map.width;
        spawnAgents();
    }
	
    void spawnAgents()
    {
        for (int i = 0;i< GameRows; i++)
        {
            for (int j = 0;j< GameCols; j++)
            {
                GameObject agent = new GameObject("Agent" + i + j);
                Vector3 position = new Vector3(((i*(rows+10))/20)+((rows+10)/2)/20, ((j * (columns + 10)) / 20) + ((columns+10) / 2) / 20, 0f);
                //Vector3 position = new Vector3(0, 0, 0);
                GameObject agents = Instantiate(Agents, position, Quaternion.identity) as GameObject;
                agents.transform.parent = agent.transform;
            }
        }
        
    }

}
