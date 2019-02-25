using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using MLAgents;

public class AiTestAgent : Agent
{
    public float laserDist = 10;
    public bool useVectorObs = true;
    public bool displayLidar = false;
    public bool spawnObstacle = false;
    public Vector2[] path;

    private Vector2[][] pathArray;
    private System.Random rnd = new System.Random();

    private int pathIdx;
    private Vector2 currentPos;
    private Vector2 nextPos;
    private Vector2 simpleVec;
    private const int nLaser = 20;
    private const int bitMask = 1 << 9;
    private const int frontStart = 180;
    private const float degreesPrLaser = 270 / nLaser;
    private const float radians = Mathf.PI / 180;

    private float moveHorizontal = 0;
    private float moveVertical = 0;

    private float targetHoizontal = 0;
    private float targetVertical = 0;

    private float drag = 0.95f;

    private float lastDistance;

    public float linearMaxAcceleration = 0.5f;
    public float angularMaxAcceleration = 0.5f;
    public float linearVelocityLimit = 1.5f;
    public float linearReverseVelocityLimit = -0.5f;
    public float angularVelocityLimit = 1.0f;

    private float virtualLinearVelocity = 0;
    private float virtualAngularVelocity = 0;

    private RaycastHit2D hit;

    public GameObject obstacle;

    private Rigidbody2D agentRB;
    private AiTestAcademy academy;
    private GameObject square;
    private bool isSpawned = false;

    public override void InitializeAgent()
    {
        if (spawnObstacle)
        {
            square = Instantiate(obstacle, new Vector2(0.0f, 0.0f), Quaternion.identity) as GameObject;
            isSpawned = true;
        }
            
        base.InitializeAgent();
        academy = FindObjectOfType<AiTestAcademy>(); 
        agentRB = GetComponent<Rigidbody2D>();
        simpleVec = new Vector2(0f, 1f);
        ReadCSVFile();
        AgentReset();
        //for (int i = 0; i < pathArray[0].Length; i++)
        //    Debug.Log(pathArray[0][i]);
    }


    public override void CollectObservations()
    {
        if (useVectorObs)
        {
            AddVectorObs(virtualLinearVelocity);
            AddVectorObs(virtualAngularVelocity);
            AddVectorObs(getTargetAngle(currentPos));
            AddVectorObs(getTargetAngle(nextPos));

            Vector3 offset = new Vector2(0.531f * Mathf.Sin((-33.09f - agentRB.rotation) * radians), 0.531f * Mathf.Cos((-33.09f - agentRB.rotation) * radians));

            for (int i = 0; i < nLaser; i++)
            {
                hit = Physics2D.Raycast(transform.position + offset, (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians))), laserDist, bitMask);

                if (displayLidar && hit)
                    Debug.DrawRay(transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians))), Color.blue);

                if (hit)
                    AddVectorObs(hit.distance);
                else
                    AddVectorObs(laserDist);
            }

            offset = new Vector2(0.531f * Mathf.Sin((146.91f - agentRB.rotation) * radians), 0.531f * Mathf.Cos((146.91f - agentRB.rotation) * radians));
            for (int i = 0; i < nLaser; i++)
            {
                hit = Physics2D.Raycast(transform.position + offset, (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * radians), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * radians))), laserDist, bitMask);
                if (displayLidar && hit)
                    Debug.DrawRay(transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * radians), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * radians))), Color.blue);

                if (hit)
                    AddVectorObs(hit.distance);
                else
                    AddVectorObs(laserDist);
            }
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        MoveAgent(vectorAction);
        CalcReward();
    }

    void FixedUpdate()
    {
        if (targetVertical != 0)
        {
            if (virtualLinearVelocity < targetVertical * (1 - Mathf.Abs(virtualAngularVelocity)))
            {
                virtualLinearVelocity += linearMaxAcceleration * 0.1f;
            }
            else
                virtualLinearVelocity -= linearMaxAcceleration * 0.1f;
        }
        else
        {
            // Drag
            virtualLinearVelocity = virtualLinearVelocity * drag;
        }

        if (targetHoizontal != 0)
        {
            if (virtualAngularVelocity < targetHoizontal)
            {
                virtualAngularVelocity += angularMaxAcceleration * Math.Abs(targetHoizontal) * 0.1f;
            }
            else
            {
                virtualAngularVelocity -= angularMaxAcceleration * Math.Abs(targetHoizontal) * 0.1f;
            }
        }
        else
        {
            // Drag
            virtualAngularVelocity = virtualAngularVelocity * drag;
        }

        // Set velocities
        agentRB.angularVelocity = virtualAngularVelocity * -Mathf.Rad2Deg;

        agentRB.velocity = new Vector2
        (
            virtualLinearVelocity * Mathf.Sin(-agentRB.rotation * Mathf.Deg2Rad), // x
            virtualLinearVelocity * Mathf.Cos(-agentRB.rotation * Mathf.Deg2Rad)  // y
        );
    }

    public void MoveAgent(float[] act)
    {
        // AI input
        moveHorizontal = Mathf.Clamp(act[0], -angularVelocityLimit, angularVelocityLimit);
        moveVertical = Mathf.Clamp(act[1], linearReverseVelocityLimit, linearVelocityLimit);

        if (moveHorizontal != 0)
        {
            targetHoizontal = moveHorizontal;
        }
            

        if (moveVertical != 0)
        {
            targetVertical = moveVertical;
        }
            
    }

    public void CalcReward()
    {
        AddReward(-0.01f);

        float nextDistance = Vector2.Distance(agentRB.transform.localPosition, nextPos);
        float distBetweenPos = Vector2.Distance(currentPos, nextPos);
        float currentDistance = Vector2.Distance(agentRB.transform.localPosition, currentPos);

        if (currentDistance < lastDistance)
        {
            AddReward(0.02f);
            lastDistance = currentDistance;
        }

        if (Mathf.Abs(getTargetAngle(currentPos)) < 5 )
            AddReward(0.01f);

        if (Mathf.Abs(getTargetAngle(nextPos)) < 15)
            AddReward(0.01f);

        if (currentDistance > lastDistance + 2)
        {
            AddReward(-1f);
            Done();
        }
            

        if (nextDistance < distBetweenPos || distBetweenPos == 0)
        {
            pathIdx++;
            currentPos = nextPos;
            if (pathIdx > path.Length-1)
            {
                if (currentDistance <0.2f)
                {
                    AddReward(1f);
                    Done();
                }
            }
            else
            {
                nextPos = path[pathIdx];
                AddReward(0.2f);
            }
            lastDistance = Vector2.Distance(agentRB.transform.localPosition, currentPos);
        }
    }

    public override void AgentReset()
    {
        int pathNr = rnd.Next(200);
        path = pathArray[pathNr];
        if (isSpawned)
        {
            int squarePlace = rnd.Next(1,path.Length-1);
            square.transform.position = (Vector2)transform.parent.position + path[squarePlace];
        }
        //Debug.Log(pathNr+" "+gameObject.name);
        for (int i = 0; i < path.Length - 1; i++)
        {
            Debug.DrawLine((Vector2)transform.parent.position + path[i], (Vector2)transform.parent.position + path[i + 1], Color.red, 10f);
        }

        pathIdx = 1;
        currentPos = path[0];
        nextPos = path[pathIdx];
        agentRB.transform.localPosition = currentPos;
        agentRB.rotation = Vector2.SignedAngle(simpleVec, nextPos - currentPos); // getTargetAngle(nextPos);
        
        lastDistance = Vector2.Distance(currentPos, nextPos);

        agentRB.velocity = new Vector2(0.0f, 0.0f);
        agentRB.angularVelocity = 0.0f;

        virtualLinearVelocity = 0.0f;
        virtualAngularVelocity = 0.0f;
        targetVertical = 0.0f;
        targetHoizontal = 0.0f;
    }

    void OnCollisionEnter2D(Collision2D col)
    {
        AddReward(-1f);
        Done();
    }

    //void OnTriggerEnter2D(Collider2D col) // OnTriggerEnter2D
    //{
    //    AddReward(-100f);
    //}
    
    float getTargetAngle(Vector2 point)
    {
        float anAngle = Vector2.SignedAngle(simpleVec, point - (Vector2)agentRB.transform.localPosition);
        float rotation = agentRB.rotation % 360;
        rotation = anAngle - rotation;

        if (rotation > 180)
            rotation = rotation - 360;
        else if (rotation < -180)
            rotation = rotation + 360;

        return rotation;
    }

    void ReadCSVFile()
    {
        pathArray = new Vector2[200][];
     
        StreamReader strReader = new StreamReader("Assets/ML-Agents/Examples/AiObstacleAvoidance/Resources/pathes.csv");
        bool endOfFile = false;
        int index = 0;
        while (!endOfFile)
        {
            string data_string = strReader.ReadLine();
            if (data_string == null)
            {
                endOfFile = true;
                break;
            }
            string[] data_values = data_string.Split(',');
            string[] ting;

            Vector2[] paths = new Vector2[data_values.Length / 150];

            float f1 = 0.0f;
            float f2 = 0.0f;

            for (int i = 0;i<data_values.Length/150;i++)
            {
                ting = data_values[i * 150].Split('.');
                f1 = Single.Parse(ting[0]) + Single.Parse(ting[1].Remove(4)) / Mathf.Pow(10, 4);

                ting = data_values[(i * 150)+1].Split('.');
                f2 = Single.Parse(ting[0]) + (Single.Parse(ting[1].Remove(4)) / Mathf.Pow(10, 4));

                paths[i] = new Vector2(f1,f2);
            }
            //Debug.Log(data_values[0]);
            pathArray[index] = paths;
            index++;
        }
    }

}
