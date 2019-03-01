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

    private Vector2[] path;

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
                virtualAngularVelocity += angularMaxAcceleration * 0.1f;
            }
            else
            {
                virtualAngularVelocity -= angularMaxAcceleration * 0.1f;
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
        moveHorizontal = Mathf.Clamp(act[0], -1, 1);
        moveVertical = Mathf.Clamp(act[1], -1, 1);

        if (moveHorizontal != 0)
        {
            targetHoizontal = moveHorizontal * angularVelocityLimit;
        }
            

        if (moveVertical != 0)
        {
            if (moveVertical>0)
                targetVertical = moveVertical * linearVelocityLimit;
            else
                targetVertical = moveVertical * linearReverseVelocityLimit;
        }
            
    }

    public void CalcReward()
    {

        float distToIndex = Vector2.Distance(agentRB.transform.localPosition,path[pathIdx]);
        float distToNextIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx+1]);
        float distanceToGoal = Vector2.Distance(agentRB.transform.localPosition, path[path.Length-1]);
        int lastIndex = pathIdx;

        while (pathIdx < path.Length-2 && distToIndex > distToNextIndex)
        {
            pathIdx += 1;
            distToIndex = distToNextIndex;
            distToNextIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx + 1]);
        }

        if (distToIndex > 5.0f)
        {
            Done();
            AddReward(-0.5f);
        }

        if (lastIndex < pathIdx)
            AddReward(0.025f * (pathIdx-lastIndex) );
        else
            AddReward(-0.001f);

        if (pathIdx > path.Length - 11)
            currentPos = path[path.Length - 1];
        else
            currentPos = path[pathIdx + 10];

        if (pathIdx > path.Length-21)
            nextPos = path[path.Length-1];
        else
            nextPos = path[pathIdx+20];

        if (distanceToGoal < 1.0f)
        {
            Done();
            AddReward(1f);
        }

    }

    public override void AgentReset()
    {
        int pathNr = rnd.Next(200);
        path = pathArray[pathNr];

        if (isSpawned)
        {
            int squarePlace = rnd.Next(20,path.Length-20);
            Vector2 squareOffest = new Vector2(0.4f, 0.4f);
            square.transform.position = (Vector2)transform.parent.position + path[squarePlace] + squareOffest;
        }
        //Debug.Log(pathNr+" "+gameObject.name);
        for (int i = 0; i < path.Length - 1; i++)
        {
            Debug.DrawLine((Vector2)transform.parent.position + path[i], (Vector2)transform.parent.position + path[i + 1], Color.red, 10f);
        }

        pathIdx = 20;
        currentPos = path[31];
        nextPos = path[41];
        agentRB.transform.localPosition = path[0];
        agentRB.rotation = Vector2.SignedAngle(simpleVec, path[31] - path[0]); // getTargetAngle(nextPos);

        agentRB.velocity = new Vector2(0.0f, 0.0f);
        agentRB.angularVelocity = 0.0f;

        virtualLinearVelocity = 0.0f;
        virtualAngularVelocity = 0.0f;
        targetVertical = 0.0f;
        targetHoizontal = 0.0f;
    }

    void OnCollisionEnter2D(Collision2D col)
    {
        Done();
        if (pathIdx < 22)
            AddReward(-1f);
    }

    void OnTriggerEnter2D(Collider2D col) // OnTriggerEnter2D
    {
        if (pathIdx < 22)
            AddReward(-0.1f);
    }


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
     
        StreamReader strReader = new StreamReader("pathes.csv");
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

            Vector2[] paths = new Vector2[data_values.Length / 8];

            float f1 = 0.0f;
            float f2 = 0.0f;

            for (int i = 0;i<data_values.Length/8;i++)
            {
                ting = data_values[i * 8].Split('.');
                f1 = Single.Parse(ting[0]) + Single.Parse(ting[1].Remove(4)) * Mathf.Pow(10, -4);

                ting = data_values[(i * 8)+1].Split('.');
                f2 = Single.Parse(ting[0]) + (Single.Parse(ting[1].Remove(4)) * Mathf.Pow(10, -4));

                paths[i] = new Vector2(f1,f2);
            }
            //Debug.Log(data_values[0]);
            pathArray[index] = paths;
            index++;
        }
    }

}
