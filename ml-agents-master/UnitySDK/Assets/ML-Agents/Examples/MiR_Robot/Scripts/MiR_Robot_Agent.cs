﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using System.IO;
using System;

public class MiR_Robot_Agent : Agent
{
    public float laserDist = 5;
    public bool useVectorObs = true;
    public bool displayLidar = false;
    public bool displayPath = false;

    public bool EnableMaxDiv = true;

    private Vector2[] path;

    private Vector2[][] pathArray;
    private System.Random rnd = new System.Random();

    private int pathIdx;
    private Vector2 currentPos;
    //private Vector2 nextPos;
    private Vector2 simpleVec;
    private const int nLaser = 40;
    private float[] hitDistances = new float[nLaser];
    private float[] safetyDistances = new float[nLaser];

    //private const int bitMask = 1 << 9;
    private const int frontStart = -180;
    private const float degreesPrLaser = 270 / (nLaser-1);
    private const float radians = Mathf.PI / 180;

    private float moveHorizontal = 0;
    private float moveVertical = 0;

    private float targetHoizontal = 0;
    private float targetVertical = 0;

    private float drag = 0.95f;

    private int descionFreq;

    private int triggers = 0;

    public float linearMaxAcceleration = 0.5f;
    public float angularMaxAcceleration = 0.5f;
    public float linearVelocityLimit = 1.5f;
    public float linearReverseVelocityLimit = 0.5f;
    public float angularVelocityLimit = 1.0f;

    private float virtualLinearVelocity = 0;
    private float virtualAngularVelocity = 0;

    private float maxDeviation = 2.0f;

    private const int overlap = 1;
    private const int zones = 8;

    private float[] lidarInput = new float[zones];

    private RaycastHit2D hit;

    public TextAsset pathesCSV;

    private Rigidbody2D agentRB;
    private CircleCollider2D safetyZone;

    public override void InitializeAgent()
    {
        base.InitializeAgent();
        safetyZone = GetComponent<CircleCollider2D>();
        agentRB = GetComponent<Rigidbody2D>();
        simpleVec = new Vector2(0f, 1f);
        ReadCSVFile();

        float vinkelB = 33.09f;
        float lengthB = safetyZone.radius;

        float lengthC = 0.531f;

        for (int i = 0;i<nLaser;i++)
        {
            vinkelB = (i * degreesPrLaser) + 33.09f;

            if (vinkelB > 180.0f)
            {
                vinkelB = 180.0f - (vinkelB - 180.0f);
            }

            safetyDistances[i] = lengthC * Mathf.Cos(vinkelB * Mathf.Deg2Rad) + Mathf.Sqrt(Mathf.Pow(lengthB, 2) + Mathf.Pow(lengthC, 2) * Mathf.Pow(Mathf.Cos(vinkelB * Mathf.Deg2Rad), 2) - Mathf.Pow(lengthC, 2));

        }

        descionFreq = agentParameters.numberOfActionsBetweenDecisions;
    }


    public override void CollectObservations()
    {
        if (useVectorObs)
        {
            AddVectorObs( (virtualLinearVelocity/linearVelocityLimit) );
            AddVectorObs(virtualAngularVelocity);
            AddVectorObs( (Mathf.Round(getTargetAngle(currentPos)/10.0f)/18.0f) );
            float dist = Vector2.Distance(currentPos, path[pathIdx]);
            AddVectorObs((Vector2.Distance(agentRB.transform.localPosition, path[pathIdx])+dist) / (maxDeviation+dist));

            // front lidar

            Vector3 offset = new Vector2(0.531f * Mathf.Sin((-33.09f - agentRB.rotation) * Mathf.Deg2Rad), 0.531f * Mathf.Cos((-33.09f - agentRB.rotation) * Mathf.Deg2Rad));

            for (int i = 0; i < nLaser; i++)
            {

                hit = Physics2D.Raycast(agentRB.transform.position+offset, (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad))), laserDist); // , bitMask

                if (displayLidar && hit)
                {
                    Debug.DrawRay(agentRB.transform.position+offset, hit.distance * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad))), Color.blue);
                    Debug.DrawRay(agentRB.transform.position+offset, safetyDistances[i] * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad))), Color.red);
                }
                    

                if (hit)
                    hitDistances[i] = hit.distance - safetyDistances[i];
                else
                    hitDistances[i] = laserDist;
            }

            for (int i = 0;i<zones;i++)
            {
                lidarInput[i] = laserDist;
            }

            for (int i = 0;i<((nLaser / zones) + 2*overlap); i++)
            {
                if (i < ((nLaser/zones)+overlap))
                {
                    if (hitDistances[i] < lidarInput[0])
                        lidarInput[0] = hitDistances[i];

                    if (hitDistances[i+((nLaser/zones)*(zones-1)-overlap)] < lidarInput[zones-1])
                        lidarInput[zones-1] = hitDistances[i + ((nLaser / zones) * (zones - 1) - overlap)];
                }
                for (int j = 1;j<zones-1;j++)
                {
                    if (hitDistances[i + ((nLaser / zones)*j)-overlap] < lidarInput[j])
                        lidarInput[j] = hitDistances[i + ((nLaser / zones) * j) - overlap];
                }
            }

            for (int i = 0; i < zones; i++)
            {
                lidarInput[i] = Mathf.Round(20*lidarInput[i]);
                lidarInput[i] /= 20*laserDist;
            }

            AddVectorObs(lidarInput);

            // Rear lidar

            offset = new Vector2(0.531f * Mathf.Sin((146.91f - agentRB.rotation) * Mathf.Deg2Rad), 0.531f * Mathf.Cos((146.91f - agentRB.rotation) * Mathf.Deg2Rad));

            for (int i = 0; i < nLaser; i++)
            {
                hit = Physics2D.Raycast(agentRB.transform.position + offset, (new Vector2(Mathf.Sin((((i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad))), laserDist); // , bitMask

                if (displayLidar && hit)
                {
                    Debug.DrawRay(agentRB.transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad))), Color.blue);
                    Debug.DrawRay(agentRB.transform.position + offset, safetyDistances[i] * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad))), Color.red);
                }


                if (hit)
                    hitDistances[i] = hit.distance - safetyDistances[i];
                else
                    hitDistances[i] = laserDist;
            }

            for (int i = 0; i < zones; i++)
            {
                lidarInput[i] = laserDist;
            }

            for (int i = 0; i < ((nLaser / zones) + 2 * overlap); i++)
            {
                if (i < ((nLaser / zones) + overlap))
                {
                    if (hitDistances[i] < lidarInput[0])
                        lidarInput[0] = hitDistances[i];

                    if (hitDistances[i + ((nLaser / zones) * (zones - 1) - overlap)] < lidarInput[zones - 1])
                        lidarInput[zones - 1] = hitDistances[i + ((nLaser / zones) * (zones - 1) - overlap)];
                }
                for (int j = 1; j < zones - 1; j++)
                {
                    if (hitDistances[i + ((nLaser / zones) * j) - overlap] < lidarInput[j])
                        lidarInput[j] = hitDistances[i + ((nLaser / zones) * j) - overlap];
                }
            }

            for (int i = 0; i < zones; i++)
            {
                lidarInput[i] = Mathf.Round(20*lidarInput[i]);
                lidarInput[i] /= 20*laserDist;
            }

            AddVectorObs(lidarInput);
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        MoveAgent(vectorAction);
        CalcReward();
        if (displayPath)
        {
            for (int i = pathIdx; i < path.Length - 1; i++)
            {
                Debug.DrawLine((Vector2)transform.parent.position + path[i], (Vector2)transform.parent.position + path[i + 1], Color.red);
            }
            //for (int i = 0; i < 200; i++)
            //{
            //    Debug.DrawLine((Vector2)transform.parent.position + pathArray[i][0], (Vector2)transform.parent.position + pathArray[i][4], Color.red);
            //    Debug.DrawLine((Vector2)transform.parent.position + pathArray[i][pathArray[i].Length - 5], (Vector2)transform.parent.position + pathArray[i][pathArray[i].Length - 1], Color.red);
            //}
        }
        
    }

    void FixedUpdate()
    {
        if (targetVertical != 0)
        {
            if (virtualLinearVelocity < targetVertical)
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
        // AI output
        moveHorizontal = Mathf.Clamp(act[0], -1, 1);
        moveVertical = Mathf.Clamp(act[1], -1, 1);
        
        //if (moveHorizontal != 0)
        //{
            if (Mathf.Abs(moveHorizontal) > 0.10f)
                targetHoizontal = moveHorizontal * angularVelocityLimit;
            else
                targetHoizontal = 0.0f; 
        //}


        //if (moveVertical != 0)
        //{
            if (moveVertical > 0)
                targetVertical = moveVertical * linearVelocityLimit * (1 - Mathf.Abs(moveHorizontal));
            else
                targetVertical = moveVertical * linearReverseVelocityLimit * (1 - Mathf.Abs(moveHorizontal));
        //}

    }

    public void CalcReward()
    {

        float distToIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx]);
        float distToNextIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx + 1]);
        float distanceToGoal = Vector2.Distance(agentRB.transform.localPosition, path[path.Length - 1]);
        int lastIndex = pathIdx;

        while (pathIdx < path.Length - 2 && distToIndex >= distToNextIndex)
        {
            pathIdx += 1;
            distToIndex = distToNextIndex;
            distToNextIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx + 1]);
        }

        if (EnableMaxDiv && distToIndex > maxDeviation) //Meters it may deviate from path
        {
            //Done();
            AddReward(-0.002f);
        }

        if (lastIndex < pathIdx)
            AddReward(0.020f);
        
        AddReward(-0.0005f * descionFreq);

        if (triggers > 0)
        {
            AddReward(-0.005f * descionFreq);
        }

        //if (virtualLinearVelocity <= 0)
        //    AddReward(-0.0005f * descionFreq);

        if (pathIdx > path.Length - 6)
            currentPos = path[path.Length - 1];
        else
            currentPos = path[pathIdx + 5];

        //if (pathIdx > path.Length - 21)
        //    nextPos = path[path.Length - 1];
        //else
        //    nextPos = path[pathIdx + 20];

        if (distanceToGoal < safetyZone.radius)
        {
            Done();
            AddReward(1f);
        }

    }

    public override void AgentReset()
    {
        int pathNr;
        RaycastHit2D hit = Physics2D.Raycast((Vector2)transform.parent.position, transform.up, 1);

        do
        {
            pathNr = rnd.Next(200);
            path = pathArray[pathNr];
            for (int i =0;i<36;i++)
            {
                hit = Physics2D.Raycast((Vector2)transform.parent.position + path[0], (new Vector2(Mathf.Sin((i * 10) * Mathf.Deg2Rad), Mathf.Cos((i * 10) * Mathf.Deg2Rad))), 1);
                if (hit)
                {
                    //Debug.Log(pathNr + " " + transform.parent.gameObject.name);
                    break;
                }
                    
            }
        } while (hit);

        pathIdx = 0;
        currentPos = path[5];
        agentRB.transform.localPosition = path[0];
        agentRB.rotation = Vector2.SignedAngle(simpleVec, path[5] - path[0]); // getTargetAngle(nextPos);

        agentRB.velocity = new Vector2(0.0f, 0.0f);
        agentRB.angularVelocity = 0.0f;

        virtualLinearVelocity = 0.0f;
        virtualAngularVelocity = 0.0f;
        targetVertical = 0.0f;
        targetHoizontal = 0.0f;

        triggers = 0;
    }

    void OnCollisionEnter2D(Collision2D col)
    {
        //Debug.Log("Collision");
        AddReward(-1f);
        //Done();
    }

    void OnTriggerEnter2D(Collider2D col) // OnTriggerEnter2D
    {
        triggers++;
    }
    
    void OnTriggerExit2D(Collider2D col)
    {
        triggers--;
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
        string[] allPathsString = pathesCSV.text.Split('\n');
        bool endOfFile = false;
        int index = 0;
        while (!endOfFile)
        {
            string data_string = allPathsString[index];
            if (index == allPathsString.Length-1)
            {
                endOfFile = true;
                break;
            }
            string[] data_values = data_string.Split(',');
            string[] ting;

            Vector2[] paths = new Vector2[data_values.Length / 16];

            float f1 = 0.0f;
            float f2 = 0.0f;

            for (int i = 0; i < data_values.Length / 16; i++)
            {
                ting = data_values[i * 16].Split('.');
                f1 = Single.Parse(ting[0]) + Single.Parse(ting[1].Remove(4)) * Mathf.Pow(10, -4);

                ting = data_values[(i * 16) + 1].Split('.');
                f2 = Single.Parse(ting[0]) + (Single.Parse(ting[1].Remove(4)) * Mathf.Pow(10, -4));

                paths[i] = new Vector2(f1, f2);
            }
            //Debug.Log(data_values[0]);
            pathArray[index] = paths;
            index++;
        }
    }

}

