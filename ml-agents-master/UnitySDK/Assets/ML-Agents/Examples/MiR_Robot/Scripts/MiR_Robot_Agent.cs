using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using System.IO;
using System;

public class MiR_Robot_Agent : Agent
{
    public float laserDist = 2;
    public bool useVectorObs = true;
    public bool displayLidar = false;
    public bool displayPath = false;

    public bool EnableMaxDiv = true;
    public bool EnableUncertainty = true;
    public bool testAgent = false;


    private Vector2[] path;

    private Vector2[][] pathArray;
    private System.Random rnd = new System.Random();

    private int pathIdx;
    private Vector2 currentPos;
    //private Vector2 nextPos;
    private Vector2 simpleVec;
    private const int nLaser = 40;
    private float[] hitDistances = new float[nLaser];
    private float[] FrontsafetyDistances = new float[nLaser];
    private float[] BacksafetyDistances = new float[nLaser];

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

    private Vector2 TimeFrameP0;
    private Vector2 TimeFrameP1;
    private Vector2 TimeFrameP2;

    private float[] lidarInput = new float[zones];

    private RaycastHit2D hit;

    private Rigidbody2D agentRB;
    private CircleCollider2D safetyZone;

    // Stream
    //private StreamWriter sw;
    // Bool test agent
    
    // Time
    private int timeStep = 0;
    // Linear 
    private float minSpeed = 0;
    private float maxSpeed = 0;
    private float averageSpeed = 0;
    private float steps90 = 0;
    private float steps10 = 0;
    private float steps0 = 0;
    // Angular
    private float maxAbsVal = 0;
    private float sumAbsVal = 0;
    // Clearance
    private float minUS = 0;
    private float averageUS = 0;
    private float minMS = 0;
    private float averageMS = 0;
    // Path
    private float averageDiv = 0;

    public override void InitializeAgent()
    {
        base.InitializeAgent();
        safetyZone = GetComponent<CircleCollider2D>();
        agentRB = GetComponent<Rigidbody2D>();
        simpleVec = new Vector2(0f, 1f);

        float lengthB = safetyZone.radius;
        float vinkelB = 30.7f;
        float lengthC = 0.4594f;
        for (int i = 0;i<nLaser;i++)
        {
            vinkelB = (i * degreesPrLaser) + 30.7f;

            if (vinkelB > 180.0f)
            {
                vinkelB = 180.0f - (vinkelB - 180.0f);
            }

            FrontsafetyDistances[i] = lengthC * Mathf.Cos(vinkelB * Mathf.Deg2Rad) + Mathf.Sqrt(Mathf.Pow(lengthB, 2) + Mathf.Pow(lengthC, 2) * Mathf.Pow(Mathf.Cos(vinkelB * Mathf.Deg2Rad), 2) - Mathf.Pow(lengthC, 2));

        }

        vinkelB = 30.96f;
        lengthC = 0.4558f;

        for (int i = 0; i < nLaser; i++)
        {
            vinkelB = (i * degreesPrLaser) + 30.96f;

            if (vinkelB > 180.0f)
            {
                vinkelB = 180.0f - (vinkelB - 180.0f);
            }

            BacksafetyDistances[i] = lengthC * Mathf.Cos(vinkelB * Mathf.Deg2Rad) + Mathf.Sqrt(Mathf.Pow(lengthB, 2) + Mathf.Pow(lengthC, 2) * Mathf.Pow(Mathf.Cos(vinkelB * Mathf.Deg2Rad), 2) - Mathf.Pow(lengthC, 2));

        }

        descionFreq = agentParameters.numberOfActionsBetweenDecisions;
    }
    

    public override void CollectObservations()
    {
        if (useVectorObs)
        {   
            AddVectorObs( roundDec(virtualLinearVelocity/linearVelocityLimit,2) );
            
            AddVectorObs( roundDec(virtualAngularVelocity,2));
            AddVectorObs( roundDec(getTargetAngle(currentPos)/180.0f,2) );
            float dist = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx]);
            AddVectorObs( roundDec(dist / maxDeviation,2));
            float randomVal = 0.0f;
            // front lidar

            Vector3 offset = new Vector2(0.4594f * Mathf.Sin((-30.7f - agentRB.rotation) * Mathf.Deg2Rad), 0.4594f * Mathf.Cos((-30.7f - agentRB.rotation) * Mathf.Deg2Rad));

            float minValAvUs = laserDist;
            float minValAvMs = laserDist;

            for (int i = 0; i < nLaser; i++)
            {
                if (EnableUncertainty)
                    randomVal = (float)((rnd.NextDouble()+rnd.NextDouble()-1)*0.1);

                hit = Physics2D.Raycast(agentRB.transform.position+offset, (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad))), laserDist); // , bitMask

                if (displayLidar && hit)
                {
                    Debug.DrawRay(agentRB.transform.position+offset, hit.distance * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad))), Color.blue);
                    Debug.DrawRay(agentRB.transform.position+offset, FrontsafetyDistances[i] * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad))), Color.red);
                }

                if (testAgent && hit)
                {
                    if (minValAvUs > hit.distance)
                        minValAvUs = hit.distance;

                    if (minValAvMs > hit.distance - BacksafetyDistances[i])
                        minValAvMs = hit.distance - BacksafetyDistances[i];

                    if (minUS > hit.distance)
                        minUS = hit.distance;

                    if (minMS > hit.distance - BacksafetyDistances[i])
                        minMS = hit.distance - BacksafetyDistances[i];
                }

                if (hit)
                    hitDistances[i] = hit.distance - FrontsafetyDistances[i] + randomVal;
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
            //Debug.Log("Front = " + String.Join(" ",
            // new List<float>(lidarInput)
            // .ConvertAll(i => i.ToString())
            // .ToArray()));

            AddVectorObs(lidarInput);

            // Rear lidar

            offset = new Vector2(0.4558f * Mathf.Sin((149.04f - agentRB.rotation) * Mathf.Deg2Rad), 0.4558f * Mathf.Cos((149.04f - agentRB.rotation) * Mathf.Deg2Rad));

            for (int i = 0; i < nLaser; i++)
            {
                if (EnableUncertainty)
                    randomVal = (float)((rnd.NextDouble() + rnd.NextDouble() - 1) * 0.1);

                hit = Physics2D.Raycast(agentRB.transform.position + offset, (new Vector2(Mathf.Sin((((i * degreesPrLaser)) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad))), laserDist); // , bitMask

                if (displayLidar && hit)
                {
                    Debug.DrawRay(agentRB.transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad))), Color.blue);
                    Debug.DrawRay(agentRB.transform.position + offset, BacksafetyDistances[i] * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * Mathf.Deg2Rad))), Color.red);
                }

                if (testAgent && hit)
                {
                    if (minValAvUs > hit.distance)
                        minValAvUs = hit.distance;

                    if (minValAvMs > hit.distance - BacksafetyDistances[i])
                        minValAvMs = hit.distance - BacksafetyDistances[i];

                    if (minUS > hit.distance)
                        minUS = hit.distance;

                    if (minMS > hit.distance - BacksafetyDistances[i])
                        minMS = hit.distance - BacksafetyDistances[i];
                }

                if (hit)
                    hitDistances[i] = hit.distance - BacksafetyDistances[i] + randomVal;
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
                lidarInput[i] = Mathf.Round(20 * lidarInput[i]);
                lidarInput[i] /= 20 * laserDist;
            }

            AddVectorObs(lidarInput);

            if (testAgent)
            {
                averageUS = averageUS + minValAvUs;
                averageMS = averageMS + minValAvMs;
            }
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
        }
        
    }

    void FixedUpdate()
    {
        int ran;
        if (EnableUncertainty)
        {
            TimeFrameP2 = TimeFrameP1;
            TimeFrameP1 = TimeFrameP0;

            ran = rnd.Next(0,1);
            switch(ran)
            {
                case 0:
                    targetVertical = TimeFrameP1[0];
                    targetHoizontal = TimeFrameP1[1];
                    break;
                case 1:
                    targetVertical = TimeFrameP2[0];
                    targetHoizontal = TimeFrameP2[1];
                    break;
            }
        }
        else
        {
            targetVertical = TimeFrameP0[0];
            targetHoizontal = TimeFrameP0[1];
        }
            
            

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
        
        if (Mathf.Abs(moveHorizontal) > 0.10f)
           targetHoizontal = moveHorizontal * angularVelocityLimit;
        else
           targetHoizontal = 0.0f;

        if (Mathf.Sqrt(Mathf.Pow(moveHorizontal, 2) + Mathf.Pow(moveVertical, 2)) > 1 && moveVertical > 0)
            moveVertical = Mathf.Sqrt(1 - Mathf.Pow(moveHorizontal,2));

        if (Mathf.Sqrt(Mathf.Pow(moveHorizontal, 2) + Mathf.Pow(moveVertical, 2)) > 1 && moveVertical < 0)
            moveVertical = -1*Mathf.Sqrt(1 - Mathf.Pow(moveHorizontal, 2));

        if (moveVertical > 0)
            targetVertical = moveVertical * linearVelocityLimit;
        else
            targetVertical = moveVertical * linearReverseVelocityLimit;

        TimeFrameP0 = new Vector2(targetVertical, targetHoizontal);
    }

    public void CalcReward()
    {

        float reward = 0;
        float distToIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx]);
        float distToNextIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx + 1]);
        float distanceToGoal = Vector2.Distance(agentRB.transform.localPosition, path[path.Length - 1]);

        while (pathIdx < path.Length - 2 && distToIndex >= distToNextIndex)
        {
            reward += 0.0025f;
            pathIdx += 1;
            distToIndex = distToNextIndex;
            distToNextIndex = Vector2.Distance(agentRB.transform.localPosition, path[pathIdx + 1]);
        }

        if (EnableMaxDiv && distToIndex > maxDeviation) //Meters it may deviate from path
        {
            //Done();
            reward += -0.001f * descionFreq;
        }

        reward += -0.0005f * descionFreq;

        if (triggers > 0)
        {
            reward += -0.005f * descionFreq;
        }

        //if (virtualLinearVelocity <= 0)
        //    AddReward(-0.0005f * descionFreq);

        if (pathIdx > path.Length - 41)
            currentPos = path[path.Length - 1];
        else
            currentPos = path[pathIdx + 40];

        //if (pathIdx > path.Length - 21)
        //    nextPos = path[path.Length - 1];
        //else
        //    nextPos = path[pathIdx + 20];

        if (testAgent)
        {
            // Time
            timeStep++;

            // Linear
            if (minSpeed > virtualLinearVelocity)
                minSpeed = virtualLinearVelocity;

            if (maxSpeed < virtualLinearVelocity)
                maxSpeed = virtualLinearVelocity;

            averageSpeed = averageSpeed + virtualLinearVelocity;

            if (virtualLinearVelocity > linearVelocityLimit * 0.9f)
                steps90++;

            if (virtualLinearVelocity < linearVelocityLimit * 0.1f)
                steps10++;

            if (virtualLinearVelocity < 0)
                steps0++;

            // Angular
            if (Mathf.Abs(virtualAngularVelocity) > maxAbsVal)
                maxAbsVal = Mathf.Abs(virtualAngularVelocity);

            sumAbsVal = sumAbsVal + Mathf.Abs(virtualAngularVelocity);

            // Path
            averageDiv = averageDiv + distToIndex;
        }

        if (distanceToGoal < safetyZone.radius)
        {
            Done();
            reward += 1f;
        }
        AddReward(reward);
    }

    public override void AgentReset()
    {
        // print test
        if (testAgent && timeStep != 0)
        {
            string output = "";
            output += "\n";
            output += timeStep;
            output += " ";
            output += minSpeed;
            output += " ";
            output += maxSpeed;
            output += " ";
            output += averageSpeed / timeStep;
            output += " ";
            output += steps90;
            output += " ";
            output += steps10;
            output += " ";
            output += steps0;
            output += " ";
            output += maxAbsVal;
            output += " ";
            output += sumAbsVal / timeStep;
            output += " ";
            output += minUS;
            output += " ";
            output += averageUS / timeStep;
            output += " ";
            output += minMS;
            output += " ";
            output += averageMS / timeStep;
            output += " ";
            output += averageDiv / timeStep;
            output += " ";


            File.AppendAllText("data.txt", output);

            // Time
            timeStep = 0;
            // Linear 
            minSpeed = 0;
            maxSpeed = 0;
            averageSpeed = 0;
            steps90 = 0;
            steps10 = 0;
            steps0 = 0;
            // Angular
            maxAbsVal = 0;
            sumAbsVal = 0;
            // Clearance
            minUS = laserDist;
            averageUS = 0;
            minMS = laserDist;
            averageMS = 0;
            // Path
            averageDiv = 0;
        }

        int pathNr;
        RaycastHit2D hit = Physics2D.Raycast((Vector2)transform.parent.position, transform.up, 1);

        do
        {
            pathNr = rnd.Next(200);
            path = pathArray[pathNr];
            for (int i =0;i<36;i++)
            {
                hit = Physics2D.Raycast((Vector2)transform.parent.position + path[0], (new Vector2(Mathf.Sin((i * 10) * Mathf.Deg2Rad), Mathf.Cos((i * 10) * Mathf.Deg2Rad))), safetyZone.radius);
                if (hit)
                {
                    //Debug.Log(pathNr + " " + transform.parent.gameObject.name);
                    break;
                }
                    
            }
        } while (hit);

        pathIdx = 0;
        currentPos = path[40];
        agentRB.transform.localPosition = path[0];
        agentRB.rotation = Vector2.SignedAngle(simpleVec, path[40] - path[0]); // getTargetAngle(nextPos);

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

    void ReadCSVFile(TextAsset pathesCSV)
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

            Vector2[] paths = new Vector2[data_values.Length / 2];

            float f1 = 0.0f;
            float f2 = 0.0f;

            for (int i = 0; i < data_values.Length / 2; i++)
            {
                ting = data_values[i * 2].Split('.');
                f1 = Single.Parse(ting[0]) + Single.Parse(ting[1].Remove(4)) * Mathf.Pow(10, -4);

                ting = data_values[(i * 2) + 1].Split('.');
                f2 = Single.Parse(ting[0]) + (Single.Parse(ting[1].Remove(4)) * Mathf.Pow(10, -4));

                paths[i] = new Vector2(f1, f2);
            }
            //Debug.Log(data_values[0]);
            pathArray[index] = paths;
            index++;
        }
    }

    float roundDec(float x, int y)
    {
        double x_d = x;
        return (float)Math.Round(x_d,y);
    }

}

