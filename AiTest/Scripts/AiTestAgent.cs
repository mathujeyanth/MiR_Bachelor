using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;

public class AiTestAgent : Agent
{
    public float laserDist = 20;
    public bool useVectorObs;
    public bool displayLidar = false;

    private const int nLaser = 30;
    private const int bitMask = 1 << 9;
    private const int frontStart = 180;
    private const float degreesPrLaser = 270 / nLaser;
    private const float radians = Mathf.PI / 180;

    private float moveHorizontal = 0;
    private float moveVertical = 0;

    public float linearMaxAcceleration = 0.5f;
    public float angularMaxAcceleration = 0.5f;
    public float linearVelocityLimit = 1.5f;
    public float linearReverseVelocityLimit = -0.5f;
    public float angularVelocityLimit = 1.0f;

    private float virtualLinearVelocity = 0;
    private float virtualAngularVelocity = 0;

    private RaycastHit2D hit;

    private Rigidbody2D agentRB;
    private AiTestAcademy academy;
    
    public override void InitializeAgent()
    {
        base.InitializeAgent();
        academy = FindObjectOfType<AiTestAcademy>();
        //rayPer = GetComponent<RayPerception>();  
        agentRB = GetComponent<Rigidbody2D>();
    }

    
    public override void CollectObservations()
    {
        if (useVectorObs)
        {
            AddVectorObs(agentRB.velocity);
            AddVectorObs(agentRB.angularVelocity);
            Vector3 offset = new Vector2(0.531f * Mathf.Sin((-33.09f - agentRB.rotation) * radians), 0.531f * Mathf.Cos((-33.09f - agentRB.rotation) * radians));

            for (int i = 0; i < nLaser; i++)
            {
                hit = Physics2D.Raycast(transform.position + offset, (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians))), laserDist, bitMask);
                if (displayLidar && hit)
                    Debug.DrawRay(transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - agentRB.rotation) * radians))), Color.blue);
                AddVectorObs(hit.distance);
            }

            offset = new Vector2(0.531f * Mathf.Sin((146.91f - agentRB.rotation) * radians), 0.531f * Mathf.Cos((146.91f - agentRB.rotation) * radians));

            for (int i = 0; i < nLaser; i++)
            {
                hit = Physics2D.Raycast(transform.position + offset, (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * radians), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * radians))), laserDist, bitMask);
                if (displayLidar && hit)
                    Debug.DrawRay(transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - agentRB.rotation) * radians), Mathf.Cos(((i * degreesPrLaser) - agentRB.rotation) * radians))), Color.blue);
                AddVectorObs(hit.distance);
            }
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        //AddReward(-1f / agentParameters.maxStep);
        MoveAgent(vectorAction);
    }

    public void MoveAgent(float[] act)
    {
        // Player input
        moveHorizontal = Mathf.Clamp(act[0], -1.0f, 1.0f);
        moveVertical = Mathf.Clamp(act[1], -1.0f, 1.0f);
  
        //if (moveHorizontal != 0.0f && moveVertical != 0.0f)
        //{
        //    moveHorizontal = moveHorizontal * 0.5f;
        //    moveVertical = moveVertical * 0.5f;
        //}

        // Linear- and angularvelocity calculation
        virtualAngularVelocity += angularMaxAcceleration * moveHorizontal * 0.1f * 1.1f;//Time.fixedDeltaTime;
        virtualLinearVelocity += linearMaxAcceleration * moveVertical * 0.1f * 1.1f;//Time.fixedDeltaTime;

        // Speed limits
        virtualAngularVelocity = Mathf.Clamp(virtualAngularVelocity, -angularVelocityLimit * (1 - Mathf.Abs(moveVertical)), angularVelocityLimit * (1 - Mathf.Abs(moveVertical)));
        virtualLinearVelocity = Mathf.Clamp(virtualLinearVelocity, linearReverseVelocityLimit * (1 - Mathf.Abs(moveHorizontal)), linearVelocityLimit * (1 - Mathf.Abs(moveHorizontal)));

        // Drag
   
        virtualAngularVelocity = virtualAngularVelocity * 0.95f;
        virtualLinearVelocity = virtualLinearVelocity * 0.95f;

        // Anuglar and linear acceleration
        agentRB.angularVelocity = virtualAngularVelocity * -57.29579f;

        agentRB.velocity = new Vector2
        (
            virtualLinearVelocity * Mathf.Sin(-agentRB.rotation * radians), // x
            virtualLinearVelocity * Mathf.Cos(-agentRB.rotation * radians)  // y
        );
        AddReward(virtualLinearVelocity);
    }

    public override void AgentReset()
    {
        agentRB.rotation = Random.value*360;
        agentRB.velocity = new Vector2(0.0f, 0.0f);
        agentRB.angularVelocity = 0.0f;
        agentRB.transform.localPosition = new Vector2(0.0f, 0.0f);
       
    }

    void OnCollisionEnter2D(Collision2D col)
    {
        AddReward(-1000f);
        Done();
    }

    void OnTriggerEnter2D(Collider2D col) // OnTriggerEnter2D
    {
        AddReward(-50f);
    }
}
