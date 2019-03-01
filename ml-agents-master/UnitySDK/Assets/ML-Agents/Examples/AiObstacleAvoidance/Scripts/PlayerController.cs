using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

[System.Serializable]
public class Boundary
{
    public float xMin, xMax, yMin, yMax;
}

public class PlayerController : MonoBehaviour
{
    //private StreamWriter sw;

    public float linearMaxAcceleration = 0.5f;
    public float angularMaxAcceleration = 0.5f;
    public float linearVelocityLimit = 1.5f;
    public float linearReverseVelocityLimit = -0.5f;
    public float angularVelocityLimit = 1.0f;
    public float laserDist = 29;


    public Boundary boundary;

    private const int nLaser = 30;
    private const int bitMask = 1 << 9;
    private const int frontStart = 180;
    private const float degreesPrLaser = 270 / nLaser;
    private const float radians = Mathf.PI / 180;

    private float moveHorizontal = 0;
    private float moveVertical = 0;

    private float virtualLinearVelocity = 0;
    private float virtualAngularVelocity = 0;

    private RaycastHit2D hit;
    private RaycastHit2D[] hits = new RaycastHit2D[nLaser];

    private Rigidbody2D rb;
   

    // Start is called before the first frame update
    void Start()
    {
        //sw = new StreamWriter(new FileStream("data", FileMode.Create));
        rb = GetComponent<Rigidbody2D>();
    }

    void Update()
    {
       
    }

    void FixedUpdate()
    {
        // Player input
        moveHorizontal = Input.GetAxis("Horizontal");
        moveVertical = Input.GetAxis("Vertical");

        if (moveHorizontal != 0.0f && moveVertical != 0.0f)
        {
            moveHorizontal = moveHorizontal * 0.5f;
            moveVertical = moveVertical * 0.5f;
        }

        // Linear- and angularvelocity calculation
        virtualAngularVelocity += angularMaxAcceleration * moveHorizontal * 0.1f;//Time.fixedDeltaTime;
        virtualLinearVelocity += linearMaxAcceleration * moveVertical * 0.1f;//Time.fixedDeltaTime;

        // Speed limits
        virtualAngularVelocity = Mathf.Clamp(virtualAngularVelocity, -angularVelocityLimit*(1 - Mathf.Abs(moveVertical)), angularVelocityLimit* (1 - Mathf.Abs(moveVertical)));
        virtualLinearVelocity = Mathf.Clamp(virtualLinearVelocity,linearReverseVelocityLimit * (1 - Mathf.Abs(moveHorizontal)), linearVelocityLimit* (1 - Mathf.Abs(moveHorizontal)));

        // Drag
        if (moveHorizontal == 0.0f)
            virtualAngularVelocity = virtualAngularVelocity * 0.80f;

        if (moveVertical == 0.0f)
            virtualLinearVelocity = virtualLinearVelocity * 0.80f;

        // Anuglar and linear acceleration
        rb.angularVelocity = virtualAngularVelocity * -57.29579f;

        rb.velocity =  new Vector2
        (
            virtualLinearVelocity * Mathf.Sin(-rb.rotation * radians), // x
            virtualLinearVelocity * Mathf.Cos(-rb.rotation * radians)  // y
        );

        // Lidar

        // Front lidar offset from center
        Vector3 offset = new Vector2(0.531f * Mathf.Sin((-33.09f - rb.rotation) * Mathf.PI / 180), 0.531f * Mathf.Cos((-33.09f - rb.rotation) * Mathf.PI / 180));

        for (int i = 0;i< nLaser; i++)
        {
            hit = Physics2D.Raycast(transform.position + offset, (new Vector2(Mathf.Sin(((frontStart+(i * degreesPrLaser)) - rb.rotation) * Mathf.PI / 180), Mathf.Cos(((frontStart+(i * degreesPrLaser)) - rb.rotation) * Mathf.PI / 180))), laserDist, bitMask);
            hits[i] = hit;
            if (hit)
            {
                Debug.DrawRay(transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((frontStart + (i * degreesPrLaser)) - rb.rotation) * Mathf.PI / 180), Mathf.Cos(((frontStart + (i * degreesPrLaser)) - rb.rotation) * Mathf.PI / 180))), Color.blue);
            }

            
        }
        //sw.Write("Front lidar: \n");
        //for (int i = 0;i<nLaser;i++)
        //{
        //    if (hits[i].distance == 0)
        //        sw.Write(laserDist);
        //    else
        //        sw.Write(hits[i].distance);

        //    if (i == nLaser - 1)
        //        sw.Write("\n");
        //    else
        //        sw.Write(" ");
        //}

        // Back lidar offset from center
        offset = new Vector2(0.531f * Mathf.Sin((146.91f - rb.rotation) * Mathf.PI / 180), 0.531f * Mathf.Cos((146.91f - rb.rotation) * Mathf.PI / 180));
        
        for (int i = 0; i < nLaser; i++)
        {
            hit = Physics2D.Raycast(transform.position + offset, (new Vector2(Mathf.Sin(((i * degreesPrLaser) - rb.rotation) * Mathf.PI / 180), Mathf.Cos(((i * degreesPrLaser) - rb.rotation) * Mathf.PI / 180))), laserDist, bitMask);
            hits[i] = hit;
            if (hit)
            {
                Debug.DrawRay(transform.position + offset, hit.distance * (new Vector2(Mathf.Sin(((i * degreesPrLaser) - rb.rotation) * Mathf.PI / 180), Mathf.Cos(((i * degreesPrLaser) - rb.rotation) * Mathf.PI / 180))), Color.blue);
            }
        }
        //sw.Write("Rear lidar: \n");
        //for (int i = 0; i < nLaser; i++)
        //{
        //    if (hits[i].distance == 0)
        //        sw.Write(laserDist);
        //    else
        //        sw.Write(hits[i].distance);

        //    if (i == nLaser - 1)
        //        sw.Write("\n");
        //    else
        //        sw.Write(" ");
        //}

    }

    //void OnCollisionEnter2D(Collision2D col)
    //{
        
    //    Debug.Log("Collision!");
    //}

    //void OnTriggerStay2D(Collider2D col) // OnTriggerEnter2D
    //{
    //    Debug.Log("Danger!");
    //}

    //void OnApplicationPause()
    //{
    //    //Remember to close your StreamWriter!
    //    sw.Close();
    //}

}   