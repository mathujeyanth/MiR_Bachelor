using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MiR_Robot_CameraControl : MonoBehaviour
{
    void Update()
    {
        float moveHorizontal = Input.GetAxis("Horizontal");
        transform.position = transform.position + new Vector3(moveHorizontal, 0);
    }
}
