using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MiR_Robot_CameraControl : MonoBehaviour
{
    //public Button Left,Right;

    //void Start()
    //{
    //    Left.onClick.AddListener(TaskLeft);
    //    Right.onClick.AddListener(TaskRight);
    //}

    void Update()
    {
        float moveHorizontal = Input.GetAxis("Horizontal");
        transform.position = transform.position + new Vector3(moveHorizontal, 0);
    }

    //void TaskLeft()
    //{
    //    transform.position = transform.position + new Vector3(-49, 0);
    //}

    //void TaskRight()
    //{
    //    transform.position = transform.position + new Vector3(49, 0);
    //}

}
