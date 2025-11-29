using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class App : MonoBehaviour
{
    public Transform target;

    public Swarm theSwarm;

    private void Update()
    {
        if(Input.GetKeyDown("space"))
        {
            theSwarm.SetGoal(target.transform.position);
        }
    }
}
