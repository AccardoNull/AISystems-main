using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;

    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    
    public float alignmentWeight = 0.5f;

    public float cohesionWeight = 1f;

    public float goalWeight = 1f;

    public float obstacleWeight = 0.9f;

    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f;

    public float initializationRadius = 1.0f;

    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;

    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;


    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        InitBoids();

    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];
        for (int i = 0; i < numberOfBoids; i++)
        {
            Vector3 local = Random.insideUnitSphere * initializationRadius;
            Vector3 world = transform.TransformPoint(local);
            Quaternion randYaw = Quaternion.AngleAxis(Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange), Vector3.up);
            Quaternion randPitch = Quaternion.AngleAxis(Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange), Vector3.right);
            Vector3 fwd = (randYaw * randPitch) * Vector3.forward;
            fwd.Normalize();
            boids[i] = new BBoid
            {
                position = world,
                forward = fwd,
                velocity = fwd * (0.5f * maxSpeed),
                alignment = Vector3.zero,
                cohesion = Vector3.zero,
                separation = Vector3.zero,
                obstacle = Vector3.zero,
                currentTotalForce = Vector3.zero
            };
            boidObjects[i] = Instantiate(boidPrefab, world, Quaternion.LookRotation(fwd, Vector3.up));
        }
       boidZeroPath = new NavMeshPath();
       currentCorner = 0;
       boidZeroNavigatingTowardGoal = false;
    }


    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for (int i = 0; i < boids.Length; i++)
        {
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;
        }
    }


    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        if (boids == null || boids.Length == 0) return;
        float dt = Time.fixedDeltaTime;
        ResetBoidForces();
        int n = boids.Length;
        for (int i = 0; i < n; i++)
        {
            Vector3 pos_i = boids[i].position;
            Vector3 fwd_i = boids[i].forward;
            int neighbourCount = 0;
            Vector3 avgVel = Vector3.zero;
            Vector3 avgPos = Vector3.zero;
            Vector3 sepSum = Vector3.zero;
            for (int j = 0; j < n; j++)
            {
                if (j == i) continue;
                Vector3 toN = boids[j].position - pos_i;
                float sqrD = toN.sqrMagnitude;
                if (sqrD > sqrNeighbourDistance) continue;
                if (Vector3.Dot(toN, fwd_i) <= 0f) continue;
                neighbourCount++;
                avgVel += boids[j].velocity;
                avgPos += boids[j].position;
                if (sqrD > 1e-6f)
                {
                    sepSum += (pos_i - boids[j].position) / sqrD;
                }
            }
            Vector3 ruleAlignment = Vector3.zero;
            Vector3 ruleCohesion = Vector3.zero;
            Vector3 ruleSeparation = Vector3.zero;
            Vector3 ruleWander = Vector3.zero;
            if (neighbourCount > 0)
            {
                Vector3 desiredVel = (avgVel / neighbourCount);
                if (desiredVel.sqrMagnitude > 1e-6f) ruleAlignment = desiredVel.normalized;
                Vector3 com = avgPos / neighbourCount;
                Vector3 toCOM = (com - pos_i);
                if (toCOM.sqrMagnitude > 1e-6f) ruleCohesion = toCOM.normalized;
                if (sepSum.sqrMagnitude > 1e-6f) ruleSeparation = sepSum.normalized;
            }
            else
            {
                if (boids[i].velocity.sqrMagnitude > 1e-6f)
                {
                    ruleWander = boids[i].velocity.normalized;
                }
                else
                {
                    ruleWander = boids[i].forward;
                }
            }
            Vector3 ruleObstacle = Vector3.zero;
            Collider[] hits = Physics.OverlapSphere(pos_i, obstacleCheckRadius);
            for (int h = 0; h < hits.Length; h++)
            {
                Collider c = hits[h];
                if (boidObjects[i] != null && c.transform == boidObjects[i]) continue;
                Vector3 closest = c.ClosestPointOnBounds(pos_i);
                Vector3 away = pos_i - closest;
                if (away.sqrMagnitude > 1e-6f)
                {
                    ruleObstacle += away.normalized;
                }
            }
            if (pos_i.x > 8f) ruleObstacle += new Vector3(-1f, 0f, 0f);
            if (pos_i.x < -8f) ruleObstacle += new Vector3( 1f, 0f, 0f);
            if (pos_i.z > 8f) ruleObstacle += new Vector3( 0f, 0f,-1f);
            if (pos_i.z < -8f) ruleObstacle += new Vector3( 0f, 0f, 1f);
            if (pos_i.y > 4f) ruleObstacle += new Vector3( 0f,-1f, 0f);
            if (pos_i.y < 1f) ruleObstacle += new Vector3( 0f, 1f, 0f);

            if (ruleObstacle.sqrMagnitude > 1e-6f) ruleObstacle.Normalize();
            boids[i].alignment = ruleAlignment;
            boids[i].cohesion = ruleCohesion;
            boids[i].separation = ruleSeparation;
            boids[i].obstacle = ruleObstacle;

            Vector3 vi = boids[i].velocity;
            if (neighbourCount > 0)
            {
                boids[i].currentTotalForce += separationWeight * ((ruleSeparation * boidForceScale) - vi);
                boids[i].currentTotalForce += alignmentWeight * ((ruleAlignment * boidForceScale) - vi);
                boids[i].currentTotalForce += cohesionWeight * ((ruleCohesion * boidForceScale) - vi);
            }
            else
            {
                boids[i].currentTotalForce += wanderWeight * ((ruleWander * boidForceScale) - vi);
            }
            boids[i].currentTotalForce += obstacleWeight * ((ruleObstacle * boidForceScale) - vi);

        }
        if (boidZeroNavigatingTowardGoal && boidZeroPath != null &&
            boidZeroPath.status == NavMeshPathStatus.PathComplete &&
            boidZeroPath.corners != null && boidZeroPath.corners.Length > 1)
        {
            NavMeshHit hit;
            Vector3 p0 = boids[0].position;
            if (NavMesh.SamplePosition(p0, out hit, 1.0f, NavMesh.AllAreas))
            {
                Vector3 corner = boidZeroPath.corners[currentCorner];
                if ((hit.position - corner).sqrMagnitude < 1.0f)
                {
                    currentCorner++;
                    if (currentCorner >= boidZeroPath.corners.Length)
                    {
                        boidZeroPath.ClearCorners();
                        boidZeroNavigatingTowardGoal = false;
                        currentCorner = 0;
                    }
                }
            }
            if (boidZeroNavigatingTowardGoal)
            {
                Vector3 target = boidZeroPath.corners[currentCorner];
                Vector3 toCorner = (target - boids[0].position);
                Vector3 ruleGoal = toCorner.sqrMagnitude > 1e-6f ? toCorner.normalized : Vector3.zero;
                boids[0].currentTotalForce += goalWeight * ((ruleGoal * boidForceScale) - boids[0].velocity);

            }
        }
        for (int i = 0; i < n; i++)
        {
            Vector3 a = boids[i].currentTotalForce;
            boids[i].velocity += a * dt;
            float speed = boids[i].velocity.magnitude;
            if (speed > maxSpeed)
            {
                boids[i].velocity = boids[i].velocity * (maxSpeed / speed);
            }
            boids[i].position += boids[i].velocity * dt;
            if (boidObjects[i] != null)
            {
                boidObjects[i].position = boids[i].position;
                Vector3 look = boids[i].velocity.sqrMagnitude > 1e-6f ? boids[i].velocity : boids[i].forward;
                Quaternion targetRot = Quaternion.LookRotation(look.normalized, Vector3.up);
                boidObjects[i].rotation = Quaternion.RotateTowards(boidObjects[i].rotation, targetRot, rotationSpeed * dt);
            }
            if (boids[i].velocity.sqrMagnitude > 1e-6f)
            {
                boids[i].forward = boids[i].velocity.normalized;
            }
        }
    }


    private void Update()
    {
        /* Render information for boidzero, useful for debugging forces and path planning*/
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        
    }


    public void SetGoal(Vector3 goal)
    {
        if (boidZeroNavigatingTowardGoal) return;
        boidZeroGoal = goal;
        NavMeshHit startHit, endHit;
        bool haveStart = NavMesh.SamplePosition(boids[0].position, out startHit, 5.0f, NavMesh.AllAreas);
        bool haveEnd   = NavMesh.SamplePosition(boidZeroGoal, out endHit,  2.0f, NavMesh.AllAreas);
        if (!haveStart || !haveEnd)
        {
            return;
        }
        boidZeroPath.ClearCorners();
        if (NavMesh.CalculatePath(startHit.position, endHit.position, NavMesh.AllAreas, boidZeroPath) &&
            boidZeroPath.status == NavMeshPathStatus.PathComplete &&
            boidZeroPath.corners != null && boidZeroPath.corners.Length > 1)
        {
            currentCorner = 0;
            boidZeroNavigatingTowardGoal = true;
        }

    }
}

