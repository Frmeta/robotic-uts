using System;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography.X509Certificates;
using UnityEditor.Experimental;
using UnityEditor.UI;
using UnityEngine;
using UnityEngine.Assertions;


public enum DriveType
{
    Manual, Automatic
}
public enum State
{
    NeedPath, WaitingForPath, Exploring, GoToBomb, PermanentStop
}
public class CarController : MonoBehaviour
{
    public DriveType driveType = DriveType.Manual;
    public State state = State.NeedPath;

    [Header("Wheel Reference")]
    public WheelCollider leftFrontWheel;
    public WheelCollider rightFrontWheel;
    public WheelCollider leftBackWheel;
    public WheelCollider rightBackWheel;
    public GameObject leftFrontWheelMesh;
    public GameObject rightFrontWheelMesh;
    public GameObject leftBackWheelMesh;
    public GameObject rightBackWheelMesh;

    
    [Header("Car Settings")]
    public float acceleration = 10f;
    public float maxSteerAngle = 30f;
    public float brakeAcceleration = 10f;
    


    [Header("Triangulation")]
    public Transform[] landmarks;
    public Transform triangulationLocationVisualizer;

    
    [Header("Odometry")]
    public Transform intertiaLocationVisualizer;
    public float wheelBase = 1f;
    private Rigidbody rb;
    private float x_inertia;
    private float z_inertia;
    private float theta_inertia;


    [Header("Average")]
    public Transform averageLocationVisualizer;
    private MapBuilder mapBuilder;
    private float inputForward;
    private float inputSteerAngle;
    private float inputBrakeAcceleration;

    [Header("Automatic Settings")]
    public float maxSpeed = 3f;

    [Header("Other")]
    public GameObject bolaBiru;
    public GameObject bolaBiruBesar;
    public bool isBestTargetManual;
    public float goToNodeAccuracy = 0.2f;

    
    // temporary variables
    private float currentAcceleration;
    private float currentSteerAngle;
    private float currentBrakeAcceleration;
    private List<Node>.Enumerator nodesEnumerator;
    private List<GameObject> nodesDebug = new List<GameObject>();

    public static CarController instance;
    // hint: motorTorque, steerAngle, brakeTorque
    // Start is called before the first frame update
    void Start()
    {
        if (instance == null){
            instance = this;
        }
        rb = GetComponent<Rigidbody>();
        mapBuilder = GetComponent<MapBuilder>();

        x_inertia = transform.position.x;
        z_inertia = transform.position.z;
        theta_inertia = transform.rotation.eulerAngles.y * Mathf.Deg2Rad;
        Assert.IsTrue(landmarks.Length == 3, "There should be exactly 3 landmarks for triangulation.");
        Assert.IsTrue(landmarks[0] != null && landmarks[1] != null && landmarks[2] != null, "Landmarks should not be null.");
        
    }

    // Update is called once per frame
    void Update()
    {
        
        switch (driveType)
        {
            case DriveType.Manual:
                // receive input
                inputForward = Input.GetAxis("Vertical");
                inputSteerAngle = Input.GetAxis("Horizontal");
                inputBrakeAcceleration = Input.GetKey(KeyCode.Space) ? 1 : 0;

                if (Input.GetKeyDown(KeyCode.Q)){
                    GoHybridAStar();
                }
                break;

            case DriveType.Automatic:
                if (state == State.NeedPath){
                    inputBrakeAcceleration = 0.2f;
                    GoHybridAStar();
                    state = State.WaitingForPath;
                } else if (state == State.Exploring){
                    
                    Node node = nodesEnumerator.Current;
                    if (node == null){
                        Debug.Log("node null??");
                        state = State.NeedPath;
                        break;
                    }
                    Vector2 myPos = new Vector2(transform.position.x, transform.position.z);
                    Vector2 nodePos = new Vector2(node.worldPosition.x, node.worldPosition.z);

                    // following node
                    bolaBiruBesar.transform.position = new Vector3(node.worldPosition.x, transform.position.y, node.worldPosition.z);

                    // gas
                    float avgSpeed = (leftFrontWheel.rotationSpeed + rightFrontWheel.rotationSpeed)/2;
                    inputForward = (avgSpeed > maxSpeed || avgSpeed < -maxSpeed) ? 0 : (node.isReversing ? -1 : 1);

                    // steer
                    
                    Vector2 diff = (nodePos - myPos).normalized;

                    float myAngle = (90f - transform.eulerAngles.y) * Mathf.Deg2Rad;
                    Vector2 myDir = new Vector2(Mathf.Cos(myAngle), Mathf.Sin(myAngle));

                    float crossProduct = myDir.x * diff.y - myDir.y * diff.x;
                    float noSteer = 0.01f;
                    float maxSteer = 0.1f;
                    if (Mathf.Abs(crossProduct) < noSteer){
                        // no difference
                        inputSteerAngle = 0;
                    } else {
                        // belok
                        inputSteerAngle = -Mathf.Sign(crossProduct) * Mathf.Clamp((Mathf.Abs(crossProduct)-noSteer)/(maxSteer - noSteer), 0, 1);
                    }

                    // brake
                    inputBrakeAcceleration = 0;


                    // if near nextNode
                    if (nodesEnumerator.Current == null ||
                    Vector2.Distance(nodePos, myPos) < goToNodeAccuracy ||
                    (node.isReversing && Vector2.Distance(nodePos, myPos) < goToNodeAccuracy*4)){
                        bool hasNext = nodesEnumerator.MoveNext();
                        if (!hasNext){
                            Debug.Log("sampe");
                            inputBrakeAcceleration = 1;
                            state = State.NeedPath;
                        }
                        // if node is not walkable (maybe htere's wall revealed nearby)
                        Vector2Int nodeCell = MapBuilder.instance.WorldToMap(nodesEnumerator.Current.worldPosition);
                        if (MapBuilder.instance.walkableMap[nodeCell.x, nodeCell.y] == MapBuilder.TileTypeWalkable.NotWalkable){
                            Debug.Log("path canceled");
                            inputBrakeAcceleration = 1;
                            state = State.NeedPath;
                        }
                    }

                    
                    
                }
                



                break;
        }

        currentAcceleration = acceleration * inputForward;
        currentSteerAngle = maxSteerAngle * inputSteerAngle;
        currentBrakeAcceleration = brakeAcceleration * inputBrakeAcceleration;
        

        // set motor torque
        leftFrontWheel.motorTorque = currentAcceleration;
        rightFrontWheel.motorTorque = currentAcceleration;
        leftBackWheel.motorTorque = currentAcceleration;
        rightBackWheel.motorTorque = currentAcceleration;

        // set steer angle
        leftFrontWheel.steerAngle = currentSteerAngle;
        rightFrontWheel.steerAngle = currentSteerAngle;
        // leftBackWheel.steerAngle = currentSteerAngle;
        // rightBackWheel.steerAngle = currentSteerAngle;

        // set brake torque
        leftFrontWheel.brakeTorque = currentBrakeAcceleration;
        rightFrontWheel.brakeTorque = currentBrakeAcceleration;
        leftBackWheel.brakeTorque = currentBrakeAcceleration;
        rightBackWheel.brakeTorque = currentBrakeAcceleration;

        // Update mesh
        UpdateWheelMesh(leftFrontWheel, leftFrontWheelMesh);
        UpdateWheelMesh(rightFrontWheel, rightFrontWheelMesh);
        UpdateWheelMesh(leftBackWheel, leftBackWheelMesh);
        UpdateWheelMesh(rightBackWheel, rightBackWheelMesh);

        // Estimate position using triangulation
        Vector3? triangulationEstimation = Triangulate3D(landmarks[0].position
            , landmarks[1].position
            , landmarks[2].position
            , Vector3.Distance(landmarks[0].position, transform.position)
            , Vector3.Distance(landmarks[1].position, transform.position)
            , Vector3.Distance(landmarks[2].position, transform.position));
        if (triangulationEstimation != null){
            Vector3 pos = (Vector3)triangulationEstimation;
            triangulationLocationVisualizer.position = new Vector3(pos.x, triangulationLocationVisualizer.position.y, pos.z);
            triangulationLocationVisualizer.gameObject.SetActive(true);
        } else {
            triangulationLocationVisualizer.gameObject.SetActive(false);
        }

        // Estimate position using inertia
        float dt = Time.fixedDeltaTime;

        float velocity = Vector3.Dot(rb.velocity, transform.forward);
        //float velocity = rb.velocity.magnitude;
        


        float avgSteeringAngle = (leftFrontWheel.steerAngle + rightFrontWheel.steerAngle)/2f;
        float phi = avgSteeringAngle * Mathf.Deg2Rad;

        float dx = velocity * Mathf.Sin(theta_inertia) * dt;
        float dz = velocity * Mathf.Cos(theta_inertia) * dt;
        float dtheta = (velocity / wheelBase) * Mathf.Tan(phi) * dt;

        x_inertia += dx;
        z_inertia += dz;
        theta_inertia += dtheta;

        intertiaLocationVisualizer.position = new Vector3(x_inertia, intertiaLocationVisualizer.position.y, z_inertia);
        intertiaLocationVisualizer.rotation = Quaternion.Euler(0, theta_inertia * Mathf.Rad2Deg, 0);

        // Average the two estimations
        Vector3 averageEstimation = (triangulationLocationVisualizer.position + intertiaLocationVisualizer.position) / 2f;
        averageLocationVisualizer.position = new Vector3(averageEstimation.x, averageLocationVisualizer.position.y, averageEstimation.z);
    }

    private void GoHybridAStar(){
        Debug.Log("starting hybrid A*");
        StartCoroutine(mapBuilder.getPathToBestTarget((nodes) => AfterHybridAStar(nodes)));
        
    }
    private void AfterHybridAStar(List<Node> nodes){
        if (nodes != null && nodes.Count>0){
            state = State.Exploring;

            nodesEnumerator = nodes.GetEnumerator();
            nodesEnumerator.MoveNext();

            // remove existing nodesDebug
            foreach (GameObject u in nodesDebug){
                Destroy(u);
            }
            nodesDebug.Clear();

            // add new nodesDebug
            foreach (Node node in nodes){
                GameObject bolaBiruu = Instantiate(bolaBiru,
                    new Vector3(node.worldPosition.x, transform.position.y, node.worldPosition.z),
                    Quaternion.identity);
                nodesDebug.Add(bolaBiruu);
            }
        } else {
            Debug.LogWarning("Path failed");
            state = State.PermanentStop;
            StartCoroutine(Recovering());
        }
    }

    private IEnumerator Recovering(){
        yield return new WaitForSeconds(1);
        state = State.NeedPath;
    }
    
    private Vector3? Triangulate3D(Vector3 pos1, Vector3 pos2, Vector3 pos3, float dist1, float dist2, float dist3){
        Vector3 ex = (pos2 - pos1).normalized;
        float i = Vector3.Dot(ex, pos3 - pos1);
        Vector3 ey = (pos3 - pos1 - i * ex).normalized;
        Vector3 ez = Vector3.Cross(ex, ey).normalized;
        float d = Vector3.Distance(pos2, pos1);
        float j = Vector3.Dot(ey, pos3 - pos1);
        float x = (dist1 * dist1 - dist2 * dist2 + d * d)/(2*d);
        float y = (dist1 * dist1 - dist3 * dist3 + i * i + j * j)/(2 * j) - (i/j) * x;
        float temp = dist1 * dist1 - x * x - y * y;

        if (temp < 0){
            return null;
        }
        float z = Mathf.Sqrt(temp);

        Vector3 calculatedPos = pos1 + x*ex + y*ey + z*ez;
        calculatedPos.y = 0;

        return calculatedPos;
    }

    private void UpdateWheelMesh(WheelCollider collider, GameObject wheelMesh)
    {
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        wheelMesh.transform.position = position;
        wheelMesh.transform.rotation = rotation;
    }
}
