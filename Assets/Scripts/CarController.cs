using System;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography.X509Certificates;
using UnityEditor.UI;
using UnityEngine;
using UnityEngine.Assertions;


public enum DriveType
{
    Manual, Automatic
}
public enum State
{
    Stop, Exploring, GoToBomb
}
public class CarController : MonoBehaviour
{
    public DriveType driveType = DriveType.Manual;
    public State state = State.Stop;

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
    public float goToNodeAccuracy = 0.2f;
    


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

    [Header("Other")]
    public GameObject bolaBiru;

    
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
    void FixedUpdate()
    {
        switch (driveType)
        {
            case DriveType.Manual:
                // receive input
                inputForward = Input.GetAxis("Vertical");
                inputSteerAngle = Input.GetAxis("Horizontal");
                inputBrakeAcceleration = Input.GetKey(KeyCode.Space) ? 1 : 0;
                break;

            case DriveType.Automatic:
                //TODO
                if (state == State.Stop){
                    Debug.Log("starting hybrid A*");
                    state = State.Exploring;
                    List<Node> nodes = MapBuilder.instance.getPathToBestTarget();
                    Debug.Log("Succesfully created path with length: " + nodes.Count.ToString());
                    nodesEnumerator = nodes.GetEnumerator();

                    // remove existing nodesDebug
                    foreach (GameObject u in nodesDebug){
                        Destroy(u);
                    }
                    nodesDebug.Clear();

                    // add new nodesDebug
                    foreach (Node node in nodes){
                        Debug.Log("bola biruuu");
                        GameObject bolaBiruu = Instantiate(bolaBiru,
                            new Vector3(node.worldPosition.x, transform.position.y, node.worldPosition.y),
                            Quaternion.identity);
                        nodesDebug.Add(bolaBiru);
                    }

                } else if (state == State.Exploring){

                    // if near nextNode
                    if (Vector2.Distance(
                            nodesEnumerator.Current.worldPosition,
                            new Vector2(transform.position.x, transform.position.z)
                        ) 
                        < goToNodeAccuracy
                    ){
                        bool hasNext = nodesEnumerator.MoveNext();
                        if (!hasNext){
                            state = State.Stop;
                        }
                    }

                    // float speedScale = 0.5f;
                    // inputForward = nodesEnumerator.Current.isReversing ? speedScale : -speedScale;
                    // inputSteerAngle = ;
                    // inputBrakeAcceleration = Input.GetKey(KeyCode.Space) ? 1 : 0;
                    
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
