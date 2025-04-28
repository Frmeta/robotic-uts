using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class DroneMovement : MonoBehaviour
{
    public float moveSpeed = 10f;  // Speed of the drone
    public float turnSpeed = 50f;  // Rotation speed
    public float thrustPower = 15f; // Upward thrust power
    public float targetAltitude = 10f; // The target altitude to maintain
    public float scanRadius = 10f; // LIDAR scan range
    public float scanInterval = 0.05f; // How often to scan
    public float maxAltitudeDiff = 4f;

    // Public grid size for length and width
    public float gridLength = 200f;  // Length of the environment along the X-axis
    public float gridWidth = 200f;   // Width of the environment along the Z-axis
    public int gridRows = 20;        // Number of rows to divide the area into
    public int gridColumns = 20;     // Number of columns to divide the area into

    // Public values for the origin (X and Z values for the starting point)
    public float originX = 0f;  // X-coordinate of the origin
    public float originZ = 0f;  // Z-coordinate of the origin

    private Rigidbody rb;
    private float lastScanTime = 0f;

    private List<Vector3> obstacles = new List<Vector3>();  // List to store obstacle positions
    private List<Vector3> bombPositions = new List<Vector3>(); // List to store bomb positions

    public GameObject obstaclePrefab; // Prefab for obstacle (Red Sphere)
    public GameObject bombPrefab; // Prefab for bomb (Blue Sphere)

    // PID variables for altitude control
    private float pidP = 1.5f; // Proportional gain
    private float pidI = 0.5f; // Integral gain
    private float pidD = 0.1f; // Derivative gain

    private float altitudeError = 0f; // Error between target altitude and current altitude
    private float previousError = 0f; // Previous error for derivative term
    private float integral = 0f; // Integral of error

    // Grid movement variables
    private Vector3 targetPosition;   // The target position to move towards
    private int currentRow = 0;       // Current row in the grid
    private int currentColumn = 0;    // Current column in the grid
    private bool movingForward = true; // Whether the drone is moving forward or backward through the grid

    // Flag to track if scanning is complete
    private bool isScanningComplete = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true; // Ensure gravity is applied to the drone


        StartCoroutine(DelayedScanArea());
    }

    void Update()
    {
        if (isScanningComplete)
        {
            // Once scanning is complete, stop movement and display the environment visually
            StopDrone();
            return; // Exit update to avoid unnecessary calculations
        }

        MoveDrone();
        ApplyAltitudeControl(); // Adjust thrust to maintain altitude

        if (Time.time - lastScanTime > scanInterval)
        {

            lastScanTime = Time.time;
        }
    }

    // Move the drone toward the target position
    void MoveDrone()
    {
        // Calculate the direction to the target
        Vector3 direction = (targetPosition - transform.position).normalized;

        // Move towards the target
        rb.MovePosition(transform.position + direction * moveSpeed * Time.deltaTime);

        // Rotate the drone to face the target
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, turnSpeed * Time.deltaTime);

        // Check if the drone has reached the target
        if (Vector3.Distance(transform.position, targetPosition) < 1f)
        {
            SetTargetPosition();
        }
    }

    // Apply PID control to maintain a constant altitude
    void ApplyAltitudeControl()
    {
        // Get the current altitude error
        altitudeError = targetAltitude - transform.position.y;

        // Calculate the integral and derivative terms
        integral += altitudeError * Time.deltaTime;
        float derivative = (altitudeError - previousError) / Time.deltaTime;

        // Compute the PID output
        float pidOutput = (pidP * altitudeError) + (pidI * integral) + (pidD * derivative);

        // Apply the thrust based on PID output
        Vector3 upwardThrust = Vector3.up * Mathf.Clamp(pidOutput, -thrustPower, thrustPower);
        rb.AddForce(upwardThrust, ForceMode.Acceleration);

        // Store the current error for the next frame
        previousError = altitudeError;
    }

    // Set a new target position in a grid pattern
    void SetTargetPosition()
    {
        // Calculate the position in the grid (based on current row and column)
        float targetX = originX + (currentColumn - gridColumns / 2) * (gridLength / gridColumns);
        float targetZ = originZ + (currentRow - gridRows / 2) * (gridWidth / gridRows);

        // Set the target position at the calculated position
        targetPosition = new Vector3(targetX, targetAltitude, targetZ);

        // Update the row and column
        UpdateGridPosition();
    }

    // Update the grid position to move to the next column or row
    void UpdateGridPosition()
    {
        // Move the drone in a zigzag pattern across the grid
        if (movingForward)
        {
            if (currentColumn < gridColumns - 1)
            {
                currentColumn++;
            }
            else
            {
                currentRow++;
                movingForward = !movingForward; // Switch direction
            }
        }
        else
        {
            if (currentColumn > 0)
            {
                currentColumn--;
            }
            else
            {
                currentRow++;
                movingForward = !movingForward; // Switch direction
            }
        }

        // If we finish scanning the whole grid, stop
        if (currentRow >= gridRows)
        {
            currentRow = 0;  // You could also add logic to stop after completing the scanning.
            isScanningComplete = true; // Mark the scanning as complete
        }
    }

    // Simulate environment scanning (LIDAR) using BoxCast for a wide area
    void ScanArea()
    {
        RaycastHit bombHit;
        RaycastHit hit_forward;
        RaycastHit hit_back;
        RaycastHit hit_left;
        RaycastHit hit_right;
        for (int row = 0; row < gridRows; row++)
        {
            for (int col = 0; col < gridColumns; col++)
            {
                // Calculate the position for this grid cell
                float x = (col - gridColumns / 2) * (gridLength / gridColumns);
                float z = (row - gridRows / 2) * (gridWidth / gridRows);

                Vector3 obstacleScanPosition = new Vector3(x, 2.3f, z);
                Vector3 bombScanPosition = new Vector3(x, targetAltitude, z);

                Vector3 instantiatePosition = new Vector3(x, targetAltitude, z);

                // Calculate the size of the BoxCast (based on grid size)
                Vector3 boxSize = new Vector3(gridLength / gridColumns, 0.001f, gridWidth / gridRows);

                // Perform the RayCast: cast a ray to detect obstacles in the grid cell
                if (Physics.Raycast(obstacleScanPosition, Vector3.forward, out hit_forward, Mathf.Infinity)
                    && Physics.Raycast(obstacleScanPosition, Vector3.back, out hit_back, Mathf.Infinity)
                    && Physics.Raycast(obstacleScanPosition, Vector3.left, out hit_left, Mathf.Infinity)
                    && Physics.Raycast(obstacleScanPosition, Vector3.right, out hit_right, Mathf.Infinity))
                {
                    if (hit_forward.distance <= 0.5
                        || hit_back.distance <= 0.5
                        || hit_left.distance <= 0.5
                        || hit_right.distance <= 0.5)
                    {

                        // Obstacle detected, instantiate a red sphere for obstacle when passing the spot
                        obstacles.Add(obstacleScanPosition);
                        Instantiate(obstaclePrefab, instantiatePosition, Quaternion.identity); // Blue sphere for obstacle
                    }
                    else
                    {
                        // Not an obstacle (clear space), instantiate the prefab for clear space only when passed
                    }
                }
                if (Physics.BoxCast(bombScanPosition, boxSize / 2, Vector3.down, out bombHit, Quaternion.identity, Mathf.Infinity))
                {
                    if (bombHit.collider.CompareTag("Bombs") && bombHit.collider.gameObject.activeSelf) // Check if the bomb is active
                    {
                        bombPositions.Add(bombHit.point);
                        Instantiate(bombPrefab, instantiatePosition, Quaternion.identity); // Green sphere for bomb
                    }
                }
            }
        }
    }


    // Stop the drone once scanning is complete
    void StopDrone()
    {
        rb.velocity = Vector3.zero;  // Stop the movement
        rb.angularVelocity = Vector3.zero;  // Stop rotation
    }

    // Send data to the DDMR (Differential Drive Mobile Robot)
    void SendDataToDDMR()
    {
        DDMR ddmr = FindObjectOfType<DDMR>(); // Assuming the DDMR script is attached to an object in the scene

        // Debug the data being sent
        UnityEngine.Debug.Log("Sending data to DDMR");
        UnityEngine.Debug.Log("Number of obstacles: " + obstacles.Count);
        UnityEngine.Debug.Log("Number of bombs: " + bombPositions.Count);

        ddmr.ReceiveMapData(obstacles, bombPositions, gridLength, gridWidth, gridRows, gridColumns, originX, originZ); // Send the grid dimensions along with obstacles and bombs
    }



    IEnumerator DelayedScanArea()
    {
        // Log message to confirm delay is happening
        UnityEngine.Debug.Log("Waiting for 5 seconds before starting the scan...");

        // Wait for 5 seconds
        yield return new WaitForSeconds(5f);

        // Now call ScanArea after the delay
        UnityEngine.Debug.Log("Starting ScanArea after 5-second delay.");
        ScanArea();

        // After scanning, send data to DDMR
        SendDataToDDMR();
    }



}