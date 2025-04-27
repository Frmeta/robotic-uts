using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneMovement : MonoBehaviour
{
    public float moveSpeed = 5f;         // Kecepatan gerak horizontal
    public float liftSpeed = 5f;          // Kecepatan naik turun
    public float hoverForce = 9.81f;      // Gaya hover agar melawan gravitasi
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true; // Tetap gunakan gravitasi
    }

    void FixedUpdate()
    {
        // Ambil input
        float moveHorizontal = Input.GetAxis("Horizontal");
        float moveVertical = Input.GetAxis("Vertical");

        // Gerakan horizontal
        Vector3 horizontalMovement = (transform.forward * moveVertical + transform.right * moveHorizontal) * moveSpeed;

        // Gerakan vertikal
        float lift = 0f;
        if (Input.GetKey(KeyCode.Space))
        {
            lift = 1f;
        }
        else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.C))
        {
            lift = -1f;
        }

        // Apply force
        Vector3 force = horizontalMovement + Vector3.up * (hoverForce + lift * liftSpeed);

        rb.AddForce(force, ForceMode.Force);
    }
}
