using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PhysicsHand : MonoBehaviour {
    [Header("Input")]
    [SerializeField] InputActionReference input;

    [Space]
    [Header("PID")]
    [SerializeField] Transform target;
    [SerializeField] Rigidbody playerRigidBody;
    [SerializeField] float freq = 50f;
    [SerializeField] float damping = 1f;
    [SerializeField] float rotFreq = 100f;
    [SerializeField] float rotDamping = 0.9f;

    [Space]
    [Header("Spring")]
    [SerializeField] float climbForce = 1000f;
    [SerializeField] float climbDragForce = 500f;


    Rigidbody rb;
    Vector3 prevPosition;
    bool isGrabbing;
    // Start is called before the first frame update
    void Start() {
        transform.position = target.position;
        transform.rotation = target.rotation;
        rb = GetComponent<Rigidbody>();
        rb.maxAngularVelocity = float.PositiveInfinity;
        prevPosition = transform.position;
    }

    // Update is called once per frame
    void FixedUpdate() {
        if (input.action.IsPressed()) {
            Debug.Log($"Gripping {input.name}");
            PIDMovement();
            PIDRotation();
            if (isGrabbing) HookesLaw();
        }
    }

    private void OnCollisionEnter(Collision collision) {
        isGrabbing = true;
    }

    private void OnCollisionExit(Collision collision) {
        isGrabbing = false;
    }

    void PIDMovement() {
        float kp = (6f * freq) * (6f * freq) * 0.25f;
        float kd = 4.5f * freq * damping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;
        Vector3 force = (target.position - transform.position) * ksg + (playerRigidBody.velocity - rb.velocity) * kdg;

        rb.AddForce(force, ForceMode.Acceleration);
    }

    void PIDRotation() {
        float kp = (6f * rotFreq) * (6f * rotFreq) * 0.25f;
        float kd = 4.5f * rotFreq * rotDamping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;
        Quaternion q = target.rotation * Quaternion.Inverse(transform.rotation);

        if (q.w < 0) {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        q.ToAngleAxis(out float angle, out Vector3 axis);
        axis.Normalize();
        axis *= Mathf.Deg2Rad;
        Vector3 torque = ksg * axis * angle + -rb.angularVelocity * kdg;

        rb.AddTorque(torque, ForceMode.Acceleration);
    }

    void HookesLaw() {
        Vector3 displacement = transform.position - target.position;
        Vector3 force = displacement * climbForce;
        float drag = GetDrag();

        playerRigidBody.AddForce(force, ForceMode.Acceleration);
        playerRigidBody.AddForce(drag * -playerRigidBody.velocity * climbDragForce, ForceMode.Acceleration);
    }

    float GetDrag() {
        Vector3 handVelocity = (target.localPosition - prevPosition) / Time.fixedDeltaTime;
        float drag = 1 / handVelocity.magnitude + float.Epsilon;
        Debug.Log("Before clamp: " + drag);
        drag = Mathf.Clamp(drag, 0.03f, 1f);
        Debug.Log("After clamp: " + drag);
        prevPosition = transform.position;
        return drag;
    }
}
