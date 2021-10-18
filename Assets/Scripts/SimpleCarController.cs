using UnityEngine;

enum SpeedType
{
    KPH,
    MPH
}

public class SimpleCarController : MonoBehaviour
{
    [SerializeField] private float maxSteerAngle;
    [SerializeField] private float motorForce;
    [SerializeField] private float brakeForce;
    [SerializeField] private float topSpeed;
    [SerializeField] private SpeedType speedType;
    [SerializeField] private float antiRoll = 1000f;
    [SerializeField] private bool tractionControl = true;
    [SerializeField] private float slipLimit = 0.3f;
    [SerializeField] private bool steeringAssist = true;
    [SerializeField] private float steeringAssistRatio = 0.5f;
    [SerializeField] private int numberOfGears;
    [SerializeField] private WheelCollider[] wheelColliders = new WheelCollider[4];
    [SerializeField] private Transform[] wheelMeshes = new Transform[4];
    [SerializeField] AudioSource audioSource;
    [SerializeField] private float minimumPitch;
    [SerializeField] private float maximumPitch;

    private Rigidbody rb;
    
    private float horizontalInput;
    private float verticalInput;
    private bool isReversing = false;
    private float rotationInPreviousFrame;
    private int currentGear = 0;
    private float currentSpeed;
    private float gearFactor;
    private float engineRpm;
 
    private void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        horizontalInput = Input.GetAxis("Horizontal");
        verticalInput = Input.GetAxis("Vertical");

        UpdateCurrentSpeed();
        HandleSteering();
        HandleDrive();
        HandleWheelTransform();
        
        AntiRoll();
        DetectReverse();
        TractionControl();
        SteeringAssist();
        HandleGearChange();
        CalculateEngineRevs();
        HandleAudio();
    }

    private void UpdateCurrentSpeed()
    {
        if (speedType == SpeedType.KPH)
        {
            currentSpeed = rb.velocity.magnitude * 3.6f;
        }
        else
        {
            currentSpeed = rb.velocity.magnitude * 2.23693629f;
        }
    }

    private void HandleSteering()
    {
        wheelColliders[0].steerAngle = maxSteerAngle * horizontalInput;
        wheelColliders[1].steerAngle = maxSteerAngle * horizontalInput;
    }

    private void HandleDrive()
    {
        wheelColliders[0].motorTorque = motorForce * verticalInput;
        wheelColliders[1].motorTorque = motorForce * verticalInput;
        
        if (!isReversing && verticalInput < 0 && rb.velocity.magnitude > 1)
        { // Braking
            for (int i = 0; i < wheelColliders.Length; i++)
            {
                wheelColliders[i].brakeTorque = -brakeForce * verticalInput;
            }
        }
        else
        {
            ResetBrakes();
        }
    }

    private void ResetBrakes()
    {
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            wheelColliders[i].brakeTorque = 0f;
        }
    }

    private void HandleWheelTransform()
    {
        for (int i = 0; i < wheelMeshes.Length; i++)
        {
            Vector3 pos = wheelMeshes[i].position;
            Quaternion quat = wheelMeshes[i].rotation;
            
            wheelColliders[i].GetWorldPose(out pos, out quat);

            wheelMeshes[i].position = pos;
            wheelMeshes[i].rotation = quat;
        }
    }

    private void AntiRoll()
    {
        // Front axle
        ApplyAntiRoll(wheelColliders[0], wheelColliders[1]);
        // Back axle
        ApplyAntiRoll(wheelColliders[2], wheelColliders[3]);
    }

    private void ApplyAntiRoll(WheelCollider left, WheelCollider right)
    {
        // Credits: http://projects.edy.es/trac/edy_vehicle-physics/wiki/TheStabilizerBars
        WheelHit hit;
        float travelLeft = 1f;
        float travelRight = 1f;

        bool isGroundedLeft = left.GetGroundHit(out hit);
        if (isGroundedLeft)
        {
            travelLeft = (-left.transform.InverseTransformPoint(hit.point).y - left.radius) / left.suspensionDistance;
        }
        bool isGroundedRight = right.GetGroundHit(out hit);
        if (isGroundedRight)
        {
            travelRight = (-right.transform.InverseTransformPoint(hit.point).y - right.radius) / right.suspensionDistance;
        }

        float antirollForce = (travelLeft - travelRight) * antiRoll;

        if (isGroundedLeft)
        {
            rb.AddForceAtPosition(left.transform.up * -antirollForce, left.transform.position);
        }
        if (isGroundedRight)
        {
            rb.AddForceAtPosition(right.transform.up * antirollForce, right.transform.position);
        }
    }

    private void DetectReverse()
    {
        float rpmSum = 0f;
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            rpmSum += wheelColliders[i].rpm;
        }
        isReversing = rpmSum / wheelColliders.Length < 0;
    }

    private void TractionControl()
    {
        if (tractionControl)
        {
            WheelHit hit;
            wheelColliders[0].GetGroundHit(out hit);
            if (hit.forwardSlip >= slipLimit && wheelColliders[0].motorTorque > 0)
            {
                wheelColliders[0].motorTorque *= 0.9f;
            }
            wheelColliders[1].GetGroundHit(out hit);
            if (hit.forwardSlip >= slipLimit && wheelColliders[1].motorTorque > 0)
            {
                wheelColliders[1].motorTorque *= 0.9f;
            }
        }
    }

    private void SteeringAssist()
    {
        if (Mathf.Abs(rotationInPreviousFrame - transform.eulerAngles.y) < 10f && steeringAssist)
        {
            var turnadjust = (transform.eulerAngles.y - rotationInPreviousFrame) * steeringAssistRatio;
            Quaternion velocityRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
            rb.velocity = velocityRotation * rb.velocity;
        }
        rotationInPreviousFrame = transform.eulerAngles.y;
    }

    private void HandleGearChange()
    {
        float speedRatio = Mathf.Abs(currentSpeed / topSpeed);
        float upshiftLimit = 1 / (float) numberOfGears * (currentGear + 1);
        float downshiftLimit = 1 / (float) numberOfGears * currentGear;
        
        if (currentGear > 0 && speedRatio < downshiftLimit)
        {
            currentGear--;
        }

        if (speedRatio > upshiftLimit && (currentGear < (numberOfGears - 1)))
        {
            currentGear++;
        }
    }
    
    private static float ULerp(float from, float to, float value)
    {
        return (1.0f - value) * from + value * to;
    }
    
    private static float CurveFactor(float factor)
    {
        return 1 - (1 - factor) * (1 - factor);
    }

    private void CalculateGearFactor()
    {
        float f = (1/(float) numberOfGears);
        var targetGearFactor = Mathf.InverseLerp(f*currentGear, f*(currentGear + 1), Mathf.Abs(currentSpeed/topSpeed));
        gearFactor = Mathf.Lerp(gearFactor, targetGearFactor, Time.deltaTime*5f);
    }
    
    private void CalculateEngineRevs()
    {
        CalculateGearFactor();
        var gearNumFactor = currentGear/(float) numberOfGears;
        var revsRangeMin = ULerp(0f, 1f, CurveFactor(gearNumFactor));
        var revsRangeMax = ULerp(1f, 1f, gearNumFactor);
        engineRpm = ULerp(revsRangeMin, revsRangeMax, gearFactor);
    }

    private void HandleAudio()
    {
        float pitch = ULerp(minimumPitch, maximumPitch, engineRpm);

        if (pitch < minimumPitch)
        {
            pitch = minimumPitch;
        }

        audioSource.pitch = pitch;
    }
}
