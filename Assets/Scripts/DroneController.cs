using UnityEngine;
using UnityEngine.InputSystem;

// This script requires a Rigidbody component on the same GameObject.
[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    // Enum to define the flight modes
    public enum FlightMode { Angle, Horizon, Acro }

    [Header("Current Status")]
    [Tooltip("The current active flight mode.")]
    public FlightMode currentMode = FlightMode.Acro;

    [Header("Base Flight Controls")]
    [Tooltip("The force applied upwards for throttle.")]
    public float thrustForce = 20f; // User's value
    [Tooltip("Torque for rotating left and right (yaw). This is your 'Yaw Rate'.")]
    public float yawTorque = 10f; // User's value
    [Tooltip("NEW: Actively stops yaw rotation when stick is centered. Fixes yaw drift in Angle mode.")]
    public float yawDamping = 5f; // NEW
    [Tooltip("Base torque for pitch in Acro/Horizon modes. This is your 'Pitch Rate'.")]
    public float pitchTorque = 15f; // User's value
    [Tooltip("Base torque for roll in Acro/Horizon modes. This is your 'Roll Rate'.")]
    public float rollTorque = 15f; // User's value

    [Header("Stabilization (Angle/Horizon)")]
    [Tooltip("The maximum angle (in degrees) the drone can tilt in Angle Mode.")]
    public float maxTiltAngle = 30f; // User's value
    [Tooltip("How strongly the drone tries to self-level. (Kp) Your value of 7 was a bit low.")]
    public float stabilizationStrength = 12f; // Reset to a more stable default
    [Tooltip("How much the drone dampens rotation to prevent wobbling. (Kd) Your value of 11 was too high.")]
    public float stabilizationDamping = 3f; // Reset to a more stable default

    [Header("Input Processing")]
    [Tooltip("The size of the 'dead' area in the center of the sticks to prevent drift.")]
    public float inputDeadzone = 0.07f; // User's value
    [Tooltip("The amount of exponential curve for Pitch/Roll/Yaw.")]
    [Range(0, 1)]
    public float acroExpo = 0.5f; // User's value
    [Tooltip("NEW: Flattens the throttle curve around the center to make hovering easier.")]
    [Range(0, 1)]
    public float throttleExpo = 0.4f; // NEW

    // --- PUBLIC PROPERTIES ---
    public FlightMode CurrentMode => currentMode;
    public float ThrottleInput => throttleInput;
    public Rigidbody Rb => rb;
    // --- END OF PUBLIC PROPERTIES ---

    // Private variables
    private Rigidbody rb;
    private float throttleInput, yawInput, pitchInput, rollInput;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        // We set the Rigidbody's angular drag here. 
        // 0.5 is a good value, but we add our own yawDamping on top.
        rb.angularDamping = 0.5f;
    }

    void FixedUpdate()
    {
        HandleMovement();
    }

    private void HandleMovement()
    {
        // --- Throttle (Processed) ---
        // We now apply throttle expo to make hovering easier
        float processedThrottle = ApplyExpo(throttleInput, throttleExpo);
        rb.AddForce(transform.up * (processedThrottle * thrustForce), ForceMode.Acceleration);

        // --- Yaw (Processed + Damped) ---
        // This is the FIX for your yaw spin in Angle Mode.
        float processedYaw = ApplyExpo(yawInput, acroExpo) * yawTorque;
        
        // Get the current yaw rotation speed
        float currentYawRate = transform.InverseTransformDirection(rb.angularVelocity).y;
        // Apply a damping force to stop rotation when the stick is centered
        float yawDampingTorque = -currentYawRate * yawDamping;

        // Add the user's input torque and the damping torque together
        rb.AddRelativeTorque(Vector3.up * (processedYaw + yawDampingTorque), ForceMode.Acceleration);


        // --- Pitch and Roll (Mode-dependent) ---
        switch (currentMode)
        {
            case FlightMode.Acro:
                HandleAcroMode();
                break;
            case FlightMode.Angle:
                HandleAngleMode();
                break;
            case FlightMode.Horizon:
                HandleHorizonMode();
                break;
        }
    }

    /// <summary>
    /// ACRO MODE: Processed stick input controls the rate of rotation.
    /// </summary>
    private void HandleAcroMode()
    {
        // Apply Expo to our deadzoned input
        float processedPitch = ApplyExpo(pitchInput, acroExpo);
        float processedRoll = ApplyExpo(rollInput, acroExpo);

        // Apply torque from *processed* input
        Vector3 acroTorque = new Vector3(processedPitch * pitchTorque, 0, -processedRoll * rollTorque);
        rb.AddRelativeTorque(acroTorque, ForceMode.Acceleration);
    }

    /// <summary>
    /// ANGLE MODE: Raw (but deadzoned) stick input controls the target angle.
    /// </summary>
    private void HandleAngleMode()
    {
        // We use the *raw* (but deadzoned) input here because we want a linear
        // mapping from stick position to target angle.
        float targetPitch = pitchInput * maxTiltAngle;
        float targetRoll = -rollInput * maxTiltAngle;

        Vector3 localEuler = transform.localEulerAngles;
        float currentPitch = ConvertAngle(localEuler.x);
        float currentRoll = ConvertAngle(localEuler.z);

        float pitchError = targetPitch - currentPitch;
        float rollError = targetRoll - currentRoll;

        Vector3 localAngularVel = transform.InverseTransformDirection(rb.angularVelocity);

        float pitchPD = (pitchError * stabilizationStrength) - (localAngularVel.x * stabilizationDamping);
        float rollPD = (rollError * stabilizationStrength) - (localAngularVel.z * stabilizationDamping);

        rb.AddRelativeTorque(new Vector3(pitchPD, 0, rollPD), ForceMode.Acceleration);
    }

    /// <summary>
    /// HORIZON MODE: Mix of Acro and Angle.
    /// </summary>
    private void HandleHorizonMode()
    {
        // 1. Apply Acro torque (with Expo)
        float processedPitch = ApplyExpo(pitchInput, acroExpo);
        float processedRoll = ApplyExpo(rollInput, acroExpo);
        Vector3 acroTorque = new Vector3(processedPitch * pitchTorque, 0, -processedRoll * rollTorque);
        rb.AddRelativeTorque(acroTorque, ForceMode.Acceleration);

        // 2. Apply P-only stabilization (the "spring" back to center)
        Vector3 localEuler = transform.localEulerAngles;
        float currentPitch = ConvertAngle(localEuler.x);
        float currentRoll = ConvertAngle(localEuler.z);
        Vector3 stabilizingTorque = new Vector3(-currentPitch, 0, -currentRoll) * (stabilizationStrength * 0.5f);
        rb.AddRelativeTorque(stabilizingTorque, ForceMode.Acceleration);
    }

    /// <summary>
    /// Helper to convert Euler angles from 0-360 to -180-180 range.
    /// </summary>
    private float ConvertAngle(float eulerAngle)
    {
        return (eulerAngle > 180) ? eulerAngle - 360 : eulerAngle;
    }

    /// <summary>
    /// Applies a deadzone to the raw input.
    /// </summary>
    private float ApplyDeadzone(float input)
    {
        if (Mathf.Abs(input) < inputDeadzone)
        {
            return 0f;
        }
        // Rescale input to still reach 1.0 at the edge of the deadzone
        return Mathf.Sign(input) * ((Mathf.Abs(input) - inputDeadzone) / (1f - inputDeadzone));
    }

    /// <summary>
    // Applies an exponential curve to the input.
    /// </summary>
    private float ApplyExpo(float input, float expo)
    {
        // Simple cubic expo formula: (expo * input^3) + ((1-expo) * input)
        return (expo * (input * input * input)) + ((1f - expo) * input);
    }

    #region Input Handlers (Called by PlayerInput component)

    public void OnThrottle(InputValue value)
    {
        float rawInput = value.Get<float>();
        // We still remap from [-1, 1] to [0, 1]
        // But the *application* of this value is now handled by HandleMovement
        throttleInput = (rawInput + 1f) / 2f; 
    }

    public void OnYaw(InputValue value)
    {
        // Apply deadzone as soon as we get the input
        yawInput = ApplyDeadzone(value.Get<float>());
    }

    public void OnPitch(InputValue value)
    {
        // Apply deadzone as soon as we get the input
        pitchInput = ApplyDeadzone(value.Get<float>());
    }

    public void OnRoll(InputValue value)
    {
        // Apply deadzone as soon as we get the input
        rollInput = ApplyDeadzone(value.Get<float>());
    }
    
    public void OnFlightModes(InputValue value)
    {
        float rzValue = value.Get<float>();
        if (rzValue < -0.5f)
        {
            currentMode = FlightMode.Acro;
        }
        else if (rzValue > 0.5f)
        {
            currentMode = FlightMode.Angle;
        }
        else
        {
            currentMode = FlightMode.Horizon;
        }
    }

    #endregion
}

