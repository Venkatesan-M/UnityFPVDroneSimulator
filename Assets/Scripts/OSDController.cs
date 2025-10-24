using UnityEngine;
using TMPro; // Import TextMeshPro!

/// <summary>
/// Manages the On-Screen Display (OSD) elements.
/// This script should be placed on the Canvas GameObject.
/// </summary>
public class OSDController : MonoBehaviour
{
    [Header("Object References")]
    [Tooltip("Drag your Drone object here.")]
    public DroneController droneController;

    [Header("UI Text Elements")]
    [Tooltip("Drag your Flight Mode Text (TMP) element here.")]
    public TextMeshProUGUI flightModeText;
    
    [Tooltip("Drag your Throttle Text (TMP) element here.")]
    public TextMeshProUGUI throttleText;
    
    [Tooltip("Drag your Altitude Text (TMP) element here.")]
    public TextMeshProUGUI altitudeText;
    
    [Tooltip("Drag your Speed Text (TMP) element here.")]
    public TextMeshProUGUI speedText;

    private float startAltitude;

    // Start is called before the first frame update
    void Start()
    {
        // Check if the drone controller is assigned
        if (droneController == null)
        {
            Debug.LogError("Drone Controller not assigned to OSD!");
            enabled = false; // Disable this script if no drone is found
            return;
        }

        // Set the starting altitude so we can measure altitude from our takeoff point
        startAltitude = droneController.transform.position.y;
    }

    // Update is called once per frame
    void Update()
    {
        if (droneController == null) return;

        // --- Update Flight Mode ---
        // Access the public property from the drone controller
        flightModeText.text = droneController.CurrentMode.ToString().ToUpper();

        // --- Update Throttle ---
        // Access the public property, multiply by 100, and format as a whole number (F0)
        throttleText.text = $"THR: {droneController.ThrottleInput * 100:F0}%";

        // --- Update Altitude ---
        // Get the drone's current Y position and subtract the starting altitude
        float altitude = droneController.transform.position.y - startAltitude;
        altitudeText.text = $"ALT: {altitude:F1} m"; // Format to 1 decimal place (F1)

        // --- Update Speed ---
        // Get the velocity from the drone's public Rigidbody reference
        float speed = droneController.Rb.linearVelocity.magnitude;
        speedText.text = $"SPD: {speed:F1} m/s"; // Format to 1 decimal place (F1)
    }
}
