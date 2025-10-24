using UnityEngine;

public class CameraController : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("The drone's transform that the camera will follow.")]
    public Transform droneTransform;

    [Header("Settings")]
    [Tooltip("How smoothly the camera follows the drone. Lower values are slower/smoother.")]
    [Range(0.01f, 1.0f)]
    public float smoothSpeed = 0.125f;
    
    // This is called after all Update functions have been called.
    // It's the best place to put camera movement code to avoid jitter.
    void LateUpdate()
    {
        // If the droneTransform is not set, do nothing.
        if (droneTransform == null)
        {
            Debug.LogWarning("Drone Transform not assigned in CameraController.");
            return;
        }

        // Use linear interpolation (Lerp) to smoothly move the camera holder
        // from its current position to the drone's position.
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, droneTransform.position, smoothSpeed);
        transform.position = smoothedPosition;
    }
}

