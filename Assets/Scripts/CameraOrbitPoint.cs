using UnityEngine;

/// <summary>
/// Script para hacer orbitar la cámara alrededor de un objetivo (ej. la Pinza/EndoWrist)
/// usando las teclas A y D, manteniéndola siempre apuntando fijamente al objetivo.
/// </summary>
public class CameraOrbitPoint : MonoBehaviour
{
    [Header("Objetivo de Rotación (Ej. Pinza / EndoWrist)")]
    [Tooltip("Arrastra aquí el GameObject de la Pinza (EndoWrist, tool0, etc.).")]
    public Transform targetPoint;

    [Header("Velocidad de Rotación")]
    [Tooltip("Velocidad de rotación orbital en grados por segundo.")]
    public float rotationSpeed = 60.0f;

    [Header("Invertir Controles")]
    public bool invertDirection = false;

    private void Start()
    {
        // Si no se asignó manualmente en el Inspector, busca la pinza por nombre en la escena
        if (targetPoint == null)
        {
            GameObject endo = GameObject.Find("endo_wrist");
            if (endo == null) endo = GameObject.Find("EndoWrist");
            if (endo == null) endo = GameObject.Find("tool0");
            
            if (endo != null)
            {
                targetPoint = endo.transform;
            }
        }

        // Apuntar inicialmente al objetivo
        if (targetPoint != null)
        {
            transform.LookAt(targetPoint);
        }
    }

    private void LateUpdate()
    {
        if (targetPoint == null) return;

        float inputDirection = 0f;

        // Detectar teclas A y D
        if (Input.GetKey(KeyCode.A))
        {
            inputDirection = -1f; // Rota alrededor de la pinza a la izquierda
        }
        else if (Input.GetKey(KeyCode.D))
        {
            inputDirection = 1f;  // Rota alrededor de la pinza a la derecha
        }

        if (inputDirection != 0f)
        {
            if (invertDirection)
            {
                inputDirection *= -1f;
            }

            // 1. Orbitar la posición de la cámara alrededor del centro de la pinza sobre el eje Y
            transform.RotateAround(targetPoint.position, Vector3.up, inputDirection * rotationSpeed * Time.deltaTime);

            // 2. Mantener la cámara apuntando siempre al centro de la pinza
            transform.LookAt(targetPoint);
        }
    }
}
