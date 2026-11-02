/***************************************************************
Autores:    Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
***************************************************************/
using System;
using System.Globalization;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// <summary>
/// Suscriptor para el tópico /current_endowrist_position (std_msgs/String).
/// Lee la posición real de las articulaciones de la pinza (servos) en GRADOS:
/// Formato: "shaft,wrist,jaw_dx,jaw_sx"
/// Actualiza la rotación de la pinza virtual en Unity.
/// </summary>
public class EndoWristSubscriber : UnitySubscriber<RosString>
{
    [Header("Componentes Robot URDF")]
    [Tooltip("Escritores de estado para actualizar la rotación de cada articulación de la pinza.")]
    public JointStateWriter shaft;
    public JointStateWriter wrist;
    public JointStateWriter jawDx;
    public JointStateWriter jawSx;

    [Header("Estado Interno")]
    private bool isUpdating = true;
    private float[] lastPositions = new float[4];

    protected override void Start()
    {
        Topic = "/current_endowrist_position";
        base.Start();

        Debug.Log("EndoWristSubscriber: Suscrito al tópico de posición real: " + Topic);
    }

    protected override void ReceiveMessage(RosString message)
    {
        if (!isUpdating)
            return;

        if (message == null || string.IsNullOrEmpty(message.data))
            return;

        string[] parts = message.data.Split(',');
        if (parts.Length < 4)
        {
            Debug.LogWarning("EndoWristSubscriber: se esperaban 4 valores en grados (shaft,wrist,jaw_dx,jaw_sx), recibido: " + message.data);
            return;
        }

        if (float.TryParse(parts[0].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float shaftDeg) &&
            float.TryParse(parts[1].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float wristDeg) &&
            float.TryParse(parts[2].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float jawDxDeg) &&
            float.TryParse(parts[3].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float jawSxDeg))
        {
            float[] newPositions = new float[] { shaftDeg, wristDeg, jawDxDeg, jawSxDeg };
            float maxDiff = 0f;
            for (int i = 0; i < 4; i++)
            {
                float diff = Mathf.Abs(newPositions[i] - lastPositions[i]);
                if (diff > maxDiff) maxDiff = diff;
            }

            if (maxDiff >= 0.001f)
            {
                if (shaft != null) shaft.Write(shaftDeg);
                if (wrist != null) wrist.Write(wristDeg);
                if (jawDx != null) jawDx.Write(jawDxDeg);
                if (jawSx != null) jawSx.Write(jawSxDeg);
                lastPositions = newPositions;
            }
        }
        else
        {
            Debug.LogError("EndoWristSubscriber: Error al convertir grados desde: " + message.data);
        }
    }

    /// <summary>
    /// Devuelve las posiciones actuales de la pinza en GRADOS (configuradas en el panel EndoWrist o recibidas del robot).
    /// Se leen las posiciones exactas en grados desde los JointStateWriters.
    /// </summary>
    public float[] GetLastKnownPositions()
    {
        float shaftDeg = shaft != null ? shaft.GetCurrentUnityAngle() : 0f;
        float wristDeg = wrist != null ? wrist.GetCurrentUnityAngle() : 0f;
        float jawDxDeg = jawDx != null ? jawDx.GetCurrentUnityAngle() : 0f;
        float jawSxDeg = jawSx != null ? jawSx.GetCurrentUnityAngle() : 0f;

        return new float[] { shaftDeg, wristDeg, jawDxDeg, jawSxDeg };
    }

    /// <summary>
    /// Detiene la actualización desde el tópico físico para conservar los ángulos virtuales del panel EndoWrist.
    /// </summary>
    public void StopUpdating()
    {
        isUpdating = false;
    }

    /// <summary>
    /// Reanuda la actualización desde el tópico físico (/current_endowrist_position).
    /// </summary>
    public void StartUpdating()
    {
        isUpdating = true;
    }
}
