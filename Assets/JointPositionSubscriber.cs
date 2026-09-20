/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/

using System;
using System.Globalization;
using UnityEngine;
using RosSharp.RosBridgeClient;

// Alias para el tipo de mensaje ROS estándar String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class JointPositionSubscriber : UnitySubscriber<RosString>
{
    public JointStateWriter[] jointStateWriters;

    private bool isUpdating = true;

    public event Action<float[]> OnJointPositionsUpdated;

    private float[] lastPositions;

    [Header("Modo Freeze")]
    public bool freezeMode = false;

    public float freezeThresholdDeg = 0.5f;

    protected override void Start()
    {
        base.Start();

        if (jointStateWriters != null && jointStateWriters.Length > 0)
            lastPositions = new float[jointStateWriters.Length];
        else
            lastPositions = new float[6];

        Topic = "current_joint_position";
    }

    protected override void ReceiveMessage(RosString message)
    {
        Debug.Log("RAW: " + message.data);

        if (!isUpdating)
            return;

        string[] parts = message.data.Split(',');

        if (parts.Length != jointStateWriters.Length)
        {
            Debug.LogError($"Cantidad de joints incorrecta. Esperados: {jointStateWriters.Length}, recibidos: {parts.Length}");
            return;
        }

        float[] newPositions = new float[jointStateWriters.Length];

        for (int i = 0; i < jointStateWriters.Length; i++)
        {
            if (!float.TryParse(
                parts[i],
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out float degValue))
            {
                Debug.LogError($"No pudo convertir Joint {i}: {parts[i]}");
                return;
            }

            newPositions[i] = degValue;

            Debug.Log($"Joint {i}: {degValue} grados");
        }

        float computedMaxDiff = 0f;

        for (int i = 0; i < newPositions.Length; i++)
        {
            float diff = Mathf.Abs(newPositions[i] - lastPositions[i]);

            if (diff > computedMaxDiff)
                computedMaxDiff = diff;
        }

        Debug.Log($"MaxDiff = {computedMaxDiff}");

        // Reducido para permitir movimientos muy pequeños
        if (computedMaxDiff >= 0.0001f)
        {
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = false;
                writer.UnlockWriting();
            }

            for (int i = 0; i < jointStateWriters.Length; i++)
            {
                float jointRad = newPositions[i] * Mathf.Deg2Rad;

                Debug.Log($"Escribiendo Joint {i}: {jointRad} rad ({newPositions[i]}°)");

                jointStateWriters[i].Write(jointRad);
            }

            lastPositions = (float[])newPositions.Clone();
        }
        else if (freezeMode && computedMaxDiff < freezeThresholdDeg)
        {
            OnJointPositionsUpdated?.Invoke(newPositions);
            return;
        }
        else
        {
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = true;
                writer.UnlockWriting();
            }
        }

        OnJointPositionsUpdated?.Invoke(newPositions);
    }

    public float[] GetLastKnownPositions()
    {
        return (float[])lastPositions.Clone();
    }

    public void StopUpdating()
    {
        isUpdating = false;
    }

    public void StartUpdating()
    {
        isUpdating = true;
    }
}
