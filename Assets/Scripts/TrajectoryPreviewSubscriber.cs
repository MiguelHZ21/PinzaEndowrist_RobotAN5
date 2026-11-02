/***************************************************************
Autores:    Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
***************************************************************/

using UnityEngine;
using RosSharp.RosBridgeClient;
using System;
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// <summary>
/// Suscriptor para vista previa (preview) visual de trayectorias articulares en Unity.
/// Tópico: /preview_joint_position (std_msgs/String)
/// </summary>
public class TrajectoryPreviewSubscriber : MonoBehaviour
{
    [Header("Componentes Robot URDF")]
    [Tooltip("Escritores de estado para actualizar la rotación del robot virtual durante el preview.")]
    public JointStateWriter[] jointStateWriters;

    [Header("Enlaces ROS 2")]
    private RosConnector rosConnector;
    private RosSocket rosSocket;
    private string topicId;
    private string topic = "/preview_joint_position";

    // --- Sincronización ROS a Unity (Hilos) ---
    private readonly object lockObj = new object();
    private float[] pendingJoints = null;
    private float[] pendingPinza = null;

    private JointPositionSubscriber jointPositionSubscriber;
    private EndoWristSubscriber endoWristSubscriber;

    void Start()
    {
        rosConnector = GetComponent<RosConnector>();
        jointPositionSubscriber = GetComponent<JointPositionSubscriber>();
        endoWristSubscriber = GetComponent<EndoWristSubscriber>();
        if (rosConnector == null)
        {
            Debug.LogError("TrajectoryPreviewSubscriber: RosConnector no encontrado.");
            return;
        }
        StartCoroutine(WaitAndSubscribe());
    }

    private System.Collections.IEnumerator WaitAndSubscribe()
    {
        while (!rosConnector.IsConnected.WaitOne(0))
        {
            yield return null;
        }
        rosSocket = rosConnector.RosSocket;
        topicId = rosSocket.Subscribe<StringMsg>(topic, OnMessageReceived, queue_length: 1);
        Debug.Log("TrajectoryPreviewSubscriber: Suscrito a " + topic);
    }

    private void OnMessageReceived(StringMsg message)
    {
        try
        {
            string[] parts = message.data.Split(',');
            if (parts.Length < 6) return;

            float[] degrees = new float[6];
            for (int i = 0; i < 6; i++)
            {
                degrees[i] = float.Parse(parts[i], System.Globalization.CultureInfo.InvariantCulture);
            }

            float[] pinzaDeg = null;
            if (parts.Length >= 10)
            {
                pinzaDeg = new float[4];
                for (int j = 0; j < 4; j++)
                {
                    pinzaDeg[j] = float.Parse(parts[6 + j], System.Globalization.CultureInfo.InvariantCulture);
                }
            }

            lock (lockObj)
            {
                pendingJoints = degrees;
                pendingPinza = pinzaDeg;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("TrajectoryPreviewSubscriber error: " + ex.Message);
        }
    }

    void Update()
    {
        float[] degrees = null;
        float[] pinzaDeg = null;
        lock (lockObj)
        {
            if (pendingJoints != null)
            {
                degrees = pendingJoints;
                pinzaDeg = pendingPinza;
                pendingJoints = null;
                pendingPinza = null;
            }
        }

        if (degrees != null && jointStateWriters != null && jointStateWriters.Length == 6)
        {
            // Pausar lectura física durante el preview para que solo se mueva el modelo virtual en Unity
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StopUpdating();

            for (int i = 0; i < 6; i++)
            {
                if (jointStateWriters[i] != null)
                {
                    jointStateWriters[i].Write(degrees[i] * Mathf.Deg2Rad);
                }
            }
        }

        if (pinzaDeg != null && endoWristSubscriber != null)
        {
            endoWristSubscriber.StopUpdating();
            if (endoWristSubscriber.shaft != null) endoWristSubscriber.shaft.Write(pinzaDeg[0]);
            if (endoWristSubscriber.wrist != null) endoWristSubscriber.wrist.Write(pinzaDeg[1]);
            if (endoWristSubscriber.jawDx != null) endoWristSubscriber.jawDx.Write(pinzaDeg[2]);
            if (endoWristSubscriber.jawSx != null) endoWristSubscriber.jawSx.Write(pinzaDeg[3]);
        }
    }

    void OnDestroy()
    {
        if (rosSocket != null && !string.IsNullOrEmpty(topicId))
        {
            rosSocket.Unsubscribe(topicId);
        }
    }
}
