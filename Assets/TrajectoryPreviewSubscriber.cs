/*******************
Suscriptor para vista previa (preview) visual de trayectorias articulares en Unity.
Tópico: /preview_joint_position (std_msgs/String)
Formato: "j1,j2,j3,j4,j5,j6" (en grados)
*******************/

using UnityEngine;
using RosSharp.RosBridgeClient;
using System;
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class TrajectoryPreviewSubscriber : MonoBehaviour
{
    [Header("JointStateWriter de las 6 articulaciones del robot")]
    public JointStateWriter[] jointStateWriters;

    private RosConnector rosConnector;
    private RosSocket rosSocket;
    private string topicId;
    private string topic = "/preview_joint_position";

    private readonly object lockObj = new object();
    private float[] pendingJoints = null;

    void Start()
    {
        rosConnector = GetComponent<RosConnector>();
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

            lock (lockObj)
            {
                pendingJoints = degrees;
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
        lock (lockObj)
        {
            if (pendingJoints != null)
            {
                degrees = pendingJoints;
                pendingJoints = null;
            }
        }

        if (degrees != null && jointStateWriters != null && jointStateWriters.Length == 6)
        {
            for (int i = 0; i < 6; i++)
            {
                if (jointStateWriters[i] != null)
                {
                    jointStateWriters[i].Write(degrees[i] * Mathf.Deg2Rad);
                }
            }
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
