/*******************
Suscriptor que recibe posiciones de la pinza (EndoWrist) publicadas
por el nodo de Python al reproducir una trayectoria cargada desde TXT.
Tópico: /output_endowrist_path  (std_msgs/String)
Formato del mensaje: "shaft,wrist,jaw_dx,jaw_sx" (4 valores float en radianes)
*******************/

using UnityEngine;
using RosSharp.RosBridgeClient;
using System;

// Alias para diferenciar entre RosSharp y System
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class EndoWristPathSubscriber : MonoBehaviour
{
    [Header("JointStateWriter de cada articulación de la pinza")]
    public JointStateWriter shaft;
    public JointStateWriter wrist;
    public JointStateWriter jawDx;
    public JointStateWriter jawSx;

    private RosConnector rosConnector;
    private RosSocket rosSocket;
    private string topicId;
    private string topic = "/output_endowrist_path";

    // Cola para pasar datos del hilo ROS al hilo principal de Unity
    private readonly object lockObj = new object();
    private float[] pendingValues = null;

    void Start()
    {
        rosConnector = GetComponent<RosConnector>();
        if (rosConnector == null)
        {
            Debug.LogError("EndoWristPathSubscriber: RosConnector no encontrado en el mismo GameObject.");
            return;
        }

        StartCoroutine(WaitAndSubscribe());
    }

    private System.Collections.IEnumerator WaitAndSubscribe()
    {
        // Esperar a que RosConnector esté conectado
        while (!rosConnector.IsConnected.WaitOne(0))
        {
            yield return null;
        }

        rosSocket = rosConnector.RosSocket;
        topicId = rosSocket.Subscribe<StringMsg>(topic, OnMessageReceived, queue_length: 1);
        Debug.Log("EndoWristPathSubscriber: Suscrito al tópico " + topic);
    }

    private void OnMessageReceived(StringMsg message)
    {
        try
        {
            string[] parts = message.data.Split(',');
            if (parts.Length < 4)
            {
                Debug.LogWarning("EndoWristPathSubscriber: mensaje con menos de 4 valores: " + message.data);
                return;
            }

            float[] values = new float[4];
            for (int i = 0; i < 4; i++)
            {
                values[i] = float.Parse(parts[i], System.Globalization.CultureInfo.InvariantCulture);
            }

            // Encolar para el hilo principal
            lock (lockObj)
            {
                pendingValues = values;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("EndoWristPathSubscriber: Error al parsear mensaje: " + ex.Message);
        }
    }

    void Update()
    {
        float[] values = null;
        lock (lockObj)
        {
            if (pendingValues != null)
            {
                values = pendingValues;
                pendingValues = null;
            }
        }

        if (values != null)
        {
            // Escribir los valores en los JointStateWriter de la pinza
            if (shaft != null) shaft.Write(values[0]);
            if (wrist != null) wrist.Write(values[1]);
            if (jawDx != null) jawDx.Write(values[2]);
            if (jawSx != null) jawSx.Write(values[3]);

            Debug.Log($"EndoWristPathSubscriber: Pinza aplicada -> shaft={values[0]:F4}, wrist={values[1]:F4}, jaw_dx={values[2]:F4}, jaw_sx={values[3]:F4}");
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
