/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/

using System;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// <summary>
/// Suscriptor para resultados de cinemática directa enviados por el MGD_Node.
/// Tópico: /output_cartesian_position
/// </summary>
public class MGD_Subscriber : UnitySubscriber<RosString>
{
    /// <summary>Evento para notificar a ControlArticular sobre la nueva posición cartesiana.</summary>
    public Action<string> OnInverseKinematicsResultReceived;

    // Inicializa la suscripción al tópico ROS correspondiente
    protected override void Start()
    {
        // Configurar el tópico al que se suscribirá
        Topic = "output_cartesian_position"; // topico que recibe el resultado de la cinematica directa en posciones cartesianas 
        Debug.Log("MGD_Subscriber suscrito al tópico: " + Topic); // Mensaje en español

        base.Start(); // Ahora se suscribe al tópico correcto
    }

    // Método que se llama al recibir un mensaje del tópico suscrito
    protected override void ReceiveMessage(RosString message)
    {
        Debug.Log("MGD_Subscriber recibió mensaje: " + message.data); // Mensaje de depuración en español

        // Invocar el evento para notificar al ControlArticular
        if (OnInverseKinematicsResultReceived != null)
        {
            OnInverseKinematicsResultReceived.Invoke(message.data);
        }
        else
        {
            Debug.LogWarning("No hay suscriptores al evento OnInverseKinematicsResultReceived."); // Mensaje en español
        }
    }
}
