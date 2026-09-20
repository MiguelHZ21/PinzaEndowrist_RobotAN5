/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/

using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Collections;
using System.Collections.Generic;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// <summary>
/// Nodo publicador central hacia ROS 2. 
/// Encargado de enrutar comandos de movimiento a la API, publicar posiciones para cinemática inversa/directa
/// y coordinar comandos de Paro (Stop) y cambio de Modos (Manual/Automático).
/// </summary>
public class Ros2CommandSender : MonoBehaviour
{
    [Header("Enlaces ROS 2")]
    private RosSocket rosSocket;
    private Dictionary<string, string> advertisedTopics = new Dictionary<string, string>();

    [Header("Tópicos")]
    public string commandTopic = "api_command"; 
    public string inverseInputTopic = "input_cartesian_position"; 
    public string directaInputTopic = "input_joint_position"; 
    public string endoWristCommandTopic = "endowrist_command"; 

    [Header("UI - Controles de Paro/Comando")]
    private InputField commandInputField; 
    private Button sendCommandButton;     
    public Button stopCommandButton;         
    public Button duplicateStopCommandButton; 

    [Header("UI - Paneles de Modo")]
    public GameObject modeManualPanel; 
    public GameObject modeAutoPanel;   

    // --- Estado Interno ---
    private bool lastManualState; 
    private bool lastAutoState;   

    void Start()
    {
        // Inicializar RosSocket con la URL del servidor ROSBridge
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketSharpProtocol("ws://localhost:9090"));

        // Asignar referencias de UI si están disponibles
        if (sendCommandButton != null && commandInputField != null)
        {
            sendCommandButton.onClick.AddListener(() => SendCommand(commandInputField.text.Trim()));
        }

        // Asignar funcionalidad al botón stop original
        if (stopCommandButton != null)
        {
            stopCommandButton.onClick.AddListener(() => SendCommand("SplineEnd()"));
            stopCommandButton.onClick.AddListener(() => SendCommand("StopMotion()"));
            stopCommandButton.onClick.AddListener(() => SendCommand("ResetAllError()"));
            stopCommandButton.onClick.AddListener(() => StartCoroutine(SendJogCommandsWithDelay()));
        }

        // Asignar la misma funcionalidad al botón stop duplicado
        if (duplicateStopCommandButton != null)
        {
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("SplineEnd()"));
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("StopMotion()"));
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("ResetAllError()"));
            duplicateStopCommandButton.onClick.AddListener(() => StartCoroutine(SendJogCommandsWithDelay()));
        }

        // Registrar el publicador para el tópico de comandos
        rosSocket.Advertise<StringMsg>(commandTopic);
        advertisedTopics[commandTopic] = commandTopic;
        Debug.Log("Conectado y tópico anunciado: " + commandTopic);

        // Registrar el publicador para el tópico de cinemática inversa
        rosSocket.Advertise<StringMsg>(inverseInputTopic);
        advertisedTopics[inverseInputTopic] = inverseInputTopic;
        Debug.Log("Tópico anunciado: " + inverseInputTopic);

        // Registrar el publicador para el tópico de cinemática directa (si es necesario)
        rosSocket.Advertise<StringMsg>(directaInputTopic);
        advertisedTopics[directaInputTopic] = directaInputTopic;
        Debug.Log("Tópico anunciado: " + directaInputTopic);

        // Registrar el publicador para el tópico de posiciones de la pinza
        rosSocket.Advertise<StringMsg>(endoWristCommandTopic);
        advertisedTopics[endoWristCommandTopic] = endoWristCommandTopic;
        Debug.Log("Tópico anunciado: " + endoWristCommandTopic);

        lastManualState = modeManualPanel.activeSelf;
        lastAutoState = modeAutoPanel.activeSelf;
    }

    void Update()
    {
        // Detectar cambios en el estado del panel de modo manual
        if (modeManualPanel.activeSelf != lastManualState)
        {
            lastManualState = modeManualPanel.activeSelf;
            if (lastManualState) // Si el modo manual se activa
            {
                modeAutoPanel.SetActive(false);
                SendCommand("DragTeachSwitch(0)");
                Debug.Log("Modo manual activado, enviando comando: DragTeachSwitch(0)");
                SendCommand("SplineEnd()");
                SendCommand("ResetAllError()");
                SendCommand("StartJOG(0,6,0,100)");
                SendCommand("StartJOG(0,6,1,100)");
            }
        }

        // Detectar cambios en el estado del panel de modo automático
        if (modeAutoPanel.activeSelf != lastAutoState)
        {
            lastAutoState = modeAutoPanel.activeSelf;
            if (lastAutoState) // Si el modo automático se activa
            {
                modeManualPanel.SetActive(false);
                SendCommand("DragTeachSwitch(1)");
                Debug.Log("Modo automático activado, enviando comando: DragTeachSwitch(1)");
            }
        }
    }

    // Método para enviar un comando al tópico principal de comandos
    public void SendCommand(string command)
    {
        Debug.Log("Preparando para enviar comando: " + command);
        rosSocket.Publish(commandTopic, new StringMsg { data = command });
    }

    // Método para publicar las posiciones de la pinza (shaft, wrist, jaw_dx, jaw_sx) al tópico /endowrist_command
    // Formato del mensaje: "shaft,wrist,jaw_dx,jaw_sx"
    public void SendEndoWristCommand(float[] pinzaPositions)
    {
        if (pinzaPositions == null || pinzaPositions.Length < 4) return;
        string msg = string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "{0:F6},{1:F6},{2:F6},{3:F6}",
            pinzaPositions[0], pinzaPositions[1], pinzaPositions[2], pinzaPositions[3]);
        rosSocket.Publish(endoWristCommandTopic, new StringMsg { data = msg });
        Debug.Log("Pinza publicada en /" + endoWristCommandTopic + ": " + msg);
    }

    // Método para enviar un comando a un tópico específico
    public void SendCommandToTopic(string topic, string command)
    {
        if (!advertisedTopics.ContainsKey(topic))
        {
            rosSocket.Advertise<StringMsg>(topic);
            advertisedTopics[topic] = topic;
            Debug.Log("Tópico anunciado: " + topic);
        }
        Debug.Log("Preparando para enviar comando a " + topic + ": " + command);
        rosSocket.Publish(topic, new StringMsg { data = command });
    }

    // Corrutina para enviar comandos de JOG con retraso
    private IEnumerator SendJogCommandsWithDelay()
    {
        SendCommand("StartJOG(0,6,0,100)");
        yield return new WaitForSeconds(1.32f); // Retraso de 1.32 segundos
        SendCommand("StartJOG(0,6,1,100)");
    }

    void OnDestroy()
    {
        // Cerrar la conexión RosSocket al destruir el objeto
        if (rosSocket != null)
        {
            foreach (var topic in advertisedTopics.Values)
            {
                rosSocket.Unadvertise(topic);
                Debug.Log("Tópico desanunciado: " + topic);
            }
            rosSocket.Close();
            Debug.Log("RosSocket cerrado.");
        }
    }
}
