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
using System.IO;
using System.Diagnostics; // Necesario para usar Process
using System.Collections;
using System.Collections.Generic;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// <summary>
/// Gestiona la carga de archivos TXT con trayectorias y su envío a ROS 2.
/// Permite previsualizar la trayectoria en la simulación o enviarla al robot real.
/// </summary>
public class Sendtxt : MonoBehaviour
{
    [Header("Enlaces ROS 2")]
    private RosSocket rosSocket; 
    private string filePathTopic = "/input_cartesian_path";
    private string previewPathTopic = "/input_cartesian_path_preview";

    [Header("UI - Controles de Archivo")]
    public Button loadTxtButton; 
    public Button previewButton; 
    public Button sendHelloButton; 
    public Button resetButton; 

    // --- Estado Interno ---
    private string selectedFilePath = ""; 
    private string initialPath = "/home/miguel/Interfaz Unity AN5"; // TODO: Migrar a Application.dataPath

    [Header("Seguridad - Aviso UI")]
    [Tooltip("Asigna aquí tu caja de texto para mostrar los avisos de seguridad.")]
    public Text safetyWarningText;

    void Start()
    {
        // Conectar con el servidor ROSBridge
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketSharpProtocol("ws://localhost:9090"));

        // Anunciar los tópicos en ROS2
        rosSocket.Advertise<RosString>(filePathTopic);
        rosSocket.Advertise<RosString>(previewPathTopic);
        UnityEngine.Debug.Log("Conectado y tópicos anunciados: " + filePathTopic + ", " + previewPathTopic);

        // Asignar la función al botón de cargar archivo
        if (loadTxtButton != null)
        {
            loadTxtButton.onClick.AddListener(() => StartCoroutine(OpenFileBrowser()));
        }

        // Asignar la función al botón de enviar mensaje (Robot Real)
        if (sendHelloButton != null)
        {
            sendHelloButton.onClick.AddListener(() => PublishFilePath());
        }

        // Asignar la función al botón de reproducir/preview (Solo Unity)
        if (previewButton != null)
        {
            previewButton.onClick.AddListener(() => PublishPreviewFilePath());
        }
    }

    // Corrutina para abrir el explorador de archivos usando Zenity
    private IEnumerator OpenFileBrowser()
    {
        string filePath = "";
        bool done = false;

        yield return new WaitForEndOfFrame(); // Esperar al final del frame para asegurar que se inicia correctamente

        // Ejecutar Zenity en un hilo separado para evitar bloquear el hilo principal
        System.Threading.Thread t = new System.Threading.Thread(() =>
        {
            // Construir el comando de Zenity para seleccionar archivos .txt
            string arguments = $"--file-selection --filename={initialPath}/ --title=\"Selecciona un archivo .txt\" --file-filter=*.txt";

            Process process = new Process();
            process.StartInfo.FileName = "zenity";
            process.StartInfo.Arguments = arguments;
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.RedirectStandardOutput = true;

            try
            {
                process.Start();

                filePath = process.StandardOutput.ReadLine(); // Leer la ruta del archivo seleccionado
                process.WaitForExit(); // Esperar a que Zenity termine
            }
            catch (System.Exception e)
            {
                UnityEngine.Debug.LogError("Error al ejecutar Zenity: " + e.Message); // Error en español
            }
            finally
            {
                done = true; // Indicar que el proceso ha finalizado
            }
        });

        t.Start(); // Iniciar el hilo

        // Esperar hasta que el hilo termine
        while (!done)
        {
            yield return null;
        }

        if (!string.IsNullOrEmpty(filePath) && File.Exists(filePath))
        {
            selectedFilePath = filePath;
            UnityEngine.Debug.Log("Archivo .txt seleccionado: " + selectedFilePath);

            // Validar el archivo al cargarlo (SafetyRobot)
            string validationError;
            if (!SafetyRobot.ValidateTxtFile(selectedFilePath, out validationError))
            {
                UnityEngine.Debug.LogError("[SafetyRobot] " + validationError);
                if (safetyWarningText != null)
                    safetyWarningText.text = validationError;
            }
            else
            {
                UnityEngine.Debug.Log("[SafetyRobot] Archivo validado. Listo para enviar.");
                if (safetyWarningText != null)
                    safetyWarningText.text = "✅ Archivo validado. Listo para enviar.";
            }
        }
        else
        {
            UnityEngine.Debug.Log("No se seleccionó ningún archivo.");
            selectedFilePath = "";
        }
    }

    // Método que publica la ruta del archivo en el tópico
    private void PublishFilePath()
    {
        if (!string.IsNullOrEmpty(selectedFilePath))
        {
            string validationError;
            if (!SafetyRobot.ValidateTxtFile(selectedFilePath, out validationError))
            {
                UnityEngine.Debug.LogError("[SafetyRobot] " + validationError);
                if (safetyWarningText != null)
                    safetyWarningText.text = validationError;
                return;
            }
            RosString message = new RosString { data = selectedFilePath };
            rosSocket.Publish(filePathTopic, message);
            UnityEngine.Debug.Log("Ruta del archivo publicada en el tópico: " + filePathTopic);
            if (safetyWarningText != null)
                safetyWarningText.text = "✅ Archivo enviado al robot.";
        }
        else
        {
            UnityEngine.Debug.Log("No hay archivo seleccionado. Por favor, selecciona un archivo .txt antes de enviar.");
        }
    }

    // Método que publica la ruta del archivo en el tópico de preview (solo animación en Unity)
    private void PublishPreviewFilePath()
    {
        if (!string.IsNullOrEmpty(selectedFilePath))
        {
            string validationError;
            if (!SafetyRobot.ValidateTxtFile(selectedFilePath, out validationError))
            {
                UnityEngine.Debug.LogError("[SafetyRobot] " + validationError);
                if (safetyWarningText != null)
                    safetyWarningText.text = validationError;
                return;
            }
            RosString message = new RosString { data = selectedFilePath };
            rosSocket.Publish(previewPathTopic, message);
            UnityEngine.Debug.Log("Ruta del archivo enviada para PREVIEW: " + previewPathTopic);
            if (safetyWarningText != null)
                safetyWarningText.text = "✅ Preview enviado.";
        }
        else
        {
            UnityEngine.Debug.Log("No hay archivo seleccionado para reproducir.");
        }

    }

    // Cerrar la conexión al destruir el objeto
    void OnDestroy()
    {
        if (rosSocket != null)
        {
            rosSocket.Close(); // Cerrar la conexión con ROS
            UnityEngine.Debug.Log("RosSocket cerrado."); // Mensaje en español
        }
    }
}