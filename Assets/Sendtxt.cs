/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
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

public class Sendtxt : MonoBehaviour
{
    private RosSocket rosSocket; // Conexión con ROS
    private string filePathTopic = "/input_cartesian_path";
    private string previewPathTopic = "/input_cartesian_path_preview";

    public Button sendHelloButton; // Botón para enviar el mensaje con la ruta del archivo (Robot real)
    public Button resetButton; // Botón para solucionar errores
    public Button loadTxtButton; // Botón para abrir el explorador de archivos
    public Button previewButton; // Botón para reproducir la trayectoria en Unity (preview visual)

    private string selectedFilePath = ""; // Ruta del archivo seleccionado
    private string initialPath = "/home/miguel/Interfaz Unity AN5"; // Ruta inicial para el explorador de archivos

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
            UnityEngine.Debug.Log("Archivo .txt seleccionado: " + selectedFilePath); // Mensaje en español
        }
        else
        {
            UnityEngine.Debug.Log("No se seleccionó ningún archivo."); // Mensaje en español
            selectedFilePath = ""; // Reiniciar la selección si no se seleccionó un archivo
        }
    }

    // Método que publica la ruta del archivo en el tópico
    private void PublishFilePath()
    {
        if (!string.IsNullOrEmpty(selectedFilePath))
        {
            // Crear el mensaje con la ruta del archivo
            RosString message = new RosString { data = selectedFilePath };
            rosSocket.Publish(filePathTopic, message);
            UnityEngine.Debug.Log("Ruta del archivo publicada en el tópico: " + filePathTopic); // Mensaje en español
        }
        else
        {
            UnityEngine.Debug.Log("No hay archivo seleccionado. Por favor, selecciona un archivo .txt antes de enviar."); // Mensaje en español
        }
    }

    // Método que publica la ruta del archivo en el tópico de preview (solo animación en Unity)
    private void PublishPreviewFilePath()
    {
        if (!string.IsNullOrEmpty(selectedFilePath))
        {
            RosString message = new RosString { data = selectedFilePath };
            rosSocket.Publish(previewPathTopic, message);
            UnityEngine.Debug.Log("Ruta del archivo enviada para PREVIEW: " + previewPathTopic);
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