/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using UnityEngine;
using RosSharp.RosBridgeClient;
using System;
using System.Linq;
using System.Collections;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class MGD_Node : MonoBehaviour
{
    // Parámetros DH del FR5
    double[,] DH_params = new double[,] {
        {0, Math.PI / 2, 0.152, 0},
        {-0.425, 0, 0, 0},
        {-0.395, 0, 0, 0},
        {0, Math.PI / 2, 0.102, 0},
        {0, -Math.PI / 2, 0.102, 0},
        {0, 0, 0.457, 0}
    };

    private RosConnector rosConnector; // Referencia al RosConnector
    private RosSocket rosSocket; // Referencia al RosSocket

    private string inputTopic = "input_joint_position"; // topico que publica en la cinematica directa 
    private string outputTopic = "output_cartesian_position"; // topico que recibe el resultado de la cinematica directa en posciones cartesianas 

    private string inputTopicId; // ID de suscripción al tópico de entrada
    private string outputTopicId; // ID de publicador al tópico de salida

    void Start()
    {
        // Obtener la instancia de RosConnector
        rosConnector = GetComponent<RosConnector>();
        if (rosConnector == null)
        {
            Debug.LogError("RosConnector no encontrado en el mismo GameObject."); // Error en español
            return;
        }

        // Esperar a que la conexión se establezca
        StartCoroutine(WaitForConnectionAndSubscribe());
    }

    // Corrutina para esperar la conexión y suscribirse/publicar a los tópicos
    private IEnumerator WaitForConnectionAndSubscribe()
    {
        // Esperar hasta que RosConnector esté conectado
        while (!rosConnector.IsConnected.WaitOne(0))
        {
            yield return null;
        }

        // Obtener el RosSocket una vez conectado
        rosSocket = rosConnector.RosSocket;

        // Suscribirse al tópico input_joint_position
        inputTopicId = rosSocket.Subscribe<StringMsg>(
            inputTopic,
            ListenerCallback,
            queue_length: 1);

        // Registrar el publicador para output_cartesian_position
        outputTopicId = rosSocket.Advertise<StringMsg>(outputTopic);

        Debug.Log("Suscrito al tópico: " + inputTopic); // Mensaje en español
        Debug.Log("Publicador registrado para el tópico: " + outputTopic); // Mensaje en español
    }

    // Callback que se ejecuta al recibir un mensaje del tópico suscrito
    void ListenerCallback(StringMsg message)
    {
        // Convertir la cadena de texto a una lista de ángulos
        string[] angleStrings = message.data.Split(',');
        double[] theta_deg;

        try
        {
            theta_deg = Array.ConvertAll(angleStrings, Double.Parse);
        }
        catch (FormatException ex)
        {
            Debug.LogError("Error al convertir los ángulos: " + ex.Message); // Error en español
            return;
        }

        // Convertir los ángulos de grados a radianes
        double[] theta = theta_deg.Select(angle => angle * Math.PI / 180).ToArray();

        // Inicializar la matriz de transformación total como identidad
        double[,] T_total = MatrixIdentity(4);

        // Calcular la matriz de transformación para cada articulación
        for (int i = 0; i < theta.Length; i++)
        {
            double a = DH_params[i, 0];
            double alpha = DH_params[i, 1];
            double d = DH_params[i, 2];
            double th = theta[i] + DH_params[i, 3];

            double[,] T_i = dh_matrix(a, alpha, d, th);
            T_total = MatrixMultiply(T_total, T_i);
        }

        // Extraer la posición del efector final y convertir a milímetros
        double Px = Math.Round(T_total[0, 3] * 1000, 2);
        double Py = Math.Round(T_total[1, 3] * 1000, 2);
        double Pz = Math.Round(T_total[2, 3] * 1000, 2);

        // Extraer rx, ry, rz con manejo de Gimbal Lock
        var (rx, ry, rz) = ExtractRPY(T_total);

        // Crear la cadena de texto de salida con la posición y los ángulos de rotación
        string salida = string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2}",
            Px, Py, Pz, rx, ry, rz);

        // Publicar la salida en output_cartesian_position
        StringMsg outputMsg = new StringMsg();
        outputMsg.data = salida;
        rosSocket.Publish(outputTopicId, outputMsg);

        // Mostrar la salida en consola
        Debug.Log($"Salida [Px, Py, Pz, rx, ry, rz]: {salida}"); // Mensaje en español
    }

    // ---------------------------------------------------------------
    // Método estático público: calcula el MGD directamente en Unity
    // sin pasar por ROS. Recibe los 6 ángulos en grados y devuelve
    // la cadena "Px,Py,Pz,rx,ry,rz" lista para guardar en el TXT.
    // ---------------------------------------------------------------
    public static string ComputeMGD(float[] jointDegrees)
    {
        double[,] DH = new double[,] {
            {0, Math.PI / 2, 0.152, 0},
            {-0.425, 0, 0, 0},
            {-0.395, 0, 0, 0},
            {0, Math.PI / 2, 0.102, 0},
            {0, -Math.PI / 2, 0.102, 0},
            {0, 0, 0.457, 0}
        };

        double[] theta = new double[6];
        for (int i = 0; i < jointDegrees.Length; i++)
            theta[i] = jointDegrees[i] * Math.PI / 180.0;

        double[,] T = MatrixIdentityStatic(4);
        for (int i = 0; i < 6; i++)
        {
            double a     = DH[i, 0];
            double alpha = DH[i, 1];
            double d     = DH[i, 2];
            double th    = theta[i] + DH[i, 3];
            T = MatrixMultiplyStatic(T, DhMatrixStatic(a, alpha, d, th));
        }

        double Px = Math.Round(T[0, 3] * 1000, 2);
        double Py = Math.Round(T[1, 3] * 1000, 2);
        double Pz = Math.Round(T[2, 3] * 1000, 2);

        // Extraer rx, ry, rz con manejo de Gimbal Lock
        var (rx, ry, rz) = ExtractRPY(T);

        return string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2}",
            Px, Py, Pz, rx, ry, rz);
    }

    // Método estático para extraer RPY con manejo de Gimbal Lock (singularidad en Ry = ±90°)
    public static (double rx, double ry, double rz) ExtractRPY(double[,] T)
    {
        double sy = -T[2, 0];
        sy = Math.Max(-1.0, Math.Min(1.0, sy));
        double ry_rad = Math.Asin(sy);

        double rx_rad, rz_rad;

        if (Math.Abs(Math.Cos(ry_rad)) < 1e-6) // Gimbal lock: ry = ±90°
        {
            rz_rad = 0.0;
            rx_rad = Math.Atan2(-T[1, 2], T[1, 1]);
        }
        else
        {
            rz_rad = Math.Atan2(T[1, 0], T[0, 0]);
            rx_rad = Math.Atan2(T[2, 1], T[2, 2]);
        }

        double rx = Math.Round(rx_rad * 180.0 / Math.PI, 2);
        double ry = Math.Round(ry_rad * 180.0 / Math.PI, 2);
        double rz = Math.Round(rz_rad * 180.0 / Math.PI, 2);

        return (rx, ry, rz);
    }

    private static double[,] DhMatrixStatic(double a, double alpha, double d, double theta)
    {
        return new double[4, 4] {
            { Math.Cos(theta), -Math.Sin(theta)*Math.Cos(alpha),  Math.Sin(theta)*Math.Sin(alpha), a*Math.Cos(theta) },
            { Math.Sin(theta),  Math.Cos(theta)*Math.Cos(alpha), -Math.Cos(theta)*Math.Sin(alpha), a*Math.Sin(theta) },
            { 0,                Math.Sin(alpha),                   Math.Cos(alpha),                 d                },
            { 0,                0,                                 0,                               1                }
        };
    }

    private static double[,] MatrixIdentityStatic(int n)
    {
        double[,] m = new double[n, n];
        for (int i = 0; i < n; i++) m[i, i] = 1;
        return m;
    }

    private static double[,] MatrixMultiplyStatic(double[,] A, double[,] B)
    {
        int r = A.GetLength(0), c = B.GetLength(1), k = A.GetLength(1);
        double[,] R = new double[r, c];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                for (int l = 0; l < k; l++)
                    R[i, j] += A[i, l] * B[l, j];
        return R;
    }

    // Función para calcular la matriz de transformación de Denavit-Hartenberg
    double[,] dh_matrix(double a, double alpha, double d, double theta)
    {
        double[,] matrix = new double[4, 4];

        matrix[0, 0] = Math.Cos(theta);
        matrix[0, 1] = -Math.Sin(theta) * Math.Cos(alpha);
        matrix[0, 2] = Math.Sin(theta) * Math.Sin(alpha);
        matrix[0, 3] = a * Math.Cos(theta);

        matrix[1, 0] = Math.Sin(theta);
        matrix[1, 1] = Math.Cos(theta) * Math.Cos(alpha);
        matrix[1, 2] = -Math.Cos(theta) * Math.Sin(alpha);
        matrix[1, 3] = a * Math.Sin(theta);

        matrix[2, 0] = 0;
        matrix[2, 1] = Math.Sin(alpha);
        matrix[2, 2] = Math.Cos(alpha);
        matrix[2, 3] = d;

        matrix[3, 0] = 0;
        matrix[3, 1] = 0;
        matrix[3, 2] = 0;
        matrix[3, 3] = 1;

        return matrix;
    }

    // Función para crear una matriz identidad de tamaño n x n
    double[,] MatrixIdentity(int n)
    {
        double[,] identity = new double[n, n];
        for (int i = 0; i < n; i++)
            identity[i, i] = 1;
        return identity;
    }

    // Función para multiplicar dos matrices
    double[,] MatrixMultiply(double[,] A, double[,] B)
    {
        int rowsA = A.GetLength(0);
        int colsA = A.GetLength(1);
        int colsB = B.GetLength(1);
        double[,] result = new double[rowsA, colsB];

        for (int i = 0; i < rowsA; i++)
            for (int j = 0; j < colsB; j++)
            {
                double sum = 0;
                for (int k = 0; k < colsA; k++)
                    sum += A[i, k] * B[k, j];
                result[i, j] = sum;
            }
        return result;
    }

    // Función para truncar números a un número específico de decimales sin redondear
    double Truncate(double number, int decimals)
    {
        double factor = Math.Pow(10, decimals);
        return Math.Floor(number * factor) / factor;
    }

    void OnDestroy()
    {
        // Cancelar suscripciones y anuncios
        if (rosSocket != null)
        {
            if (!string.IsNullOrEmpty(inputTopicId))
                rosSocket.Unsubscribe(inputTopicId);
            if (!string.IsNullOrEmpty(outputTopicId))
                rosSocket.Unadvertise(outputTopicId);
        }
    }
}