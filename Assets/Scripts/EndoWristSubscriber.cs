/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

/// <summary>
/// Suscriptor para el tópico /endowrist (sensor_msgs/JointState).
/// Actualiza la posición de las articulaciones de la pinza virtual en tiempo real
/// en función de la información proveniente de ROS 2.
/// NOTA: ROS publica en RADIANES; este suscriptor convierte a GRADOS
/// antes de pasar los valores al JointStateWriter.
/// </summary>
public class EndoWristSubscriber : UnitySubscriber<JointState>
{
    [Header("Componentes Robot URDF")]
    [Tooltip("Escritores de estado para actualizar la rotación de cada articulación de la pinza.")]
    public JointStateWriter shaft;
    public JointStateWriter wrist;
    public JointStateWriter jawDx;
    public JointStateWriter jawSx;

    protected override void Start()
    {
        Topic = "/endowrist";
        base.Start();

        Debug.Log("Suscrito al tópico: " + Topic);
    }

    protected override void ReceiveMessage(JointState message)
    {
        for (int i = 0; i < message.name.Length; i++)
        {
            string jointName = message.name[i];
            // ROS publica en RADIANES → convertir a GRADOS para el sistema interno
            float positionDeg = (float)message.position[i] * Mathf.Rad2Deg;

            Debug.Log($"{jointName} -> {positionDeg}°");

            switch (jointName)
            {
                case "shaft":
                case "Shaft":
                case "Endo_eje":
                    if (shaft != null)
                        shaft.Write(positionDeg);
                    
                    break;

                case "wrist":
                case "Wrist":
                case "Endo_muneca":
                    if (wrist != null)
                        wrist.Write(positionDeg);
                    
                    break;

                case "jaw_dx":
                case "Jaw_Dx":
                case "Endo_mandibula_dx":
                    if (jawDx != null)
                        jawDx.Write(positionDeg);
                
                    break;

                case "jaw_sx":
                case "Jaw_Sx":
                case "Endo_mandibula_sx":
                    if (jawSx != null)
                        jawSx.Write(positionDeg);
                    
                    break;

                default:
                    Debug.LogWarning("Articulación no reconocida: " + jointName);
                    break;
            }
        }
    }

    /// <summary>
    /// Devuelve las posiciones actuales de la pinza en GRADOS.
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
}
