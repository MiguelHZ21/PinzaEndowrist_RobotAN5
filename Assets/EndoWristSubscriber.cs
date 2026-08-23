using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

public class EndoWristSubscriber : UnitySubscriber<JointState>
{
    [Header("JointStateWriter de cada articulación")]
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
            float position = (float)message.position[i];

            Debug.Log($"{jointName} -> {position}");

            switch (jointName)
            {
                case "shaft":
                    if (shaft != null)
                        shaft.Write(position);
                    
                    break;

                case "wrist":
                    if (wrist != null)
                        wrist.Write(position);
                    
                    break;

                case "jaw_dx":
                    if (jawDx != null)
                        jawDx.Write(position);
                
                    break;

                case "jaw_sx":
                    if (jawSx != null)
                        jawSx.Write(position);
                    
                    break;

                default:
                    Debug.LogWarning("Articulación no reconocida: " + jointName);
                    break;
            }
        }
    }

    public float[] GetLastKnownPositions()
    {
        // Leer directamente los ángulos actuales en Unity (en grados)
        float shaftDeg = shaft != null ? shaft.GetCurrentUnityAngle() : 0f;
        float wristDeg = wrist != null ? wrist.GetCurrentUnityAngle() : 0f;
        float jawDxDeg = jawDx != null ? jawDx.GetCurrentUnityAngle() : 0f;
        float jawSxDeg = jawSx != null ? jawSx.GetCurrentUnityAngle() : 0f;

        // Convertir de grados a radianes y revertir la lógica de JointStateWriter
        // En JointStateWriter: targetValue = radianes * Rad2Deg * -1f
        // Por tanto: radianes = grados * Deg2Rad * -1f
        float shaftRad = shaftDeg * Mathf.Deg2Rad * -1f;
        float wristRad = wristDeg * Mathf.Deg2Rad * -1f;
        
        // jaw_dx tiene una inversión doble en JointStateWriter (targetValue * -1f)
        // Por tanto: grados_jawDx = radianes * Rad2Deg
        // Entonces: radianes = grados * Deg2Rad
        float jawDxRad = jawDxDeg * Mathf.Deg2Rad;
        
        // jaw_sx sigue la misma lógica normal
        float jawSxRad = jawSxDeg * Mathf.Deg2Rad * -1f;

        return new float[] { shaftRad, wristRad, jawDxRad, jawSxRad };
    }
}
