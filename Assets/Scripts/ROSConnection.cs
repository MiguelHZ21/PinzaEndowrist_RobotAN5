/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Desactiva el log nativo de Unity para optimizar rendimiento 
/// y evitar saturación en la consola durante transmisiones de ROS masivas.
/// </summary>
public class ROSConnection : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
         Debug.unityLogger.logEnabled = false;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
