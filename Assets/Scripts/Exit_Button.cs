/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/
using UnityEngine;

/// <summary>
/// Provee la funcionalidad para cerrar la aplicación o detener la simulación en el Editor de Unity.
/// </summary>
public class ExitButton : MonoBehaviour
{
    public void ExitGame()
    {
        #if UNITY_EDITOR
            // Aplicable solo en el editor de Unity
            UnityEditor.EditorApplication.isPlaying = false;
        #else
            // Cierra la aplicación
            Application.Quit();
        #endif
    }
}
