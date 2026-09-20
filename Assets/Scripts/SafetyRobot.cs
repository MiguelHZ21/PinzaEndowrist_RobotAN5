/*******************
Autores:    Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/

using System.IO;
using UnityEngine;

/// <summary>
/// Provee validaciones estáticas para trayectorias robóticas previniendo saltos bruscos 
/// que puedan causar daños mecánicos o colisiones en el robot físico.
/// </summary>
public static class SafetyRobot
{
    /// <summary>Límite máximo de variación articular (grados) entre dos puntos de la trayectoria.</summary>
    public static float MaxJointJumpDeg = 10f;

    /// <summary>Límite máximo de desplazamiento lineal XYZ (mm) del TCP entre dos puntos consecutivos.</summary>
    public static float MaxCartesianJumpMm = 50f;

    /// <summary>Límite máximo de rotación cartesiana (grados) en cualquier eje (Rx, Ry, Rz).</summary>
    public static float MaxOrientationJumpDeg = 15f;

    /// <summary>
    /// Valida que la interpolación entre dos configuraciones articulares no exceda el límite establecido.
    /// </summary>
    public static bool CheckJointJump(float[] newPositions, float[] prevPositions, out string warningMsg)
    {
        warningMsg = string.Empty;

        if (prevPositions == null || newPositions == null) return true;

        int dof = Mathf.Min(newPositions.Length, prevPositions.Length);
        for (int i = 0; i < dof; i++)
        {
            float delta = Mathf.Abs(newPositions[i] - prevPositions[i]);
            if (delta > MaxJointJumpDeg)
            {
                warningMsg = $"Salto de {delta:F1}° en J{i + 1} excede el límite permitido ({MaxJointJumpDeg}°).";
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Valida que el desplazamiento lineal y angular del TCP no exceda las tolerancias mecánicas.
    /// </summary>
    public static bool CheckCartesianJump(float x, float y, float z, float rx, float ry, float rz, float[] prev, out string warningMsg)
    {
        warningMsg = string.Empty;

        if (prev == null || prev.Length < 6) return true;

        // Distancia euclidiana en el espacio 3D
        float distMm = Mathf.Sqrt(Mathf.Pow(x - prev[0], 2) + Mathf.Pow(y - prev[1], 2) + Mathf.Pow(z - prev[2], 2));
        if (distMm > MaxCartesianJumpMm)
        {
            warningMsg = $"Desplazamiento XYZ de {distMm:F1} mm excede el límite ({MaxCartesianJumpMm} mm).";
            return false;
        }

        // Variación máxima de orientación en cualquiera de los ejes
        float maxOriDelta = Mathf.Max(Mathf.Abs(rx - prev[3]), Mathf.Abs(ry - prev[4]), Mathf.Abs(rz - prev[5]));
        if (maxOriDelta > MaxOrientationJumpDeg)
        {
            warningMsg = $"Variación de orientación de {maxOriDelta:F1}° excede el límite ({MaxOrientationJumpDeg}°).";
            return false;
        }

        return true;
    }

    /// <summary>
    /// Analiza un archivo de trayectoria (CSV) línea por línea para asegurar su viabilidad cinemática.
    /// </summary>
    public static bool ValidateTxtFile(string filePath, out string errorMsg)
    {
        errorMsg = string.Empty;
        try
        {
            string[] lines = File.ReadAllLines(filePath);
            if (lines.Length == 0) return true;

            // Identificar el tipo de trayectoria para aplicar el contexto de validación
            string header = lines[0].Trim().ToLower();
            bool isArticular = (header == "articular");

            float[] prevXYZ = null;
            float[] prevOri = null;

            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i].Trim();

                // Omitir cabeceras o separadores visuales
                if (string.IsNullOrEmpty(line) || (!char.IsDigit(line[0]) && line[0] != '-')) continue;

                string[] parts = line.Split(',');
                if (parts.Length < 6) continue;

                var invCulture = System.Globalization.CultureInfo.InvariantCulture;
                var numStyle = System.Globalization.NumberStyles.Float;

                if (!float.TryParse(parts[0], numStyle, invCulture, out float x) ||
                    !float.TryParse(parts[1], numStyle, invCulture, out float y) ||
                    !float.TryParse(parts[2], numStyle, invCulture, out float z) ||
                    !float.TryParse(parts[3], numStyle, invCulture, out float rx) ||
                    !float.TryParse(parts[4], numStyle, invCulture, out float ry) ||
                    !float.TryParse(parts[5], numStyle, invCulture, out float rz))
                {
                    continue;
                }

                // La validación de saltos cartesianos se omite en trayectorias articulares, 
                // ya que la interpolación en el espacio de juntas genera curvas amplias en el espacio cartesiano.
                if (prevXYZ != null && !isArticular)
                {
                    float[] prev = { prevXYZ[0], prevXYZ[1], prevXYZ[2], prevOri[0], prevOri[1], prevOri[2] };
                    
                    if (!CheckCartesianJump(x, y, z, rx, ry, rz, prev, out string checkMsg))
                    {
                        errorMsg = $"Línea {i + 1}: {checkMsg}";
                        return false;
                    }
                }

                prevXYZ = new float[] { x, y, z };
                prevOri = new float[] { rx, ry, rz };
            }
            return true;
        }
        catch (System.Exception e)
        {
            errorMsg = $"Excepción de E/S al leer la trayectoria: {e.Message}";
            return false;
        }
    }
}
