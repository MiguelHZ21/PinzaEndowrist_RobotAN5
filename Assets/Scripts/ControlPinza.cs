/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

/// <summary>
/// Gestiona la recolección visual y envío de las posiciones del instrumento quirúrgico (EndoWrist).
/// Vincula los sliders de la interfaz con los escritores de estado URDF.
/// </summary>
public class ControlPinza : MonoBehaviour
{
    [Header("Sliders del EndoWrist")]
    [Tooltip("SliderArt1 -> shaft (Endo_eje)")]
    public Slider sliderShaft;
    [Tooltip("SliderArt2 -> wrist (Endo_muneca)")]
    public Slider sliderWrist;
    [Tooltip("SliderArt3 -> jaw_dx (Endo_mandibula_dx)")]
    public Slider sliderJawDx;
    [Tooltip("SliderArt4 -> jaw_sx (Endo_mandibula_sx)")]
    public Slider sliderJawSx;

    [Header("Componentes Robot URDF")]
    [Tooltip("Escritores de estado para actualizar la rotación de la pinza virtual.")]
    public JointStateWriter writerShaft;
    public JointStateWriter writerWrist;
    public JointStateWriter writerJawDx;
    public JointStateWriter writerJawSx;

    [Header("Textos de valor EndoWrist")]
    public TMPro.TextMeshProUGUI textValueShaft;
    public TMPro.TextMeshProUGUI textValueWrist;
    public TMPro.TextMeshProUGUI textValueJawDx;
    public TMPro.TextMeshProUGUI textValueJawSx;

    void Start()
    {
        // Configurar listeners de los sliders
        if (sliderShaft != null)
            sliderShaft.onValueChanged.AddListener(OnShaftChanged);
        if (sliderWrist != null)
            sliderWrist.onValueChanged.AddListener(OnWristChanged);
        if (sliderJawDx != null)
            sliderJawDx.onValueChanged.AddListener(OnJawDxChanged);
        if (sliderJawSx != null)
            sliderJawSx.onValueChanged.AddListener(OnJawSxChanged);
    }

    // --- EndoWrist: callbacks de sliders ---
    // Los valores se envían en radianes al JointStateWriter.

    private void OnShaftChanged(float value)
    {
        if (writerShaft != null)
            writerShaft.Write(value);
        if (textValueShaft != null)
            textValueShaft.text = (value * Mathf.Rad2Deg).ToString("F1") + "°";
    }

    private void OnWristChanged(float value)
    {
        if (writerWrist != null)
            writerWrist.Write(value);
        if (textValueWrist != null)
            textValueWrist.text = (value * Mathf.Rad2Deg).ToString("F1") + "°";
    }

    private void OnJawDxChanged(float value)
    {
        if (writerJawDx != null)
            writerJawDx.Write(value);
        if (textValueJawDx != null)
            textValueJawDx.text = (value * Mathf.Rad2Deg).ToString("F1") + "°";
    }

    private void OnJawSxChanged(float value)
    {
        if (writerJawSx != null)
            writerJawSx.Write(value);
        if (textValueJawSx != null)
            textValueJawSx.text = (value * Mathf.Rad2Deg).ToString("F1") + "°";
    }
}
