using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

public class ControlPinza : MonoBehaviour
{
    [Header("Sliders del EndoWrist")]
    public Slider sliderShaft;    // SliderArt1 -> shaft (Endo_eje)
    public Slider sliderWrist;    // SliderArt2 -> wrist (Endo_muneca)
    public Slider sliderJawDx;    // SliderArt3 -> jaw_dx (Endo_mandibula_dx)
    public Slider sliderJawSx;    // SliderArt4 -> jaw_sx (Endo_mandibula_sx)

    [Header("JointStateWriter del EndoWrist")]
    public JointStateWriter writerShaft;
    public JointStateWriter writerWrist;
    public JointStateWriter writerJawDx;
    public JointStateWriter writerJawSx;

    [Header("Textos de valor EndoWrist (opcional)")]
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
