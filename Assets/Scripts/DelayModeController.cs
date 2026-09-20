/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
Modificado: Miguel Hernandez (miguelhernandez@unicauca.edu.co)
            Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
*******************/

using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Controla la activación del modo Delay en la interfaz. 
/// Actúa como un Singleton para proveer el tiempo de retraso a otros scripts.
/// </summary>
public class DelayModeController : MonoBehaviour
{
    [Header("UI - Toggles de Activación")]
    public Toggle delayModeToggle;
    public Toggle secondaryDelayToggle;
    public Toggle tertiaryDelayToggle;

    [Header("UI - Campos de Entrada (Segundos)")]
    public InputField delayInputField;
    public InputField secondaryInputField;
    public InputField tertiaryInputField;

    // --- Estado Interno ---
    private bool isDelayModeActive = false;
    private float delayTime = 0f;

    /// <summary>Singleton para acceder fácilmente desde otros scripts.</summary>
    public static DelayModeController Instance { get; private set; }

    void Awake()
    {
        // Implementación del patrón Singleton
        if (Instance == null)
        {
            Instance = this;
            // DontDestroyOnLoad(gameObject); // Persistir entre escenas si es necesario
        }
        else
        {
            Destroy(gameObject);
        }
    }

    void Start()
    {
        ConfigurarToggle(delayModeToggle);
        ConfigurarToggle(secondaryDelayToggle);
        ConfigurarToggle(tertiaryDelayToggle);

        ConfigurarInputField(delayInputField);
        ConfigurarInputField(secondaryInputField);
        ConfigurarInputField(tertiaryInputField);
    }

    private void ConfigurarToggle(Toggle toggle)
    {
        if (toggle != null)
        {
            toggle.isOn = false; // Desactivar modo delay por defecto
            toggle.onValueChanged.AddListener(OnCambioToggleDelay);
        }
    }

    private void ConfigurarInputField(InputField inputField)
    {
        if (inputField != null)
        {
            inputField.onEndEdit.AddListener(OnCambioInputFieldDelay);
            inputField.text = delayTime.ToString();
            inputField.interactable = false; // Deshabilitar campo de delay inicialmente
        }
    }

    private void OnCambioToggleDelay(bool isOn)
    {
        // Verificar el estado de todos los toggles para determinar si el modo delay está activo
        isDelayModeActive = (delayModeToggle?.isOn ?? false) || 
                            (secondaryDelayToggle?.isOn ?? false) || 
                            (tertiaryDelayToggle?.isOn ?? false);

        // Activar o desactivar los campos de InputField según el estado del modo delay
        if (delayInputField != null) delayInputField.interactable = isDelayModeActive;
        if (secondaryInputField != null) secondaryInputField.interactable = isDelayModeActive;
        if (tertiaryInputField != null) tertiaryInputField.interactable = isDelayModeActive;

        Debug.Log($"Modo Delay {(isDelayModeActive ? "activado" : "desactivado")}");
    }

    private void OnCambioInputFieldDelay(string value)
    {
        if (float.TryParse(value, out float nuevoDelay))
        {
            delayTime = Mathf.Max(0, nuevoDelay); // Asegurar que el delay no sea negativo

            // Actualizar todos los InputField con el nuevo valor de delay
            if (delayInputField != null) delayInputField.text = delayTime.ToString();
            if (secondaryInputField != null) secondaryInputField.text = delayTime.ToString();
            if (tertiaryInputField != null) tertiaryInputField.text = delayTime.ToString();

            Debug.Log($"Delay ajustado a: {delayTime} segundos");
        }
        else
        {
            // Restaurar el valor anterior si la entrada no es válida
            if (delayInputField != null) delayInputField.text = delayTime.ToString();
            if (secondaryInputField != null) secondaryInputField.text = delayTime.ToString();
            if (tertiaryInputField != null) tertiaryInputField.text = delayTime.ToString();
        }
    }

    // Método para verificar si el modo delay está activo
    public bool IsDelayModeActive()
    {
        return isDelayModeActive;
    }

    // Método para obtener el tiempo de delay configurado
    public float GetDelayTime()
    {
        return delayTime;
    }
}
