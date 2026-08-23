/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/
using UnityEngine;
using UnityEngine.UI;

public class ButtonPanel : MonoBehaviour
{
    public GameObject cartesianPanel;
    public GameObject jointPanel;
    public GameObject recordPanel;
    public GameObject txtPanel;
    public GameObject manualModePanel;
    public GameObject autoModePanel;

    public GameObject manualButtonImage;
    public GameObject autoButtonImage;

    public CartesianStateWriterNew cartesianWriter;
    public JointPositionSubscriber jointPositionSubscriber;

    // --- EndoWrist ---
    [Header("Panel EndoWrist")]
    public GameObject endoWristPanel;


    private bool lastManualState;
    private bool lastAutoState;

    void Start()
    {
        manualModePanel.SetActive(true);
        autoModePanel.SetActive(false);
        UpdateButtonVisuals();

        // El panel EndoWrist inicia oculto
        if (endoWristPanel != null)
            endoWristPanel.SetActive(false);

        // El panel EndoWrist inicia oculto
        if (endoWristPanel != null)
            endoWristPanel.SetActive(false);
    }

    public void OpenCartesianPanel()
    {
        OpenPanel(cartesianPanel);
    }

    public void OpenJointPanel()
    {
        OpenPanel(jointPanel);
    }

    public void OpenRecordPanel()
    {
        OpenPanel(recordPanel);
    }

    public void OpenTXTPanel()
    {
        OpenPanel(txtPanel);
    }

    /// <summary>
    /// Abre el panel EndoWrist y cierra todos los demás.
    /// Asignar al OnClick del Boton_Wrist.
    /// </summary>
    public void OpenEndoWristPanel()
    {
        OpenPanel(endoWristPanel);
    }

    public void ToggleManualMode()
    {
        bool newState = !manualModePanel.activeSelf;
        manualModePanel.SetActive(newState);
        autoModePanel.SetActive(!newState);
        UpdateButtonVisuals();
    }

    public void ToggleAutoMode()
    {
        bool newState = !autoModePanel.activeSelf;
        autoModePanel.SetActive(newState);
        manualModePanel.SetActive(!newState);
        UpdateButtonVisuals();
    }

    private void OpenPanel(GameObject panelToOpen)
    {
        // Activa sólo el panel seleccionado y desactiva los demás (con seguridad contra referencias nulas)
        if (cartesianPanel != null)
            cartesianPanel.SetActive(panelToOpen == cartesianPanel);
        if (jointPanel != null)
            jointPanel.SetActive(panelToOpen == jointPanel);
        if (recordPanel != null)
            recordPanel.SetActive(panelToOpen == recordPanel);
        if (txtPanel != null)
            txtPanel.SetActive(panelToOpen == txtPanel);
        if (endoWristPanel != null)
            endoWristPanel.SetActive(panelToOpen == endoWristPanel);

        // Si es jointPanel, recordPanel o txtPanel, se comportan igual que el panel articular:
        // Se activa la actualización normal de articulaciones y cartesiano.
        if (panelToOpen == jointPanel ||
            panelToOpen == recordPanel ||
            panelToOpen == txtPanel)
        {
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StartUpdating(); // Activa actualización normal en el panel articular.
            if (cartesianWriter != null)
                cartesianWriter.StartUpdating();         // Desactiva interpolación en el panel cartesiano (o lógica que tú uses).
        }
        // Si es el panel cartesiano, se detiene la actualización normal y se activa la interpolación.
        else if (panelToOpen == cartesianPanel)
        {
            if (cartesianWriter != null)
                cartesianWriter.StopUpdating();  // Activa interpolación en el panel cartesiano.
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StopUpdating(); // Detiene actualización en el panel articular.
            // Además, aquí se reinicia la suscripción de jointPositionSubscriber sólo cuando se presione Send (desde el otro script).
        }
        // Si es el panel EndoWrist, se comporta como el articular.
        else if (panelToOpen == endoWristPanel)
        {
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StartUpdating();
            if (cartesianWriter != null)
                cartesianWriter.StartUpdating();
        }
    }

    private void UpdateButtonVisuals()
    {
        if (manualButtonImage != null && autoButtonImage != null)
        {
            manualButtonImage.SetActive(manualModePanel.activeSelf);
            autoButtonImage.SetActive(autoModePanel.activeSelf);
        }
    }

}

