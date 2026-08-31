/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/
using System;
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.IO;

public class CartesianStateWriterNew : MonoBehaviour
{
    // --- Referencias a la interfaz de usuario ---
    public InputField posXInputField;
    public InputField posYInputField;
    public InputField posZInputField;
    public InputField oriXInputField;
    public InputField oriYInputField;
    public InputField oriZInputField;

    public Button sendCommandButton;
    public Button addButton;
    public Button removeButton; // Botón para eliminar la última posición
    public Button saveTxtButton; // Botón para guardar en txt

    public Text coordinatesDisplay;

    // --- Control de velocidad ---
    public Slider speedSlider;
    public InputField speedValueInput;
    public Button increaseSpeedButton; // Botón para aumentar la velocidad
    public Button decreaseSpeedButton; // Botón para disminuir la velocidad

    // --- Referencias a ROS2 y sus suscriptores ---
    public Ros2CommandSender ros2CommandSender;
    public InverseKinematicsSubscriber ikSubscriber;
    public CartesianPositionSubscriber cartesianPositionSubscriber;
    public JointPositionSubscriber jointPositionSubscriber;

    // --- Referencia a los escritores de estado de articulaciones (para actualizar el URDF) ---
    public JointStateWriter[] jointStateWriters;

    // --- EndoWrist ---
    public EndoWristSubscriber endoWristSubscriber;

    // --- Listas para almacenar comandos y estados ---
    // Lista de comandos articulares (para IK/URDF; no se usan para mover al robot)
    private List<string> jointCommands = new List<string>();
    // Lista para mostrar en pantalla (texto formateado con posiciones cartesianas y velocidad)
    private List<string> cartesianCoordinates = new List<string>();
    // Lista con las posiciones cartesianas en bruto (tomadas de los inputs)
    private List<string> cartesianCommandsList = new List<string>();
    // Lista con los valores [x, y, z, rx, ry, rz] en float (tomadas de los inputs)
    private List<float[]> cartesianPositionsList = new List<float[]>();
    // Lista de comandos que se enviarán al robot (formato CARTPoint(...))
    private List<string> cartPointCommands = new List<string>();
    // Listas de velocidad y delay
    private List<float> speedList = new List<float>();
    private List<float> delayList = new List<float>();
    // *** NUEVO: Lista con los resultados IK (posición de joints en grados) que serán la posición objetivo
    private List<float[]> ikCartesianPositionsList = new List<float[]>();
    
    // Lista para almacenar las posiciones de la pinza
    private List<float[]> endoWristPositionsList = new List<float[]>();

    private int pointIndex = 1;  // Índice para los puntos (aumenta al agregar, se reinicia al enviar cada lote)
    private float[] currentPositions = new float[6];

    // Cola segura para pasar datos desde el hilo de ROS al hilo principal de Unity
    private bool hasNewIkResult = false;
    private float[] pendingIkResult = null;
    private bool pendingNaNError = false;  // Bandera para mostrar error de NaN en hilo principal
    private bool awaitingInverseKinematics = false;
    private bool isListening = true;
    private bool isManualEditing = false;

    private string savePath = "/home/miguel/Interfaz AppDesigner AN5"; // Ruta de guardado

    void Start()
    {
        // Configuración de botones
        addButton.onClick.AddListener(AddCurrentPosition);
        sendCommandButton.onClick.AddListener(() => StartCoroutine(SendCommands()));

        if (removeButton != null)
            removeButton.onClick.AddListener(RemoveLastPosition);

        // Configuración del Slider de velocidad
        speedSlider.onValueChanged.AddListener(OnSpeedSliderValueChanged);
        speedSlider.minValue = 1f;
        speedSlider.maxValue = 100f;
        speedSlider.wholeNumbers = true;
        speedSlider.value = 10;
        speedValueInput.text = speedSlider.value.ToString();
        speedValueInput.onEndEdit.AddListener(OnSpeedInputFieldValueChanged);

        // Botones para aumentar y disminuir velocidad
        if (increaseSpeedButton != null)
            increaseSpeedButton.onClick.AddListener(IncreaseSpeedValue);
        if (decreaseSpeedButton != null)
            decreaseSpeedButton.onClick.AddListener(DecreaseSpeedValue);

        // Suscriptor de cinemática inversa (IK)
        if (ikSubscriber != null)
        {
            ikSubscriber.cartesianStateWriter = this;
            ikSubscriber.OnInverseKinematicsResultReceived += ReceiveJointPositions;
        }
        else
        {
            Debug.LogWarning("ikSubscriber no está asignado en el Inspector.");
        }

        // Iniciar la corrutina que actualiza los InputFields
        StartCoroutine(UpdateInputFieldsContinuously());

        // Listener para guardar en .txt
        if (saveTxtButton != null)
            saveTxtButton.onClick.AddListener(GuardarEnTxt);
    }

    void Update()
    {
        // Detecta si el usuario está editando manualmente los InputFields
        if (!isManualEditing &&
           (posXInputField.isFocused || posYInputField.isFocused || posZInputField.isFocused ||
            oriXInputField.isFocused || oriYInputField.isFocused || oriZInputField.isFocused))
        {
            isListening = false;
            isManualEditing = true;
        }

        // Procesar el resultado IK en el hilo principal de Unity.
        // ROS usa hilos secundarios: llamar StartCoroutine o modificar la UI desde ahí
        // causa un colapso silencioso. Por eso solo guardamos el resultado en el hilo de ROS
        // y lo procesamos aquí, en el hilo principal (Update), donde Unity sí lo permite.
        if (hasNewIkResult && pendingIkResult != null)
        {
            hasNewIkResult = false;
            float[] resultToProcess = pendingIkResult;
            pendingIkResult = null;
            ProcesarJointPositions(resultToProcess);
        }

        // Mostrar error de NaN en el hilo principal (la UI solo puede modificarse aqui)
        if (pendingNaNError)
        {
            pendingNaNError = false;
            coordinatesDisplay.text = "Sin solucion IK para este punto.\nIntenta una posicion diferente.";
            UpdateCoordinatesDisplay();
        }
    }

    // Método para recibir la respuesta de la cinemática inversa (IK)
    // Se mantiene para actualizar el URDF y otros usos.
    public void ReceiveJointPositions(string message)
    {
        if (!awaitingInverseKinematics)
        {
            Debug.LogWarning("Se recibió resultado de IK sin haberlo solicitado.");
            return;
        }

        // Dividir el mensaje
        string[] parts = message.Split(',');

        // Si la respuesta es "Fuera de rango"
        if (parts[0] == "Fuera de rango")
        {
            coordinatesDisplay.text = "Posiciones fuera del rango (IK)";
            awaitingInverseKinematics = false;
            Debug.LogWarning("IK: Posiciones fuera del rango.");
            return;
        }

        // Parsear a float[] con formato invariante (evita problemas con comas/puntos segun el idioma del SO)
        float[] ikResult = new float[6];
        for (int i = 0; i < parts.Length; i++)
        {
            if (!float.TryParse(parts[i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out ikResult[i]))
            {
                Debug.LogError($"Error al parsear la posición {i}: {parts[i]}");
                awaitingInverseKinematics = false;
                return;
            }
        }

        // Verificar si algún valor es NaN: ocurre cuando Matlab no encuentra solución IK
        // para las coordenadas dadas (singularidad, fuera del espacio de trabajo real, etc.).
        // En ese caso se revierte el punto que se estaba agregando y se avisa al usuario.
        bool hasNaN = false;
        for (int i = 0; i < ikResult.Length; i++)
        {
            if (float.IsNaN(ikResult[i]))
            {
                hasNaN = true;
                break;
            }
        }

        if (hasNaN)
        {
            Debug.LogWarning("IK: Matlab devolvio NaN. No existe solucion para este punto.");
            awaitingInverseKinematics = false;

            // Revertir los datos ya guardados en las listas para este punto fallido
            if (cartesianPositionsList.Count > 0)   cartesianPositionsList.RemoveAt(cartesianPositionsList.Count - 1);
            if (cartesianCommandsList.Count > 0)    cartesianCommandsList.RemoveAt(cartesianCommandsList.Count - 1);
            if (cartPointCommands.Count > 0)        cartPointCommands.RemoveAt(cartPointCommands.Count - 1);
            if (speedList.Count > 0)                speedList.RemoveAt(speedList.Count - 1);
            if (delayList.Count > 0)                delayList.RemoveAt(delayList.Count - 1);
            if (cartesianCoordinates.Count > 0)     cartesianCoordinates.RemoveAt(cartesianCoordinates.Count - 1);
            if (endoWristPositionsList.Count > 0)   endoWristPositionsList.RemoveAt(endoWristPositionsList.Count - 1);

            // Guardar la notificacion en la cola para mostrarla en el hilo principal
            pendingIkResult = null;
            hasNewIkResult = false;
            pendingNaNError = true;
            return;
        }

        // NO llamamos ProcesarJointPositions() directamente desde aquí.
        // Este método es invocado por el hilo secundario de ROS, y Unity no permite
        // lanzar corrutinas (StartCoroutine) ni modificar la UI desde hilos secundarios.
        // En cambio, guardamos el resultado en una variable y Update() lo procesa de forma segura.
        pendingIkResult = ikResult;
        hasNewIkResult = true;
    }

    // Procesa el resultado de la cinemática inversa (IK)
    private void ProcesarJointPositions(float[] jointPositions)
    {
        // Formatear la cadena con dos decimales para mostrar
        string jointPositionsStr = $"J1: {jointPositions[0]:F2} " +
                                   $"J2: {jointPositions[1]:F2} " +
                                   $"J3: {jointPositions[2]:F2} " +
                                   $"J4: {jointPositions[3]:F2} " +
                                   $"J5: {jointPositions[4]:F2} " +
                                   $"J6: {jointPositions[5]:F2}";

        // Guardar el comando de articulaciones (para uso interno)
        string jointCommand = $"JNTPoint({pointIndex},{string.Join(",", jointPositions)})";
        jointCommands.Add(jointCommand);
        awaitingInverseKinematics = false;
        pointIndex++;

        // Agregar la información de articulaciones a la última línea del display
        if (cartesianCoordinates.Count > 0)
        {
            cartesianCoordinates[cartesianCoordinates.Count - 1] += $"\n{jointPositionsStr}";
        }

        // Actualizar el URDF a través de JointStateWriters (convirtiendo a radianes)
        if (jointStateWriters != null && jointStateWriters.Length == 6)
        {
            for (int i = 0; i < jointStateWriters.Length; i++)
            {
                jointStateWriters[i].Write(jointPositions[i] * Mathf.Deg2Rad);
            }
        }
        else
        {
            Debug.LogError("jointStateWriters no está asignado correctamente (se esperan 6 articulaciones).");
        }

        // *** NUEVO ***
        // Guardar el resultado IK (posición de joints en grados) para usarlo como posición objetivo en modo delay
        ikCartesianPositionsList.Add(jointPositions);

        // Refrescar el display después de un pequeño delay
        StartCoroutine(UpdateCoordinatesDisplayAfterDelay());

        Debug.Log("IK procesada y agregada para URDF: " + jointCommand);
    }
    // Agrega la posición cartesiana tomada de los InputFields.
    // Se envía a la IK para obtener la solución (para URDF) y se crea el comando CARTPoint().
    // AGREGAMOS LOS MÉTODOS STOP/START UPDATING
    public void StopUpdating()
    {
        // En este script no se implementa interpolación; este método es dummy.
        Debug.Log("[CartesianStateWriterNew] StopUpdating() llamado.");
    }

    public void StartUpdating()
    {
        Debug.Log("[CartesianStateWriterNew] StartUpdating() llamado.");
    }
       private void AddCurrentPosition()
    {
        // Al presionar Add, se detiene la actualización de los joints para permitir la interpolación.
        if (jointPositionSubscriber != null)
            jointPositionSubscriber.StopUpdating();

        if (jointStateWriters != null && jointStateWriters.Length > 0)
        {
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = true;
                writer.UnlockWriting();
            }
        }

        if (!awaitingInverseKinematics)
        {
            float posX = ParseFloatInvariant(posXInputField.text);
            float posY = ParseFloatInvariant(posYInputField.text);
            float posZ = ParseFloatInvariant(posZInputField.text);
            float oriX = ParseFloatInvariant(oriXInputField.text);
            float oriY = ParseFloatInvariant(oriYInputField.text);
            float oriZ = ParseFloatInvariant(oriZInputField.text);

            float[] x_lim = { -830f, -320f };
            float[] y_lim = { -500f, 500f };
            float[] z_lim = { 0f, 720f };
            float[] rx_lim_1 = { -180f, -20f };
            float[] rx_lim_2 = { 20f, 180f };

            if (posX < x_lim[0] || posX > x_lim[1] ||
                posY < y_lim[0] || posY > y_lim[1] ||
                posZ < z_lim[0] || posZ > z_lim[1] ||
                !((oriX >= rx_lim_1[0] && oriX <= rx_lim_1[1]) || (oriX >= rx_lim_2[0] && oriX <= rx_lim_2[1])))
            {
                coordinatesDisplay.text = "Posiciones fuera del rango (Cartesiano)";
                return;
            }

            string cartesianCommand = FormCartesianCommand();
            float[] cartesianPositions = new float[] { posX, posY, posZ, oriX, oriY, oriZ };

            cartesianPositionsList.Add(cartesianPositions);
            cartesianCommandsList.Add(cartesianCommand);

            string cartPointCmd = string.Format(
                System.Globalization.CultureInfo.InvariantCulture,
                "CARTPoint({0},{1},{2},{3},{4},{5},{6})",
                pointIndex, posX, posY, posZ, oriX, oriY, oriZ);
            cartPointCommands.Add(cartPointCmd);

            float currentSpeed = speedSlider.value;
            speedList.Add(currentSpeed);

            float pointDelay = (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive()) ?
                               DelayModeController.Instance.GetDelayTime() : 0f;
            delayList.Add(pointDelay);

            // Obtener posiciones de la pinza
            if (endoWristSubscriber != null)
            {
                endoWristPositionsList.Add(endoWristSubscriber.GetLastKnownPositions());
            }
            else
            {
                endoWristPositionsList.Add(new float[4]); // Si no está asignado, ceros por defecto
            }

            float[] currentPinza = endoWristPositionsList[endoWristPositionsList.Count - 1];
            cartesianCoordinates.Add(FormatForDisplay(cartesianCommand, currentSpeed, currentPinza));
            UpdateCoordinatesDisplay();

            ros2CommandSender.SendCommandToTopic(ros2CommandSender.inverseInputTopic, cartesianCommand);
            awaitingInverseKinematics = true;
            Debug.Log("Enviando a IK (para URDF): " + cartesianCommand);
        }
        else
        {
            Debug.LogWarning("Esperando la respuesta de IK antes de agregar otra posición.");
        }
    }

    // Elimina la última posición añadida (se eliminan de todas las listas)
    private void RemoveLastPosition()
    {
        if (jointCommands.Count > 0)
            jointCommands.RemoveAt(jointCommands.Count - 1);

        if (cartesianCoordinates.Count > 0)
            cartesianCoordinates.RemoveAt(cartesianCoordinates.Count - 1);

        if (cartesianCommandsList.Count > 0)
            cartesianCommandsList.RemoveAt(cartesianCommandsList.Count - 1);

        if (cartesianPositionsList.Count > 0)
            cartesianPositionsList.RemoveAt(cartesianPositionsList.Count - 1);

        if (cartPointCommands.Count > 0)
            cartPointCommands.RemoveAt(cartPointCommands.Count - 1);

        if (speedList.Count > 0)
            speedList.RemoveAt(speedList.Count - 1);

        if (delayList.Count > 0)
            delayList.RemoveAt(delayList.Count - 1);

        // Eliminar también el último resultado IK (posición objetivo)
        if (ikCartesianPositionsList.Count > 0)
            ikCartesianPositionsList.RemoveAt(ikCartesianPositionsList.Count - 1);

        if (endoWristPositionsList.Count > 0)
            endoWristPositionsList.RemoveAt(endoWristPositionsList.Count - 1);

        if (pointIndex > 1)
            pointIndex--;

        UpdateCoordinatesDisplay();
        Debug.Log("Se eliminó la última posición añadida.");
    }

    // Corrutina para enviar los comandos al robot en lotes de 5.
    // En cada lote se envían primero los CARTPoint (re-indexados) y luego los MoveL.
    private IEnumerator SendCommands()
    {
        int totalPoints = cartPointCommands.Count;
        if (totalPoints == 0)
        {
            Debug.LogWarning("No hay comandos CARTPoint para enviar.");
            isManualEditing = false;
            isListening = true;
            yield break;
        }

        // Al presionar Send, se reactiva la suscripción para actualizar los joints normalmente.
        if (jointPositionSubscriber != null)
            jointPositionSubscriber.StartUpdating();

        // Desactivar la interpolación en los JointStateWriters.
        if (jointStateWriters != null)
        {
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = false;
            }
        }
           // Limpiar el display de coordenadas al iniciar el envío.
        coordinatesDisplay.text = "";
          // Enviar comandos de preparación antes de los JNTPoint()
        ros2CommandSender.SendCommand("DragTeachSwitch(0)");
        Debug.Log("Enviado: DragTeachSwitch(0)");
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("StopMotion()");
        Debug.Log("Enviado: StopMotion()");
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("ResetAllError()");
        Debug.Log("Enviado: ResetAllError()");
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("StartJOG(0,6,0,100)");
        Debug.Log("Enviado: StartJOG(0,6,0,100)");
        yield return new WaitForSeconds(0.5f);

        ros2CommandSender.SendCommand("StartJOG(0,6,1,100)");
        Debug.Log("Enviado: StartJOG(0,6,1,100)");
        yield return new WaitForSeconds(0.5f);

        // Añadir un breve delay antes de iniciar los comandos principales
        yield return new WaitForSeconds(1f);


        int batchSize = 5;
        int batches = Mathf.CeilToInt(cartPointCommands.Count / (float)batchSize);

        // Procesar lote a lote
        for (int b = 0; b < batches; b++)
        {
            int start = b * batchSize;
            int end = Mathf.Min((b + 1) * batchSize, totalPoints);

            // --- Enviar los CARTPoint del lote (re-indexados de 1 a batchSize) ---
            for (int i = start; i < end; i++)
            {
                int reIndex = i - start + 1;
                // Generar el comando CARTPoint usando los parámetros almacenados en cartesianPositionsList
                float[] pos = cartesianPositionsList[i];
                string cartCmd = $"CARTPoint({reIndex},{pos[0]},{pos[1]},{pos[2]},{pos[3]},{pos[4]},{pos[5]})";
                ros2CommandSender.SendCommand(cartCmd);
                Debug.Log("Enviado CARTPoint: " + cartCmd);

                // Publicar posición de la pinza para este punto en /endowrist_command
                if (endoWristPositionsList.Count > i)
                    ros2CommandSender.SendEndoWristCommand(endoWristPositionsList[i]);

                yield return new WaitForSeconds(0.05f);
            }

            // --- Enviar los comandos MoveL correspondientes al lote ---
            if (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive())
            {
                // Modo con delay: para cada punto del lote se espera que el robot alcance la posición objetivo (resultado IK)
                for (int i = start; i < end; i++)
                {
                    int reIndex = i - start + 1;
                    float currentSpeed = speedList[i];
                    string moveCmd = $"MoveL(CART{reIndex},{currentSpeed})";
                    ros2CommandSender.SendCommand(moveCmd);
                    Debug.Log("Enviado MoveL: " + moveCmd);

                    // Usar la posición objetivo (resultado IK) del punto
                    float[] targetJointPositions = ikCartesianPositionsList[i];
                    yield return StartCoroutine(WaitForRobotToReachPosition(targetJointPositions, 1f));

                    float pointDelay = delayList[i];
                    // Si es el primer punto del lote y no hay delay, se fuerza 1 segundo
                    if ((i == start) && pointDelay == 0)
                    {
                        pointDelay = 1f;
                    }
                    Debug.Log($"Posición alcanzada para CART{reIndex} (lote {b + 1}). Delay de {pointDelay} segundos.");
                    yield return new WaitForSeconds(pointDelay);
                }
            }
            else
            {
                // Modo sin delay (Spline) en lote:
                ros2CommandSender.SendCommand("SplineStart()");
                Debug.Log("Enviado: SplineStart() para lote " + (b + 1));
                yield return new WaitForSeconds(0.5f);

                for (int i = start; i < end; i++)
                {
                    int reIndex = i - start + 1;
                    float currentSpeed = speedList[i];
                    string splineCmd = $"MoveL(CART{reIndex},{currentSpeed})";
                    ros2CommandSender.SendCommand(splineCmd);
                    Debug.Log("Enviado: " + splineCmd);
                    yield return new WaitForSeconds(0.5f);
                }
                ros2CommandSender.SendCommand("SplineEnd()");
                Debug.Log("Enviado: SplineEnd() para lote " + (b + 1));
            }
        }

        // Limpiar listas y restablecer variables.
        jointCommands.Clear();
        cartesianCoordinates.Clear();
        cartesianCommandsList.Clear();
        cartesianPositionsList.Clear();
        cartPointCommands.Clear();
        speedList.Clear();
        delayList.Clear();
        ikCartesianPositionsList.Clear();
        endoWristPositionsList.Clear();
        pointIndex = 1;

        UpdateCoordinatesDisplay();
        isManualEditing = false;
        isListening = true;
    }

    // Corrutina: Refresca el display después de un pequeño delay
    private IEnumerator UpdateCoordinatesDisplayAfterDelay()
    {
        yield return new WaitForSeconds(0.5f);
        UpdateCoordinatesDisplay();
    }
    // Actualiza el texto del display con las posiciones guardadas
    private void UpdateCoordinatesDisplay()
    {
        coordinatesDisplay.text = "";
        for (int i = 0; i < cartesianCoordinates.Count; i++)
        {
            coordinatesDisplay.text += $"P {i + 1}: {cartesianCoordinates[i]}";
            if (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive())
            {
                coordinatesDisplay.text += $"  Delay: {delayList[i]}s";
            }
            coordinatesDisplay.text += "\n";
        }
    }

    // Formatea los valores de los InputFields en un string "x,y,z,rx,ry,rz"
    // Convierte cualquier string numerico (usando punto o coma) a float usando InvariantCulture
    private float ParseFloatInvariant(string input)
    {
        if (string.IsNullOrEmpty(input)) return 0f;
        input = input.Replace(',', '.');
        if (float.TryParse(input, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float result))
        {
            return result;
        }
        return 0f;
    }

    // Formatea los valores de los InputFields en un string "x,y,z,rx,ry,rz" usando siempre punto decimal
    private string FormCartesianCommand()
    {
        float posX = ParseFloatInvariant(posXInputField.text);
        float posY = ParseFloatInvariant(posYInputField.text);
        float posZ = ParseFloatInvariant(posZInputField.text);
        float oriX = ParseFloatInvariant(oriXInputField.text);
        float oriY = ParseFloatInvariant(oriYInputField.text);
        float oriZ = ParseFloatInvariant(oriZInputField.text);

        return string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "{0},{1},{2},{3},{4},{5}",
            posX, posY, posZ, oriX, oriY, oriZ);
    }
    // Formatea para mostrar en pantalla (ej.: "X:100,Y:200,Z:300,Rx:10,Ry:20,Rz:30, Speed: 10, Pinza: [0,0,0,0]")
    private string FormatForDisplay(string cartesianCommand, float speed, float[] pinza)
    {
        string[] parts = cartesianCommand.Split(',');
        string pinzaStr = pinza != null ? $", Pinza: [{pinza[0]:F2}, {pinza[1]:F2}, {pinza[2]:F2}, {pinza[3]:F2}]" : "";
        return $"X:{parts[0]},Y:{parts[1]},Z:{parts[2]},Rx:{parts[3]},Ry:{parts[4]},Rz:{parts[5]}, Speed: {speed}{pinzaStr}";
    }

    // Corrutina: Espera a que el robot alcance la posición objetivo (con tolerancia)
    // Se utiliza el JointPositionSubscriber para obtener la posición actual (en grados)
    // Timeout modificado a 600 segundos.
    private IEnumerator WaitForRobotToReachPosition(float[] targetPositions, float tolerance)
    {
        bool positionReached = false;
        float timeout = 600f;  // Timeout aumentado a 600 segundos
        float elapsedTime = 0f;

        while (!positionReached && elapsedTime < timeout)
        {
            // Obtener la posición actual de las articulaciones usando JointPositionSubscriber
            float[] currentPos = jointPositionSubscriber.GetLastKnownPositions();
            if (currentPos == null || currentPos.Length != targetPositions.Length)
            {
                Debug.LogWarning("No se pudieron obtener las posiciones articulares actuales del robot.");
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
                continue;
            }

            positionReached = true;
            for (int i = 0; i < targetPositions.Length; i++)
            {
                float difference = Mathf.Abs(currentPos[i] - targetPositions[i]);
                if (difference > tolerance)
                {
                    positionReached = false;
                    break;
                }
            }

            if (!positionReached)
            {
                Debug.Log($"Esperando posición. Tiempo transcurrido: {elapsedTime}s");
                Debug.Log($"Posición actual: {string.Join(",", currentPos)}");
                Debug.Log($"Posición objetivo (IK): {string.Join(",", targetPositions)}");
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
            }
            else
            {
                Debug.Log("Robot ha alcanzado la posición objetivo.");
            }
        }

        if (elapsedTime >= timeout)
        {
            Debug.LogWarning("Tiempo de espera excedido. Continuando con el siguiente comando.");
        }
    }
    // Guarda en un archivo .txt las posiciones cartesianas, velocidad y delay.
    // Se guarda únicamente: los 6 valores de posición, seguidos de velocidad y delay, separados por comas.
    // Ejemplo de línea guardada: -497,-102,466,180,0,0,17,0
    private void GuardarEnTxt()
    {
        string filePath = Path.Combine(savePath, "cartesian_positions.txt");
        using (StreamWriter writer = new StreamWriter(filePath))
        {
            // Escribir la primera línea con la palabra "cartesiano"
            writer.WriteLine("cartesiano");

            // Usar cartesianPositionsList para extraer los 6 valores (ya que estos vienen de los inputs)
            // y luego agregar velocidad, delay, y los 4 valores de la pinza.
            for (int i = 0; i < cartesianPositionsList.Count; i++)
            {
                float[] pos = cartesianPositionsList[i];
                float[] pinza = (endoWristPositionsList.Count > i) ? endoWristPositionsList[i] : new float[4];
                string line = string.Format(
                    System.Globalization.CultureInfo.InvariantCulture,
                    "{0},{1},{2},{3},{4},{5},{6},{7},{8:F6},{9:F6},{10:F6},{11:F6}",
                    pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
                    speedList[i], delayList[i],
                    pinza[0], pinza[1], pinza[2], pinza[3]);
                writer.WriteLine(line);
            }
        }
        Debug.Log($"Archivo guardado en: {filePath}");
    }

    // Métodos de control de escucha de los InputFields
    public void StopListening()
    {
        isListening = false;
        Debug.Log("Se detuvo la escucha de posiciones cartesianas.");
    }

    public void StartListening()
    {
        isListening = true;
        Debug.Log("Se inició la escucha de posiciones cartesianas.");
    }

    public bool IsListening()
    {
        return isListening;
    }
    // Métodos para el manejo del Slider/InputField de velocidad
    private void OnSpeedSliderValueChanged(float value)
    {
        speedValueInput.text = value.ToString();
    }

    private void OnSpeedInputFieldValueChanged(string value)
    {
        if (float.TryParse(value, out float result))
        {
            result = Mathf.Clamp(result, speedSlider.minValue, speedSlider.maxValue);
            speedSlider.value = result;
        }
    }

    private void IncreaseSpeedValue()
    {
        float newSpeed = speedSlider.value + 1f;
        newSpeed = Mathf.Clamp(newSpeed, speedSlider.minValue, speedSlider.maxValue);
        speedSlider.value = newSpeed;
        speedValueInput.text = newSpeed.ToString();
        Debug.Log($"Velocidad incrementada a: {newSpeed}");
    }

    private void DecreaseSpeedValue()
    {
        float newSpeed = speedSlider.value - 1f;
        newSpeed = Mathf.Clamp(newSpeed, speedSlider.minValue, speedSlider.maxValue);
        speedSlider.value = newSpeed;
        speedValueInput.text = newSpeed.ToString();
        Debug.Log($"Velocidad decrementada a: {newSpeed}");
    }

    // Corrutina: Actualiza continuamente los InputFields con la posición actual del robot
    // (Se sigue utilizando cartesianPositionSubscriber para actualizar la UI)
    private IEnumerator UpdateInputFieldsContinuously()
    {
        while (true)
        {
            if (isListening)
            {
                if (!posXInputField.isFocused && !posYInputField.isFocused && !posZInputField.isFocused &&
                    !oriXInputField.isFocused && !oriYInputField.isFocused && !oriZInputField.isFocused)
                {
                    float[] positions = cartesianPositionSubscriber.GetLastKnownCartesianPositions();
                    posXInputField.text = positions[0].ToString("F2");
                    posYInputField.text = positions[1].ToString("F2");
                    posZInputField.text = positions[2].ToString("F2");
                    oriXInputField.text = positions[3].ToString("F2");
                    oriYInputField.text = positions[4].ToString("F2");
                    oriZInputField.text = positions[5].ToString("F2");

                    currentPositions = positions;
                }
            }
            yield return new WaitForSeconds(0.5f);
        }
    }
}

