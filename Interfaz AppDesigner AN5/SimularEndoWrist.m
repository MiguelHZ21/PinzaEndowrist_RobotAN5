% =========================================================================
% Script de MATLAB para simular la pinza EndoWrist en ROS 2
% Publica el estado cíclico en el tópico /endowrist a 10 Hz
% Autores: Adaptado para Interfaz Unity AN5
% =========================================================================

% Limpiar espacio de trabajo y consola
clear;
clc;

% 1. Crear el nodo de ROS 2 en MATLAB
node = ros2node("endowrist_cyclic_pub");

% 2. Configurar la política de QoS (Best Effort) para coincidir con Unity
qos = ros2qos;
qos.Reliability = "besteffort";
qos.Depth = 10;

% 3. Crear el publicador en el tópico /endowrist
pub = ros2publisher(node, "/endowrist", "sensor_msgs/JointState", qos);

% 4. Instanciar el tipo de mensaje JointState y asignar nombres de articulaciones
msg = ros2message("sensor_msgs/JointState");
msg.name = {'shaft', 'wrist', 'jaw_dx', 'jaw_sx'};

% 5. Configurar la frecuencia de envío a 10 Hz
rate = ros2rate(node, 10);

fprintf('Publicando en /endowrist a 10 Hz desde MATLAB...\n');
fprintf('Presiona Ctrl+C en la ventana de comandos de MATLAB para detener.\n');

% Registrar el tiempo inicial
t0 = tic;

% Bucle infinito de simulación
while true
    % Calcular tiempo transcurrido en segundos
    t = toc(t0);

    % Calcular las trayectorias sinusoidales
    pos_shaft  = 0.8 * sin(0.5 * t);          % rotación lenta
    pos_wrist  = 0.6 * sin(0.8 * t + 1.0);    % muñeca
    pos_jaw_dx = 0.4 * sin(1.2 * t);          % mandíbula derecha
    pos_jaw_sx = 0.4 * sin(1.2 * t + 3.14);   % mandíbula izquierda (en contrafase)

    % Asignar las posiciones calculadas al mensaje (en forma de vector columna)
    msg.position = [pos_shaft; pos_wrist; pos_jaw_dx; pos_jaw_sx];

    % Asignar frame id en el header
    msg.header.frame_id = 'endowrist';

    % Enviar el mensaje de estado a ROS 2 / Unity
    send(pub, msg);

    % Esperar al siguiente intervalo (100 ms)
    waitfor(rate);
end
