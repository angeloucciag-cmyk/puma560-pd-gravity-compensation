clear,clc,close all
startup_rvc;
mdl_puma560;
%aggiunta massa puntiforme all'end effector
m_obj = 1.5;
p560.links(6).m = p560.links(6).m + m_obj;
%% yoshikawa index
[x_p_d,best_q,T_d] = bestPoint(p560,qs);
z_des = rad2deg(tr2eul(T_d));
fprintf('ZYZ Desiderati -> Alpha: %.2f°, Beta: %.2f°, Gamma: %.2f°\n', z_des);
%%  Parametri di simulazione
dt = 0.001;
t_final = 5;
t_vec = 0:dt:t_final;
tau_sensor = 160e-6; %trasduttore di posizione  (circa 1000Hz)
% Parametri del disturbo (durata definita)
t_inizio_disturbo = 1.0;  
durata_disturbo = 1.5;    
t_fine_disturbo = t_inizio_disturbo + durata_disturbo; 
W_ext = [0; 0; -40; 0; 0; 0];      % Forza esterna di 90N verso il basso
% Condizioni iniziali
q = best_q; 
q_meas = best_q;   % Inizializzazione misura (uguale al reale all'inizio)
qdot = zeros(1,6);
error_norm = zeros(length(t_vec), 1);
q_history = zeros(length(t_vec), 6);
q_meas_history = zeros(length(t_vec), 6);
wn = 8;  %puls naturale
zeta = 0.9; %coeff smorz
%J_geom = p560.jacob0(q);
%[Kp,Kd] = pd_gravity_gains(p560,q,wn,zeta,J_geom); 
%% Simulazione Unica: Mantenimento + Disturbo + Ritorno
%preallocazione vettore segnali di controllo
u_h = zeros(length(t_vec),6);
u_c = zeros(length(t_vec),6);
I_history = zeros(length(t_vec),6);
zyz_history = zeros(length(t_vec), 3); % Roll, Pitch, Yaw [rad]
figure('Name', 'Simulazione PUMA 560: PD + Gravità + Disturbo');
for i = 1:length(t_vec)
    % 1. SIMULAZIONE MISURA (Trasduttore)
    q_meas = trasduttore_giunti(q, q_meas, tau_sensor, dt);
    
    % Salvataggio per analisi post-simulazione
    q_history(i,:) = q;
    q_meas_history(i,:) = q_meas;
   
    % 2. Cinematica 
    T_curr = p560.fkine(q_meas);
    x_p_curr = T_curr.t;
    x_p_tilde = x_p_d- x_p_curr;
    error_norm(i) = norm(x_p_tilde);
    zyz_curr = tr2eul(T_curr);
    zyz_history(i,:) = zyz_curr;
    Ja =jacobanalitic(p560,T_curr,q);
   
    [Kp,Kd] = pd_gravity_gains(p560,q_meas,wn,zeta,Ja); 
    % 3.azione di controllo
    u_control = pd_gravity_controller(p560, q_meas, qdot, T_d, T_curr, Kp, Kd,Ja);
    
    % 4. Applicazione del Disturbo Temporaneo 
    tau_ext = zeros(6, 1);
    J_curr = p560.jacob0(q);   % Se T_d è un oggetto SE3, lo convertiamo in matrice 4x4 per sicurezza
    if isa(T_d, 'SE3')
        T_d_mat = T_d.T;
    else
        T_d_mat = T_d;
    end
    if isa(T_curr, 'SE3')
        T_c_mat = T_curr.T;
    else
        T_c_mat = T_curr;
    end
   
    
    if t_vec(i) >= t_inizio_disturbo && t_vec(i) < t_fine_disturbo
        % La forza esterna agisce solo nell'intervallo definito
        tau_ext = J_curr' * W_ext; 
    end
    
    % % Coppia totale applicata al manipolatore 
    u_tot = u_control + tau_ext;
   
    u_h(i,:) = u_tot';
    u_c(i,:) = u_control';
    % 5. Simulazione della dinamica del manipolatore 
    qddot = p560.accel(q_meas, qdot, u_tot');
    
    % Integrazione numerica (Eulero)
    qdot = qdot + qddot' * dt;
    q = q + qdot * dt;
    %salvataggio corrente assorbita dai motori
    Kt = 0.223;
    N = [62.6 53.7 76.7 71.9 76.0 107.3];  %valori reali presio da NASA  %rapporti trasmissione
    tau_joint = u_tot;
    I = tau_joint ./ (Kt .* N');
    I_history(i,:) = I';
    
    % Visualizzazione dinamica
    if mod(i, 30) == 0
        p560.plot(q);
        if t_vec(i) >= t_inizio_disturbo && t_vec(i) < t_fine_disturbo
            title(sprintf('DISTURBO ATTIVO - Tempo: %.2f s', t_vec(i)));
        else
            title(sprintf('Mantenimento/Ritorno - Tempo: %.2f s', t_vec(i)));
        end
    end
end
%% Plot finale dell'errore
figure;
plot(t_vec, error_norm, 'LineWidth', 2, 'Color', 'b');
hold on;
% Evidenzia l'intervallo del disturbo
fill([t_inizio_disturbo t_fine_disturbo t_fine_disturbo t_inizio_disturbo], ...
     [0 0 max(error_norm) max(error_norm)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xline(t_inizio_disturbo, '--r', 'Inizio Disturbo');
xline(t_fine_disturbo, '--g', 'Fine Disturbo');
title('Analisi dell''Errore: Effetto del Disturbo e Ritorno in Posizione');
xlabel('Tempo [s]'); ylabel('x tilde [m]');
grid on;
%% --- ANALISI ORIENTAMENTO END-EFFECTOR (CON GESTIONE PERIODICITÀ) ---
figure('Name', 'Orientamento End-Effector (ZYZ)', 'NumberTitle', 'off');
% Estrazione angoli desiderati (target) per confronto
zyz_d = tr2eul(T_d); 
% Applicazione unwrap: corregge i salti di +/- 2*pi
% Specifichiamo la dimensione 1 per operare lungo il tempo
zyz_unwrapped = unwrap(zyz_history, [], 1); 
zyz_labels = {'Precessione (\alpha - Z)', 'Nutazione (\beta - Y)', 'Rotazione (\gamma - Z)'};
colors = [0.8500 0.3250 0.0980; 0 0.4470 0.7410; 0.4660 0.6740 0.1880];
for j = 1:3
    subplot(3, 1, j);
    % Plot dell'orientamento reale
    plot(t_vec, rad2deg(zyz_unwrapped(:,j)), 'LineWidth', 2, 'Color', colors(j,:));
    hold on;
    
    % Linea di riferimento desiderata
    yline(rad2deg(zyz_d(j)), 'k--', 'Target', 'LineWidth', 1.1);
    
    % Evidenzia area del disturbo
    y_lims = ylim;
    fill([t_inizio_disturbo t_fine_disturbo t_fine_disturbo t_inizio_disturbo], ...
         [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    
    grid on;
    ylabel([zyz_labels{j}, ' [deg]']);
    if j == 1, title('Evoluzione Angoli ZYZ'); end
end
xlabel('Tempo [s]');
%% --- ANALISI ERRORE ANGOLARE (ZYZ) ---
figure('Name', 'Errore Orientamento End-Effector (ZYZ)', 'NumberTitle', 'off');

% 1. Calcolo dell'errore istantaneo (differenza tra target e storia)
% zyz_d è il vettore 1x3 calcolato all'inizio
e_zyz = zeros(length(t_vec), 3);
for k = 1:length(t_vec)
    % Differenza angolare corretta per la periodicità
    diff_raw = zyz_d - zyz_history(k,:);
    e_zyz(k,:) = rad2deg(atan2(sin(diff_raw), cos(diff_raw)));
end

zyz_labels = {'Errore Precessione (\alpha)', 'Errore Nutazione (\beta)', 'Errore Rotazione (\gamma)'};

for j = 1:3
    subplot(3, 1, j);
    plot(t_vec, e_zyz(:,j), 'LineWidth', 2, 'Color', [0.6350 0.0780 0.1840]);
    hold on;
    
    % Linea dello zero (errore nullo)
    yline(0, 'k--', 'LineWidth', 1);
    
    % Evidenzia area del disturbo
    y_lims = [min(e_zyz(:,j))-2, max(e_zyz(:,j))+2];
    fill([t_inizio_disturbo t_fine_disturbo t_fine_disturbo t_inizio_disturbo], ...
         [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    
    grid on;
    ylabel([zyz_labels{j}, ' [deg]']);
    if j == 1, title('Analisi dell''Errore Statico e Dinamico (ZYZ)'); end
end
xlabel('Tempo [s]');
%% --- CONFRONTO MISURA REALE VS MISURATO (TUTTI I GIUNTI) ---
figure('Name', 'CONFRONTO MISURA REALE VS MISURATO', 'NumberTitle', 'off');
for j = 1:6
    subplot(3, 2, j); % Griglia 3x2
    
     % Plot posizione misurata (filtrata dal trasduttore)
    plot(t_vec, q_meas_history(:,j), 'r--', 'LineWidth', 1.2);
    hold on;
    
    
    % Plot posizione reale
    plot(t_vec, q_history(:,j), 'b', 'LineWidth', 1.5);
    grid on;
    title(['Giunto ', num2str(j)]);
    ylabel('Angolo [rad]');
        
    if j == 1
        legend('Reale', 'Misurato', 'Location', 'best');
    end
   
    if j >= 5
        xlabel('Tempo [s]');
    end
end
sgtitle('Confronto Misura Reale vs Misura Filtrata + rumore (Tutti i Giunti)'); % Migliora la spaziatura tra i grafici
%% Corrente assorbita dai motori
figure('Name','Corrente Motori')
I_max =[6.99; 15.55; 5.23; 1.51; 1.18; 0.89];  %valori reali preesi da NASA 
for j=1:6
subplot(3,2,j)
plot(t_vec,I_history(:,j),'LineWidth',1.5)
hold on
yline(I_max(j),'r--')
yline(-I_max(j),'r--')
grid on
title(['Giunto ',num2str(j)])
ylabel('Corrente [A]')
if j>=5
xlabel('Tempo [s]')
end
end
sgtitle('Corrente assorbita dai motori')
%% CONTROLLER
  function u_control = pd_gravity_controller(robot, q, qdot, T_d, T_curr, Kp, Kd, Ja)
    T_d_mat = T_d;
    T_c_mat = T_curr.T;

    % Errore di Posizione
    e_pos = T_d_mat(1:3, 4) - T_c_mat(1:3, 4);
    
    % Errore di Orientamento ZYZ
    zyz_d = tr2eul(T_d_mat)'; 
    zyz_c = tr2eul(T_c_mat)';
    e_ori = zyz_d - zyz_c;
    
    % Correzione periodicità (percorso più breve)
    e_ori = atan2(sin(e_ori), cos(e_ori));
    
    % Vettore errore totale
    xtilde = [e_pos; e_ori];
    
    % Compensazione Gravità
    gq = robot.gravload(q)';
   
    % Legge di controllo
    u_control = gq + Ja' * (Kp * xtilde - Kd * (Ja * qdot'));
    
    % Saturazione
    tau_limit = [97.6; 186.4; 89.4; 24.2; 20.1; 21.3];
    u_control = max(min(u_control, tau_limit), -tau_limit);
end
%% SIMULAZIONE TRASDUTTORE REALE
function q_next = trasduttore_giunti(q_real, q_prev, tau, dt)
%metodo differenze all'indietro (rispetto ad eulero in avanti permette tau
%piccoli)
    % Simulazione del rumore bianco sul sensore
    % randn genera numeri con distribuzione normale (media 0, varianza 1)
    sigma = 0.001;
    rumore = sigma * randn(size(q_real));
    q_real = q_real + rumore;
    % Coefficiente di filtro (Alpha)
    % Se dt è piccolo rispetto a tau, alpha è piccolo (molto filtraggio)
    % Se dt è grande, alpha tende a 1 (nessun filtraggio)
    alpha = dt / (tau + dt);
    
    % Equazione del filtro:
    q_next = alpha * q_real + (1 - alpha) * q_prev;
end
%% ANALISI RAGGIUNGIBILITÀ
function [x_d, best_q, best_T] = bestPoint(p560, qs)
    % Area di ricerca (espandila se necessario)
    x_range = 0.3:0.05:0.7;   
    y_range = -0.4:0.05:0.4;  
    z_range = -0.2:0.1:0.5;  
    % Inizializzazione con valori di default per evitare l'errore di output
    best_w = -1;
    best_q = qs;
    best_T = p560.fkine(qs).T; 
    x_d = best_T(1:3, 4);
    
    tau_limit = [97.6; 186.4; 89.4; 24.2; 20.1; 21.3];
    found_any = false;
    for x = x_range
        for y = y_range
            for z = z_range
                T_target = transl(x,y,z) * trotx(pi);
                q_sol = p560.ikine6s(T_target, 'up');
                
                if ~isempty(q_sol)
                    % Calcolo coppie necessarie per contrastare la gravità
                    tg = abs(p560.gravload(q_sol)'); 
                    
                    % Vincolo di sicurezza: usiamo l'80% per essere meno restrittivi
                    if all(tg < 0.8 * tau_limit)
                        w = p560.maniplty(q_sol);
                        
                        if w > best_w
                            best_w = w;
                            best_q = q_sol;
                            best_T = T_target;
                            x_d = T_target(1:3, 4);
                            found_any = true;
                        end
                    end
                end
            end
        end
    end
    if ~found_any
        warning('Nessun punto trovato con i vincoli di coppia. Uso la posa iniziale.');
    else
        fprintf('Punto ottimale trovato! W = %.4f\n', best_w);
    end
end
%% CRITERIO TARATURA P e D
function [Kp, Kd] = pd_gravity_gains(robot, q, wn, zeta, J)
    M = robot.inertia(q);
    % Lambda = inv(J * inv(M) * J' + eye(6)*1e-4); % Versione precedente
    
    % Versione più robusta:
    A = J * (M \ J');
    % Aggiungiamo un piccolo offset sulla diagonale per limitare i guadagni massimi
    Lambda = inv(A + eye(6) * 0.005); 
    
    Kp = Lambda * wn^2;
    Kd = 2 * zeta * Lambda * wn;
end
function Ja = jacobanalitic(robot, T_curr, q)
    zyz = tr2eul(T_curr); 
    phi = zyz(1);   
    theta = zyz(2); 

    E = [ 0, -sin(phi),  cos(phi)*sin(theta);
          0,  cos(phi),  sin(phi)*sin(theta);
          1,  0,         cos(theta) ];

    % Damped Least Squares Inverse (Metodo di Levenberg-Marquardt)
    % Impedisce a singole componenti di esplodere
    epsilon = 0.05; 
    invE = E' * inv(E * E' + epsilon^2 * eye(3));
    
    Ta = [eye(3),       zeros(3,3);
          zeros(3,3),   invE];
    Ja = Ta * robot.jacob0(q);
end