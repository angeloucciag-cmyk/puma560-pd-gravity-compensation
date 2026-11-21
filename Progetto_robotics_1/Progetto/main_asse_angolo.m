clear,clc,close all
startup_rvc;
mdl_puma560;
%aggiunta massa puntiforme all'end effector
m_obj = 2.5;
p560.links(6).m = p560.links(6).m + m_obj;
%% yoshikawa index

[x_p_d,best_q,T_d] = bestPoint(p560,qs);

%%  Parametri di simulazione
dt = 0.005;
t_final = 6;
t_vec = 0:dt:t_final;
tau_sensor = 160e-6; %trasduttore di posizione  (circa 1000Hz)

% Parametri del disturbo (durata definita)
t_inizio_disturbo = 2.0;  
durata_disturbo = 1.5;    
t_fine_disturbo = t_inizio_disturbo + durata_disturbo; 
W_ext = [0; 0; -90; 0; 0; 0];      % Forza esterna di 90N verso il basso

% Condizioni iniziali
q = best_q; 
q_meas = best_q;   % Inizializzazione misura (uguale al reale all'inizio)
qdot = zeros(1,6);
error_norm = zeros(length(t_vec), 1);
q_history = zeros(length(t_vec), 6);
q_meas_history = zeros(length(t_vec), 6);
wn = 8;  %puls naturale
zeta = 0.9; %coeff smorz
[Kp,Kd] = pd_gravity_gains(p560,q,wn,zeta); 

%% Simulazione Unica: Mantenimento + Disturbo + Ritorno

%preallocazione vettore segnali di controllo
u_h = zeros(length(t_vec),6);
u_c = zeros(length(t_vec),6);
I_history = zeros(length(t_vec),6);
eori_history = zeros(length(t_vec), 3); % asse-angolo

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

    % ===== ERRORE ORIENTAMENTO (ASSE-ANGOLO) =====
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

    R_d = T_d_mat(1:3,1:3);
    R_c = T_c_mat(1:3,1:3);

    R_err = R_d * R_c';

    cos_theta = (trace(R_err)-1)/2;
    cos_theta = max(min(cos_theta,1),-1);
    theta = acos(cos_theta);

    if theta < 1e-6
        e_ori_plot = [0;0;0];
    else
        e_ori_plot = theta/(2*sin(theta)) * [
            R_err(3,2) - R_err(2,3);
            R_err(1,3) - R_err(3,1);
            R_err(2,1) - R_err(1,2)
        ];
    end

    eori_history(i,:) = e_ori_plot';
  
    % 3.azione di controllo
    u_control = pd_gravity_controller(p560, q_meas, qdot, T_d, T_curr, Kp, Kd);
    
    % 4. Applicazione del Disturbo Temporaneo 
    tau_ext = zeros(6, 1);
    J_curr = p560.jacob0(q);

    if t_vec(i) >= t_inizio_disturbo && t_vec(i) < t_fine_disturbo
        tau_ext = J_curr' * W_ext; 
    end
    
    % Coppia totale applicata al manipolatore 
    u_tot = u_control + tau_ext;
   
    u_h(i,:) = u_tot';
    u_c(i,:) = u_control';

    % 5. Simulazione della dinamica del manipolatore 
    qddot = p560.accel(q, qdot, u_tot');
    
    % Integrazione numerica (Eulero)
    qdot = qdot + qddot' * dt;
    q = q + qdot * dt;

    %salvataggio corrente assorbita dai motori
    Kt = 0.223;
    N = [62.6 53.7 76.7 71.9 76.0 107.3];
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

%% Plot errore posizione
figure;
plot(t_vec, error_norm, 'LineWidth', 2, 'Color', 'b');
hold on;
fill([t_inizio_disturbo t_fine_disturbo t_fine_disturbo t_inizio_disturbo], ...
     [0 0 max(error_norm) max(error_norm)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xline(t_inizio_disturbo, '--r', 'Inizio Disturbo');
xline(t_fine_disturbo, '--g', 'Fine Disturbo');
title('Analisi dell''Errore: Effetto del Disturbo e Ritorno in Posizione');
xlabel('Tempo [s]'); ylabel('x tilde [m]');
grid on;

%% --- ANALISI ORIENTAMENTO (ASSE-ANGOLO CON UNWRAP) ---
figure('Name', 'Errore Orientamento End-Effector', 'NumberTitle', 'off');
eori_labels = {'Ex (X)', 'Ey (Y)', 'Ez (Z)'};

eori_unwrapped = unwrap(eori_history); 

for j = 1:3
    subplot(3, 1, j);
    plot(t_vec, rad2deg(eori_unwrapped(:,j)), 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]);
    hold on;
    
    y_limits = [min(rad2deg(eori_unwrapped(:,j)))-5, max(rad2deg(eori_unwrapped(:,j)))+5];
    fill([t_inizio_disturbo t_fine_disturbo t_fine_disturbo t_inizio_disturbo], ...
         [y_limits(1) y_limits(1) y_limits(2) y_limits(2)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    
    grid on;
    ylabel([eori_labels{j}, ' [deg]']);
    if j == 1, title('Errore Orientamento (Asse-Angolo con unwrap)'); end
    if j == 3, xlabel('Tempo [s]'); end
end
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
function u_control = pd_gravity_controller(robot, q, qdot, T_d, T_curr, Kp, Kd)
    
    T_d_mat = T_d;
    T_c_mat = T_curr.T;
    % Posizione
    pos_d = T_d_mat(1:3, 4);
    pos_c = T_c_mat(1:3, 4);
    e_pos = pos_d - pos_c;
    
    % ===== ORIENTAMENTO ASSE-ANGOLO =====
    R_d = T_d_mat(1:3,1:3);
    R_c = T_c_mat(1:3,1:3);

    R_err = R_d * R_c';

    cos_theta = (trace(R_err)-1)/2;
    cos_theta = max(min(cos_theta,1),-1);
    theta = acos(cos_theta);

    if theta < 1e-6
        e_ori = [0;0;0];
    else
        e_ori = theta/(2*sin(theta)) * [
            R_err(3,2) - R_err(2,3);
            R_err(1,3) - R_err(3,1);
            R_err(2,1) - R_err(1,2)
        ];
    end
    
    xtilde = [e_pos; e_ori];
    
    J = robot.jacob0(q);
    gq = robot.gravload(q)';
   
    u_control = gq + J' * (Kp * xtilde - Kd * (J * qdot'));
    %saturazione
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
%% CRITERIO TARATURA P e D
function [Kp,Kd] = pd_gravity_gains(robot,q,wn,zeta)

J = robot.jacob0(q);

M = robot.inertia(q);

% inerzia nello spazio operativo
Lambda = inv(J * inv(M) * J');

Kp = Lambda * wn^2;
Kd = 2*zeta*Lambda*wn;

end
%% ANALISI RAGGIUNGIBILITÀ
function [x_d,best_q,best_T] = bestPoint(p560,qz)

% Area di ricerca
x_range = 0.4:0.05:0.6;   
y_range = -0.3:0.05:0.3;  
z_range = -0.1:0.05:0.3;  

best_w = -1;               
best_q = qz;              
best_T = eye(4);          

% vettori per salvare workspace
X = [];
Y = [];
Z = [];
W = [];

% Loop di ottimizzazione
for x = x_range
    for y = y_range
        for z = z_range
            
            % matrice posa
            T_target = transl(x,y,z) * trotx(pi);
            
            % cinematica inversa
            q_sol = p560.ikine6s(T_target,'up');
            
            if ~isempty(q_sol)
                
                % indice Yoshikawa
                w = p560.maniplty(q_sol);
                
                % salvataggio punti
                X(end+1) = x;
                Y(end+1) = y;
                Z(end+1) = z;
                W(end+1) = w;
                
                if w > best_w
                    best_w = w;
                    best_q = q_sol;
                    best_T = T_target;
                end
                
            end
        end
    end
end

pos_x = best_T(1,4); 
pos_y = best_T(2,4); 
pos_z = best_T(3,4);
x_d = [pos_x,pos_y,pos_z]';
fprintf('--------------------------------------------------\n');
fprintf('Punto di Pick Ottimale trovato:\n');
fprintf('X: %.2f, Y: %.2f, Z: %.2f\n', pos_x, pos_y, pos_z);
fprintf('Indice di Yoshikawa (w): %.4f\n', best_w);
fprintf('--------------------------------------------------\n');

%% Plot workspace manipulability

figure(1)

% normalizzazione indice manipolabilità
Wn = (W - min(W)) / (max(W) - min(W));

% nuvola di punti sfumata
scatter3(X,Y,Z,8,Wn,'filled')

hold on

% punto migliore evidenziato
scatter3(pos_x,pos_y,pos_z,120,'r','filled')

colormap(turbo)
colorbar

xlabel('X')
ylabel('Y')
zlabel('Z')

title('Workspace Manipolabilità (Indice Yoshikawa)')
grid on
view(3)
axis equal

% plot robot
p560.plot(best_q,'workspace',[-0.8 0.8 -0.8 0.8 -0.2 1.2])
zlim([-1.2 1.2])
xlim([-1.2 1.2])

end
