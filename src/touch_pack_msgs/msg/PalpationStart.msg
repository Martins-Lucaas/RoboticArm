# Comando de início do experimento de palpação (/palpation/start).
# Substitui o JSON em std_msgs/String: schema tipado, introspectável
# e legível via `ros2 topic echo`.

float32 speed_mms           # velocidade de deslizamento (mm/s)
float32 depth_mm            # curso máximo de descida — segurança (mm)
float32 force_n             # setpoint do PID de força (N, compressão)
float32 kp                  # ganhos do PID de força, em (m/s)/N …
float32 ki
float32 kd
float32 slide_dist_mm       # distância lateral do SLIDING (mm)
float32 approach_speed_mms  # velocidade de aproximação CONTACT/RETRACT (mm/s)
string slide_dir            # '+X' | '-X' | '+Y' | '-Y'
int32 repeats               # ciclos automáticos do experimento (>= 1)
float32 speed_factor_pct    # SpeedFactor do braço real (%)
float64[6] home_deg         # home do braço em graus, ordem joint1..joint6

# Estabilização do setpoint no HOLD — 0.0 = usar o default do explorer.
float32 hold_tol_n          # tolerância absoluta da banda (N)
float32 hold_stable_s       # janela contínua dentro da banda (s)
float32 hold_timeout_s      # teto de espera pela estabilização (s)

# Modo de palpação:
#   'SLIDE' = deslizamento (descida → hold → deslizamento lateral → recuo)
#   'TOUCH' = toque (descida → hold com força controlada → recuo, sem deslizar)
# Vazio = 'SLIDE' (compatibilidade). Em TOUCH, `repeats` = quantidade de toques.
string mode
