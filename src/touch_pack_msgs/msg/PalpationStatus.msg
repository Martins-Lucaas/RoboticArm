# Status do experimento de palpação (/palpation/status, ~10 Hz).

string phase                # IDLE|HOME|DESCENDING|HOLD|SLIDING|RETRACT|DONE|ABORTED
int32 cycle                 # ciclo atual (1-based; 0 = fora de experimento)
int32 cycles_total          # total de ciclos do experimento
float32 target_depth_mm
float32 target_force_n
float32 force_net_n         # força tare-compensada (N, + compressão)
float32 speed_mms
bool paused                 # true enquanto o experimento está pausado
