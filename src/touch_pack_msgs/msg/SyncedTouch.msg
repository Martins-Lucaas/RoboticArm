# Par sincronizado célula de carga (ESP32) × touch sensor (STM32).
# Publicado em /touch_sync/data pelo force_sync_node a 50 Hz, somente
# quando AMBAS as fontes têm amostra fresca (< SYNC_MAX_AGE_S).
#
# header.stamp é o instante do pareamento no relógio do PC do ROS;
# *_age_ms diz quanto tempo antes disso cada amostra chegou via UDP —
# permite avaliar a qualidade da sincronização a posteriori.

std_msgs/Header header
float32 load_cell_n       # força calibrada da célula de carga (N, + compressão)
float32 touch_value       # leitura do touch sensor (unidade enviada pelo STM32)
float32 load_cell_age_ms  # idade da amostra da célula no pareamento (ms)
float32 touch_age_ms      # idade da amostra do touch no pareamento (ms)
