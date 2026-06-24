"""
constants.py — Constantes compartilhadas do touch_pack.

Fonte única para valores que antes viviam duplicados na GUI, no explorer,
no logger e nos nós auxiliares ("atualizei um lado só" era uma classe de
bug real: o limite de 15 N e a pose POINTING existiam em 2 cópias).

Regra: valores usados por MAIS de um módulo moram aqui; valores privados
de um único módulo ficam nele.
"""
from __future__ import annotations

import math
import os

# ── Cadeia do braço CR10 (convenção URDF) ────────────────────────────────────
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Pose "apontar para a mesa": braço vertical com o efetuador perpendicular.
# É a home default da GUI e o seed POINTING do explorer.
POINTING_SEED_DEG = {'joint1': 0.0, 'joint2': 0.0, 'joint3': -90.0,
                     'joint4': 0.0, 'joint5': 90.0, 'joint6': 0.0}

# ── Mão COVVI — juntas primárias ─────────────────────────────────────────────
HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Pose POINTING (palpação com o Index estendido).
HAND_POINT_DEG = {'Thumb': 30.0, 'Index': 0.0, 'Middle': 80.0,
                  'Ring': 80.0, 'Little': 80.0, 'Rotate': 0.0}
HAND_POINTING_RAD = {j: math.radians(v) for j, v in HAND_POINT_DEG.items()}

# ── Controle de força ────────────────────────────────────────────────────────
# Limite de segurança: a medição é CANCELADA se a compressão exceder este
# valor (explorer aborta; GUI exibe DANGER ao se aproximar).
FORCE_ABORT_LIMIT_N = 15.0
# Setpoint máximo selecionável na GUI.
FORCE_SETPOINT_MAX_N = 10.0

# ── Célula de carga (ESP32 via UDP) ──────────────────────────────────────────
LOAD_CELL_UDP_PORT = 8080
# A ESP32 amostra a 1 kHz (1 ms) e ENVIA EM LOTE: empacota
# LOAD_CELL_BATCH_N amostras por datagrama, transmitindo ~100 pacotes/s. Isso
# entrega 1 kHz de dados mantendo a taxa de pacotes na faixa que o WiFi sustenta
# de forma confiável (1000 datagramas minúsculos/s estouravam o airtime e a
# perda). Cada amostra é auto-descritiva (seq + timestamp + tensão), então um
# pacote perdido aparece como salto de seq, sem desalinhar o stream.
#
# Amostra (little-endian, 12 bytes): uint32 seq + uint32 t_us + float v_sensor.
#   seq    — contador incremental por AMOSTRA (não por pacote); o salto revela
#            amostras perdidas.
#   t_us   — micros() da ESP32 no instante da amostra; é o relógio de 1 ms usado
#            para colocar força e toque numa grade temporal comum (sincronização).
#   v_sensor — tensão calibrada (V), com filtro LEVE na ESP (só média do
#            oversampling); o filtro pesado roda no force_receiver (PC).
# Espelhado em sensors/ForceDriver/src/main.cpp (struct Sample).
LOAD_CELL_SAMPLE_FMT = '<IIf'
# Quantas amostras a ESP agrupa por datagrama. 1 kHz / 10 = 100 pacotes/s.
# Espelhado no firmware (BATCH_N).
LOAD_CELL_BATCH_N = 10

# Auto-descoberta para enviar a telemetria por UNICAST (broadcast no WiFi não
# é reconhecido/retransmitido pela 802.11 → ~30% de perda). O force_receiver
# manda um "hello" periódico para o IP FIXO do ESP nesta porta; o ESP grava o
# remetente e passa a enviar a telemetria unicast de volta (com fallback a
# broadcast se nunca receber/ficar obsoleto). Espelhado no firmware.
LOAD_CELL_ESP_IP        = '192.168.5.105'   # IP estático do ESP (WiFi.config)
LOAD_CELL_DISCOVERY_PORT = 8090             # porta onde o ESP escuta o hello
LOAD_CELL_DISCOVERY_MAGIC = b'FRCV'         # tag do hello (ignora tráfego alheio)

# Transformação AFIM tensão_ADC→v_sensor aplicada no firmware da ESP32.
# ESPELHO EXATO de sensors/ForceDriver/src/main.cpp:
#     v_sensor = v_adc * V_GAIN - V_OFFSET,   V_GAIN = (R1+R2)/R2
# com R1=220000, R2=98600 (divisor) e V_OFFSET=0.4544 V. O firmware NÃO
# multiplica por AMP_GAIN — o ×10 que existia aqui era espúrio.
# Estes dois valores formam a ASSINATURA da configuração de hardware/firmware:
# a GUI grava ambos no load_cell_calib.json ao calibrar e avisa quando a
# calibração vigente foi feita com ganho/offset diferentes (firmware alterado
# → slope/intercept salvos ficam inválidos silenciosamente).
LC_FW_R1 = 220000.0
LC_FW_R2 =  98600.0
LC_FW_VOLTAGE_SCALE  = (LC_FW_R1 + LC_FW_R2) / LC_FW_R2   # ganho do divisor (≈3.23)
LC_FW_VOLTAGE_OFFSET = 0.4544                             # V subtraído no firmware

# ── Touch sensor (STM32 → PC plotter → UDP) ──────────────────────────────────
# Porta DIFERENTE da célula de carga: o force_receiver aceita qualquer
# datagrama ≥ 8 bytes sem filtrar remetente — na mesma porta os fluxos
# se misturariam silenciosamente.
TOUCH_SENSOR_UDP_PORT = 8081
# Payload (little-endian, 8 bytes): uint32 seq + float valor.
# Espelhado em sensors/Touch_sensor/touch_sensor.py (roda fora do ROS).
TOUCH_PAYLOAD_FMT = '<If'
# IP de broadcast para o qual o plotter (TouchSensorSource) reemite o I_final
# por UDP a cada TOTAL — exatamente o papel do plotter original. O receptor é
# o touch_receiver_node (porta 8081, mesmo formato). É o MESMO broadcast que a
# ESP32 da célula usa (DEST_IP em sensors/ForceDriver/src/main.cpp); a porta
# 8081 (≠ 8080) é o que evita misturar os fluxos toque/força.
TOUCH_UDP_BROADCAST_IP = '192.168.5.255'

# Idade máxima de uma amostra para entrar no par sincronizado (s).
# A célula amostra a 100 Hz (10 ms); 0.25 s = ~25 amostras perdidas antes
# de considerarmos a fonte morta.
SYNC_MAX_AGE_S = 0.25

# ── Arquivos de configuração persistente (~/.config/touch_pack/) ─────────────
CONFIG_DIR            = os.path.expanduser('~/.config/touch_pack')
HOME_POSE_FILE        = os.path.join(CONFIG_DIR, 'home_pose.json')
ROBOT_CONFIG_FILE     = os.path.join(CONFIG_DIR, 'robot.json')
LC_CALIB_FILE         = os.path.join(CONFIG_DIR, 'load_cell_calib.json')
POSES_FILE            = os.path.join(CONFIG_DIR, 'poses.json')
PALPATION_PARAMS_FILE = os.path.join(CONFIG_DIR, 'palpation_params.json')

# ── Saída dos runs ───────────────────────────────────────────────────────────
# Os dados (CSV de runs e do stream força+toque) são gravados em
# <raiz_do_repo>/sensors/Data. A raiz é localizada subindo a partir deste
# arquivo até achar um diretório que contenha `sensors/` — funciona tanto
# rodando do código-fonte (src/...) quanto do espaço instalado (install/...),
# ambos sob a raiz do repositório. Override explícito: TOUCH_PACK_DATA_DIR.
def _resolve_repo_root() -> str | None:
    """Sobe a partir deste arquivo até achar um diretório que contenha
    `sensors/` — funciona rodando do código-fonte (src/...) ou do espaço
    instalado (install/...), ambos sob a raiz do repositório. None se o pacote
    estiver instalado fora da árvore do repo."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(d, 'sensors')):
            return d
        parent = os.path.dirname(d)
        if parent == d:        # chegou na raiz do filesystem
            break
        d = parent
    return None


_REPO_ROOT = _resolve_repo_root()


def _resolve_runs_dir() -> str:
    env = os.environ.get('TOUCH_PACK_DATA_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    if _REPO_ROOT:
        return os.path.join(_REPO_ROOT, 'sensors', 'Data')
    # Fallback: repo não encontrado (pacote instalado fora da árvore) —
    # mantém o comportamento antigo para não perder dados.
    return os.path.expanduser('~/touch_pack_runs')


RUNS_DIR = _resolve_runs_dir()

# ── Calibração da célula COMPARTILHADA via git ───────────────────────────────
# A calibração pertence ao SENSOR (célula+amp+firmware), não ao PC. Para que
# toda máquina que clonar o repo já venha calibrada, guardamos uma cópia
# VERSIONADA aqui no repo. Precedência de LEITURA: a calibração local em
# ~/.config (recalibrada NESTA máquina) ganha; se não houver, cai nesta cópia do
# repo. None se o pacote estiver instalado fora da árvore do repo.
LC_CALIB_REPO_FILE = (os.path.join(_REPO_ROOT, 'sensors', 'load_cell_calib.json')
                      if _REPO_ROOT else None)


def lc_calib_read_path() -> str:
    """Caminho de onde LER a calibração: prefere a local (~/.config); se não
    existir, cai na versionada no repo (compartilhada via git). Gravar sempre vai
    para LC_CALIB_FILE (local) — e é espelhado no repo para virar diff de commit."""
    if os.path.exists(LC_CALIB_FILE):
        return LC_CALIB_FILE
    if LC_CALIB_REPO_FILE and os.path.exists(LC_CALIB_REPO_FILE):
        return LC_CALIB_REPO_FILE
    return LC_CALIB_FILE   # nenhum existe: o loader trata a ausência
