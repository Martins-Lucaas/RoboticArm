"""
palpation_report.py — Análise pós-run dos CSVs do palpation_logger.

Para cada run (<ts>__samples.csv + <ts>__params.json) produz:
  <ts>__summary.json   métricas por ciclo/fase (ver compute_summary)
  <ts>__plot.png       força×tempo colorido por fase (requer matplotlib;
                       sem ele o relatório gera só o JSON)

Métricas calculadas (por ciclo, nas fases com controle de força):
  DESCENDING  duração, força máx, overshoot vs setpoint
  HOLD        duração (≈ tempo de estabilização do setpoint), força média/
              desvio no último segundo (qualidade da estabilização)
  SLIDING     duração, força média/desvio/mín/máx, erro médio absoluto vs
              setpoint, distância lateral percorrida (via TCP da FK)

Uso (CLI):
  ros2 run touch_pack palpation_report -- --latest
  ros2 run touch_pack palpation_report -- ~/touch_pack_runs/20260611_*.csv
  ros2 run touch_pack palpation_report -- --latest --no-plot

O logger também chama generate_report() automaticamente ao fechar cada run.

Compatibilidade: lê tanto o schema novo (t_rel_s, cycle, phase, force_net_n,
q1..q6, tcp_*, touch_value, touch_age_ms) quanto os antigos (sem as colunas
de touch; ou t_rel_s, phase, fx..tz — usa |fz| como força, cycle=1, sem TCP).
Quando o run tem touch_value, cada fase ganha um bloco 'touch' com as mesmas
estatísticas da força e o plot ganha um eixo direito com o sinal do toque.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys

OUTPUT_DIR = os.path.expanduser('~/touch_pack_runs')

# Fases com controle/medição de força — as únicas resumidas por ciclo.
_FORCE_PHASES = ('DESCENDING', 'HOLD', 'SLIDING')

# Cores por fase no gráfico — espelham a paleta da GUI.
_PHASE_COLORS = {
    'HOME': '#94a3b8', 'DESCENDING': '#d97706', 'HOLD': '#16a34a',
    'SLIDING': '#2563eb', 'RETRACT': '#64748b',
    'IDLE': '#cbd5e1', 'DONE': '#16a34a', 'ABORTED': '#dc2626',
}


# ──────────────────────────────────────────────────────────────────────
# Leitura
# ──────────────────────────────────────────────────────────────────────

def _load_rows(csv_path: str) -> list[dict]:
    """Lê o CSV num formato normalizado:
    [{t, cycle, phase, force, tcp(x,y,z)|None, touch|None}, ...]"""
    rows: list[dict] = []
    with open(csv_path, newline='') as fh:
        reader = csv.DictReader(fh)
        fields = reader.fieldnames or []
        new_schema = 'force_net_n' in fields
        for r in reader:
            try:
                t = float(r['t_rel_s'])
                if new_schema:
                    force = float(r['force_net_n'])
                    cycle = int(r.get('cycle') or 1) or 1
                    tcp = None
                    if r.get('tcp_x'):
                        tcp = (float(r['tcp_x']), float(r['tcp_y']),
                               float(r['tcp_z']))
                else:   # schema antigo: wrench — |fz| era a força normal
                    force = abs(float(r['fz']))
                    cycle = 1
                    tcp = None
                # Coluna opcional (runs novos): vazia = sem amostra fresca.
                touch = None
                if r.get('touch_value'):
                    touch = float(r['touch_value'])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append({'t': t, 'cycle': cycle,
                         'phase': r.get('phase', '?'), 'force': force,
                         'tcp': tcp, 'touch': touch})
    return rows


def _load_params(csv_path: str) -> dict:
    params_path = csv_path.replace('__samples.csv', '__params.json')
    try:
        with open(params_path) as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def _segments(rows: list[dict]) -> list[dict]:
    """Agrupa amostras contíguas de mesma (cycle, phase)."""
    segs: list[dict] = []
    for row in rows:
        if (segs and segs[-1]['phase'] == row['phase']
                and segs[-1]['cycle'] == row['cycle']):
            segs[-1]['rows'].append(row)
        else:
            segs.append({'cycle': row['cycle'], 'phase': row['phase'],
                         'rows': [row]})
    return segs


# ──────────────────────────────────────────────────────────────────────
# Métricas
# ──────────────────────────────────────────────────────────────────────

def _stats(forces: list[float]) -> dict:
    return {
        'mean_n': round(statistics.fmean(forces), 3),
        'std_n':  round(statistics.pstdev(forces), 3) if len(forces) > 1 else 0.0,
        'min_n':  round(min(forces), 3),
        'max_n':  round(max(forces), 3),
        'n_samples': len(forces),
    }


def _seg_summary(seg: dict, target: float | None) -> dict:
    rows = seg['rows']
    forces = [r['force'] for r in rows]
    t0, t1 = rows[0]['t'], rows[-1]['t']
    out = {'duration_s': round(t1 - t0, 2), **_stats(forces)}

    if target is not None:
        out['target_n'] = target
        if seg['phase'] == 'DESCENDING':
            out['overshoot_n'] = round(max(forces) - target, 3)
        if seg['phase'] == 'SLIDING':
            out['mae_vs_target_n'] = round(
                statistics.fmean(abs(f - target) for f in forces), 3)
    if seg['phase'] == 'HOLD':
        # Qualidade da estabilização: estatística do último segundo do HOLD
        # (a janela que o critério de _HOLD_STABLE_S validou).
        tail = [r['force'] for r in rows if r['t'] >= t1 - 1.0]
        if tail:
            out['final_window'] = _stats(tail)
    if seg['phase'] == 'SLIDING':
        tcps = [r['tcp'] for r in rows if r['tcp'] is not None]
        if len(tcps) >= 2:
            dx = tcps[-1][0] - tcps[0][0]
            dy = tcps[-1][1] - tcps[0][1]
            out['lateral_dist_mm'] = round(math.hypot(dx, dy) * 1e3, 1)
    touch_vals = [r['touch'] for r in rows if r.get('touch') is not None]
    if touch_vals:
        out['touch'] = _stats(touch_vals)
    return out


def compute_summary(rows: list[dict], params: dict) -> dict:
    target = params.get('force_n')
    target = float(target) if target is not None else None

    cycles: dict[int, dict] = {}
    for seg in _segments(rows):
        if seg['phase'] not in _FORCE_PHASES or not seg['rows']:
            continue
        cyc = cycles.setdefault(seg['cycle'], {})
        # Fases repetidas no mesmo ciclo (não deveria ocorrer) ganham sufixo.
        key = seg['phase']
        k = 2
        while key in cyc:
            key = f'{seg["phase"]}_{k}'; k += 1
        cyc[key] = _seg_summary(seg, target)

    summary: dict = {
        'n_samples': len(rows),
        'duration_s': round(rows[-1]['t'] - rows[0]['t'], 2) if rows else 0.0,
        'cycles_detected': len(cycles),
        'target_force_n': target,
        'params': params,
        'cycles': {str(c): cycles[c] for c in sorted(cycles)},
    }

    # Repetibilidade entre ciclos: força média do SLIDING por ciclo.
    sl_means = [c['SLIDING']['mean_n'] for c in cycles.values()
                if 'SLIDING' in c]
    if len(sl_means) >= 2:
        summary['sliding_repeatability'] = {
            'mean_of_means_n': round(statistics.fmean(sl_means), 3),
            'std_of_means_n': round(statistics.pstdev(sl_means), 3),
        }
    return summary


# ──────────────────────────────────────────────────────────────────────
# Gráfico
# ──────────────────────────────────────────────────────────────────────

def _make_plot(rows: list[dict], summary: dict, out_png: str) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except ImportError:
        return False

    fig, ax = plt.subplots(figsize=(11, 4.5), dpi=110)
    phases_seen: list[str] = []
    for seg in _segments(rows):
        srows = seg['rows']
        color = _PHASE_COLORS.get(seg['phase'], '#0f172a')
        ax.plot([r['t'] for r in srows], [r['force'] for r in srows],
                color=color, linewidth=1.0)
        if seg['phase'] not in phases_seen:
            phases_seen.append(seg['phase'])

    target = summary.get('target_force_n')
    if target is not None:
        ax.axhline(target, color='#dc2626', linestyle='--',
                   linewidth=0.9, label=f'setpoint {target:g} N')

    # Touch sensor no eixo direito (unidade arbitrária do STM32) — só nos
    # runs novos que têm a coluna; lacunas (amostra estale) quebram a linha.
    touch_pts = [(r['t'], r['touch']) for r in rows
                 if r.get('touch') is not None]
    if touch_pts:
        ax_t = ax.twinx()
        ax_t.plot([p[0] for p in touch_pts], [p[1] for p in touch_pts],
                  color='#7c3aed', linewidth=0.8, alpha=0.7)
        ax_t.set_ylabel('touch sensor (u.a.)', color='#7c3aed')
        ax_t.tick_params(axis='y', labelcolor='#7c3aed')

    ax.set_xlabel('tempo (s)')
    ax.set_ylabel('força de compressão (N)')
    n_cyc = summary.get('cycles_detected', 0)
    ax.set_title(f'Palpação — {os.path.basename(out_png).split("__")[0]}'
                 + (f'  ({n_cyc} ciclos)' if n_cyc > 1 else ''))
    handles = [Patch(color=_PHASE_COLORS.get(p, '#0f172a'), label=p)
               for p in phases_seen]
    if target is not None:
        handles += ax.get_legend_handles_labels()[0]
    ax.legend(handles=handles, fontsize=8, ncol=min(6, len(handles)),
              loc='upper right')
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True


# ──────────────────────────────────────────────────────────────────────
# API pública + CLI
# ──────────────────────────────────────────────────────────────────────

def generate_report(csv_path: str, make_plot: bool = True) -> dict:
    """Gera <ts>__summary.json (+ <ts>__plot.png) ao lado do CSV.
    Retorna o summary (com as chaves extras summary_path/plot_path)."""
    rows = _load_rows(csv_path)
    if not rows:
        raise ValueError(f'CSV sem amostras válidas: {csv_path}')
    params = _load_params(csv_path)
    summary = compute_summary(rows, params)

    base = csv_path.replace('__samples.csv', '')
    summary_path = base + '__summary.json'
    with open(summary_path, 'w') as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
    summary['summary_path'] = summary_path

    if make_plot:
        plot_path = base + '__plot.png'
        if _make_plot(rows, summary, plot_path):
            summary['plot_path'] = plot_path
    return summary


def _print_summary(summary: dict) -> None:
    tgt = summary.get('target_force_n')
    print(f'  amostras: {summary["n_samples"]}  '
          f'duração: {summary["duration_s"]:.1f}s  '
          f'ciclos: {summary["cycles_detected"]}  '
          f'setpoint: {tgt if tgt is not None else "?"} N')
    for cyc, phases in summary.get('cycles', {}).items():
        for phase, m in phases.items():
            extra = ''
            if 'overshoot_n' in m:
                extra = f'  overshoot={m["overshoot_n"]:+.2f}N'
            if 'mae_vs_target_n' in m:
                extra = f'  MAE={m["mae_vs_target_n"]:.2f}N'
            if 'lateral_dist_mm' in m:
                extra += f'  percurso={m["lateral_dist_mm"]:.0f}mm'
            if 'touch' in m:
                extra += (f'  touch={m["touch"]["mean_n"]:.2f}'
                          f'±{m["touch"]["std_n"]:.2f}u.a.')
            print(f'    ciclo {cyc} {phase:<11} {m["duration_s"]:5.1f}s  '
                  f'F={m["mean_n"]:.2f}±{m["std_n"]:.2f}N '
                  f'[{m["min_n"]:.2f}, {m["max_n"]:.2f}]{extra}')
    rep = summary.get('sliding_repeatability')
    if rep:
        print(f'    repetibilidade SLIDING: '
              f'{rep["mean_of_means_n"]:.2f} ± {rep["std_of_means_n"]:.2f} N '
              '(desvio entre ciclos)')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Gera summary JSON + gráfico de runs de palpação.')
    parser.add_argument('csvs', nargs='*',
                        help='CSV(s) de run (<ts>__samples.csv)')
    parser.add_argument('--latest', action='store_true',
                        help=f'usa o run mais recente de {OUTPUT_DIR}')
    parser.add_argument('--no-plot', action='store_true',
                        help='gera apenas o summary JSON')
    args = parser.parse_args(argv)

    paths = list(args.csvs)
    if args.latest or not paths:
        candidates = sorted(glob.glob(
            os.path.join(OUTPUT_DIR, '*__samples.csv')))
        if not candidates:
            sys.exit(f'Nenhum run encontrado em {OUTPUT_DIR}.')
        paths = [candidates[-1]]

    for path in paths:
        print(f'▶ {os.path.basename(path)}')
        try:
            summary = generate_report(path, make_plot=not args.no_plot)
        except (OSError, ValueError) as exc:
            print(f'  ERRO: {exc}')
            continue
        _print_summary(summary)
        print(f'  → {os.path.basename(summary["summary_path"])}'
              + (f'  +  {os.path.basename(summary["plot_path"])}'
                 if 'plot_path' in summary else '  (sem matplotlib: só JSON)'))


if __name__ == '__main__':
    main()
