"""
ui_helpers.py — Tema visual + widgets reutilizáveis da GUI do touch_pack.

Primeira fatia da modularização da palpation_gui: tudo aqui é Tk puro,
sem ROS, importável por qualquer janela/aba futura.

IMPORTANTE — bug do fontconfig (Tk 8.6 / Ubuntu 22.04): qualquer `font=`
com família/tamanho fora dos built-ins força o Tk a criar um novo XftFont
via fontconfig, que corrompe o heap nesta máquina (~50 widgets → segfault).
Por isso TODOS os FONT_* abaixo são NAMED FONTS embutidos do Tk,
pré-alocados durante tk.Tk() sem nenhuma chamada ao fontconfig.
NÃO substitua por tuples ('Família', tamanho, 'peso').
"""
from __future__ import annotations

import tkinter as tk

# ── Paleta (tema claro do laboratório) ─────────────────────────────────
BG          = '#f1f5f9'
PANEL       = '#ffffff'
HEADER      = '#1d4ed8'
HEADER_FG   = 'white'
TEXT        = '#0f172a'
TEXT_MUTED  = '#475569'
TEXT_DIM    = '#94a3b8'
PRIMARY     = '#2563eb'
PRIMARY_HV  = '#1d4ed8'
OK          = '#16a34a'
WARN        = '#d97706'
DANGER      = '#dc2626'
DANGER_HV   = '#b91c1c'
BORDER      = '#cbd5e1'
BTN_NEUTRAL = '#e2e8f0'

# ── Fontes — APENAS named fonts do Tk (ver docstring do módulo) ────────
FONT_TITLE  = 'TkCaptionFont'       # 12 pt bold
FONT_HEAD   = 'TkCaptionFont'       # 12 pt bold
FONT_LBL    = 'TkDefaultFont'       # 10 pt
FONT_SMALL  = 'TkSmallCaptionFont'  #  9 pt
FONT_BIG    = 'TkCaptionFont'       # 12 pt bold (displays de força)
FONT_MONO   = 'TkFixedFont'         # 10 pt mono
FONT_MONO_S = 'TkFixedFont'         # 10 pt mono


def _shade(hex_color: str, factor: float) -> str:
    """Clareia/escurece uma cor hex (factor positivo = mais claro)."""
    h = hex_color.lstrip('#')
    r, g, b = (int(h[i:i + 2], 16) for i in (0, 2, 4))
    if factor >= 0:
        r = int(r + (255 - r) * factor)
        g = int(g + (255 - g) * factor)
        b = int(b + (255 - b) * factor)
    else:
        f = 1.0 + factor
        r = int(r * f); g = int(g * f); b = int(b * f)
    return f'#{r:02x}{g:02x}{b:02x}'


class _Tooltip:
    """Tooltip flutuante minimalista (hover) — substitui os textos de ajuda
    inline, reduzindo o ruído visual das abas. Usa apenas named fonts do Tk
    (TkSmallCaptionFont) para não disparar o bug do fontconfig."""

    def __init__(self, widget, text: str):
        self.widget = widget
        self.text = text
        self.tip: tk.Toplevel | None = None
        widget.bind('<Enter>', self._show, add='+')
        widget.bind('<Leave>', self._hide, add='+')
        widget.bind('<ButtonPress>', self._hide, add='+')

    def _show(self, _e=None):
        if self.tip is not None or not self.text:
            return
        try:
            x = self.widget.winfo_rootx() + 10
            y = (self.widget.winfo_rooty()
                 + self.widget.winfo_height() + 6)
            self.tip = tw = tk.Toplevel(self.widget)
            tw.wm_overrideredirect(True)
            tw.wm_geometry(f'+{x}+{y}')
            tk.Label(tw, text=self.text, justify='left',
                     bg=TEXT, fg='white', font='TkSmallCaptionFont',
                     padx=10, pady=6, wraplength=340).pack()
        except tk.TclError:
            self.tip = None

    def _hide(self, _e=None):
        if self.tip is not None:
            try:
                self.tip.destroy()
            except tk.TclError:
                pass
            self.tip = None


def _hdr_btn(parent, icon: str, label: str, command, *,
              bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL, padx=12, pady=5):
    """Botão estilizado da barra superior — ícone Unicode + label,
    com troca dinâmica de estado via `btn.set_state(icon, label, bg, fg)`."""
    state = {'bg': bg, 'fg': fg}
    text = f' {icon}  {label} ' if icon else f' {label} '
    btn = tk.Button(parent, text=text, command=command,
                    bg=bg, fg=fg,
                    activebackground=_shade(bg, -0.08),
                    activeforeground=fg,
                    relief='flat', bd=0, padx=padx, pady=pady,
                    font=font, cursor='hand2',
                    highlightthickness=0)
    btn.bind('<Enter>',
              lambda _e: btn.config(bg=_shade(state['bg'], -0.08)))
    btn.bind('<Leave>',
              lambda _e: btn.config(bg=state['bg']))

    def set_state(icon: str, label: str, bg: str, fg: str = 'white'):
        state['bg'] = bg; state['fg'] = fg
        new = f' {icon}  {label} ' if icon else f' {label} '
        btn.config(text=new, bg=bg, fg=fg,
                    activebackground=_shade(bg, -0.08),
                    activeforeground=fg)
    btn.set_state = set_state  # type: ignore[attr-defined]
    return btn
