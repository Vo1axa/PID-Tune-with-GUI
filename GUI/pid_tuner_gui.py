#!/usr/bin/env python3
"""PID Tuner - tampilan modern (customtkinter, tema gelap).
Install: pip install customtkinter pyserial matplotlib  (di dalam venv)
"""
import queue
import re
import threading
import time
from collections import deque

import customtkinter as ctk
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from matplotlib.figure import Figure

BAUD = 115200
MAXPTS = 2500  # 50 detik @ 20 ms

BG, CARD, BORDER = "#0f1117", "#1a1d27", "#2a2e3d"
TXT, MUTED = "#e6e8ef", "#8b90a3"
BLUE, ORANGE, GREEN, RED = "#4f8cff", "#ffa94d", "#22c55e", "#ef4444"

ctk.set_appearance_mode("dark")


class App(ctk.CTk):
    def __init__(self):
        super().__init__(fg_color=BG)
        self.title("PID Tuner")
        self.geometry("1180x740")
        self.minsize(980, 620)
        self.ser = None
        self.running = False
        self.q = queue.Queue()
        self.t0 = time.time()
        self.tt, self.sp, self.rpm, self.pwm = (deque(maxlen=MAXPTS) for _ in range(4))

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.build_sidebar()
        self.build_main()
        self.refresh_ports()
        self.after(100, self.poll)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- helpers ----------
    def section(self, parent, text):
        ctk.CTkLabel(parent, text=text.upper(), text_color=MUTED,
                     font=ctk.CTkFont(size=11, weight="bold")).pack(anchor="w", padx=18, pady=(18, 6))

    def button(self, parent, text, cmd, color=BLUE, hover=None, **kw):
        return ctk.CTkButton(parent, text=text, command=cmd, fg_color=color,
                             hover_color=hover or color, corner_radius=8, height=36,
                             font=ctk.CTkFont(size=13, weight="bold"), **kw)

    # ---------- sidebar ----------
    def build_sidebar(self):
        sb = ctk.CTkFrame(self, width=270, fg_color=CARD, corner_radius=0)
        sb.grid(row=0, column=0, sticky="ns")
        sb.grid_propagate(False)
        sb.pack_propagate(False)

        ctk.CTkLabel(sb, text="PID Tuner", font=ctk.CTkFont(size=22, weight="bold"),
                     text_color=TXT).pack(anchor="w", padx=18, pady=(20, 0))
        ctk.CTkLabel(sb, text="ESP8266 · BTS7960", text_color=MUTED).pack(anchor="w", padx=18)

        self.section(sb, "Koneksi")
        row = ctk.CTkFrame(sb, fg_color="transparent")
        row.pack(fill="x", padx=18)
        self.port_cb = ctk.CTkComboBox(row, values=[""], height=34, corner_radius=8,
                                       fg_color=BG, border_color=BORDER, button_color=BORDER)
        self.port_cb.pack(side="left", fill="x", expand=True)
        ctk.CTkButton(row, text="⟳", width=34, height=34, fg_color=BG, border_width=1,
                      border_color=BORDER, hover_color=BORDER,
                      command=self.refresh_ports).pack(side="left", padx=(6, 0))
        self.btn_conn = self.button(sb, "Connect", self.toggle_conn)
        self.btn_conn.pack(fill="x", padx=18, pady=(8, 4))
        self.status = ctk.CTkLabel(sb, text="●  Terputus", text_color=RED)
        self.status.pack(anchor="w", padx=18)

        self.section(sb, "Gain PID")
        grid = ctk.CTkFrame(sb, fg_color="transparent")
        grid.pack(fill="x", padx=18)
        grid.grid_columnconfigure((1, 3), weight=1)
        self.ent = {}
        for i, (name, val) in enumerate([("Kp", "3"), ("Ki", "8"), ("Kd", "0"), ("SP", "100")]):
            r, c = divmod(i, 2)
            ctk.CTkLabel(grid, text=name, text_color=MUTED, width=26).grid(row=r, column=c * 2, pady=4)
            e = ctk.CTkEntry(grid, height=34, corner_radius=8, fg_color=BG, border_color=BORDER, width=80)
            e.insert(0, val)
            e.grid(row=r, column=c * 2 + 1, padx=(2, 8), pady=4, sticky="ew")
            e.bind("<Return>", lambda ev: self.apply())
            self.ent[name] = e
        self.button(sb, "Apply", self.apply, color="#2f3447", hover=BORDER).pack(fill="x", padx=18, pady=(6, 0))

        self.section(sb, "Aksi")
        row = ctk.CTkFrame(sb, fg_color="transparent")
        row.pack(fill="x", padx=18)
        self.auto_rpm = ctk.CTkEntry(row, width=64, height=36, corner_radius=8,
                                     fg_color=BG, border_color=BORDER)
        self.auto_rpm.insert(0, "80")
        self.auto_rpm.pack(side="left")
        self.button(row, "Auto-tune", self.do_auto, color="#7c5cff", hover="#6a4be0").pack(
            side="left", fill="x", expand=True, padx=(8, 0))
        self.button(sb, "▶  GO (uji 8 s)", self.do_go, color=GREEN, hover="#16a34a").pack(
            fill="x", padx=18, pady=(8, 0))
        self.button(sb, "■  STOP", lambda: self.send("stop"), color=RED, hover="#dc2626").pack(
            fill="x", padx=18, pady=(8, 0))
        self.button(sb, "Show gain", lambda: self.send("show"), color="#2f3447", hover=BORDER).pack(
            fill="x", padx=18, pady=(8, 0))

    # ---------- main ----------
    def build_main(self):
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.grid(row=0, column=1, sticky="nsew", padx=18, pady=18)
        main.grid_columnconfigure(0, weight=1)
        main.grid_rowconfigure(1, weight=1)

        cards = ctk.CTkFrame(main, fg_color="transparent")
        cards.grid(row=0, column=0, sticky="ew")
        cards.grid_columnconfigure((0, 1, 2, 3), weight=1, uniform="c")
        self.val = {}
        for i, (key, title, color) in enumerate([("rpm", "RPM", BLUE), ("sp", "TARGET", ORANGE),
                                                  ("err", "ERROR", TXT), ("pwm", "PWM", GREEN)]):
            f = ctk.CTkFrame(cards, fg_color=CARD, corner_radius=12)
            f.grid(row=0, column=i, sticky="ew", padx=(0 if i == 0 else 8, 0))
            ctk.CTkLabel(f, text=title, text_color=MUTED,
                         font=ctk.CTkFont(size=11, weight="bold")).pack(anchor="w", padx=16, pady=(12, 0))
            self.val[key] = ctk.CTkLabel(f, text="--", text_color=color,
                                         font=ctk.CTkFont(size=28, weight="bold"))
            self.val[key].pack(anchor="w", padx=16, pady=(0, 12))

        plot = ctk.CTkFrame(main, fg_color=CARD, corner_radius=12)
        plot.grid(row=1, column=0, sticky="nsew", pady=12)
        self.fig = Figure(figsize=(8, 4.5), dpi=100, facecolor=CARD)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212, sharex=self.ax1)
        for ax, yl in ((self.ax1, "rpm"), (self.ax2, "pwm")):
            ax.set_facecolor(CARD)
            ax.grid(True, color=BORDER, linewidth=0.6)
            ax.tick_params(colors=MUTED, labelsize=8)
            ax.set_ylabel(yl, color=MUTED, fontsize=9)
            for s in ax.spines.values():
                s.set_color(BORDER)
        self.ax1.tick_params(labelbottom=False)
        self.ax2.set_xlabel("detik", color=MUTED, fontsize=9)
        self.l_sp, = self.ax1.plot([], [], color=ORANGE, ls="--", lw=1.4, label="target")
        self.l_rpm, = self.ax1.plot([], [], color=BLUE, lw=1.8, label="rpm")
        self.l_pwm, = self.ax2.plot([], [], color=GREEN, lw=1.6)
        leg = self.ax1.legend(loc="upper left", facecolor=CARD, edgecolor=BORDER, fontsize=8)
        for t in leg.get_texts():
            t.set_color(TXT)
        self.fig.tight_layout(pad=1.2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

        bottom = ctk.CTkFrame(main, fg_color=CARD, corner_radius=12)
        bottom.grid(row=2, column=0, sticky="ew")
        self.log = ctk.CTkTextbox(bottom, height=130, fg_color=CARD, text_color=TXT,
                                  font=ctk.CTkFont(family="monospace", size=12), state="disabled")
        self.log.pack(fill="x", padx=8, pady=(8, 4))
        row = ctk.CTkFrame(bottom, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=(0, 8))
        self.raw_e = ctk.CTkEntry(row, placeholder_text="Perintah manual, mis. kp 2.5", height=34,
                                  corner_radius=8, fg_color=BG, border_color=BORDER)
        self.raw_e.pack(side="left", fill="x", expand=True)
        self.raw_e.bind("<Return>", self.send_raw)
        ctk.CTkButton(row, text="Kirim", width=70, height=34, fg_color="#2f3447",
                      hover_color=BORDER, command=self.send_raw).pack(side="left", padx=(8, 0))
        ctk.CTkButton(row, text="Clear", width=70, height=34, fg_color="#2f3447",
                      hover_color=BORDER, command=self.clear_plot).pack(side="left", padx=(8, 0))

    # ---------- serial ----------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()] or [""]
        self.port_cb.configure(values=ports)
        if not self.port_cb.get():
            self.port_cb.set(ports[0])

    def toggle_conn(self):
        self.disconnect() if self.ser else self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port_cb.get(), BAUD, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            self.logmsg(f"! Gagal membuka port: {e}")
            self.ser = None
            return
        self.running = True
        threading.Thread(target=self.reader, daemon=True).start()
        self.btn_conn.configure(text="Disconnect", fg_color="#2f3447", hover_color=BORDER)
        self.status.configure(text="●  Terhubung", text_color=GREEN)
        self.clear_plot()

    def disconnect(self):
        self.running = False
        time.sleep(0.15)
        if self.ser:
            try:
                self.ser.write(b"stop\n")
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_conn.configure(text="Connect", fg_color=BLUE, hover_color=BLUE)
        self.status.configure(text="●  Terputus", text_color=RED)

    def reader(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                self.q.put("! koneksi serial terputus")
                self.running = False
                break
            if line:
                self.q.put(line)

    def send(self, cmd):
        if not self.ser:
            self.logmsg("! Belum terhubung")
            return
        self.ser.write((cmd + "\n").encode())
        self.logmsg(f"> {cmd}")

    def send_raw(self, _=None):
        c = self.raw_e.get().strip()
        if c:
            self.send(c)
            self.raw_e.delete(0, "end")

    def apply(self):
        for k in ("Kp", "Ki", "Kd", "SP"):
            v = self.ent[k].get().strip()
            if v:
                self.send(f"{k.lower()} {v}")

    def do_go(self):
        self.apply()
        self.clear_plot()
        self.send("go")

    def do_auto(self):
        self.clear_plot()
        self.send(f"auto {self.auto_rpm.get().strip() or '80'}")

    # ---------- data ----------
    def poll(self):
        got = False
        try:
            while True:
                line = self.q.get_nowait()
                m = re.match(r"sp:([-\d.]+),rpm:([-\d.]+),pwm:(-?\d+)", line)
                if m:
                    self.tt.append(time.time() - self.t0)
                    self.sp.append(float(m.group(1)))
                    self.rpm.append(float(m.group(2)))
                    self.pwm.append(int(m.group(3)))
                    got = True
                else:
                    self.logmsg(line)
                    g = re.search(r"Kp=([-\d.]+) Ki=([-\d.]+) Kd=([-\d.]+) sp=([-\d.]+)", line)
                    if g:
                        for k, v in zip(("Kp", "Ki", "Kd", "SP"), g.groups()):
                            self.ent[k].delete(0, "end")
                            self.ent[k].insert(0, v)
        except queue.Empty:
            pass
        if got:
            self.redraw()
        self.after(100, self.poll)

    def redraw(self):
        t = list(self.tt)
        self.l_sp.set_data(t, list(self.sp))
        self.l_rpm.set_data(t, list(self.rpm))
        self.l_pwm.set_data(t, list(self.pwm))
        if t:
            self.ax1.set_xlim(t[0], max(t[-1], t[0] + 5))
            self.val["rpm"].configure(text=f"{self.rpm[-1]:.1f}")
            self.val["sp"].configure(text=f"{self.sp[-1]:.0f}")
            self.val["err"].configure(text=f"{self.sp[-1] - self.rpm[-1]:+.1f}")
            self.val["pwm"].configure(text=f"{self.pwm[-1]}")
        for ax in (self.ax1, self.ax2):
            ax.relim()
            ax.autoscale_view(scalex=False)
        self.canvas.draw_idle()

    def clear_plot(self):
        self.t0 = time.time()
        for d in (self.tt, self.sp, self.rpm, self.pwm):
            d.clear()
        for v in self.val.values():
            v.configure(text="--")
        self.redraw()

    def logmsg(self, s):
        self.log.configure(state="normal")
        self.log.insert("end", s + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()