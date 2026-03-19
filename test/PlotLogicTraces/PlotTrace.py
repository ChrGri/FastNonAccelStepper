import numpy as np
import struct
import matplotlib.pyplot as plt

def parse_saleae_digital(file_path, limit=10**7):
    with open(file_path, 'rb') as f:
        magic = f.read(8)
        if magic != b'<SALEAE>':
            raise ValueError(f"Datei {file_path} ist kein gültiges Saleae-Format.")
        
        version, f_type = struct.unpack('<ii', f.read(8))
        initial_state = struct.unpack('<I', f.read(4))[0]
        begin_t, end_t = struct.unpack('<dd', f.read(16))
        num_transitions_total = struct.unpack('<Q', f.read(8))[0]

        num_to_read = min(num_transitions_total, limit)
        transition_times = np.fromfile(f, dtype='<f8', count=num_to_read)
        
        if num_transitions_total > limit:
            end_t = transition_times[-1]
        
    return transition_times, initial_state, begin_t, end_t

def get_stepper_data(times, initial_state, begin_t, end_t):
    t_plot = [begin_t]
    y_plot = [initial_state]
    current_state = initial_state
    for t in times:
        t_plot.append(t)
        y_plot.append(current_state)
        current_state = 1 - current_state
        t_plot.append(t)
        y_plot.append(current_state)
    t_plot.append(end_t)
    y_plot.append(current_state)
    return np.array(t_plot), np.array(y_plot)

def calculate_frequency(times, initial_state):
    """Berechnet die Frequenz und filtert initiale Ausreißer."""
    # Wir brauchen die Zeitpunkte der steigenden Flanken (0 -> 1)
    start_idx = 0 if initial_state == 0 else 1
    rising_edges = times[start_idx::2]
    
    if len(rising_edges) < 3: # Mindestens 3 Kanten für 2 Intervalle
        return np.array([]), np.array([])

    # Differenz zwischen steigenden Flanken
    deltas = np.diff(rising_edges)
    
    # Frequenz berechnen
    frequencies = 1.0 / deltas
    freq_time = rising_edges[:-1] + (deltas / 2)
    
    # --- Filter-Logik ---
    # 1. Den allerersten Punkt löschen (oft Artefakt vom Aufzeichnungsstart)
    freq_time = freq_time[1:]
    frequencies = frequencies[1:]
    
    # 2. Physikalisch unmögliche Werte filtern (z.B. > 300kHz)
    # Da dein MCPWM Limit bei 250kHz liegt, ist alles darüber ein Messfehler
    valid_mask = frequencies < 260000 
    
    return freq_time[valid_mask], frequencies[valid_mask]

def calculate_position(t_step, s_step, t_dir, s_dir, b_t, e_t):
    """Rekonstruiert die Position basierend auf Step-Flanken und Dir-Level."""
    # Wir zählen nur steigende Flanken auf Kanal 1
    start_idx = 0 if s_step == 0 else 1
    rising_edges = t_step[start_idx::2]
    
    if len(rising_edges) == 0:
        return np.array([b_t, e_t]), np.array([0, 0])

    # Bestimme den Dir-Zustand für jede steigende Flanke
    # searchsorted findet effizient heraus, wo die Flanke zeitlich im Dir-Signal liegt
    dir_indices = np.searchsorted(t_dir, rising_edges, side='right')
    
    # Der aktuelle Dir-Zustand ergibt sich aus (initial_state + Anzahl der Wechsel) % 2
    dir_levels = (s_dir + dir_indices) % 2
    
    # Umrechnung in Richtung (-1 oder 1). 
    # Annahme: Dir-Low (0) = Vorwärts, Dir-High (1) = Rückwärts (wie in deinem Code)
    directions = np.where(dir_levels == 0, 1, -1)
    
    # Kumulative Summe ergibt die Position
    positions = np.cumsum(directions)
    
    # Zeit- und Positionsarrays für den Treppen-Plot vorbereiten
    pos_time = np.concatenate(([b_t], rising_edges, [e_t]))
    pos_val = np.concatenate(([0], positions, [positions[-1]]))
    
    return pos_time, pos_val

# === Daten laden ===
file_ch1 = "test/PlotLogicTraces/digital_1.bin"
file_ch5 = "test/PlotLogicTraces/digital_5.bin"

t1, s1, b1, e1 = parse_saleae_digital(file_ch1)
t5, s5, b5, e5 = parse_saleae_digital(file_ch5)

# === Daten aufbereiten ===
x1, y1 = get_stepper_data(t1, s1, b1, e1)
x5, y5 = get_stepper_data(t5, s5, b5, e5)
freq_t, freq_val = calculate_frequency(t1, s1)
pos_t, pos_val = calculate_position(t1, s1, t5, s5, b1, e1)

# === Plotten mit 3 Subplots ===
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 12), sharex=True, 
                                   gridspec_kw={'height_ratios': [1.5, 1, 1.5]})

# --- Subplot 1: Logikpegel ---
ax1.step(x1, y1 + 1.5, where='post', color='blue', label='Kanal 1 (Step)')
ax1.step(x5, y5, where='post', color='red', label='Kanal 5 (Dir)')
ax1.set_yticks([0.5, 2.0])
ax1.set_yticklabels(["CH 5 (Dir)", "CH 1 (Step)"])
ax1.set_ylabel("Logikpegel")
ax1.set_title("Stepper Signal Analyse")
ax1.grid(True, alpha=0.3)
ax1.legend(loc='upper right')

# --- Subplot 2: Frequenz ---
if len(freq_val) > 0:
    ax2.plot(freq_t, freq_val, 'g-', label='PWM Frequenz (Hz)')
    ax2.fill_between(freq_t, freq_val, color='green', alpha=0.1)
    ax2.set_ylabel("Frequenz [Hz]")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    ax2.set_ylim(0, np.max(freq_val) * 1.2)

# --- Subplot 3: Position ---
ax3.step(pos_t, pos_val, where='post', color='purple', label='Rekonstruierte Position')
ax3.set_ylabel("Position [Pulse]")
ax3.set_xlabel("Zeit [s]")
ax3.grid(True, alpha=0.3)
ax3.legend(loc='upper right')

plt.tight_layout()
plt.show()

tmp = 5