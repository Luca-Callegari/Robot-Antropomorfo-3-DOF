""" import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- PARAMETRI ---
n = 9000
nome_file_output = 'frequenza_robot1_tr.png'
# -----------------------------

# 1. Caricamento e pulizia
df = pd.read_csv('Robot1.csv') # Usato Robot1 come da file caricato
df.columns = df.columns.str.strip()
df = df.head(n)
df['Time'] = pd.to_datetime(df['Time'])

# 2. Calcolo della frequenza di campionamento (Fs)
# Calcoliamo il tempo medio tra i campioni in secondi
delta_t = df['Time'].diff().mean().total_seconds()
fs = 1 / delta_t  # Frequenza di campionamento (Hz)

# 3. Funzione per calcolare la FFT
def calcola_fft(signal, fs):
    n_samples = len(signal)
    # Calcolo della trasformata
    fft_values = np.fft.fft(signal)
    # Frequenze corrispondenti
    freqs = np.fft.fftfreq(n_samples, 1/fs)
    
    # Prendiamo solo la metà positiva dello spettro
    half_n = n_samples // 2
    f_pos = freqs[:half_n]
    # Magnitudo (normalizzata per il numero di campioni)
    amplitude = np.abs(fft_values[:half_n]) * (2.0 / n_samples)
    
    return f_pos, amplitude

# 4. Creazione dei Grafici
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# --- FFT Accelerometro (X, Y, Z) ---
for col, color in zip(['Accel-X (g)', 'Accel-Y (g)', 'Accel-Z (g)'], ['red', 'green', 'blue']):
    f, amp = calcola_fft(df[col].values, fs)
    ax1.plot(f, amp, label=col, color=color, alpha=0.8)

ax1.set_title(f'Analisi in Frequenza - Accelerometro (Fs ≈ {fs:.1f} Hz)')
ax1.set_ylabel('Ampiezza')
ax1.legend()
ax1.grid(True, linestyle='--', alpha=0.6)

# --- FFT Giroscopio (X, Y, Z) ---
for col, color in zip(['Gyro-X (d/s)', 'Gyro-Y (d/s)', 'Gyro-Z (d/s)'], ['darkred', 'darkgreen', 'darkblue']):
    f, amp = calcola_fft(df[col].values, fs)
    ax2.plot(f, amp, label=col, color=color, alpha=0.8)

ax2.set_title('Analisi in Frequenza - Giroscopio')
ax2.set_ylabel('Ampiezza')
ax2.set_xlabel('Frequenza (Hz)')
ax2.legend()
ax2.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
plt.savefig(nome_file_output, dpi=300)
plt.show() """

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- PARAMETRI ---
n = 3400
nome_file_output = 'frequenza_robot3_tr.png'
# -----------------------------

# 1. Caricamento e pulizia
df = pd.read_csv('Robot3.csv') 
df.columns = df.columns.str.strip()
df = df.head(n)
df['Time'] = pd.to_datetime(df['Time'])

# 2. Calcolo della frequenza di campionamento (Fs)
delta_t = df['Time'].diff().mean().total_seconds()
fs = 1 / delta_t  # Frequenza di campionamento (Hz)

# 3. Funzione per calcolare della FFT
def calcola_fft(signal, fs):
    n_samples = len(signal)
    fft_values = np.fft.fft(signal)
    freqs = np.fft.fftfreq(n_samples, 1/fs)
    
    half_n = n_samples // 2
    f_pos = freqs[:half_n]
    amplitude = np.abs(fft_values[:half_n]) * (2.0 / n_samples)
    
    return f_pos, amplitude

# 4. Creazione dei Grafici
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# --- FFT Accelerometro (X, Y, Z) ---
for col, color in zip(['Accel-X (g)', 'Accel-Y (g)', 'Accel-Z (g)'], ['red', 'green', 'blue']):
    f, amp = calcola_fft(df[col].values, fs)
    ax1.plot(f, amp, label=col, color=color, alpha=0.8)

ax1.set_title(f'Analisi in Frequenza - Accelerometro (Fs ≈ {fs:.1f} Hz)')
ax1.set_ylabel('Ampiezza')
ax1.set_xlim(0, 40)

# ---> MODIFICA ASSE VERTICALE AX1
ax1.spines['left'].set_position(('outward', 15))  # Sposta l'asse Y a sinistra (esterno) di 15 punti
ax1.spines['right'].set_visible(False)            # Nasconde il bordo destro per un look più pulito
ax1.spines['top'].set_visible(False)              # Nasconde il bordo superiore

ax1.legend()
ax1.grid(True, linestyle='--', alpha=0.6)

# --- FFT Giroscopio (X, Y, Z) ---
for col, color in zip(['Gyro-X (d/s)', 'Gyro-Y (d/s)', 'Gyro-Z (d/s)'], ['darkred', 'darkgreen', 'darkblue']):
    f, amp = calcola_fft(df[col].values, fs)
    ax2.plot(f, amp, label=col, color=color, alpha=0.8)

ax2.set_title('Analisi in Frequenza - Giroscopio')
ax2.set_ylabel('Ampiezza')
ax2.set_xlabel('Frequenza (Hz)')
ax2.set_xlim(0, 40)

# ---> MODIFICA ASSE VERTICALE AX2
ax2.spines['left'].set_position(('outward', 15))  # Sposta l'asse Y a sinistra (esterno) di 15 punti
ax2.spines['right'].set_visible(False)            # Nasconde il bordo destro
ax2.spines['top'].set_visible(False)              # Nasconde il bordo superiore

ax2.legend()
ax2.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
plt.savefig(nome_file_output, dpi=300)
plt.show()