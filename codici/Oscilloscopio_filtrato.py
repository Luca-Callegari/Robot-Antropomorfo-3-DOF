import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURAZIONE ---
n = 3000  # Numero di righe da analizzare
nome_file_output = 'grafico_robot3_filtrato.png'

# Parametri Kalman (puoi regolarli se vuoi più o meno "morbidezza")
Q = 1e-5  # Incertezza del processo
R = 0.01  # Incertezza della misura (rumore sensore)
# ----------------------

# 1. Caricamento e pulizia dati
df = pd.read_csv('Robot3.csv', skipinitialspace=True)
df.columns = df.columns.str.strip()
df = df.head(n)
df['Time'] = pd.to_datetime(df['Time'])

# Funzione Filtro di Kalman 1D
def kalman_filter(data, Q, R):
    n_iter = len(data)
    x_hat = np.zeros(n_iter)
    P = np.zeros(n_iter)
    x_hat[0] = data[0]
    P[0] = 1.0
    for k in range(1, n_iter):
        # Predict
        x_hat_minus = x_hat[k-1]
        P_minus = P[k-1] + Q
        # Correct
        K = P_minus / (P_minus + R)
        x_hat[k] = x_hat_minus + K * (data[k] - x_hat_minus)
        P[k] = (1 - K) * P_minus
    return x_hat

# Identificazione delle colonne
colonne_sensori = [
    'Accel-X (g)', 'Accel-Y (g)', 'Accel-Z (g)',
    'Gyro-X (d/s)', 'Gyro-Y (d/s)', 'Gyro-Z (d/s)'
]

# 2. Creazione della figura (3 righe, 2 colonne)
fig, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
fig.suptitle(f'Analisi Sensori con Filtro di Kalman (Prime {n} righe)', fontsize=16)

# Colori per i grafici
colors = ['red', 'green', 'blue', 'darkred', 'darkgreen', 'darkblue']

# Loop per filtrare e graficare ogni sensore
for i, col in enumerate(colonne_sensori):
    # Applica il filtro
    dati_filtrati = kalman_filter(df[col].values, Q, R)
    
    # Trova la posizione corretta nella griglia (riga, colonna)
    riga = i % 3
    colonna_plot = i // 3
    ax = axes[riga, colonna_plot]
    
    # Plot segnale originale (chiaro) e filtrato (scuro)
    ax.plot(df['Time'], df[col], label='Originale', color=colors[i], alpha=0.3)
    ax.plot(df['Time'], dati_filtrati, label='Kalman', color=colors[i], linewidth=1.5)
    
    ax.set_title(col)
    ax.legend(loc='upper right', fontsize='small')
    ax.grid(True, linestyle='--', alpha=0.5)

# Aggiunta etichette assi X finali
axes[2, 0].set_xlabel('Tempo')
axes[2, 1].set_xlabel('Tempo')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig(nome_file_output, dpi=300)
plt.show()