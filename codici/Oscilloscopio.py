import pandas as pd
import matplotlib.pyplot as plt

# --- PARAMETRO DA DECIDERE ---
n = 2266 
nome_file_output = 'grafico_robot3.png' 
# -----------------------------

# 1. Caricamento dei dati
df = pd.read_csv('Robot3.csv')

# Pulizia: rimuove eventuali spazi bianchi nei nomi delle colonne
df.columns = df.columns.str.strip()

# Prende solo le prime n righe
df = df.head(n)

# Converte la colonna Time in formato datetime
df['Time'] = pd.to_datetime(df['Time'])

# 2. Creazione della figura
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# --- Grafico Accelerometro ---
ax1.plot(df['Time'], df['Accel-X (g)'], label='Accel-X', color='red', alpha=0.8)
ax1.plot(df['Time'], df['Accel-Y (g)'], label='Accel-Y', color='green', alpha=0.8)
ax1.plot(df['Time'], df['Accel-Z (g)'], label='Accel-Z', color='blue', alpha=0.8)
ax1.set_title(f'Dati Accelerometro (Prime {n} righe)')
ax1.set_ylabel('Accelerazione (g)')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.6)

# --- Grafico Giroscopio ---
ax2.plot(df['Time'], df['Gyro-X (d/s)'], label='Gyro-X', color='darkred', alpha=0.8)
ax2.plot(df['Time'], df['Gyro-Y (d/s)'], label='Gyro-Y', color='darkgreen', alpha=0.8)
ax2.plot(df['Time'], df['Gyro-Z (d/s)'], label='Gyro-Z', color='darkblue', alpha=0.8)
ax2.set_title(f'Dati Giroscopio (Prime {n} righe)')
ax2.set_ylabel('Velocità Angolare (d/s)')
ax2.set_xlabel('Tempo')
ax2.legend(loc='upper right')
ax2.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()

# --- SALVATAGGIO NELLA CARTELLA CORRENTE ---
# dpi=300 serve per avere un'alta risoluzione, ottima per le relazioni
plt.savefig(nome_file_output, dpi=300)
print(f"Grafico salvato correttamente come: {nome_file_output}")

# Mostra il grafico a video
plt.show()