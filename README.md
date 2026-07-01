# 3DOF-Manipulator-Control 🤖⚙️

Questo repository raccoglie lo studio dinamico e la progettazione del sistema di controllo per un **manipolatore robotico industriale a 3 gradi di libertà (3 DOF)**. Il progetto affronta l'intero flusso ingegneristico: dalla definizione della geometria della struttura fino alla taratura di un controllore in grado di muovere il braccio robotico annullando le forze di gravità e i disturbi esterni.

---

## 1. Architettura e Modellistica del Robot

Il robot analizzato è un braccio articolato spaziale con tre giunti rotazionali. Per mappare il suo comportamento ci siamo mossi attraverso tre fasi principali:

* **Cinematica Diretta:** Abbiamo utilizzato la convenzione di *Denavit-Hartenberg* per mappare il passaggio dalle coordinate dei singoli giunti (gli angoli di rotazione) alla posizione finale dell'organo terminale (l'end-effector) nello spazio cartesiano tridimensionale.
* **Dinamica del Sistema:** Per ricavare le equazioni del moto abbiamo sfruttato l'approccio energetico di *Eulero-Lagrange*. Questo ci ha permesso di ricavare la matrice d'inerzia dei link, i termini centrifughi/Coriolis e l'effetto della gravità agenti sulla struttura in base alle masse reali del robot.
* **Pianificazione delle Traiettorie:** Il robot è stato testato su compiti di tracking, ovvero compiti in cui l'end-effector deve seguire una traiettoria fluida nel tempo all'interno del suo spazio di lavoro operativo.

---

## 2. Le Sfide di Controllo e Soluzioni Adottate

Durante lo sviluppo ci siamo scontrati con alcune problematiche tipiche della robotica reale, risolte implementando strategie matematiche e di controllo mirate:

### 🎯 La gestione delle singolarità (Damped Least Squares)
Quando si calcola la cinematica inversa per far seguire una traiettoria al robot, si utilizza la matrice Jacobiana. Con parametri fisici reali (link molto corti e masse ridotte), il metodo del gradiente classico fallisce a causa di Jacobiane malcondizionate o vicine a punti di singolarità geometrica. 
Per superare questo blocco numerico, abbiamo adottato l'algoritmo **Damped Least Squares (pseudoinversa smorzata)**, che garantisce la convergenza matematica e la stabilità dei calcoli anche vicino alle singolarità dello spazio di lavoro.

### 🎛️ Taratura del Controllore PID
Il cuore del controllo del movimento si basa su una struttura **PID (Proporzionale-Integrale-Derivativo)** per ogni giunto, la cui equazione classica nel dominio del tempo è:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

L'analisi nello spazio di stato a ciclo chiuso ha evidenziato tre pilastri fondamentali per il bilanciamento dei guadagni:
1. **Azione Derivativa ($K_d$):** È fondamentale per lo smorzamento. Un valore insufficiente porta il braccio a oscillare vistosamente o a diventare instabile.
2. **Azione Integrale ($K_i$):** È l'elemento chiave per azzerare l'errore di posizione a regime. Senza la componente integrale, la forza di gravità costante impedirebbe al robot di raggiungere l'altezza desiderata. Tuttavia, un valore eccessivo rischia di destabilizzare la risposta dinamica.
3. **Coppia di Guadagni Ottimale:** Attraverso l'analisi dei poli del sistema, abbiamo individuato una configurazione di terna ideale (es. $K_p = 64, K_d = 8, K_i = 24$) in grado di garantire un movimento fluido, privo di sovraelongazioni pericolose e robusto rispetto ai disturbi.

---

## 3. Struttura del Progetto

Il framework permette di analizzare l'andamento delle coppie ai giunti e l'errore di tracking cartesiano, dimostrando come un controllo PID ben tarato, unito a una corretta gestione dell'inversione cinematica, sia perfettamente in grado di domare le non linearità naturali di un manipolatore seriale.

---
**Autori:** Joan Reynaldo Bautista Delgado & Luca Callegari  
**Corso:** Meccanica delle Vibrazioni / Robotica  
**Università:** Università degli Studi di Roma "Tor Vergata"
