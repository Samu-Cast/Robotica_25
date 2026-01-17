# Documentazione: Plan Node (`plan_node.py`)

Il **Plan Node** è il cervello strategico del robot "Charlie". Utilizza un **Behavior Tree (BT)** per prendere decisioni in tempo reale basate sui dati dei sensori e coordina l'esecuzione della missione principale: trovare e attivare una valvola.

## 1. Architettura Generale
Il nodo funge da ponte tra la percezione (`sense`) e l'azione (`act`).
*   **Input (Iscrizioni ROS2):**
    *   `/clock`: Sincronizzazione startup (attende che la simulazione sia attiva).
    *   `/sense/proximity/*`: Dati dai sensori a ultrasuoni (distanze).
    *   `/sense/odometry`: Posizione stimata del robot (`x`, `y`, `theta`).
    *   `/sense/detection`: Rilevamenti visivi (persone, target colorati, ostacoli) dalla telecamera.
    *   `/sense/battery`: Livello batteria.
*   **Output (Pubblicazioni ROS2):**
    *   `/plan/command`: Comandi di movimento testuali per il nodo Act (es. "Front", "Left", "Stop").
    *   `/plan/signals`: Segnali di eventi importanti (es. "PersonFound", "ValveActivated").

---

## 2. Sincronizzazione Startup
Il nodo **attende** che la simulazione Gazebo sia pronta prima di iniziare a inviare comandi:

1.  Si iscrive al topic `/clock` (pubblicato da Gazebo).
2.  Quando riceve un messaggio con `clock.sec > 0`, sa che la simulazione è attiva.
3.  Attende ulteriori **10 secondi** per permettere al robot di stabilizzarsi (spawn completo, sensori inizializzati).
4.  Solo dopo avvia il tick del Behavior Tree a 10Hz.

Questo previene l'invio di comandi prima che il robot sia pronto.

---

## 3. Behavior Tree (Albero Comportamentale)
La logica è strutturata gerarchicamente. Ogni nodo del BT viene eseguito (tick) a **10Hz**.

### Struttura Principale (`build_tree`)
L'albero segue una **Sequenza Principale** di priorità:
1.  **InitialRetreat**: Arretramento iniziale dalla piattaforma di spawn (eseguito una sola volta per 4 secondi).
2.  **Battery Management**: Controllo prioritario. Se la batteria è < 20%, il robot ignora tutto il resto e torna alla base per ricaricarsi (`GoCharge`).
3.  **Mission Loop (TargetSearch)**:
    *   **Calcolo Target**: Sceglie il prossimo obiettivo non visitato.
    *   **Navigazione (`GoToTarget`)**: Si muove verso il target *mentre* cerca oggetti visivamente (`Parallel`).
    *   **Verifica**: Controlla se il target raggiunto è la valvola rossa.
4.  **ActiveValve**: Se la valvola è trovata, la attiva.
5.  **GoHome**: Ritorno alla piattaforma iniziale a missione conclusa.

---

## 4. Logiche Dettagliate

### A. Selezione del Target (`CalculateTarget`)
Il robot possiede una mappa statica di 3 punti di interesse (`KNOWN_TARGETS`: Verde, Blu, Rosso).
*   Non naviga a caso: sceglie sempre il target **più vicino in linea d'aria** tra quelli non ancora visitati.
*   Utilizza la posizione odometrica *corretta* (vedi punto C) per il calcolo delle distanze.

### B. Navigazione e Evitamento (`MoveToTarget`)
Gestisce il movimento verso il target selezionato.
*   **Hysteresis**: Mantiene la direzione corrente finché l'errore angolare non supera una certa soglia, evitando oscillazioni (jitter).
*   **Evitamento Ostacoli**:
    *   **Ultrasuoni**: Se un ostacolo è troppo vicino (< 35cm frontale o < 25cm laterale), il robot entra in modalità `AVOID` e gira verso lo spazio libero. Dopo aver evitato, procede in avanti per ~1.5s per superare l'ostacolo prima di ricalcolare il percorso.
    *   **Camera**: Se rileva una persona o un ostacolo visivo con alta confidenza (> 50%) e vicino (< 2m), devia preventivamente basandosi sulla zona di rilevamento (sinistra/destra).

### C. Approccio di Precisione (`AtTarget`)
Quando il robot è vicino al target (< 0.5m), attiva una macchina a stati in **5 Fasi** per l'allineamento:
1.  **Fase 0 (Rotate & Approach)**: Ruota verso il target usando le coordinate note e avanza fino al contatto quasi fisico (< 0.6m dai sensori o < 0.15m da odometria).
2.  **Fase 1 (Visual Centering)**: Usa la telecamera per ruotare finché l'oggetto colorato non è al centro dell'immagine.
3.  **Fase 2 (Sensor Align)**: Usa i sensori laterali (Left/Right) per allinearsi perpendicolarmente (differenza < 5cm).
4.  **Fase 3 (Distance Fix)**: Usa il sensore centrale per portarsi a **0.35m** di distanza.
5.  **Fase 4 (Odom Reset)**: Una volta in posizione, ricalcola l'errore odometrico (`odom_correction`) basandosi sulle coordinate note del target.

---

## 5. Blackboard e Traduzione Comandi
*   **Blackboard**: È la memoria condivisa del BT. I sensori scrivono qui i dati, i nodi decisionali leggono e scrivono l'azione scelta (`plan_action`).
*   **Traduzione**: Un dizionario `ACTION_TO_COMMAND` converte le azioni logiche interne (es. `MOVE_FRONT_LEFT`) nei comandi specifici per il driver dei motori (es. `"FrontLeft"`).

---

## 6. Riepilogo Classi

| Classe | Tipo | Descrizione |
|--------|------|-------------|
| `BatteryCheck` | Condition | Verifica batteria > 20% |
| `GoCharge` | Action | Naviga verso la stazione di ricarica |
| `CalculateTarget` | Action | Seleziona il prossimo target non visitato |
| `AtTarget` | Action | Allineamento preciso in 5 fasi |
| `InitialRetreat` | Action | Arretramento iniziale (4s) |
| `MoveToTarget` | Action | Navigazione con evitamento ostacoli |
| `SearchObj` | Action | Aggiorna `found` in base ai rilevamenti |
| `RecognitionPerson/Obstacle/Valve` | Condition | Controlla il valore di `found` |
| `SignalPerson` | Action | Aggiunge "PersonFound" ai segnali |
| `GoAroundP/O` | Action | Evita persona/ostacolo basandosi sulla zona |
| `ActiveValve` | Action | Attiva la valvola, missione completata |
| `GoHome` | Action | Ritorna alla posizione iniziale |

