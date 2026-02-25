# Charlie Robot - Autonomous Hazardous Environment Navigation

## Descrizione del Progetto

**Charlie** è un robot mobile autonomo progettato per operare in ambienti interni pericolosi colpiti da fuoriuscite accidentali di sostanze tossiche. Il progetto simula uno scenario in un laboratorio chimico dove una perdita di contaminante ha compromesso la qualità dell'aria e la visibilità, rendendo pericoloso l'intervento umano.

### Missione Principale
Il robot deve:
- Raggiungere e simulare l'attivazione del sistema di ventilazione d'emergenza (rappresentato da un target rosso)
- Esplorare autonomamente l'ambiente navigando verso punti candidati predefiniti
- Reagire e aggirare ostacoli imprevisti durante il percorso
- Identificare potenziali vittime e segnalare la loro posizione ai soccorritori

---

## Struttura del Repository

Il progetto è organizzato in due cartelle principali, una per la simulazione e una per il robot fisico:

```
Robotica_25/
├── Charlie_simulation/       # Versione simulata (Gazebo + Docker)
│   ├── docker/
│   │   └── docker-compose.yml
│   ├── models/gazebo/        # Modelli 3D per Gazebo
│   ├── ros2_create3_sim/     # Simulatore ROS 2 + Gazebo
│   ├── scripts/
│   │   └── demo.py
│   └── src/
│       ├── sense/            # Modulo percezione (YOLO + sensori)
│       ├── plan/             # Modulo decisionale (Behavior Tree)
│       └── act/              # Modulo attuazione
│
├── Charlie_physical/         # Versione robot fisico (iRobot Create 3 + Jetson Nano)
│   ├── docker/
│   │   └── docker-compose.yml
│   ├── scripts/
│   │   ├── configuration.bash
│   │   ├── run_jetson.bash
│   │   └── fotocamera.bash
│   └── src/
│       ├── sense/
│       ├── plan/
│       └── act/
│
├── documentation/
│   └── design_documentation.md
└── README.md
```

---

## Charlie Simulation

### Prerequisiti
- **Docker** installato
- **8GB+ RAM** consigliati
- **Browser web** per accesso VNC alla simulazione

### Installazione

1. **Clonare il repository** (con submodules):
   ```bash
   git clone --recurse-submodules https://github.com/Samu-Cast/Robotica_25.git
   cd Robotica_25
   ```

2. **Build dei container Docker**:
   ```bash
   cd Charlie_simulation/docker
   docker compose build
   ```
   > ⚠️ Il primo build potrebbe richiedere 15-30 minuti per compilare ROS 2 e Gazebo.

### Esecuzione

1. **Avviare i container**:
   ```bash
   cd Charlie_simulation/docker
   docker compose up
   ```

2. **Accesso alla simulazione Gazebo** — aprire il browser e navigare a:
   ```
   http://localhost:8080
   ```
   - **Password VNC**: `password`

3. **Avviare la simulazione** tramite il terminale VNC:
   ```bash
   ~/ros2_ws/start.bash
   ```

### Monitoraggio dei Moduli

```bash
# Log del modulo Sense
docker logs -f charlie_sense

# Log del modulo Plan (Behavior Tree)
docker logs -f charlie_plan

# Log del modulo Act
docker logs -f charlie_act
```

### Visualizzazione

- **Gazebo**: Visualizza l'ambiente 3D completo con il robot, gli ostacoli e i target. Si apre automaticamente dopo `start.bash`.
- **RViz**: Topic utili visualizzabili: `/tf`, `/odom`, `/sense/debug_image`.

### Fermare la Simulazione

```bash
cd Charlie_simulation/docker
docker compose down
```

---

## Charlie Physical

### Piattaforma Hardware

| Componente | Dettaglio |
|---|---|
| **Robot** | [iRobot Create 3](https://edu.irobot.com/create3) |
| **Computer di bordo** | NVIDIA Jetson Nano |
| **Fotocamera** | IMX219 (CSI camera module) |

### Prerequisiti

- **Jetson Nano 4gb** con microSD (minimo 32 GB consigliati)
- **iRobot Create 3** acceso e connesso alla stessa rete Wi-Fi
- **Camera CSI IMX219** collegata al Jetson Nano
- Alimentazione adeguata per il Jetson Nano (power bank 5V 3A o superiore)

### Setup del Jetson Nano

1. **Flash dell'immagine sulla microSD**:
   - Scaricare l'immagine ufficiale NVIDIA Jetson Nano Developer Kit SD Card Image dal sito:
     [https://developer.nvidia.com/embedded/downloads](https://developer.nvidia.com/embedded/downloads)
   - Flashare l'immagine sulla microSD utilizzando [balenaEtcher](https://etcher.balena.io/) o un tool equivalente.
   - Inserire la microSD nel Jetson Nano e completare il primo avvio.

2. **Configurazione dell'ambiente**:
   Eseguire lo script di configurazione che installa Docker, docker-compose e clona il repository:
   ```bash
   cd Charlie_physical/scripts
   bash configuration.bash
   ```

### Configurazione di Rete e Middleware

1. **Connessione alla rete**: assicurarsi che il Jetson Nano e l'iRobot Create 3 siano connessi alla **stessa rete Wi-Fi**.

2. **Impostare il middleware DDS sul robot**: connettersi all'**Access Point** del Create 3 e, dalla pagina di configurazione web, impostare il middleware RMW su **CycloneDDS** (`rmw_cyclonedds_cpp`).
   > 💡 Lato Jetson non è necessario esportare manualmente la variabile `RMW_IMPLEMENTATION`: è già definita nel `docker-compose.yml`.

### Esecuzione

Avviare il sistema completo (fotocamera + container Docker) eseguendo:
```bash
cd Charlie_physical/scripts
bash run_jetson.bash
```

Lo script avvia in sequenza:
1. `camera_host.py` — acquisizione immagini dalla fotocamera IMX219
2. `docker-compose up` — avvio dei container Sense, Plan e Act

### Fermare il Robot

```bash
cd Charlie_physical/docker
docker-compose down
```

---

## Testing

### Test Unitari
```bash
# Test del modulo Plan (Behavior Tree)
cd Charlie_simulation/src/plan
python -m pytest test_plan_behaviors.py -v

# Test del modulo Sense (helper functions)
cd Charlie_simulation/src/sense
python -m pytest test_sense_node.py -v
```

### Test del Color Detector
```bash
cd Charlie_simulation/src/sense
python -m pytest test_color_detector.py -v
```

---

## Autori

Realizzato da:
- Samuele Castellani
- Giammarco Ubaldi
- Samuele Verna

Progetto sviluppato per il corso **Intelligent Systems and Robotics Laboratory - University of L'Aquila**. (https://www.disim.univaq.it/)