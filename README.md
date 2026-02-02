# Charlie Robot - Autonomous Hazardous Environment Navigation

![iRobot Create 3](documentation/irobot.jpg)

## 📋 Descrizione del Progetto

**Charlie** è un robot mobile autonomo progettato per operare in ambienti interni pericolosi colpiti da fuoriuscite accidentali di sostanze tossiche. Il progetto simula uno scenario in un laboratorio chimico dove una perdita di contaminante ha compromesso la qualità dell'aria e la visibilità, rendendo pericoloso l'intervento umano.

### Missione Principale
Il robot deve:
- 🎯 Raggiungere e simulare l'attivazione del sistema di ventilazione d'emergenza (rappresentato da un target rosso)
- 🔍 Esplorare autonomamente l'ambiente navigando verso punti candidati predefiniti
- 🚧 Reagire e aggirare ostacoli imprevisti durante il percorso
- 🧍 Identificare potenziali vittime e segnalare la loro posizione ai soccorritori


## 🚀 Installazione

### Prerequisiti

- **Docker** installato
- **8GB+ RAM** consigliati
- **Browser web** per accesso VNC alla simulazione

### Procedura

1. **Clonare il repository** (con submodules):
   ```bash
   git clone --recurse-submodules https://github.com/<your-repo>/Robotica_25.git
   cd Robotica_25
   ```

2. **Build dei container Docker**:
   ```bash
   cd docker
   docker compose build
   ```
   > ⚠️ Il primo build potrebbe richiedere 15-30 minuti per compilare ROS 2 e Gazebo.

---

## ▶️ Esecuzione

### Avviare la Simulazione Completa

```bash
cd docker
docker compose up
```

### Accesso alla Simulazione Gazebo

Aprire il browser e navigare a:
```
http://localhost:8080
```
- **Password VNC**: `password`

Avviare la simulazione tramite l'esecuzione del file: 
```
~/ros2_ws/start.bash
```
### Monitoraggio dei Moduli

Visualizzare i log di ciascun container:
```bash
# Log del modulo Sense
docker logs -f charlie_sense

# Log del modulo Plan (Behavior Tree)
docker logs -f charlie_plan

# Log del modulo Act
docker logs -f charlie_act
```


### Visualizzazione della Simulazione

La simulazione può essere monitorata attraverso due strumenti:

- **Gazebo**: Visualizza l'ambiente 3D completo con il robot, gli ostacoli e i target. La finestra Gazebo si apre automaticamente dopo aver eseguito `start.bash`.

- **RViz**: Per visualizzare i dati dei sensori, la posizione del robot e i topic ROS 2:

  I topic più utili visualizzati sono:
  - `/tf` - Trasformazioni del robot
  - `/odom` - Odometria
  - `/sense/debug_image` - Feed camera


### Fermare la Simulazione

```bash
cd docker
docker compose down
```

---

## 📁 Struttura del Progetto

```
Robotica_25/
├── docker/
│   └── docker-compose.yml    # Orchestrazione container
├── documentation/
│   └── design_documentation.md
├── models/
│   └── gazebo/               # Modelli 3D per Gazebo
├── ros2_create3_sim/         # Simulatore ROS 2 + Gazebo
├── src/
│   ├── sense/                # Modulo percezione (YOLO + sensori)
│   │   ├── sense_node.py
│   │   ├── color_detector.py
│   │   └── human_detector.py
│   ├── plan/                 # Modulo decisionale (Behavior Tree)
│   │   ├── plan_node.py
│   │   └── behaviors.py
│   └── act/                  # Modulo attuazione
│       └── action_node.py
└── README.md
```

---

## 🧪 Testing

### Test Unitari
```bash
# Test del modulo Plan (Behavior Tree)
cd src/plan
python -m pytest test_plan_behaviors.py -v

# Test del modulo Sense (helper functions)
cd src/sense
python -m pytest test_sense_node.py -v
```

### Test del Color Detector
```bash
cd src/sense
python -m pytest test_color_detector.py -v
```

---

## 👥 Autori

Realizzato da:
- Samuele Castellani
- Giammarco Ubaldi
- Samuele Verna

Progetto sviluppato per il corso **ISRLAB**.

---

