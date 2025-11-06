# Autonomous Agent Navigation using Value Iteration

Questo progetto implementa un agente autonomo in grado di navigare in un ambiente 2D a griglia con ostacoli, utilizzando un approccio tabulare basato su **Value Iteration**. L'agente controlla un robot non puntiforme (con ingombro fisico e orientamento) e apprende una policy ottimale per raggiungere una posizione e un orientamento target `(x, y, theta)`, evitando collisioni e rispettando i vincoli di movimento.

## Caratteristiche Principali

* **Ambiente a Griglia:** Mondo discreto `NX x NY` con ostacoli poligonali.
* **Stato Agente:** Lo stato dell'agente è definito da `(x, y, theta_idx)`, dove `theta_idx` rappresenta l'orientamento discreto (72 angoli possibili).
* **Azioni Agente:** L'agente ha tre azioni discrete: `TURN_LEFT`, `TURN_RIGHT`, `MOVE_FORWARD`.
* **Collision Detection:** Utilizza la libreria `Shapely` per calcolare l'intersezione esatta tra il footprint ruotato del robot e gli ostacoli, includendo anche i bordi della mappa come barriere invalicabili.
* **Reward Shaping Avanzato:** Utilizza un sistema di ricompense e penalità per guidare l'apprendimento:
    * `R_GOAL`: Ricompensa positiva per il raggiungimento del target.
    * `R_COLLISION`: Penalità severa per le collisioni con ostacoli o bordi.
    * `R_STEP`: Costo per ogni passo per incentivare percorsi brevi.
    * `R_ROTATE`: Costo per le rotazioni per evitare movimenti inutili.
    * `R_DRIFT_PENALTY`: Penalità specifica per scoraggiare movimenti fisicamente irrealistici ("drift") causati dalla discretizzazione della griglia, forzando manovre di guida pulite.

## Struttura del Progetto

    autonomous_agent_pjwk/
    ├── src/
    │   ├── __init__.py
    │   ├── config.py        # Parametri di configurazione (mappa, robot, reward)
    │   ├── environment.py   # Logica del mondo, fisica e collisioni
    │   ├── planning.py      # Algoritmo di Value Iteration
    │   └── visualizer.py    # Funzioni per generare plot e animazioni
    ├── main.py              # Script principale per training e simulazione
    ├── README.md            # Documentazione del progetto (Versione Inglese)
    ├── README.it.md         # Documentazione del progetto (Versione Italiana)
    └── requirements.txt     # Dipendenze Python

## Dettagli Implementativi

### 1. Ambiente e Fisica (`src/environment.py`)
Il cuore della simulazione è la classe `Environment`, che gestisce la fisica del mondo discreto.
* **Stato:** Ogni stato è una tupla `(x, y, theta_idx)`.
* **Transizioni:** La funzione `step(state, action)` calcola il prossimo stato applicando la cinematica del robot. I movimenti continui vengono discretizzati ("snapped") alla cella più vicina della griglia.
* **Collisioni:** La funzione `is_collision(state)` utilizza `Shapely` per creare un poligono ruotato che rappresenta l'ingombro esatto del robot e verifica l'intersezione con gli ostacoli o l'uscita dai bordi della mappa.

### 2. Pianificazione con Value Iteration (`src/planning.py`)
Il `ValueIterationPlanner` risolve il problema di navigazione calcolando la Value Function $V(s)$ per ogni stato.
* **Pre-calcolo:** Per efficienza, una `collision_map` booleana viene pre-calcolata per tutti i 720.000 stati possibili, velocizzando drasticamente l'addestramento.
* **Equazione di Bellman:** L'algoritmo itera attraverso tutti gli stati applicando l'aggiornamento:
    $$V_{k+1}(s) = \max_a [ R(s,a,s') + \gamma V_k(s') ]$$
    fino a convergenza.
* **Drift Penalty:** Durante il calcolo delle ricompense $R(s,a,s')$, viene calcolato il prodotto scalare tra il vettore di movimento intenzionale (basato sull'angolo) e il vettore di movimento reale (basato sulla griglia). Una penalità viene applicata se questi vettori divergono, scoraggiando i movimenti "sporchi".

### 3. Visualizzazione (`src/visualizer.py`)
Utilizza `Matplotlib` per creare rappresentazioni grafiche delle policy.
* **Plot Statici:** Disegna il percorso completo, gli ostacoli e il goal su una griglia 2D.
* **Animazioni:** Genera GIF animate che mostrano il robot muoversi passo dopo passo.

## 4. Configurazione (`src/config.py`)

Questo file centralizza tutti i parametri modificabili.

* **Dimensioni del Mondo (`NX`, `NY`):** Definiscono la risoluzione della griglia.
* **Parametri Robot:** `ROBOT_LENGTH` e `ROBOT_WIDTH` definiscono l'ingombro fisico, mentre `DELTA_THETA_DEG` definisce la granularità delle rotazioni possibili (es. 5°).
* **Mappa:** `OBSTACLES_VERTICES` contiene la lista dei poligoni che formano gli ostacoli e `GOAL_STATE` definisce la posizione e l'orientamento target.
* **Sistema di Ricompense (`R_*`):** Definisce i pesi scalari per tutti i tipi di eventi (goal, collisioni, passi, rotazioni, drift), che determinano direttamente il comportamento dell'agente.

### 5. Main Script (`main.py`)
Il punto di ingresso dell'applicazione che orchestra l'intero processo.
* **Inizializzazione:** Crea le istanze di `Environment` e `ValueIterationPlanner`.
* **Gestione Modello:** Controlla se esistono modelli pre-addestrati (`v.npy`, `policy.npy`). Se non li trova, avvia automaticamente il pre-calcolo e il training.
* **Testing e Validazione:** Esegue una suite di test automatici per verificare che la policy in stati chiave (es. vicino al goal o agli ostacoli) sia logica.
* **Simulazione:** Esegue episodi di prova partendo da diversi stati iniziali e utilizza il visualizzatore per generare i risultati grafici.

## Come Eseguire

1.  **Installare le dipendenze:**
    Assicurarsi di avere Python installato, quindi eseguire:
    ```bash
    pip install -r requirements.txt
    ```

2.  **Avviare la simulazione:**
    Eseguire lo script principale:
    ```bash
    python main.py
    ```
    * Al primo avvio, lo script eseguirà il **pre-calcolo della mappa delle collisioni** (può richiedere alcuni minuti) e l'algoritmo di **Value Iteration** (su una CPU Intel Core i3 di 7ª generazione, l'addestramento completo ha richiesto circa 4 ore).
    * I modelli addestrati (`v.npy`, `policy.npy`) verranno salvati per esecuzioni future più rapide.
    * Verranno eseguite diverse simulazioni di test, salvando i risultati come immagini statiche (`.png`) e animazioni (`.gif`).


## Risultati

### Risultati Finali (Policy Ottimizzata)
Questi risultati mostrano il comportamento dell'agente con la configurazione finale, che include una penalità severa per il drift (`R_DRIFT_PENALTY = -90.0`) e controlli corretti per i bordi della mappa. L'agente dimostra una guida pulita e sicura.

| Simulazione da (10, 10, 0°) - Statico | Simulazione da (10, 10, 0°) - Animazione |
| :---: | :---: |
| <img src="img/results_without_drift/Simulazione_da_10_10_0.png" width="100%"> | <img src="img/results_without_drift/Animazione_da_10_10_0.gif" width="100%"> |

| Simulazione da (50, 50, 90°) - Statico | Simulazione da (50, 50, 90°) - Animazione |
| :---: | :---: |
| <img src="img/results_without_drift/Simulazione_da_50_50_18.png" width="100%"> | <img src="img/results_without_drift/Animazione_da_50_50_18.gif" width="100%"> |

| Simulazione da (70, 72, 0°) - Statico | Simulazione da (70, 72, 0°) - Animazione |
| :---: | :---: |
| <img src="img/results_without_drift/Simulazione_da_70_72_0.png" width="100%"> | <img src="img/results_without_drift/Animazione_da_70_72_0.gif" width="100%"> |

---

### Analisi dei Problemi Risolti
Durante lo sviluppo, sono state affrontate due sfide critiche che compromettevano il realismo della simulazione.

#### 1. Uscita dai Bordi della Mappa
Inizialmente, il sistema controllava solo se il *centro* del robot si trovasse all'interno della griglia. Questo permetteva al robot, avendo un ingombro fisico, di "sfondare" parzialmente i muri esterni della mappa con il suo perimetro.
* **Soluzione:** È stato implementato un controllo geometrico rigoroso in `environment.py` che verifica se l'intero poligono del robot (footprint) è contenuto all'interno dei confini del mondo, trattando i bordi della mappa come muri invalicabili.

#### 2. "Drifting" e Movimenti Irrealistici
A causa della discretizzazione della griglia, l'agente poteva muoversi in una direzione (es. Nord) pur essendo orientato leggermente diversamente (es. 85°), creando un effetto di "drift" o scivolamento laterale irrealistico.
* **Soluzione:** È stata introdotta una **Drift Penalty** (`R_DRIFT_PENALTY`). Questa penalità calcola il prodotto scalare tra il vettore di movimento intenzionale (basato sull'angolo) e quello reale (sulla griglia). Impostando una penalità molto alta (-90.0), l'agente è stato forzato ad apprendere che solo i movimenti perfettamente allineati sono accettabili, eliminando completamente il comportamento di drift.

### Comportamento Prima delle Correzioni
Questo esempio mostra il comportamento dell'agente *prima* delle correzioni sopra descritte. Si noti come il robot tenda a "scivolare" lateralmente nelle curve strette per evitare rotazioni complesse e come in alcuni casi possa parzialmente uscire dai bordi superiori della mappa.

| Simulazione con Problemi - Statico | Simulazione con Problemi - Animazione |
| :---: | :---: |
| <img src="img/breaking_boundries_and_low_drift_penalty/Simulazione_da_10_10_0.png" width="100%"> | <img src="img/breaking_boundries_and_low_drift_penalty/Animazione_da_10_10_0(1).gif" width="100%"> |