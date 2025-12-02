# Navigazione di Agente Autonomo tramite Value Iteration

Questo progetto implementa un agente autonomo capace di navigare in un ambiente a griglia 2D con ostacoli, utilizzando un approccio tabellare basato sulla **Value Iteration**. L'agente controlla un robot non puntiforme (con ingombro fisico e orientamento) e apprende una politica ottima per raggiungere una posizione e un orientamento target `(x, y, theta)`, evitando collisioni e rispettando i vincoli di movimento.

## Caratteristiche Principali

* **Ambiente a Griglia:** Mondo discreto `NX x NY` con ostacoli poligonali.
* **Stato dell'Agente:** Lo stato è definito da `(x, y, theta_idx)`, dove `theta_idx` rappresenta l'orientamento discreto (72 angoli possibili).
* **Azioni dell'Agente:** L'agente dispone di tre azioni discrete: `TURN_LEFT` (Gira a Sx), `TURN_RIGHT` (Gira a Dx), `MOVE_FORWARD` (Vai Avanti).
* **Validazione Sim-to-Real:** Il progetto implementa una pipeline di valutazione che testa la politica discreta in un Ambiente Continuo, evidenziando il divario "Sim-to-Real".
* **Rilevamento Collisioni:** Utilizza la libreria `Shapely` per calcolare l'intersezione esatta tra l'ingombro ruotato del robot e gli ostacoli, inclusi i confini della mappa come barriere invalicabili.
* **Reward Shaping Avanzato:** Utilizza un sistema di ricompense e penalità per guidare l'apprendimento:
    * `R_GOAL`: Ricompensa positiva per il raggiungimento del target.
    * `R_COLLISION`: Penalità severa per collisioni con ostacoli o confini.
    * `R_STEP`: Costo per ogni passo per incentivare percorsi brevi.
    * `R_ROTATE`: Costo per le rotazioni per evitare movimenti inutili.
    * `R_DRIFT_PENALTY`: Penalità specifica per scoraggiare movimenti fisicamente non realistici ("drift") causati dalla discretizzazione della griglia, forzando manovre di guida pulite.

## Struttura del Progetto

    autonomous_agent_pjwk/
    ├── src/
    │   ├── __init__.py
    │   ├── config.py        # Parametri di configurazione (mappa, robot, ricompense)
    │   ├── environment.py   # Logica del mondo, fisica e collisioni
    │   ├── planning.py      # Algoritmo di Value Iteration
    │   └── visualizer.py    # Funzioni per generare grafici e animazioni
    ├── main.py              # Script principale per training e simulazione
    ├── README.md            # Documentazione del progetto (Versione Inglese)
    ├── README.it.md         # Documentazione del progetto (Versione Italiana)
    ├── results              # Immagini e animazioni di esperimenti multipli
    └── requirements.txt     # Dipendenze Python

## Dettagli Implementativi

### 1. Ambiente e Fisica (`src/environment.py`)
Il cuore della simulazione è la classe `Environment`, che gestisce la fisica del mondo.
* **Stato:** Ogni stato è una tupla `(x, y, theta_idx)`.
* **Transizioni:** La funzione `step(state, action)` calcola lo stato successivo applicando la cinematica del robot. I movimenti continui vengono discretizzati ("snapped") alla cella della griglia più vicina.
* **Collisioni:** La funzione `is_collision(state)` usa `Shapely` per creare un poligono ruotato che rappresenta l'ingombro esatto del robot e controlla l'intersezione con ostacoli o l'uscita dai confini della mappa.
* **Modalità di Valutazione (Continua):** I movimenti utilizzano la precisione in virgola mobile (floating-point). Il robot si muove nello spazio continuo, simulando la fisica del mondo reale dove non esiste lo "snapping" alla griglia.

### 2. Pianificazione con Value Iteration (`src/planning.py`)
Il `ValueIterationPlanner` risolve il problema di navigazione calcolando discretamente la Funzione Valore $V(s)$ per ogni stato.
* **Pre-calcolo:** Per efficienza, una `collision_map` booleana viene pre-calcolata per tutti i 720.000 stati possibili, velocizzando drasticamente il training.
* **Equazione di Bellman:** L'algoritmo itera attraverso tutti gli stati applicando l'aggiornamento:
    $$V_{k+1}(s) = \max_a [ R(s,a,s') + \gamma V_k(s') ]$$
    fino a convergenza.
* **Drift Penalty:** Durante il calcolo della ricompensa $R(s,a,s')$, viene calcolato il prodotto scalare tra il vettore di movimento inteso (basato sull'angolo) e il vettore di movimento effettivo (basato sulla griglia). Viene applicata una penalità se questi vettori divergono, scoraggiando movimenti "sporchi".

### 3. Visualizzazione (`src/visualizer.py`)
Usa `Matplotlib` per creare rappresentazioni grafiche delle politiche.
* **Grafici Statici:** Disegna il percorso completo, gli ostacoli e l'obiettivo su una griglia 2D.
* **Animazioni:** Genera GIF animate che mostrano il robot muoversi passo dopo passo.

## 4. Configurazione (`src/config.py`)

Questo file centralizza tutti i parametri modificabili.

* **Dimensioni del Mondo (`NX`, `NY`):** Definiscono la risoluzione della griglia.
* **Parametri Robot:** `ROBOT_LENGTH` e `ROBOT_WIDTH` definiscono l'ingombro fisico, mentre `DELTA_THETA_DEG` definisce la granularità di rotazione possibile (es. 5°).
* **Mappa:** `OBSTACLES_VERTICES` contiene la lista dei poligoni che formano gli ostacoli, e `GOAL_STATE` definisce la posizione e l'orientamento target.
* **Sistema di Ricompense (`R_*`):** Definisce i pesi scalari per tutti i tipi di eventi (goal, collisioni, passi, rotazioni, drift), che determinano direttamente il comportamento dell'agente.

### 5. Script Principale (`main.py`)
Il punto di ingresso dell'applicazione che orchestra l'intero processo.
* **Inizializzazione:** Crea istanze di `Environment` e `ValueIterationPlanner`.
* **Gestione Modelli:** Controlla se esistono modelli pre-addestrati (`v.npy`, `policy.npy`). Se non trovati, avvia automaticamente il pre-calcolo e il training.
* **Test e Validazione:** Testa l'agente nel mondo a griglia ideale dove ha appreso.
* **Simulazione Continua:** Testa l'agente in un mondo continuo realistico. Lo script gestisce la traduzione tra la posizione continua del robot e il lookup della policy discreta (usando l'arrotondamento al vicino più prossimo).

## Come Eseguire

1.  **Installare le dipendenze:**
    Esegui la seguente istruzione (Python richiesto):
    ```bash
    pip install -r requirements.txt
    ```

2.  **Avviare la simulazione:**
    Esegui lo script principale:
    ```bash
    python main.py
    ```
    * Al primo avvio, lo script eseguirà il **pre-calcolo della mappa collisioni** (potrebbe richiedere alcuni minuti) e l'algoritmo di **Value Iteration** (su una CPU Intel Core i3 di 7a gen, il training completo ha impiegato circa 4 ore).
    * I modelli addestrati (`v.npy`, `policy.npy`) verranno salvati per esecuzioni future più rapide.
    * Verranno eseguite diverse simulazioni di test, salvando i risultati come immagini statiche (`.png`) e animazioni (`.gif`).

## Risultati

### Risultati Finali (Policy Ottimizzata)
Questi risultati mostrano il comportamento dell'agente con la configurazione finale, che include una severa penalità per il drift (`R_DRIFT_PENALTY = -90.0`) e controlli corretti dei confini della mappa. L'agente dimostra una guida pulita e sicura.

| Simulazione da (10, 10, 0°) - Statica | Simulazione da (10, 10, 0°) - Animazione |
| :---: | :---: |
| <img src="results/results_without_drift/Simulazione_da_10_10_0.png" width="100%"> | <img src="results/results_without_drift/Animazione_da_10_10_0.gif" width="100%"> |

| Simulazione da (50, 50, 90°) - Statica | Simulazione da (50, 50, 90°) - Animazione |
| :---: | :---: |
| <img src="results/results_without_drift/Simulazione_da_50_50_18.png" width="100%"> | <img src="results/results_without_drift/Animazione_da_50_50_18.gif" width="100%"> |

| Simulazione da (70, 72, 0°) - Statica | Simulazione da (70, 72, 0°) - Animazione |
| :---: | :---: |
| <img src="results/results_without_drift/Simulazione_da_70_72_0.png" width="100%"> | <img src="results/results_without_drift/Animazione_da_70_72_0.gif" width="100%"> |

---

### Analisi dei Problemi Risolti
Durante lo sviluppo, sono state affrontate due sfide critiche che compromettevano il realismo della simulazione.

#### 1. Uscita dai Confini della Mappa
Inizialmente, il sistema controllava solo se il *centro* del robot fosse all'interno della griglia. Questo permetteva al robot, avendo un ingombro fisico, di "sfondare" parzialmente i muri esterni con il perimetro.
* **Soluzione:** È stato implementato un rigoroso controllo geometrico in `environment.py` che verifica se l'intero poligono del robot (ingombro) è contenuto nei confini del mondo, trattando i bordi della mappa come muri invalicabili.

#### 2. "Drifting" e Movimenti Irrealistici
A causa della discretizzazione della griglia, l'agente poteva muoversi in una direzione (es. Nord) pur essendo orientato leggermente in modo diverso (es. 85°), creando un effetto irrealistico di "drift" o scivolamento laterale.
* **Soluzione:** È stata introdotta una **Drift Penalty** (`R_DRIFT_PENALTY`). Questa penalità calcola il prodotto scalare tra il vettore di movimento inteso (basato sull'angolo) e quello effettivo (sulla griglia). Impostando una penalità molto alta (-90.0), l'agente è stato costretto ad imparare che solo i movimenti perfettamente allineati sono accettabili, eliminando completamente il comportamento di drift.

### Comportamento Prima delle Correzioni
Questo esempio mostra il comportamento dell'agente *prima* delle correzioni sopra citate. Notare come il robot tenda a "scivolare" lateralmente nelle curve strette per evitare rotazioni complesse e come possa uscire parzialmente dai confini superiori della mappa in alcuni casi.

| Simulazione con Problemi - Statica | Simulazione con Problemi - Animazione |
| :---: | :---: |
| <img src="results/breaking_boundries_and_low_drift_penalty/Simulazione_da_10_10_0.png" width="100%"> | <img src="results/breaking_boundries_and_low_drift_penalty/Animazione_da_10_10_0(1).gif" width="100%"> |

---

### Analisi del Divario Sim-to-Real
Un obiettivo chiave di questo progetto è valutare come una politica appresa in un mondo discreto performi in un ambiente continuo.

* **Casi di Successo:** In aree aperte (es. partendo da `50, 50` o `70, 72`), l'agente naviga con successo in modalità continua. La "Drift Penalty" ha insegnato efficacemente all'agente ad allinearsi con gli assi della griglia, minimizzando gli errori di traiettoria.
* **Casi Limite:** In spazi ristretti (es. partendo da `10, 10` vicino agli ostacoli), la simulazione continua può fallire dove quella discreta ha successo.

| Simulazione Continua da (10, 10, 0°) - Statica | Simulazione Continua da (10, 10, 0°) - Animazione |
| :---: | :---: |
| <img src="results/continous_simulations/Sim_1_Continuous_10_10_0.png" width="100%"> | <img src="results/continous_simulations/Anim_1_Continuous_10_10_0.gif" width="100%"> |

| Simulazione Continua da (50, 50, 90°) - Statica | Simulazione Continua da (50, 50, 90°) - Animazione |
| :---: | :---: |
| <img src="results/continous_simulations/Sim_2_Continuous_50_50_18.png" width="100%"> | <img src="results/continous_simulations/Anim_2_Continuous_50_50_18.gif" width="100%"> |

| Simulazione Continua da (70, 72, 0°) - Statica | Simulazione Continua da (70, 72, 0°) - Animazione |
| :---: | :---: |
| <img src="results/continous_simulations/Sim_3_Continuous_70_72_0.png" width="100%"> | <img src="results/continous_simulations/Anim_3_Continuous_70_72_0.gif" width="100%"> |