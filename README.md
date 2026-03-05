# Spiegazione dei file per cartelle

## Carla utils

'building_extraction.py' è uno script fatto in python che si collega alla mappa di Carla e salva tutte le dimensioni degli edifici della suddetta mappa.

Generato da ChatGPT con il prompt "I want to recreate the building of carla in sumo, I need to extract information of building and convert it to a file of format .poly.xml, take reference to retrieve buildings information from carla.readthedocs.io/en/latest/core_map/ , and for the format of the file from: https://sumo.dlr.de/docs/Simulation/Shapes.html"

## DrivingSimulatorCarlaControl

Qui ci sono i file modificati del simulatore di base fornitovi da Sebastiano.

### Generate traffic

In questa versione 'generate_traffic.py' fa solo da "broker" tra la simulazione di Sumo e Carla, in modo che Carla possa andare senza che una simulazione di Sumo sia obligatoria e si è dovuto stravolgere la logica del Simulatore.

### Carla Slave

Uguale a quello base, cambia solo la logica di spawn dei veicoli.

ATTENZIONE: Dato che i modelli dei veicoli vengono scelti randomicamente per assicurarmi che in tutti e tre le simulazioni il rand è inizializzato con lo stesso seed presente nei settings. Un possibile miglioramento, dato che come tipo veicolo usiamo quelli di Carla anche in Sumo, è quello di unire anche il tipo veicolo ai messaggi in modo tale che le dimensioni che vediamo in Carla sono corrispondenti a quelle in Sumo.

###  Carla Master

Oltre alla logica di spawn e update dei veicoli, nella funzione 'advance' si controlla prima se nessun segnale di emergency brake è arrivato, e poi si manda la posizione dell'egoveicolo al client di Sumo.

## Scenario

Il file 'carlatypes' è stato preso direttamente dalla cartella di Co-simulazione di Carla, come deto in precendeza dato che si usano comunque tipo veicoli di Carla si possono far combaciare quello che viene visto a schermo e quello in Sumo.

Nel file 'net.xml', anchesso preso dalla cartella Co-simulation di Carla è stata aggiunta una route al di fuori della mappa per far spawnare li' l'egoveicolo per poi essere spostato nella posizione di dell'egoveicolo in Carla

Il file 'poly.xml' è quell dove sono salvati le informazione degli edifici, o ostacoli in generale, per permettere a veins di calacolare le possibili perdite di pacchetti o simili, ottenuto tramite 'building_extraction.py'.

'client1' avvia semplicemente la simulazione facendo riferimento al file 'sumo.cfg'

IMPORTANTE: ordine di avvio "client1 -> Veins -> client2"

'client2' è l'istanza di Sumo che si interfaccia e comunica con Carla, prima gli manda le posizioni dei vari veicoli in Sumo e poi aggiorna la posizione dell'egoveicolo.

IMPORTANTE: la funzione 'MoveToXY' deve avvenire dopo il simulation step.

## Veins

La maggior parte di questi file sono file base di Veins che sono stati modificati per permettere la connessione di Veins a Sumo

Gli unici davvero imporanti sono i file presenti nella cartella examples/veins:

Qui sono presenti solo il file omnetpp.ini per far partire la simulazione e lo script 'run_veins' che è un miniscript in bash che serve automizza l'avvio della simulazione tramite terminale.

E l'applicazione c++ presente in src/veins/modules/appliation/traci con messaggio custom annesso

## WebServerOBS

È l'applet in flask per avere l'avviso grafico si può modificare con quello che si vuole, se conoscete abbastaqnza di sviluppo web e avete vogli di divertirvi potete addiruttura fare render 3D delle zone circostanti della macchina alla "Tesla" maniera.

Non lo fate era solo per dire fino a che si riuscirebbe a fare, il lavoro necessario supera di gran lunga le funzionalità.

Non dimenticate l'ip dell'app quando lo usate nel Simulatore