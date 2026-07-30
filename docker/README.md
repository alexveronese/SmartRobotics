# ROS 2 Rolling development container

Immagine di sviluppo basata su Ubuntu 24.04 (Noble) con:

- ROS 2 Rolling Desktop installato dai pacchetti deb ufficiali;
- `ros-dev-tools`, `rosdep`, `colcon`, Git e utility da terminale;
- il repository
  [Mastering ROS 2 for Robotics Programming](https://github.com/PacktPublishing/Mastering-ROS-2-for-Robotics-Programming)
  clonato in `/workspaces/mastering_ros2`;
- helper per installare le dipendenze e compilare ogni capitolo come overlay
  separato;
- Gazebo Sim Jetty, `gz_ros2_control`, bridge ROS-Gazebo e un'estensione
  completa del Chapter 9 per il riconoscimento e lo smistamento di forme;
- utente non-root `ros` con `sudo` senza password per il normale lavoro di
  sviluppo.

L'installazione di ROS segue la
[guida ufficiale per Ubuntu](https://docs.ros.org/en/rolling/Get-Started/Installation/Ubuntu-Install-Debs.html).
Noble è mantenuto qui per la compatibilità con gli esempi del libro e con il
relativo stack Gazebo. Rolling è nel frattempo migrato a Ubuntu 26.04: i
pacchetti Noble usati dall'immagine sono quindi congelati e ROS stampa un
avviso informativo all'avvio. Per seguire Rolling più recente occorre
riconvalidare il Chapter 9 su Ubuntu 26.04.

## Avvio rapido

Costruisci l'immagine:

```bash
docker compose build
```

Avvia una shell:

```bash
docker compose run --rm ros2-dev
```

Verifica l'ambiente:

```bash
printenv ROS_DISTRO
colcon list --base-paths /workspaces/mastering_ros2/Chapter03
ros2 run demo_nodes_cpp talker
```

Il volume Docker `ros2-workspaces` conserva modifiche, build e installazioni
fra un avvio e l'altro.

## Demo Chapter 9: smistamento dei quadrati

La demo usa il Panda e la configurazione `ros2_control` del Chapter 9. La
scena contiene due tavoli, due prismi quadrati e due prismi triangolari dello
stesso colore. Una camera zenitale segmenta gli oggetti con OpenCV e li
classifica dal numero di vertici del contorno; il colore viene usato soltanto
per separare gli oggetti dallo sfondo.

Il Panda:

1. attende tre rilevamenti stabili;
2. seleziona soltanto i contorni a quattro vertici;
3. calcola le pose articolari con una IK numerica sul modello URDF;
4. comanda braccio e pinza tramite `FollowJointTrajectory`;
5. porta i due quadrati sul tavolo inizialmente vuoto, lasciando fermi i
   triangoli.

Costruisci e avvia la simulazione:

```bash
docker compose build sorting-demo
docker compose up sorting-demo
```

Apri quindi
[http://localhost:6080/vnc.html?autoconnect=1&resize=scale](http://localhost:6080/vnc.html?autoconnect=1&resize=scale)
per vedere Gazebo nel browser. Su macOS, incluso Apple Silicon, la GUI viene
renderizzata nel container con Xvfb e Mesa e pubblicata tramite noVNC, quindi
non occorre configurare XQuartz.

Per eseguirla senza GUI:

```bash
DEMO_HEADLESS=true docker compose up sorting-demo
```

Al completamento il log mostra:

```text
COMPLETED: both squares are on the destination table; triangles remain on the source table
```

Per disabilitare l'avvio automatico:

```bash
DEMO_AUTO_START=false docker compose up sorting-demo
```

Poi, da un secondo terminale:

```bash
docker compose exec sorting-demo \
  ros2 topic pub --once /square_sorting/start std_msgs/msg/Bool '{data: true}'
```

Topic utili:

- `/shape_detections`: risultato JSON con forma, vertici, pixel e posizione;
- `/shape_detector/annotated`: immagine con contorni ed etichette;
- `/square_sorting/status`: stato leggibile della sequenza;
- `/joint_states`: stato simulato del Panda;
- `/arm_controller/follow_joint_trajectory` e
  `/eef_controller/follow_joint_trajectory`: action dei controller.

Il sorgente modificabile è in
[`sorting_ws/src/square_sorting_demo`](sorting_ws/src/square_sorting_demo).
Per provare una modifica senza ricostruire tutta l'immagine:

```bash
docker compose run --rm ros2-dev
build-sorting-demo
source /workspaces/sorting_ws/install/setup.bash
run-square-sorting-demo
```

Le compatibilità necessarie per usare il materiale originale con Rolling sono
isolate in [`docker/chapter09-rolling.patch`](docker/chapter09-rolling.patch):
rinominano la risorsa hardware con un nome ROS valido e rimuovono il secondo
plugin Ogre legacy della camera D435. Il clone del repository Packt resta
integro.

La simulazione si appoggia alle interfacce documentate di
[`gz_ros2_control`](https://control.ros.org/master/doc/gz_ros2_control/doc/index.html),
al
[`ros_gz_bridge`](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
e al servizio
[`SetEntityPose`](https://docs.ros.org/en/rolling/p/ros_gz_sim_demos/index.html).
Non è richiesto un server MCP Gazebo per compilare, eseguire o sviluppare la
demo.

## Lavorare sui capitoli

Il repository non è un unico workspace: alcuni capitoli contengono pacchetti
con lo stesso nome e altri richiedono stack o hardware specifici. Per questo
gli overlay vengono mantenuti separati.

Per esempio, per preparare e compilare il capitolo 5:

```bash
install-chapter-deps Chapter05
build-chapter Chapter05
source /workspaces/chapter_ws/Chapter05/install/setup.bash
```

Per rendere un capitolo l'overlay predefinito all'avvio:

```bash
ROS_ACTIVE_CHAPTER=Chapter05 docker compose run --rm ros2-dev
```

`install-chapter-deps` usa `rosdep` con Rolling. Poiché il codice del libro è
stato sviluppato per ROS 2 Jazzy, alcuni capitoli possono richiedere modifiche
per API cambiate o dipendenze non pubblicate su Rolling. Per esempio,
`Chapter03/master_ros2_pkg` usa la macro CMake
`ament_target_dependencies`, non più esposta allo stesso modo dalla Rolling
installata: il clone viene lasciato intenzionalmente integro, così ogni patch
di porting resta visibile nel normale flusso Git.

## Personalizzazione

Copia `.env.example` in `.env` per impostare UID/GID, ROS domain, capitolo
attivo o ref Git:

```bash
cp .env.example .env
docker compose build
```

Per ricreare da zero anche il workspace persistente:

```bash
docker compose down
docker volume rm robotic_ros2-workspaces
```

Quest'ultimo comando elimina in modo permanente le modifiche salvate nel
volume.
