# eye-in-hand
/*BRIEF*/
In questo report verrà presentato l'algoritmo di visione per una camera monoculare per il controllo di un braccio robotico posto su di una carozzina al fine di premere un pulsante scelto dal utente. 
Il codice sviluppato aspetta che l'utente, tramite l'interfaccia, prema un pulsante. Una volta identificato il pulsante premuto, la camera fornisce al robot le informazioni di posizione ad orientazione in modo da posizionarsi sul percorso corretto. Visto che la camera utilizzata è monoculare non si hanno informazioni sulla distanza. Per ovviare a questa mancanza si è deciso di utilizzare un algoritmo esterno capace di utilizzare le informazioni delle features trovate per stimare la posizione relativa. Questo algoritmo è affetto da un errore di scala e per ridurne gli effetti si utilizza il movimento del robot relativo alla posizione precedente.

/*Requisiti*/
Attenzione: In questo codice si utilizza Opencv 2.4 e ROS indigo.
Questo algoritmo non funziona con Opencv 3. Nei forum di Opencv c'è scritto che i metodi di cv_bridge non sono ancora supportati in opencv3.
L'utilizzo di pcl 1.7.2 è facoltativo, se non si dispone di tale versione basta cambiare il package dentro al Cmakelist find_package(PCL 1.7.2 REQUIRED) e cambiarlo con la versione installata.
Si è scelto di usare delle scorciatoie per inviare i comandi usando la testiera. Installare il pacchetto ros keycommand.

/*Calibrazione*/
Bisogna effettuare due calibrazioni. 
1) lanciare il codie calibration fornito da opencv
2) lanciare ptam_cal.launch
Il primo codice calibra la camera con modello pinhole il secondo con il metodo delle arcotangenti. Dopo la calibrazione occorre salvare i dati forniti. 
1) dentro al file ~/eye_in_hand/Config/code_param.yalm 
2) dentro al file ~/ethzasl_ptam/ptam/PtamFixParams.yalm

ATTENZIONE: per la calibrazione di Ptam occorre utilizzare la chessboard fornita dal pacchetto.

/*PROCEDURA*/
Quando la camera è attiva invia due messaggi Ros, uno invia lo streaming dal nodo di PTAM l'altro invia una immagine statica al nodo di opencv. Una volta che l'immagine è arrivata ( nel topic /camera/output_video) è possibile effettuare la fase di riconoscimento del bottone premuto.
Per inizializzare Ptam occorre premere la barra spazziatrice, traslare la camera e ripremere la barra. Bisonga assicurarsi che la griglia creata sia stabile. Se balla occorre ripetere la procedura premendo reset nell'interfaccia Ptam.
Dopo questa fase di iniziallizazione Ptam inizia ad inviare la posa del frame word rispetto la camera. Per la stima della scala si è creato un nodo ros. Bisogna inviare: 

1) topic : /moverobot. In questo topic si invia un geometry_msgs/Pose con la traslazione effettuata.
	i.e se ci si è spostato di 2 cm su z bisogna inviare rostopic pub -1 /moverobot geometry_msgs/Pose '{position: {x: 0.0, y: 0.0, z: 0.02}, orientation: {w: 1}}'
2) topic: /stopandgo. Questo messaggio serve per fermare momentaneamente il calcolo della stima della scala. 
	rostopic pub -1 /stopandgo std_msgs/Bool 'true'
	Per farla ripartire
	rostopic pub -1 /stopandgo std_msgs/Bool 'false'

Quando l'algoritmo raggiunge la convergenza invia un messaggio ros al nodo della camera per calcolare la posa 3d del bottone.
Ogni volta che la camera si muove bisogna inviare sul topic /robot il valore true
	rostopic pub -1 /robot std_msgs/Bool true

Riassumendo, occorre:
1) inizializzare Ptam
2) inviare la traslazione sul topic /moverobot
3) inviare messaggio di start sul topic /stopandgo --> rostopic pub -1 /stopandgo std_msgs/Bool 'false'
4) inviare messaggio di stop sul topic /stopandgo --> rostopic pub -1 /stopandgo std_msgs/Bool 'true' appena ha stimato la scala
5) inviare rosrun ptam ptam_visualizer
6) inviare ad ogni roto/traslazione il messaggio di movimento sul topic /stopandgo --> rostopic pub -1 /robot std_msgs/Bool true

Metodo veloce:
1) inizializzare Ptam
2) inviare la traslazione sul topic /moverobot
3) inviare messaggio di start premendo g sulla tastiera
4) inviare messaggio di stop premendo s sulla tastiera
5) inviare rosrun ptam ptam_visualizer
6) premere ad ogni roto/traslazione il tasto r sulla tastiera

Per questo metodo assicurarsi di avere la finestra nera di ros keycommand accesa e selezionata