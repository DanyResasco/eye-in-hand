# eye-in-hand
/*BRIEF*/
In questo report verrà presentato l'algoritmo di visione per una camera monoculare per il controllo di un braccio robotico posto su di una carozzina al fine di premere un pulsante scelto dal utente. 
Il codice sviluppato aspetta che l'utente, tramite l'interfaccia, prema un pulsante. Una volta identificato il pulsante premuto, la camera fornisce al robot le informazioni di posizione ad orientazione in modo da posizionarsi sul percorso corretto. Visto che la camera utilizzata è monoculare non si hanno informazioni sulla distanza. Per ovviare a questa mancanza si è deciso di utilizzare un algoritmo esterno capace di utilizzare le informazioni delle features trovate per stimare la posizione relativa. Questo algoritmo è affetto da un errore di scala e per ridurne gli effetti si utilizza il movimento del robot relativo alla posizione precedente.

/*Requisiti*/
Attenzione: In questo codice si utilizza Opencv 2.4 e ROS indigo.
Questo algoritmo non funziona con Opencv 3. Nei forum di Opencv c'è scritto che i metodi di cv_bridge non sono ancora supportati in opencv3.
L'utilizzo di pcl 1.7.2 è facoltativo, se non si dispone di tale versione basta cambiare il package dentro al Cmakelist find_package(PCL 1.7.2 REQUIRED) e cambiarlo con la versione installata.

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
Per inizializzare Ptam occorre premere la barra spazziatric, traslare la camera e ripremere la barra. Dopo questa fase di iniziallizazione Ptam inizia ad inviare la posa del frame word rispetto la camera. Per la stima della scala si è scelto di utilizzare lo stimatore a massima verosimiglianza. Per questo tipo di stimatore occorrono più di cento campioni. Per far queto si sono creati due messaggi:
1) topic : /moverobot. In questo topic si invia un geometry_msgs/Pose con la posizione reale del
