# eye-in-hand
Progetto che si occupa della visione con una camera monoculare montata su un braccio robotico. Questo braccio, montato su una carrozzina, viene utilizzato come aiuto per persone disabili per premere dei pulsanti. 
Il codice sviluppato è generale per qualsiasi tipo di pulsantiera. 
L'utente premendo sul tablet, il bottone che vuole premere, fornisce al codice il pulsante desiderato. Utilizzando degli algoritmi di riconoscimento delle forme e di matching, si riesce a ricreare la posizione in 3D del punto. Con questo dato è possibile creare la posa dell'end-effector e a portare il braccio nel punto desiderato.

Attenzione: In questo codice si utilizza Opencv 2.4 e ROS indigo.
Questo algoritmo non funziona con Opencv 3. Nei forum di Opencv c'è scritto che i metodi di cv_bridge non sono ancora supportati in opencv3.
