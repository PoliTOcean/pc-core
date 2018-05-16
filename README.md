# pc-core

HEAD
Control station software
=======
Per far funzionare i pacchetti, è necessario importare l'intera cartella "politocean" dentro "src", nel workspace di ROS.
Successivamente bisogna lanciare il comando "catkin_make" all'interno della cartella principale del workspace.

Fatto ciò, si può tranquillamente usare la piattaforma lanciando i comandi:

roslaunch politocean launcherGUI.launch -> per avviare l'interfaccia grafica

roslaunch politocean rov.launch -> per avviare il nodo che dovrà girare sul ROV

Questi due script farannno partire tutti i nodi necessari alla loro corretta esecuzione, così come saranno loro a fermarli una volta chiuso il programma.

