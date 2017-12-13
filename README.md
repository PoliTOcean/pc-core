# pc-core

Per far funzionare i pacchetti, è necessario importare l'intera cartella "politocean" dentro "src", nel workspace di ROS.
Successivamente bisogna lanciare il comando "catkin_make" all'interno della cartella principale del workspace.

Fatto ciò, si può tranquillamente usare la piattaforma lanciando i comandi:

rosrun politocean mainGUI.py -> per avviare l'interfaccia grafica

rosrun politocean mainROV.py -> per avviare il nodo che dovrà girare sul ROV

Questi due script farannno partire tutti i nodi necessari alla loro corretta esecuzione, così come saranno loro a fermarli una volta chiuso il programma.
