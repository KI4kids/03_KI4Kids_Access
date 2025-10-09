
## Linux Befehle	




| Befehl| Erklärung | 
| -------- | -------- | 
|sudo rpi-eeprom-update|EEPROM Update|
|sudo apt update && sudo apt upgrade -y	|Update. Apt steht für Advanced Packaging Tool|
|pwd|	zeigt den aktuellen Pfad an|
|ls	|zeig alle Dateien in dem aktuellen Pfad an|
|ls -lha|	versteckte Dateien anzeigen|
|ls *png|	zeigt alle .png Dateien an|
|cd yolov5|	wechselt in den Ordner yolov5|
|cd /usr/local/bin/	|wechselt in den Ordner ..bin|
|cd -	|wechselt in den vorherigen Ordner|
|cd ~	|wechelt ins home Verzeichnis|
|cd ..	|geht einen Ordner zurück|
|rm test.sh|	löscht die test.sh|
|cp test.sh yolov5/	|kopiert test.sh in yolov5|
|cp -r yolov5/Test/ tmp/	|kopiert den Ordner Test in den Ordner tmp|
|mkdir Bilder|	erstellt den Ordner Bilder|
|cat test.txt|	zeigt den Inhalt von test.txt|
|cat -n test.txt	|.. Mit Zeilennummern|
|nano test.txt	|zeigt text.txt im Editor nano|
|sudo apt install filezilla|	installiert filezilla|
|curl http://fh.com/abc.jpg -o download.jpg	|lädt Dateien herunter|
|help cd	|Hilfe für den Befehl cd|
|mv text.txt text2.txt	|so wird so text.txt in text2.txt umbenannt|
|mv Schreibtisch/text.txt Dokumente/text.txt	|Datei verschieben|
|ping 192.168.1.1	|Erreichbarkeit eines anderen Computers überprüfen|
|ping google.com	| |
|tar xfv archiv.tar	|tape archiver; Beispiel um ein Archiv zu erstellen|
|gzip test.txt	|Beispiel test.txt komprimieren|
|touch text.txt	|Datei erstellen|
|service ssh stop	|Services starten und stoppen|
|exit|	ein Programm, eine Shell oder ein UNIX-Netzwerk verlassen|
|shutdown -h now|	Computer herunterfahren|
|reboot -h now|	Computer neu starten|
|---|---|
|sudo lsblk -e 1,7 -o NAME,FSTYPE,ROTA,SIZE,VENDOR,MODEL| Listet sd/ssd Infos|
|sudo parted /dev/nvme0n1| Startet das Programm parted |
|rm 1| SSD 1 neu partitionieren |
|---|---|
|conda --version | Anaconda Versionsabfrage |
|conda create -n myenv | Anaconda env erstellen  |
|conda env list  |  Anaconda env auflisten |
|conda remove -n myenv --all |Anaconda env löschen  |
|conda deactivate  |  Anaconda deaktivieren |
|conda env export > myenv.yml  |  Anaconda env exportieren |
|conda env create -f myenv.yml  | Anaconda env importieren |
|conda rename -n old_name new_name  | Anaconda env umbenennen |
|---|---|
|---|---|




