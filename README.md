# Teleoperazione subacquea in condizioni di comunicazione degradate
### Un caso di studio con un prototipo a base fissa

[![ROS](https://img.shields.io/badge/ROS-1-22314E?logo=ros)](http://wiki.ros.org/)
[![Franka Emika](https://img.shields.io/badge/Franka%20Emika-Panda-00A0DF)](https://www.franka.de/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu)](https://ubuntu.com/)

Questa repository contiene il materiale sperimentale e i risultati principali della mia tesi di laurea magistrale in Ingengneria Informatica.  
L’obiettivo del lavoro è stato studiare un sistema di **teleoperazione subacquea** (onshore - offshore) in scenari con **comunicazione degradata**, con particolare focus sulla latenza, utilizzando un prototipo a base fissa.

## Panoramica

Il progetto investiga le sfide della teleoperazione in ambienti subacquei caratterizzati da comunicazione degradata. Attraverso l'utilizzo del robot collaborativo **Franka Emika Panda**, sono state simulate condizioni di latenza tipiche delle comunicazioni subacquee per valutare le performance del sistema di controllo.

**Obiettivi principali:**
- Analisi degli effetti della latenza sulla teleoperazione
- Valutazione delle performance in compiti di pick-and-place e di inseguimento di riferimento
- Esecuzione delle operazioni con ritardi costanti crescenti
- Confronto tra modalità di teleoperazione manuale e automatica

---

### Architettura realizzata
![Mappa Concettuale Strategia Branstorming Idea Bianco Nero Moderno Professionale Lavoro](https://github.com/user-attachments/assets/c59fe8df-ef2f-4676-b024-0ae1c09d6ad4)

### Gestione del ritardo bidirezionale simulato
<img width="1920" height="1080" alt="img2" src="https://github.com/user-attachments/assets/ab035cec-8814-4d01-b0c9-b986567e7c48" />

Il sistema implementa un'architettura distribuita che simula realisticamente le condizioni di comunicazione degradata tipiche degli ambienti subacquei.

---

## Risultati Sperimentali

### Tempo di Completamento - Operazione Pick and Place 
  <img width="989" height="590" alt="pick" src="https://github.com/user-attachments/assets/220f88ad-589e-4831-ac25-2cae993bd01b" />
  
*Analisi del tempo richiesto per completare operazioni di pick-and-place al variare della latenza di comunicazione sia per la modalità manuale che automatica.*

### Errore Medio di Tracking - Operazione Follow Cube
<img width="1189" height="590" alt="image" src="https://github.com/user-attachments/assets/8a9adb69-19fc-4214-b91a-c6d64ec10dc9" />

*Valutazione della precisione nell'errore di tracking di riferimento in condizioni di ritardo di comunicazione confrontando modalità manuale e automatica all'aumentare del ritardo.*


---

## Tecnologie Utilizzate

### Hardware
- **Robot collaborativo**: Franka Emika Panda (7 DoF)
- **Sensori visivi**: Telecamera RGB Intel RealSense
- **Workstation**: Ubuntu 20.04

### Software
- **Framework**: Robot Operating System (ROS1)
- **Simulazione**: Gazebo
- **Visualizzazione**: RViz
- **Linguaggi**: C++, Python

---

## Video dimostrativi 
- Follow cube in modalità manuale con (500 ms di delay)
[Guarda il video](Video/manual_follow_delay500.mp4)

- Follow cube in modalità automatica con (500 ms di delay)
[Guarda il video](Video/automatico_follow_delay500.gif)

# Autore

**Noemi La Torre**

- Email: latorre.noemi17@gmail.com
- LinkedIn: [linkedin.com/in/noemilatorre](https://linkedin.com/in/noemilatorre)
- GitHub: [github.com/noemilatorre](https://github.com/noemilatorre)
- Portfolio: [noemilatorre.github.io](https://noemilatorre.github.io)

---

*Tesi di Laurea Magistrale in Ingegneria Informatica - Curriculum Intelligenza Artificiale e Robotica*  
*Università degli Studi di Cassino e del Lazio Meridionale - Settembre 2025*

  
