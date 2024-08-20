
import cv2
import numpy as np
import socket

# Fonction pour vérifier si la couleur détectée est verte
def est_verte(mask):
    # Trouve les contours dans le masque binaire
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Retourne True si des contours sont trouvés (c'est-à-dire que quelque chose est vert)
    return len(contours) > 0

# Fonction pour envoyer le résultat au serveur RobotStudio
def envoyer_resultat(soquete, resultat):
    # Convertit le résultat en chaîne "TRUE" ou "FALSE"
    message = "TRUE" if resultat else "FALSE"
    # Envoie le message au serveur
    soquete.send(message.encode())

# Configuration de la capture vidéo depuis la caméra
cap = cv2.VideoCapture(0)

# Définition des plages de couleur en HSV pour le vert
vertBajo = np.array([35, 100, 100], np.uint8)   # Plage basse pour le vert
vertAlto = np.array([85, 255, 255], np.uint8)   # Plage haute pour le vert

# Création du socket pour la communication avec RobotStudio
soquete = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
soquete.connect(("127.0.0.1", 8080))  # Remplacez l'adresse IP et le port par ceux de RobotStudio

while True:
    # Capture une image de la caméra
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convertit l'image capturée de BGR à HSV
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Crée un masque pour détecter les pixels verts dans l'image
    maskVert = cv2.inRange(frameHSV, vertBajo, vertAlto)
    
    # Vérifie si l'objet détecté est vert
    est_vert = est_verte(maskVert)
    envoyer_resultat(soquete, est_vert)
    
    # Affiche l'image originale et le masque vert dans des fenêtres séparées
    cv2.imshow('Image', frame)
    cv2.imshow('Masque Vert', maskVert)
    
    # Quitte le programme lorsque la touche 'q' est pressée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret:
    cv2.imshow('Image', frame)
    cv2.waitKey(0)
# Libère les ressources utilisées
cap.release()
cv2.destroyAllWindows()
soquete.close()  # Fermeture du socket

#conda activate base
#python C:\Users\chaima\Desktop\Client.py

