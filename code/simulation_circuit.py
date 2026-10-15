import numpy as np
import math


class Voiture:
    """Liste pour stocker les caractéristiques d'une voiture"""

    def __init__(self, nom, masse, accel_moy, longueur, largeur, hauteur, Cx, Cz, mu):
        self.nom = nom
        self.masse = masse  # kg
        self.accel_moy = accel_moy  # m/s²
        self.longueur = longueur  # m
        self.largeur = largeur  # m
        self.hauteur = hauteur  # m
        self.Cx = Cx  # coefficient de traînée
        self.Cz = Cz  # coefficient de portance
        self.mu = mu  # coefficient de frottement sol
        self.S = largeur * hauteur  # Surface frontale (maître-couple) en m²

        # Accessoires
        self.a_booster = False
        self.portion_booster = None
        self.a_ailerons = False
        self.a_jupe = False

    def appliquer_accessoires(self, accessoires, portion_booster=None):
        """Applique les modifications dues aux accessoires"""
        voiture_modifiee = Voiture(self.nom, self.masse, self.accel_moy,
                                   self.longueur, self.largeur, self.hauteur,
                                   self.Cx, self.Cz, self.mu)

        # Booster: +30% accélération sur une portion
        if 'booster' in accessoires:
            voiture_modifiee.a_booster = True
            voiture_modifiee.portion_booster = portion_booster

        # Ailerons: +10% portance, +30kg, +0.8m² surface
        if 'ailerons' in accessoires:
            voiture_modifiee.a_ailerons = True
            voiture_modifiee.masse += 30
            voiture_modifiee.Cz *= 1.10

        # Jupe avant: -5% traînée, +15kg
        if 'jupe' in accessoires:
            voiture_modifiee.a_jupe = True
            voiture_modifiee.masse += 15
            voiture_modifiee.Cx *= 0.95

        return voiture_modifiee


# Constantes physiques
g = 9.81  # m/s²
rho_air = 1.225  # kg/m³ (densité de l'air)

# Paramètres du circuit
h_pente = 2  # hauteur de la pente en m
L_pente = 31  # longueur de la pente en m
alpha = np.arcsin(h_pente / L_pente)  # angle de la pente en radians
R_looping = 6  # rayon du looping en m
L_ravin = 9  # largeur du ravin en m
h_ravin = 1
L_finale = 10  # longueur de la piste finale en m


def simuler_pente(voiture, dt=0.001):
    """
    Simule le mouvement sur la pente avec frottements
    Retourne: vitesse finale, temps, trajectoires
    """
    m = voiture.masse
    k = 0.5 * rho_air * voiture.S * voiture.Cx

    # Appliquer booster si activé sur la pente
    accel = voiture.accel_moy
    if voiture.a_booster and voiture.portion_booster == 'pente':
        accel *= 1.30

    # Conditions initiales
    t = 0
    x = 0
    vx = 0

    # Listes pour stocker les résultats
    temps = [t]
    positions = [x]
    vitesses = [vx]
    accelerations = []

    # Simulation avec méthode d'Euler
    while x < L_pente:
        # Calcul de l'accélération selon l'équation du livrable 2:
        # m.g.(sinα - µ.cosα) - k.vx² + m.am = m.dvx/dt
        ax = g * (np.sin(alpha) - voiture.mu * np.cos(alpha)) - (k * vx ** 2) / m + accel

        # Mise à jour vitesse et position (Euler)
        vx = vx + ax * dt
        x = x + vx * dt
        t = t + dt

        # Sauvegarde
        temps.append(t)
        positions.append(x)
        vitesses.append(vx)
        accelerations.append(ax)

    return vx, t, np.array(temps), np.array(positions), np.array(vitesses), np.array(accelerations)


def vitesse_min_looping(R):
    """Calcule la vitesse minimale pour passer le looping"""
    return np.sqrt(g * R)


def simuler_looping(voiture, v_entree, dt=0.001):
    """
    Simule le mouvement dans le looping avec frottements
    Retourne: vitesse sortie, temps, peut_passer
    """
    m = voiture.masse
    R = R_looping
    k = 0.5 * rho_air * voiture.S * voiture.Cx

    # Appliquer booster si activé sur le looping
    accel_boost = 0
    if voiture.a_booster and voiture.portion_booster == 'looping':
        accel_boost = voiture.accel_moy * 0.30

    # Conditions initiales (l'angle teta = 0 en bas du looping)
    t = 0
    theta = 0  # angle en radians
    omega = v_entree / R  # vitesse angulaire initiale

    # Listes pour stocker les résultats
    temps = [t]
    angles = [theta]
    vitesses_ang = [omega]
    vitesses_lin = [omega * R]

    peut_passer = True

    # Simulation jusqu'à θ = 2π (un tour complet)
    while theta < 2 * np.pi:
        # Équation du mouvement tangentielle avec frottements
        # m.R.dω/dt = -m.g.sin(θ) - µ.m.g.cos(θ) - k.R³.ω²
        alpha_ang = (-g * np.sin(theta) - voiture.mu * g * np.cos(
            theta) - k * R ** 3 * omega ** 2 / m + accel_boost) / R

        # Mise à jour
        omega = omega + alpha_ang * dt
        theta = theta + omega * dt
        t = t + dt

        # Vitesse linéaire
        v = omega * R

        # Vérification au sommet:
        if np.pi - 0.1 < theta < np.pi + 0.1:
            if v < vitesse_min_looping(R):
                peut_passer = False

        # Sauvegarde
        temps.append(t)
        angles.append(theta)
        vitesses_ang.append(omega)
        vitesses_lin.append(v)

        # Sécurité: si la vitesse devient négative, la voiture tombe
        if omega < 0:
            peut_passer = False
            break

    v_sortie = omega * R if omega > 0 else 0

    return v_sortie, t, np.array(temps), np.array(vitesses_lin), peut_passer


def vitesse_min_ravin(L, h):
    """
    Calcule la vitesse minimale pour franchir le ravin
    Avec chute de h mètres sur une distance L
    """
    # y = -g*x²/(2*v₀²)
    # Pour x=L, y=-h : -h = -g*L²/(2*v₀²)
    # v₀ = L * sqrt(g/(2*h))
    return L * np.sqrt(g / (2 * h))


def simuler_ravin(voiture, v_entree, dt=0.001):
    """
    Simule le saut au-dessus du ravin avec frottements
    Retourne: peut_passer, temps, trajectoire
    """
    m = voiture.masse
    T = voiture.S * voiture.Cx  # paramètre de traînée
    P = voiture.S * voiture.Cz  # paramètre de portance

    # Conditions initiales
    t = 0
    x = 0
    y = 0
    vx = v_entree
    vy = 0

    # Listes pour stocker la trajectoire
    temps = [t]
    pos_x = [x]
    pos_y = [y]

    # Simulation jusqu'à ce que la voiture atteigne x = L_ravin
    while x < L_ravin and y > -5:  # sécurité pour éviter une chute infinie
        # Calcul de la norme de la vitesse
        v_norm = np.sqrt(vx ** 2 + vy ** 2)

        # Accélérations avec frottements (équations du livrable 2)
        ax = -(rho_air / (2 * m)) * v_norm * (T * vx + P * vy)
        ay = -(rho_air / (2 * m)) * v_norm * (T * vy - P * vx) - g

        # Mise à jour (méthode d'Euler)
        vx = vx + ax * dt
        vy = vy + ay * dt
        x = x + vx * dt
        y = y + vy * dt
        t = t + dt

        # Sauvegarde
        temps.append(t)
        pos_x.append(x)
        pos_y.append(y)

    # Vérification: la voiture doit arriver à x>=L_ravin avec y>=-h_ravin
    peut_passer = (x >= L_ravin) and (y >= -h_ravin)

    return peut_passer, t, np.array(pos_x), np.array(pos_y)


def simuler_finale(voiture, v_entree, dt=0.001):
    """
    Simule la fin de piste horizontale avec frottements
    Retourne: temps pour parcourir L_finale
    """
    m = voiture.masse
    k = 0.5 * rho_air * voiture.S * voiture.Cx

    # Appliquer booster si activé sur la fin de piste
    accel = voiture.accel_moy
    if voiture.a_booster and voiture.portion_booster == 'finale':
        accel *= 1.30

    # Conditions initiales
    t = 0
    x = 0
    vx = v_entree

    # Simulation
    while x < L_finale:
        # Équation du mouvement avec frottements
        # -µ.m.g - k.vx² + m.am = m.ax
        ax = -voiture.mu * g - (k * vx ** 2) / m + accel

        # Mise à jour
        vx = vx + ax * dt
        x = x + vx * dt
        t = t + dt

        # Sécurité
        if vx < 0:
            vx = 0
            break

    return t


def tracer_vitesse_looping(temps, vitesses, v_min, nom_voiture):
    """
    Trace la courbe de vitesse dans le looping
    Sauvegarde dans un fichier texte avec coordonnées
    """
    print("\n📈 GRAPHIQUE: VITESSE DANS LE LOOPING")
    print("=" * 80)

    # Convertir en km/h
    vitesses_kmh = vitesses * 3.6
    v_min_kmh = v_min * 3.6

    # afficher les valeurs
    print(f" Vitesse min = {min(vitesses_kmh):.2f} km/h")
    print(f" Vitesse max = {max(vitesses_kmh):.2f} km/h")

    # Créer le fichier de données
    with open('vitesse_looping.txt', 'w', encoding='utf-8') as f:
        f.write("VITESSE DANS LE LOOPING (km/h) vs TEMPS (s)\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"Vitesse minimale requise: {v_min_kmh:.2f} km/h\n\n")
        f.write("Temps (s) | Vitesse (km/h)\n")
        f.write("-" * 40 + "\n")


        step = max(1, len(temps) // 50)
        for i in range(0, len(temps), step):
            f.write(f"{temps[i]:8.4f}  | {vitesses_kmh[i]:10.2f}\n")

    print("✓ Données sauvegardées dans 'vitesse_looping.txt'")

    # Afficher un résumé visuel
    print("\nRÉSUMÉ VISUEL:")
    print(f"  Vitesse initiale:  {vitesses_kmh[0]:.2f} km/h  {'█' * int(vitesses_kmh[0] / 5)}")
    print(f"  Vitesse minimale:  {v_min_kmh:.2f} km/h  {'█' * int(v_min_kmh / 5)} ← SEUIL")
    print(f"  Vitesse minimale atteinte: {min(vitesses_kmh):.2f} km/h  {'█' * int(min(vitesses_kmh) / 5)}")
    print(f"  Vitesse finale:    {vitesses_kmh[-1]:.2f} km/h  {'█' * int(vitesses_kmh[-1] / 5)}")

    # Tracer avec matplotlib(une fonction mathématique qui sert à tracer les courbes)
    try:
        import matplotlib.pyplot as plt

        # Fermer toutes les figures précédentes
        plt.close('all')

        fig = plt.figure(figsize=(12, 6))
        plt.plot(temps, vitesses_kmh, 'b-', linewidth=2, label='Vitesse de la voiture')
        plt.axhline(y=v_min_kmh, color='r', linestyle='--', linewidth=2,
                    label=f'Vitesse minimale ({v_min_kmh:.1f} km/h)')
        plt.xlabel('Temps (s)', fontsize=12)
        plt.ylabel('Vitesse (km/h)', fontsize=12)
        plt.title(f'Vitesse dans le looping - {nom_voiture}', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10)

        # Ajouter des annotations
        idx_min = np.argmin(vitesses_kmh)
        plt.plot(temps[idx_min], vitesses_kmh[idx_min], 'ro', markersize=8)
        plt.annotate(f'Min: {vitesses_kmh[idx_min]:.1f} km/h',
                     xy=(temps[idx_min], vitesses_kmh[idx_min]),
                     xytext=(10, 10), textcoords='offset points',
                     bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7),
                     fontsize=10)

        plt.tight_layout()


        import time
        filename = f'vitesse_looping_{int(time.time())}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✓ Graphique sauvegardé dans '{filename}'")
        plt.show(block=False)  # Ne pas bloquer l'exécution
        plt.pause(0.1)  # Petite pause pour afficher
    except ImportError:
        print("ℹ️  matplotlib non disponible - installez-le avec: pip install matplotlib")


def tracer_trajectoire_ravin(pos_x, pos_y, nom_voiture):
    """
    Trace la trajectoire dans le ravin
    Sauvegarde dans un fichier texte avec coordonnées
    """
    print("\n📈 GRAPHIQUE: TRAJECTOIRE DANS LE RAVIN")
    print("=" * 80)

    # Présentation de dmin et hmax
    print(f" Distance max = {max(pos_x):.2f} m")
    print(f" Hauteur min = {min(pos_y):.2f} m")


    # Créer le fichier de données
    with open('trajectoire_ravin.txt', 'w', encoding='utf-8') as f:
        f.write("TRAJECTOIRE DANS LE RAVIN\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"Largeur du ravin: {L_ravin} m\n")
        f.write(f"Dénivelé: -{h_ravin} m\n\n")
        f.write("Distance (m) | Hauteur (m)\n")
        f.write("-" * 40 + "\n")

        # Échantillonner les données
        step = max(1, len(pos_x) // 50)
        for i in range(0, len(pos_x), step):
            f.write(f"{pos_x[i]:10.4f}   | {pos_y[i]:10.4f}\n")

    print("✓ Données sauvegardées dans 'trajectoire_ravin.txt'")

    # Afficher un résumé
    print("\nRÉSUMÉ TRAJECTOIRE:")
    print(f"  Distance parcourue: {pos_x[-1]:.2f} m / {L_ravin} m")
    print(f"  Hauteur finale: {pos_y[-1]:.2f} m (min requis: -{h_ravin} m)")
    print(f"  Hauteur max atteinte: {max(pos_y):.2f} m")

    # Tracer avec matplotlib
    try:
        import matplotlib.pyplot as plt

        # Fermer toutes les figures précédentes
        plt.close('all')

        fig = plt.figure(figsize=(12, 6))
        plt.plot(pos_x, pos_y, 'g-', linewidth=2, label='Trajectoire de la voiture')
        plt.axvline(x=L_ravin, color='r', linestyle='--', linewidth=2, label=f'Arrivée ({L_ravin} m)')
        plt.axhline(y=-h_ravin, color='orange', linestyle='--', linewidth=2, label=f'Niveau sol (-{h_ravin} m)')
        plt.axhline(y=0, color='black', linestyle='-', linewidth=1, alpha=0.3, label='Niveau départ')

        # Marquer le point d'arrivée
        plt.plot(pos_x[-1], pos_y[-1], 'ro', markersize=10, label=f'Arrivée: ({pos_x[-1]:.2f}m, {pos_y[-1]:.2f}m)')

        plt.xlabel('Distance horizontale (m)', fontsize=12)
        plt.ylabel('Hauteur (m)', fontsize=12)
        plt.title(f'Trajectoire dans le ravin - {nom_voiture}', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10)
        plt.axis('equal')
        plt.tight_layout()

        # Nom de fichier unique avec timestamp
        import time
        filename = f'trajectoire_ravin_{int(time.time())}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✓ Graphique sauvegardé dans '{filename}'")
        plt.show(block=False)  # Ne pas bloquer l'exécution
        plt.pause(0.1)  # Petite pause pour afficher
    except ImportError:
        print("ℹ️  matplotlib non disponible - installez-le avec: pip install matplotlib")


def analyser_voiture(voiture):
    """
    Analyse complète d'une voiture sur le circuit
    """
    print(f"\n{'=' * 80}")
    print(f"ANALYSE DÉTAILLÉE: {voiture.nom}")
    if voiture.a_booster or voiture.a_ailerons or voiture.a_jupe:
        print(f"AVEC ACCESSOIRES:")
        if voiture.a_booster:
            print(f"  • Booster (+30% accélération sur {voiture.portion_booster})")
        if voiture.a_ailerons:
            print(f"  • Ailerons (+10% portance, +30kg)")
        if voiture.a_jupe:
            print(f"  • Jupe avant (-5% traînée, +15kg)")
    print(f"{'=' * 80}")

    resultats = {}

    # 1) PENTE
    print(f"\n{'─' * 80}")
    print(f"1️⃣  PENTE (Longueur={L_pente}m, Hauteur={h_pente}m, Angle={np.degrees(alpha):.2f}°)")
    print(f"{'─' * 80}")

    v_pente, t_pente, temps_p, pos_p, vit_p, acc_p = simuler_pente(voiture)

    print(f"\n📊 RÉSULTATS PENTE:")
    print(f"   ✓ Vitesse en bas de la pente: {v_pente:.2f} m/s ({v_pente * 3.6:.2f} km/h)")
    print(f"   ✓ Temps de parcours: {t_pente:.3f} s")
    print(f"   ✓ Accélération moyenne: {np.mean(acc_p):.2f} m/s²")

    resultats['v_pente'] = v_pente
    resultats['t_pente'] = t_pente

    # 2) LOOPING
    print(f"\n{'─' * 80}")
    print(f"2️⃣  LOOPING (Rayon={R_looping}m)")
    print(f"{'─' * 80}")

    v_min_loop = vitesse_min_looping(R_looping)

    print(f"\n📊 ANALYSE LOOPING:")
    print(f"   • Vitesse minimale requise au sommet: {v_min_loop:.2f} m/s ({v_min_loop * 3.6:.2f} km/h)")
    print(f"   • Vitesse d'entrée (sortie pente): {v_pente:.2f} m/s ({v_pente * 3.6:.2f} km/h)")

    # Comparaison vitesse entrée vs vitesse min
    if v_pente >= v_min_loop:
        diff = v_pente - v_min_loop
        print(f"   ✓ COMPARAISON: Vitesse suffisante! (+{diff:.2f} m/s de marge)")
    else:
        diff = v_min_loop - v_pente
        print(f"   ✗ COMPARAISON: Vitesse insuffisante! (-{diff:.2f} m/s manquants)")

    v_looping, t_looping, temps_l, vit_l, peut_loop = simuler_looping(voiture, v_pente)

    print(f"\n📈 TRACÉ DE LA VITESSE AU COURS DU TEMPS:")
    tracer_vitesse_looping(temps_l, vit_l, v_min_loop, voiture.nom)

    print(f"\n📊 RÉSULTATS LOOPING:")
    if peut_loop:
        print(f"   ✓ CONCLUSION: PEUT PASSER le looping!")
        print(f"   ✓ Vitesse de sortie du looping: {v_looping:.2f} m/s ({v_looping * 3.6:.2f} km/h)")
        print(f"   ✓ Temps dans le looping: {t_looping:.3f} s")
    else:
        print(f"   ✗ CONCLUSION: NE PEUT PAS passer le looping (vitesse insuffisante)")
        print(f"   ✗ La voiture tombe avant de finir le tour")
        resultats['peut_finir'] = False
        return resultats

    resultats['v_looping'] = v_looping
    resultats['t_looping'] = t_looping

    # 3) RAVIN
    print(f"\n{'─' * 80}")
    print(f"3️⃣  SAUT DU RAVIN (Largeur={L_ravin}m, Dénivelé=-{h_ravin}m)")
    print(f"{'─' * 80}")

    v_min_ravin_val = vitesse_min_ravin(L_ravin, h_ravin)

    print(f"\n📊 ANALYSE RAVIN:")
    print(f"   • Vitesse minimale requise pour franchir: {v_min_ravin_val:.2f} m/s ({v_min_ravin_val * 3.6:.2f} km/h)")
    print(f"   • Vitesse d'entrée (sortie looping): {v_looping:.2f} m/s ({v_looping * 3.6:.2f} km/h)")

    # Comparaison vitesse entrée vs vitesse min
    if v_looping >= v_min_ravin_val:
        diff = v_looping - v_min_ravin_val
        print(f"   ✓ COMPARAISON: Vitesse suffisante! (+{diff:.2f} m/s de marge)")
    else:
        diff = v_min_ravin_val - v_looping
        print(f"   ✗ COMPARAISON: Vitesse insuffisante! (-{diff:.2f} m/s manquants)")

    peut_ravin, t_ravin, pos_x_r, pos_y_r = simuler_ravin(voiture, v_looping)

    print(f"\n📈 TRACÉ DE LA TRAJECTOIRE DANS LE RAVIN:")
    tracer_trajectoire_ravin(pos_x_r, pos_y_r, voiture.nom)

    print(f"\n📊 RÉSULTATS RAVIN:")
    if peut_ravin:
        print(f"   ✓ CONCLUSION: PEUT FRANCHIR le ravin!")
        print(f"   ✓ Distance parcourue: {pos_x_r[-1]:.2f} m")
        print(f"   ✓ Hauteur finale: {pos_y_r[-1]:.2f} m (min requis: {-h_ravin:.2f} m)")
        print(f"   ✓ Temps de vol: {t_ravin:.3f} s")
    else:
        print(f"   ✗ CONCLUSION: NE PEUT PAS franchir le ravin")
        print(f"   ✗ Distance atteinte: {pos_x_r[-1]:.2f} m / {L_ravin} m")
        print(f"   ✗ Hauteur finale: {pos_y_r[-1]:.2f} m (trop bas)")
        resultats['peut_finir'] = False
        return resultats

    resultats['t_ravin'] = t_ravin

    # 4) FIN DE PISTE
    print(f"\n{'─' * 80}")
    print(f"4️⃣  FIN DE PISTE (Longueur={L_finale}m)")
    print(f"{'─' * 80}")

    t_finale = simuler_finale(voiture, v_looping, dt=0.001)

    print(f"\n📊 RÉSULTATS FIN DE PISTE:")
    print(f"   ✓ Temps de parcours: {t_finale:.3f} s")
    resultats['t_finale'] = t_finale

    # TEMPS TOTAL
    temps_total = t_pente + t_looping + t_ravin + t_finale
    resultats['temps_total'] = temps_total
    resultats['peut_finir'] = True

    print(f"\n{'=' * 80}")
    print(f"⏱️  TEMPS TOTAL POUR PARCOURIR LE CIRCUIT: {temps_total:.3f} secondes")
    print(f"{'=' * 80}")
    print(f"\nDétail par portion:")
    print(f"   • Pente:      {t_pente:.3f} s ({t_pente / temps_total * 100:.1f}%)")
    print(f"   • Looping:    {t_looping:.3f} s ({t_looping / temps_total * 100:.1f}%)")
    print(f"   • Ravin:      {t_ravin:.3f} s ({t_ravin / temps_total * 100:.1f}%)")
    print(f"   • Fin piste:  {t_finale:.3f} s ({t_finale / temps_total * 100:.1f}%)")

    print(f"\n{'=' * 80}")
    if temps_total < 8:
        print(f"🏆 VICTOIRE! GAGNE contre Owen Shaw (temps < 8s)!")
        print(f"   Marge: {8 - temps_total:.3f} secondes d'avance")
    else:
        print(f"❌ DÉFAITE! PERD contre Owen Shaw (temps >= 8s)")
        print(f"   Retard: {temps_total - 8:.3f} secondes de trop")
    print(f"{'=' * 80}")

    return resultats


# ===== BASE DE DONNÉES DES VOITURES =====

voitures_disponibles = {
    '1': Voiture("Dodge Charger R/T 1970", 1760, 5.1, 5.28, 1.95, 1.35, 0.38, 0.3, 0.1),
    '2': Voiture("Toyota Supra Mark IV 1994", 1615, 5, 4.51, 1.81, 1.27, 0.29, 0.3, 0.1),
    '3': Voiture("Chevrolet Yenko Camaro 1969", 1498, 5.3, 4.72, 1.88, 1.30, 0.35, 0.3, 0.1),
    '4': Voiture("Mazda RX-7 Fd", 1385, 5.2, 4.3, 1.75, 1.23, 0.28, 0.3, 0.1),
    '5': Voiture("Nissan Skyline GTR-R34 1999", 1540, 5.8, 4.6, 1.79, 1.36, 0.34, 0.3, 0.1),
    '6': Voiture("Mitsubichi Lancer Evolution VII", 1600, 5, 4.51, 1.81, 1.48, 0.28, 0.3, 0.1),
}


# ===== PROGRAMME PRINCIPAL =====

def menu_principal():
    print("\n" + "=" * 80)
    print("🏁 SIMULATEUR DE CIRCUIT - FAST & FURIOUS 🏁")
    print("Défi: Battre Owen Shaw (temps < 8 secondes)")
    print("=" * 80)

    # CHOIX DE LA VOITURE
    print("\n🚗 VOITURES DISPONIBLES:")
    for num, voiture in voitures_disponibles.items():
        print(f"  [{num}] {voiture.nom}")
        print(f"      Masse: {voiture.masse}kg | Accél: {voiture.accel_moy}m/s² | Cx: {voiture.Cx} | Cz: {voiture.Cz}")

    while True:
        choix_voiture = input("\n🚗 Choisissez votre voiture (1-6): ").strip()
        if choix_voiture in voitures_disponibles:
            voiture_base = voitures_disponibles[choix_voiture]
            break
        else:
            print("❌ Choix invalide. Veuillez entrer un nombre entre 1 et 6.")

    print(f"\n✓ Voiture sélectionnée: {voiture_base.nom}")

    # CHOIX DES ACCESSOIRES
    print("\n" + "=" * 80)
    print("🔧 ACCESSOIRES DISPONIBLES:")
    print("=" * 80)
    print("  [1] Booster - +30% accélération sur UNE portion (pente/looping/finale)")
    print("  [2] Ailerons - +10% portance, +30kg, surface 0.8m²")
    print("  [3] Jupe avant - -5% traînée, +15kg")
    print("  [0] Aucun accessoire")

    accessoires = []
    portion_booster = None

    while True:
        choix_acc = input("\n🔧 Choisissez vos accessoires (ex: 1,2 ou 0 pour aucun): ").strip()

        if choix_acc == '0':
            print("✓ Aucun accessoire sélectionné")
            break

        choix_liste = [c.strip() for c in choix_acc.split(',')]
        valide = True

        for c in choix_liste:
            if c == '1':
                accessoires.append('booster')
                # Demander la portion pour le booster
                print("\n📍 Sur quelle portion voulez-vous utiliser le booster?")
                print("  [1] Pente")
                print("  [2] Looping")
                print("  [3] Fin de piste")
                while True:
                    choix_portion = input("Choisissez (1-3): ").strip()
                    if choix_portion == '1':
                        portion_booster = 'pente'
                        break
                    elif choix_portion == '2':
                        portion_booster = 'looping'
                        break
                    elif choix_portion == '3':
                        portion_booster = 'finale'
                        break
                    else:
                        print("❌ Choix invalide. Entrez 1, 2 ou 3.")
            elif c == '2':
                accessoires.append('ailerons')
            elif c == '3':
                accessoires.append('jupe')
            else:
                print(f"❌ Accessoire '{c}' invalide")
                valide = False
                break

        if valide:
            break

    # APPLIQUER LES ACCESSOIRES
    if accessoires:
        print(f"\n✓ Accessoires sélectionnés: {', '.join(accessoires)}")
        if portion_booster:
            print(f"✓ Booster activé sur: {portion_booster}")
        voiture_finale = voiture_base.appliquer_accessoires(accessoires, portion_booster)
    else:
        voiture_finale = voiture_base

    # SIMULATION
    print("\n" + "=" * 80)
    print("🏁 LANCEMENT DE LA SIMULATION...")
    print("=" * 80)

    resultats = analyser_voiture(voiture_finale)

    # PROPOSER UNE NOUVELLE SIMULATION
    print("\n" + "=" * 80)
    rejouer = input("\n🔄 Voulez-vous tester une autre configuration? (oui/non): ").strip().lower()
    if rejouer in ['oui', 'o', 'yes', 'y']:
        menu_principal()
    else:
        print("\n👋 Merci d'avoir utilisé le simulateur! Bonne chance pour la course!")


# LANCER LE PROGRAMME
if __name__ == "__main__":
    menu_principal()