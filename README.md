# Modèle Géométrique Direct et Inverse d'un Robot

Ce programme implémente le Modèle Géométrique Direct (MGD) et le Modèle Géométrique Inverse (MGI) d'un robot à 4 degrés de liberté. Il permet de calculer la position et l'orientation de l'effecteur du robot en fonction des angles des articulations (MGD) et vice-versa (MGI).

## Contexte

Dans la robotique, le MGD et le MGI sont des outils fondamentaux pour :
- MGD : Déterminer la position et l'orientation de l'effecteur du robot en fonction des angles des articulations
- MGI : Calculer les angles des articulations nécessaires pour atteindre une position et une orientation données

## Structure du Programme

Le programme est composé de trois fichiers principaux :
- `header.h` : Contient les déclarations des structures et fonctions
- `main.c` : Contient l'implémentation des fonctions et le programme de test
- `README.md` : Documentation du projet

### Structures de Données

1. **Matrix4x4** : Représente une matrice 4x4 pour les transformations homogènes
```c
typedef struct {
    double m[4][4];
} Matrix4x4;
```

2. **Vector3D** : Représente un vecteur 3D
```c
typedef struct {
    double x;
    double y;
    double z;
} Vector3D;
```

3. **OrientationPosition** : Représente l'orientation et la position de l'effecteur
```c
typedef struct {
    Vector3D n;  // Vecteur n
    Vector3D o;  // Vecteur o
    Vector3D a;  // Vecteur a
    Vector3D p;  // Position
} OrientationPosition;
```

## Fonctionnalités

### 1. Modèle Géométrique Direct (MGD)
- Prend en entrée les angles θ1, θ2, θ3, θ4 en degrés
- Calcule les matrices de transformation élémentaires
- Combine ces matrices pour obtenir la transformation globale
- Retourne les vecteurs n, o, a (orientation) et p (position)

### 2. Modèle Géométrique Inverse (MGI)
- Prend en entrée une position et une orientation désirées
- Vérifie que le repère est orthonormé direct
- Calcule les angles θ1, θ2, θ3, θ4 correspondants
- Retourne les angles en degrés

### 3. Fonctions Utilitaires
- Conversion degrés/radians
- Calcul de produit vectoriel et scalaire
- Multiplication de matrices
- Calcul des matrices élémentaires de rotation

## Compilation et Exécution

### Prérequis
- Un compilateur C (gcc recommandé)
- La bibliothèque mathématique (math.h)

### Compilation
```bash
gcc -o robot main.c -lm
```
Note : L'option `-lm` est nécessaire pour lier la bibliothèque mathématique.

### Exécution
```bash
./robot
```

## Exemple de Sortie

```
Résultat du MGD:
n: (-0.014755, 0.991481, -0.129410)
o: (-0.974444, 0.014755, 0.224144)
a: (0.224144, 0.129410, 0.965926)
p: (0.000000, 0.000000, 0.000000)

Résultat du MGI:
theta1: 0.000000°
theta2: 0.000000°
theta3: 103.064313°
theta4: 90.852574°
```

## Modification des Tests

Pour tester différentes configurations, vous pouvez modifier les valeurs des angles dans la fonction `main()` du fichier `main.c` :

```c
double theta1 = 30.0;  // degrés
double theta2 = 45.0;
double theta3 = -30.0;
double theta4 = 60.0;
```

## Notes Importantes

1. Les angles sont traités en degrés dans l'interface utilisateur mais convertis en radians pour les calculs internes.
2. Le MGI vérifie que le repère cible est orthonormé direct avant de calculer les angles.
3. La précision des calculs dépend de la précision de la représentation en virgule flottante.

## Limitations Actuelles

1. Le MGI utilise une implémentation simplifiée qui pourrait nécessiter des ajustements selon la géométrie spécifique du robot.
2. Le programme ne gère pas encore les singularités du robot.
3. Les limites articulaires ne sont pas prises en compte.

## Perspectives d'Amélioration

1. Ajout de la gestion des singularités
2. Implémentation des limites articulaires
3. Ajout de visualisation 3D
4. Optimisation des calculs matriciels
5. Ajout de tests unitaires 