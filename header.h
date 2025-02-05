#ifndef HEADER_H
#define HEADER_H

#include <math.h>

// Structure pour représenter une matrice 4x4
typedef struct {
    double m[4][4];
} Matrix4x4;

// Structure pour représenter un vecteur 3D
typedef struct {
    double x;
    double y;
    double z;
} Vector3D;

// Structure pour représenter l'orientation et la position
typedef struct {
    Vector3D n;  // Vecteur n
    Vector3D o;  // Vecteur o
    Vector3D a;  // Vecteur a
    Vector3D p;  // Position
} OrientationPosition;

// Fonctions pour le MGD
Matrix4x4 calculate_elementary_matrix(double theta, int axis);
OrientationPosition calculate_mgd(double theta1, double theta2, double theta3, double theta4);

// Fonctions pour le MGI
void calculate_mgi(OrientationPosition target, double* theta1, double* theta2, double* theta3, double* theta4);

// Fonctions utilitaires
double deg_to_rad(double deg);
double rad_to_deg(double rad);
Vector3D cross_product(Vector3D v1, Vector3D v2);
double dot_product(Vector3D v1, Vector3D v2);

#endif // HEADER_H
