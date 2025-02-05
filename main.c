#include "header.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

// Conversion degrés -> radians
double deg_to_rad(double deg) {
    return deg * PI / 180.0;
}

// Conversion radians -> degrés
double rad_to_deg(double rad) {
    return rad * 180.0 / PI;
}

// Calcul du produit vectoriel
Vector3D cross_product(Vector3D v1, Vector3D v2) {
    Vector3D result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

// Calcul du produit scalaire
double dot_product(Vector3D v1, Vector3D v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// Calcul de la matrice élémentaire de rotation
Matrix4x4 calculate_elementary_matrix(double theta, int axis) {
    Matrix4x4 matrix = {0};
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    // Initialisation avec la matrice identité
    for(int i = 0; i < 4; i++) {
        matrix.m[i][i] = 1.0;
    }

    switch(axis) {
        case 1: // Rotation autour de l'axe X
            matrix.m[1][1] = cos_theta;
            matrix.m[1][2] = -sin_theta;
            matrix.m[2][1] = sin_theta;
            matrix.m[2][2] = cos_theta;
            break;
        case 2: // Rotation autour de l'axe Y
            matrix.m[0][0] = cos_theta;
            matrix.m[0][2] = sin_theta;
            matrix.m[2][0] = -sin_theta;
            matrix.m[2][2] = cos_theta;
            break;
        case 3: // Rotation autour de l'axe Z
            matrix.m[0][0] = cos_theta;
            matrix.m[0][1] = -sin_theta;
            matrix.m[1][0] = sin_theta;
            matrix.m[1][1] = cos_theta;
            break;
    }

    return matrix;
}

// Multiplication de deux matrices 4x4
Matrix4x4 multiply_matrices(Matrix4x4 m1, Matrix4x4 m2) {
    Matrix4x4 result = {0};
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                result.m[i][j] += m1.m[i][k] * m2.m[k][j];
            }
        }
    }
    return result;
}

// Calcul du MGD
OrientationPosition calculate_mgd(double theta1, double theta2, double theta3, double theta4) {
    // Conversion des angles en radians
    theta1 = deg_to_rad(theta1);
    theta2 = deg_to_rad(theta2);
    theta3 = deg_to_rad(theta3);
    theta4 = deg_to_rad(theta4);

    // Calcul des matrices élémentaires
    Matrix4x4 T1 = calculate_elementary_matrix(theta1, 3); // Rotation Z
    Matrix4x4 T2 = calculate_elementary_matrix(theta2, 2); // Rotation Y
    Matrix4x4 T3 = calculate_elementary_matrix(theta3, 2); // Rotation Y
    Matrix4x4 T4 = calculate_elementary_matrix(theta4, 3); // Rotation Z

    // Calcul de la matrice de transformation globale
    Matrix4x4 T = multiply_matrices(multiply_matrices(multiply_matrices(T1, T2), T3), T4);

    // Extraction des vecteurs n, o, a et p
    OrientationPosition result;
    
    // Vecteur n
    result.n.x = T.m[0][0];
    result.n.y = T.m[1][0];
    result.n.z = T.m[2][0];

    // Vecteur o
    result.o.x = T.m[0][1];
    result.o.y = T.m[1][1];
    result.o.z = T.m[2][1];

    // Vecteur a
    result.a.x = T.m[0][2];
    result.a.y = T.m[1][2];
    result.a.z = T.m[2][2];

    // Position p
    result.p.x = T.m[0][3];
    result.p.y = T.m[1][3];
    result.p.z = T.m[2][3];

    return result;
}

// Calcul du MGI
void calculate_mgi(OrientationPosition target, double* theta1, double* theta2, double* theta3, double* theta4) {
    // Vérification que z4 = x4 vectoriel y4
    Vector3D z4_calculated = cross_product(target.n, target.o);
    if (fabs(z4_calculated.x - target.a.x) > 1e-10 ||
        fabs(z4_calculated.y - target.a.y) > 1e-10 ||
        fabs(z4_calculated.z - target.a.z) > 1e-10) {
        printf("Erreur: Le repère cible n'est pas orthonormé direct\n");
        return;
    }

    // Calcul des angles
    // Note: Cette implémentation est simplifiée et devrait être adaptée
    // selon la géométrie spécifique du robot
    *theta1 = atan2(target.p.y, target.p.x);
    *theta2 = atan2(sqrt(target.p.x * target.p.x + target.p.y * target.p.y), target.p.z);
    *theta3 = atan2(target.a.z, -target.a.x);
    *theta4 = atan2(target.n.y, target.n.x);

    // Conversion en degrés
    *theta1 = rad_to_deg(*theta1);
    *theta2 = rad_to_deg(*theta2);
    *theta3 = rad_to_deg(*theta3);
    *theta4 = rad_to_deg(*theta4);
}

// Fonction principale pour tester
int main() {
    // Test du MGD
    double theta1 = 30.0;  // degrés
    double theta2 = 45.0;
    double theta3 = -30.0;
    double theta4 = 60.0;

    OrientationPosition result = calculate_mgd(theta1, theta2, theta3, theta4);

    printf("Résultat du MGD:\n");
    printf("n: (%f, %f, %f)\n", result.n.x, result.n.y, result.n.z);
    printf("o: (%f, %f, %f)\n", result.o.x, result.o.y, result.o.z);
    printf("a: (%f, %f, %f)\n", result.a.x, result.a.y, result.a.z);
    printf("p: (%f, %f, %f)\n", result.p.x, result.p.y, result.p.z);

    // Test du MGI
    double theta1_calc, theta2_calc, theta3_calc, theta4_calc;
    calculate_mgi(result, &theta1_calc, &theta2_calc, &theta3_calc, &theta4_calc);

    printf("\nRésultat du MGI:\n");
    printf("theta1: %f°\n", theta1_calc);
    printf("theta2: %f°\n", theta2_calc);
    printf("theta3: %f°\n", theta3_calc);
    printf("theta4: %f°\n", theta4_calc);

    return 0;
}
