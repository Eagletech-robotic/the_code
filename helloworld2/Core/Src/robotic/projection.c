#include <stdio.h>
#include <math.h>

/* Fonction pour calculer le produit scalaire (dot product) de deux vecteurs 3D */
float dotProduct(const float v1[3], const float v2[3]) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

/* Fonction pour calculer le produit vectoriel (cross product) de deux vecteurs 3D */
void crossProduct(const float v1[3], const float v2[3], float result[3]) {
    result[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

/* Fonction pour normaliser un vecteur 3D */
void normalize(float v[3]) {
    float length = sqrt(dotProduct(v, v));
    if (length > 1e-8f) {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
}

/*
 * Projection de M sur le plan perpendiculaire à A et récupération
 * des coordonnées 2D dans la base (T1, T2).
 *
 * Paramètres:
 *   A, M : vecteurs 3D
 *   M2D  : vecteur 2D (résultat)
 */
void projectOnPlane(const float A[3], const float M[3], float M2D[2]) {
    // 1) Normaliser A
    float A_unit[3] = { A[0], A[1], A[2] };
    normalize(A_unit);

    // 2) Trouver deux vecteurs T1, T2 perpendiculaires à A_unit
    //    - Pour T1, on peut prendre cross(A_unit, un vecteur "arbitraire")
    //      Pour éviter les problèmes, choisissons un vecteur simple (ex : l'axe X = {1,0,0}
    //      s'il n'est pas trop aligné avec A). Si A est presque parallèle à X, on peut choisir Y, etc.
    float arbitrary[3] = {0.0f, 0.0f, 1.0f};
    // Vérification pour éviter le cas de colinéarité
    if (fabsf(dotProduct(A_unit, arbitrary)) > 0.99f) {
        arbitrary[0] = 0.0f;
        arbitrary[1] = 1.0f;
        arbitrary[2] = 0.0f;
    }

    float T1[3];
    crossProduct(arbitrary, A_unit,  T1);
    normalize(T1);

    float T2[3];
    crossProduct(A_unit, T1, T2);
    normalize(T2);

    // 3) M_perp_A = M - (M dot A_unit)*A_unit
    float M_dot_A = dotProduct(M, A_unit);
    float M_parA[3] = { A_unit[0]*M_dot_A, A_unit[1]*M_dot_A, A_unit[2]*M_dot_A };
    float M_perp_A[3] = { M[0] - M_parA[0], M[1] - M_parA[1], M[2] - M_parA[2] };

    // 4) Coordonnées 2D = (M_perp_A dot T1, M_perp_A dot T2)
    M2D[0] = dotProduct(M_perp_A, T1);
    M2D[1] = dotProduct(M_perp_A, T2);
}


/*
 * Retourne l'angle en radians entre deux vecteurs 2D A et B.
 * Si l'un des vecteurs est nul (longueur quasi nulle),
 * la fonction renvoie 0.0f par convention (ou un autre code de votre choix).
 *
 * θ=arccos(A⋅B/∥A∥∥B∥​).
 */
float angleBetween2D(const float A[2], const float B[2]) {
    // Produit scalaire A·B
    float dot = A[0] * B[0] + A[1] * B[1];

    // Norme de A
    float normA = sqrtf(A[0]*A[0] + A[1]*A[1]);
    // Norme de B
    float normB = sqrtf(B[0]*B[0] + B[1]*B[1]);

    // Si au moins un vecteur est nul, on renvoie 0.0f par sécurité
    if (normA < 1e-8f || normB < 1e-8f) {
        return 0.0f;
    }

    // Calcul du cosinus de l'angle
    float cosAngle = dot / (normA * normB);

    // Sécuriser cosAngle dans l'intervalle [-1, 1] pour éviter
    // d'éventuels débordements numériques (ex. 1.0000002)
    if (cosAngle > 1.0f)  cosAngle = 1.0f;
    if (cosAngle < -1.0f) cosAngle = -1.0f;

    // Retourne l'angle en radians
    float a = acosf(cosAngle);
    printf(" %.2f\r\n",a);
    return a;
}

// Fonction pour calculer la norme d'un vecteur
float vectorNorm(const float v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Fonction pour calculer l'angle entre deux vecteurs 3D en radians
float angleBetweenVectors(const float v1[3], const float v2[3]) {
    float dot = dotProduct(v1, v2);
    float norm1 = vectorNorm(v1);
    float norm2 = vectorNorm(v2);

    // Vérification pour éviter une division par zéro
    if (norm1 < 1e-8f || norm2 < 1e-8f) {
         return 0.0f;
     }
    float cosTheta = dot / (norm1 * norm2);

    // S'assurer que le cosinus reste dans [-1, 1] pour éviter des erreurs d'approximation
    if (cosTheta < -1.0f) cosTheta = -1.0f;
    if (cosTheta > 1.0f) cosTheta = 1.0f;

    float a = acosf(cosTheta);
    //printf("%.2f  %.2f %.2f %.2f  %.2f %.2f    %.2f\r\n",a*(180.0f/M_PI),v2[0],v2[1],v2[2], norm1, norm2, cosTheta);
    printf("%.2f \r\n",a);
    return a; // Retourne l'angle en radians
}

// from https://github.com/pololu/lis3mdl-arduino/blob/master/examples/HeadingWithLSM6/HeadingWithLSM6.ino
// calibrate_nr() le 2025 01 05 :
//min: {-0.8, -0.144, -0.199} max: {+0.000000, +0.237, +0.00000}
const float m_min_x = -.8f;
const float m_min_y = -.144f;
const float m_min_z = -.199f;
const float m_max_x = 0.0f;
const float m_max_y = 0.237f;
const float m_max_z = 0.0f;
float computeHeading_with_base(float from[3], float mag[3], float acc[3])
{
  float temp_m[3] = {mag[0], mag[1], mag[2]};

  // copy acceleration readings from LSM6::vector into an LIS3MDL::vector
  //LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  // subtract offset (average of min and max) from magnetometer readings
  temp_m[0] -= (m_min_x + m_max_x) / 2;
  temp_m[1] -= (m_min_y + m_max_y) / 2;
  temp_m[2] -= (m_min_z + m_max_z) / 2;

  // compute E and N
  float E[3];
  float N[3];
  crossProduct(mag, acc, E);
  normalize(E);

  crossProduct(acc, E, N);
  normalize(N);

  // compute heading
  float heading = atan2(dotProduct(E, from), dotProduct(N, from)) * 180.0 / M_PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

float computeHeading(float mag[3], float acc[3]) {
	float from[]={1.0,0.0,0.0};
	return computeHeading_with_base(from, mag, acc);
}

// Permet de déterminer un offset sur la mesure qui peut être assez énorme
float running_min_x = 0.0;
float running_min_y = 0.0;
float running_min_z = 0.0;
float running_max_x = 0.0;
float running_max_y = 0.0;
float running_max_z = 0.0;
void calibrate_nr(float mag[3]) {
	  running_min_x = fmin(running_min_x, mag[0]);
	  running_min_y = fmin(running_min_y, mag[1]);
	  running_min_z = fmin(running_min_z, mag[3]);

	  running_max_x = fmax(running_max_x, mag[0]);
	  running_max_y = fmax(running_max_y, mag[1]);
	  running_max_z = fmax(running_max_z, mag[2]);

	  printf("min: {%+6f, %+6f, %+6f}   max: {%+6f, %+6f, %+6f} \r\n",
	    running_min_x, running_min_y, running_min_z,
	    running_max_x, running_max_y, running_max_z);
}

float running_max =0.0f;
float running_min =1000000.0f;
void stat_nr(float data) {
	  running_min = fmin(running_min, data);
	  running_max = fmax(running_max, data);
	  printf("%+2f min : %+2f  max: %+2f \r\n", data, running_min, running_max);
}
