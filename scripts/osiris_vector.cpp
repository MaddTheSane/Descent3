/*
* Descent 3
* Copyright (C) 2024 Parallax Software
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <cstring>
#include "osiris_vector.h"

void vm_AverageVector(simd::float3 *a, int num) {
  // Averages a vector.  ie divides each component of vector a by num
  // assert (num!=0);

  a->x = a->x / (float)num;
  a->y = a->y / (float)num;
  a->z = a->z / (float)num;
}

void vm_AddVectors(simd::float3 *result, simd::float3 *a, simd::float3 *b) {
  // Adds two vectors.  Either source can equal dest

  *result = (*a) + (*b);
}

void vm_SubVectors(simd::float3 *result, const simd::float3 *a, const simd::float3 *b) {
  // Subtracts second vector from first.  Either source can equal dest

  *result = (*a) - (*b);
}

float vm_VectorDistance(const simd::float3 *a, const simd::float3 *b) {
  // Given two vectors, returns the distance between them

  simd::float3 dest;
  float dist;

  vm_SubVectors(&dest, a, b);
  dist = vm_GetMagnitude(&dest);
  return dist;
}
float vm_VectorDistanceQuick(simd::float3 *a, simd::float3 *b) {
  // Given two vectors, returns the distance between them

  simd::float3 dest;
  float dist;

  vm_SubVectors(&dest, a, b);
  dist = vm_GetMagnitudeFast(&dest);
  return dist;
}

// Calculates the perpendicular vector given three points
// Parms:	n - the computed perp vector (filled in)
//			v0,v1,v2 - three clockwise vertices
void vm_GetPerp(simd::float3 *n, simd::float3 *a, simd::float3 *b, simd::float3 *c) {
  // Given 3 vertices, return the surface normal in n
  // IMPORTANT: B must be the 'corner' vertex

  simd::float3 x, y;

  vm_SubVectors(&x, b, a);
  vm_SubVectors(&y, c, b);

  vm_CrossProduct(n, &x, &y);
}

// Calculates the (normalized) surface normal give three points
// Parms:	n - the computed surface normal (filled in)
//			v0,v1,v2 - three clockwise vertices
// Returns the magnitude of the normal before it was normalized.
// The bigger this value, the better the normal.
float vm_GetNormal(simd::float3 *n, simd::float3 *v0, simd::float3 *v1, simd::float3 *v2) {
  vm_GetPerp(n, v0, v1, v2);

  return vm_VectorNormalize(n);
}

// Does a simple dot product calculation
float vm_DotProduct(simd::float3 *u, simd::float3 *v) { return simd::dot(*u, *v); }

// Scales all components of vector v by value s and stores result in vector d
// dest can equal source
void vm_ScaleVector(simd::float3 *d, simd::float3 *v, float s) {
  *d = *v * s;
}

void vm_ScaleAddVector(simd::float3 *d, simd::float3 *p, simd::float3 *v, float s) {
  // Scales all components of vector v by value s
  // adds the result to p and stores result in vector d
  // dest can equal source

  *d = *p + (*v * s);
}

void vm_DivVector(simd::float3 *dest, simd::float3 *src, float n) {
  // Divides a vector into n portions
  // Dest can equal src

  // assert (n!=0);

  *dest = *src / n;
}

void vm_CrossProduct(simd::float3 *dest, simd::float3 *u, simd::float3 *v) {
  // Computes a cross product between u and v, returns the result
  //	in Normal.  Dest cannot equal source.

  *dest = simd::cross(*u, *v);
}

// Normalize a vector.
// Returns:  the magnitude before normalization
float vm_VectorNormalize(simd::float3 *a) {
  float mag;

  mag = vm_GetMagnitude(a);

  if (mag > 0)
    *a /= mag;
  else {
    *a = Zero_vector;
    a->x = 1.0;
    mag = 0.0f;
  }

  return mag;
}

float vm_GetMagnitude(simd::float3 *a) {
  float f = simd::length_squared(*a);

  //  f = (a->x * a->x) + (a->y * a->y) + (a->z * a->z);

  return (sqrt(f));
}

void vm_ClearMatrix(vec::matrix *dest) { memset(dest, 0, sizeof(vec::matrix)); }

void vm_MakeIdentity(vec::matrix *dest) {
  memset(dest, 0, sizeof(vec::matrix));
  dest->rvec.x = dest->uvec.y = dest->fvec.z = 1.0;
}
void vm_MakeInverseMatrix(vec::matrix *dest) {
  memset((void *)dest, 0, sizeof(vec::matrix));
  dest->rvec.x = dest->uvec.y = dest->fvec.z = -1.0;
}

void vm_TransposeMatrix(vec::matrix *m) {
  // Transposes a matrix in place

  float t;

  t = m->uvec.x;
  m->uvec.x = m->rvec.y;
  m->rvec.y = t;
  t = m->fvec.x;
  m->fvec.x = m->rvec.z;
  m->rvec.z = t;
  t = m->fvec.y;
  m->fvec.y = m->uvec.z;
  m->uvec.z = t;
}

void vm_MatrixMulVector(simd::float3 *result, simd::float3 *v, vec::matrix *m) {
  // Rotates a vector thru a matrix

  // assert(result != v);

  result->x = simd::dot(*v, m->rvec);
  result->y = simd::dot(*v, m->uvec);
  result->z = simd::dot(*v, m->fvec);
}

// Multiply a vector times the transpose of a matrix
void vm_VectorMulTMatrix(simd::float3 *result, simd::float3 *v, vec::matrix *m) {
  // assert(result != v);

  result->x = vec::vm_Dot3Vector(m->rvec.x, m->uvec.x, m->fvec.x, v);
  result->y = vec::vm_Dot3Vector(m->rvec.y, m->uvec.y, m->fvec.y, v);
  result->z = vec::vm_Dot3Vector(m->rvec.z, m->uvec.z, m->fvec.z, v);
}

void vm_MatrixMul(vec::matrix *dest, vec::matrix *src0, vec::matrix *src1) {
  // For multiplying two 3x3 matrices together

  // assert((dest != src0) && (dest != src1));

  dest->rvec.x = vec::vm_Dot3Vector(src0->rvec.x, src0->uvec.x, src0->fvec.x, &src1->rvec);
  dest->uvec.x = vec::vm_Dot3Vector(src0->rvec.x, src0->uvec.x, src0->fvec.x, &src1->uvec);
  dest->fvec.x = vec::vm_Dot3Vector(src0->rvec.x, src0->uvec.x, src0->fvec.x, &src1->fvec);

  dest->rvec.y = vec::vm_Dot3Vector(src0->rvec.y, src0->uvec.y, src0->fvec.y, &src1->rvec);
  dest->uvec.y = vec::vm_Dot3Vector(src0->rvec.y, src0->uvec.y, src0->fvec.y, &src1->uvec);
  dest->fvec.y = vec::vm_Dot3Vector(src0->rvec.y, src0->uvec.y, src0->fvec.y, &src1->fvec);

  dest->rvec.z = vec::vm_Dot3Vector(src0->rvec.z, src0->uvec.z, src0->fvec.z, &src1->rvec);
  dest->uvec.z = vec::vm_Dot3Vector(src0->rvec.z, src0->uvec.z, src0->fvec.z, &src1->uvec);
  dest->fvec.z = vec::vm_Dot3Vector(src0->rvec.z, src0->uvec.z, src0->fvec.z, &src1->fvec);
}

// Multiply a matrix times the transpose of a matrix
void vm_MatrixMulTMatrix(vec::matrix *dest, vec::matrix *src0, vec::matrix *src1) {
  // For multiplying two 3x3 matrices together

  // assert((dest != src0) && (dest != src1));

  dest->rvec.x = src0->rvec.x * src1->rvec.x + src0->uvec.x * src1->uvec.x + src0->fvec.x * src1->fvec.x;
  dest->uvec.x = src0->rvec.x * src1->rvec.y + src0->uvec.x * src1->uvec.y + src0->fvec.x * src1->fvec.y;
  dest->fvec.x = src0->rvec.x * src1->rvec.z + src0->uvec.x * src1->uvec.z + src0->fvec.x * src1->fvec.z;

  dest->rvec.y = src0->rvec.y * src1->rvec.x + src0->uvec.y * src1->uvec.x + src0->fvec.y * src1->fvec.x;
  dest->uvec.y = src0->rvec.y * src1->rvec.y + src0->uvec.y * src1->uvec.y + src0->fvec.y * src1->fvec.y;
  dest->fvec.y = src0->rvec.y * src1->rvec.z + src0->uvec.y * src1->uvec.z + src0->fvec.y * src1->fvec.z;

  dest->rvec.z = src0->rvec.z * src1->rvec.x + src0->uvec.z * src1->uvec.x + src0->fvec.z * src1->fvec.x;
  dest->uvec.z = src0->rvec.z * src1->rvec.y + src0->uvec.z * src1->uvec.y + src0->fvec.z * src1->fvec.y;
  dest->fvec.z = src0->rvec.z * src1->rvec.z + src0->uvec.z * src1->uvec.z + src0->fvec.z * src1->fvec.z;
}

vec::matrix operator*(vec::matrix src0, vec::matrix src1) {
  // For multiplying two 3x3 matrices together
  vec::matrix dest;

  dest.rvec.x = vec::vm_Dot3Vector(src0.rvec.x, src0.uvec.x, src0.fvec.x, &src1.rvec);
  dest.uvec.x = vec::vm_Dot3Vector(src0.rvec.x, src0.uvec.x, src0.fvec.x, &src1.uvec);
  dest.fvec.x = vec::vm_Dot3Vector(src0.rvec.x, src0.uvec.x, src0.fvec.x, &src1.fvec);

  dest.rvec.y = vec::vm_Dot3Vector(src0.rvec.y, src0.uvec.y, src0.fvec.y, &src1.rvec);
  dest.uvec.y = vec::vm_Dot3Vector(src0.rvec.y, src0.uvec.y, src0.fvec.y, &src1.uvec);
  dest.fvec.y = vec::vm_Dot3Vector(src0.rvec.y, src0.uvec.y, src0.fvec.y, &src1.fvec);

  dest.rvec.z = vec::vm_Dot3Vector(src0.rvec.z, src0.uvec.z, src0.fvec.z, &src1.rvec);
  dest.uvec.z = vec::vm_Dot3Vector(src0.rvec.z, src0.uvec.z, src0.fvec.z, &src1.uvec);
  dest.fvec.z = vec::vm_Dot3Vector(src0.rvec.z, src0.uvec.z, src0.fvec.z, &src1.fvec);

  return dest;
}

vec::matrix operator*=(vec::matrix &src0, vec::matrix src1) { return (src0 = src0 * src1); }

// Computes a normalized direction vector between two points
// Parameters:	dest - filled in with the normalized direction vector
//					start,end - the start and end points used to calculate the vector
// Returns:		the distance between the two input points
float vm_GetNormalizedDir(simd::float3 *dest, simd::float3 *end, simd::float3 *start) {
  *dest = (*end) - (*start);
  return vm_VectorNormalize(dest);
}

// Returns a normalized direction vector between two points
// Just like vm_GetNormalizedDir(), but uses sloppier magnitude, less precise
// Parameters:	dest - filled in with the normalized direction vector
//					start,end - the start and end points used to calculate the vector
// Returns:		the distance between the two input points
float vm_GetNormalizedDirFast(simd::float3 *dest, simd::float3 *end, simd::float3 *start) {
  *dest = (*end) - (*start);
  return vm_VectorNormalizeFast(dest);
}

float vm_GetMagnitudeFast(simd::float3 *v) {
  float a, b, c, bc;

  a = fabs(v->x);
  b = fabs(v->y);
  c = fabs(v->z);

  if (a < b) {
    float t = a;
    a = b;
    b = t;
  }

  if (b < c) {
    float t = b;
    b = c;
    c = t;

    if (a < b) {
      float t = a;
      a = b;
      b = t;
    }
  }

  bc = (b / 4) + (c / 8);

  return a + bc + (bc / 2);
}

// Normalize a vector using an approximation of the magnitude
// Returns:  the magnitude before normalization
float vm_VectorNormalizeFast(simd::float3 *a) {
  float mag;

  mag = vm_GetMagnitudeFast(a);

  if (mag == 0.0) {
    a->x = a->y = a->z = 0.0;
    return 0;
  }

  a->x = (a->x / mag);
  a->y = (a->y / mag);
  a->z = (a->z / mag);

  return mag;
}

// Computes the distance from a point to a plane.
// Parms:	checkp - the point to check
// Parms:	norm - the (normalized) surface normal of the plane
//				planep - a point on the plane
// Returns:	The signed distance from the plane; negative dist is on the back of the plane
float vm_DistToPlane(simd::float3 *checkp, simd::float3 *norm, simd::float3 *planep) {
  simd::float3 t;

  t = *checkp - *planep;

  return simd::dot(t, *norm);
}

float vm_GetSlope(float x1, float y1, float x2, float y2) {
  // returns the slope of a line
  float r;

  if (y2 - y1 == 0)
    return (0.0);

  r = (x2 - x1) / (y2 - y1);
  return (r);
}

void vm_SinCosToMatrix(vec::matrix *m, float sinp, float cosp, float sinb, float cosb, float sinh, float cosh) {
  float sbsh, cbch, cbsh, sbch;

  sbsh = (sinb * sinh);
  cbch = (cosb * cosh);
  cbsh = (cosb * sinh);
  sbch = (sinb * cosh);

  m->rvec.x = cbch + (sinp * sbsh); // m1
  m->uvec.z = sbsh + (sinp * cbch); // m8

  m->uvec.x = (sinp * cbsh) - sbch; // m2
  m->rvec.z = (sinp * sbch) - cbsh; // m7

  m->fvec.x = (sinh * cosp); // m3
  m->rvec.y = (sinb * cosp); // m4
  m->uvec.y = (cosb * cosp); // m5
  m->fvec.z = (cosh * cosp); // m9

  m->fvec.y = -sinp; // m6
}

void vm_AnglesToMatrix(vec::matrix *m, angle p, angle h, angle b) {
  float sinp, cosp, sinb, cosb, sinh, cosh;

  sinp = FixSin(p);
  cosp = FixCos(p);
  sinb = FixSin(b);
  cosb = FixCos(b);
  sinh = FixSin(h);
  cosh = FixCos(h);

  vm_SinCosToMatrix(m, sinp, cosp, sinb, cosb, sinh, cosh);
}

// Computes a matrix from a vector and and angle of rotation around that vector
// Parameters:	m - filled in with the computed matrix
//					v - the forward vector of the new matrix
//					a - the angle of rotation around the forward vector
void vm_VectorAngleToMatrix(vec::matrix *m, simd::float3 *v, angle a) {
  float sinb, cosb, sinp, cosp, sinh, cosh;

  sinb = FixSin(a);
  cosb = FixCos(a);

  sinp = -v->y;
  cosp = sqrt(1.0 - (sinp * sinp));

  if (cosp != 0.0) {
    sinh = v->x / cosp;
    cosh = v->z / cosp;
  } else {
    sinh = 0;
    cosh = 1.0;
  }

  vm_SinCosToMatrix(m, sinp, cosp, sinb, cosb, sinh, cosh);
}

// Ensure that a matrix is orthogonal
void vm_Orthogonalize(vec::matrix *m) {
  // Normalize forward vector
  if (vm_VectorNormalize(&m->fvec) == 0) {
    return;
  }

  // Generate right vector from forward and up vectors
  m->rvec = simd::cross(m->uvec, m->fvec);

  // Normaize new right vector
  if (vm_VectorNormalize(&m->rvec) == 0) {
    vm_VectorToMatrix(m, &m->fvec, NULL, NULL); // error, so generate from forward vector only
    return;
  }

  // Recompute up vector, in case it wasn't entirely perpendiclar
  m->uvec = simd::cross(m->fvec, m->rvec);
}

// do the math for vm_VectorToMatrix()
void DoVectorToMatrix(vec::matrix *m, simd::float3 *fvec, simd::float3 *uvec, simd::float3 *rvec) {
  simd::float3 *xvec = &m->rvec, *yvec = &m->uvec, *zvec = &m->fvec;

  // ASSERT(fvec != NULL);

  *zvec = *fvec;
  if (vm_VectorNormalize(zvec) == 0) {
    return;
  }

  if (uvec == NULL) {

    if (rvec == NULL) { // just forward vec

    bad_vector2:;

      if (zvec->x == 0 && zvec->z == 0) { // forward vec is straight up or down

        m->rvec.x = 1.0;
        m->uvec.z = (zvec->y < 0) ? 1.0 : -1.0;

        m->rvec.y = m->rvec.z = m->uvec.x = m->uvec.y = 0;
      } else { // not straight up or down

        xvec->x = zvec->z;
        xvec->y = 0;
        xvec->z = -zvec->x;

        vm_VectorNormalize(xvec);

        *yvec = simd::cross(*zvec, *xvec);
      }

    } else { // use right vec

      *xvec = *rvec;
      if (vm_VectorNormalize(xvec) == 0)
        goto bad_vector2;

      *yvec = simd::cross(*zvec, *xvec);

      // normalize new perpendicular vector
      if (vm_VectorNormalize(yvec) == 0)
        goto bad_vector2;

      // now recompute right vector, in case it wasn't entirely perpendiclar
      *xvec = simd::cross(*yvec, *zvec);
    }
  } else { // use up vec

    *yvec = *uvec;
    if (vm_VectorNormalize(yvec) == 0)
      goto bad_vector2;

    *xvec = simd::cross(*yvec, *zvec);

    // normalize new perpendicular vector
    if (vm_VectorNormalize(xvec) == 0)
      goto bad_vector2;

    // now recompute up vector, in case it wasn't entirely perpendiclar
    *yvec = simd::cross(*zvec, *xvec);
  }
}

// Compute a matrix from one or two vectors.  At least one and at most two vectors must/can be specified.
// Parameters:	m - filled in with the orienation matrix
//					fvec,uvec,rvec - pointers to vectors that determine the matrix.
//						One or two of these must be specified, with the other(s) set to NULL.
void vm_VectorToMatrix(vec::matrix *m, simd::float3 *fvec, simd::float3 *uvec, simd::float3 *rvec) {
  if (!fvec) { // no forward vector.  Use up and/or right vectors.
    vec::matrix tmatrix;

    if (uvec) { // got up vector. use up and, if specified, right vectors.
      DoVectorToMatrix(&tmatrix, uvec, NULL, rvec);
      m->fvec = -tmatrix.uvec;
      m->uvec = tmatrix.fvec;
      m->rvec = tmatrix.rvec;
      return;
    } else { // no up vector.  Use right vector only.
      // ASSERT(rvec);
      DoVectorToMatrix(&tmatrix, rvec, NULL, NULL);
      m->fvec = -tmatrix.rvec;
      m->uvec = tmatrix.uvec;
      m->rvec = tmatrix.fvec;
      return;
    }
  } else {
    // ASSERT(! (uvec && rvec));		//can only have 1 or 2 vectors specified
    DoVectorToMatrix(m, fvec, uvec, rvec);
  }
}

void vm_SinCos(uint16_t a, float *s, float *c) {
  if (s)
    *s = FixSin(a);
  if (c)
    *c = FixCos(a);
}

// extract angles from a matrix
vec::angvec *vm_ExtractAnglesFromMatrix(vec::angvec *a, vec::matrix *m) {
  float sinh, cosh, cosp;

  if (m->fvec.x == 0 && m->fvec.z == 0) // zero head
    a->h = 0;
  else
    a->h = FixAtan2(m->fvec.z, m->fvec.x);

  sinh = FixSin(a->h);
  cosh = FixCos(a->h);

  if (fabs(sinh) > fabs(cosh)) // sine is larger, so use it
    cosp = (m->fvec.x / sinh);
  else // cosine is larger, so use it
    cosp = (m->fvec.z / cosh);

  if (cosp == 0 && m->fvec.y == 0)
    a->p = 0;
  else
    a->p = FixAtan2(cosp, -m->fvec.y);

  if (cosp == 0) // the cosine of pitch is zero.  we're pitched straight up. say no bank

    a->b = 0;

  else {
    float sinb, cosb;

    sinb = (m->rvec.y / cosp);
    cosb = (m->uvec.y / cosp);

    if (sinb == 0 && cosb == 0)
      a->b = 0;
    else
      a->b = FixAtan2(cosb, sinb);
  }

  return a;
}

// returns the value of a determinant
float calc_det_value(vec::matrix *det) {
  return det->rvec.x * det->uvec.y * det->fvec.z - det->rvec.x * det->uvec.z * det->fvec.y -
         det->rvec.y * det->uvec.x * det->fvec.z + det->rvec.y * det->uvec.z * det->fvec.x +
         det->rvec.z * det->uvec.x * det->fvec.y - det->rvec.z * det->uvec.y * det->fvec.x;
}

// computes the delta angle between two vectors.
// vectors need not be normalized. if they are, call vm_vec_delta_ang_norm()
// the forward vector (third parameter) can be NULL, in which case the absolute
// value of the angle in returned.  Otherwise the angle around that vector is
// returned.

angle vm_DeltaAngVec(simd::float3 *v0, simd::float3 *v1, simd::float3 *fvec) {
  simd::float3 t0, t1;

  t0 = *v0;
  t1 = *v1;

  vm_VectorNormalize(&t0);
  vm_VectorNormalize(&t1);

  return vm_DeltaAngVecNorm(&t0, &t1, fvec);
}

// computes the delta angle between two normalized vectors.
angle vm_DeltaAngVecNorm(simd::float3 *v0, simd::float3 *v1, simd::float3 *fvec) {
  angle a;

  a = FixAcos(vm_DotProduct(v0, v1));

  if (fvec) {
    simd::float3 t;

    vm_CrossProduct(&t, v0, v1);
    if (vm_DotProduct(&t, fvec) < 0)
      a = -a;
  }

  return a;
}

// Gets the real center of a polygon
// Returns the size of the passed in stuff
float vm_GetCentroid(simd::float3 *centroid, simd::float3 *src, int nv) {
  // ASSERT (nv>2);
  simd::float3 normal;
  float area, total_area;
  int i;
  simd::float3 tmp_center;

  vec::vm_MakeZero(centroid);

  // First figure out the total area of this polygon
  vm_GetPerp(&normal, &src[0], &src[1], &src[2]);
  total_area = (vm_GetMagnitude(&normal) / 2);

  for (i = 2; i < nv - 1; i++) {
    vm_GetPerp(&normal, &src[0], &src[i], &src[i + 1]);
    area = (vm_GetMagnitude(&normal) / 2);
    total_area += area;
  }

  // Now figure out how much weight each triangle represents to the overall
  // polygon
  vm_GetPerp(&normal, &src[0], &src[1], &src[2]);
  area = (vm_GetMagnitude(&normal) / 2);

  // Get the center of the first polygon
  vec::vm_MakeZero(&tmp_center);
  for (i = 0; i < 3; i++) {
    tmp_center += src[i];
  }
  tmp_center /= 3;

  *centroid += (tmp_center * (area / total_area));

  // Now do the same for the rest
  for (i = 2; i < nv - 1; i++) {
    vm_GetPerp(&normal, &src[0], &src[i], &src[i + 1]);
    area = (vm_GetMagnitude(&normal) / 2);

    vec::vm_MakeZero(&tmp_center);

    tmp_center += src[0];
    tmp_center += src[i];
    tmp_center += src[i + 1];

    tmp_center /= 3;

    *centroid += (tmp_center * (area / total_area));
  }

  return total_area;
}

// Gets the real center of a polygon, but uses fast magnitude calculation
// Returns the size of the passed in stuff
float vm_GetCentroidFast(simd::float3 *centroid, simd::float3 *src, int nv) {
  // ASSERT (nv>2);
  simd::float3 normal;
  float area, total_area;
  int i;
  simd::float3 tmp_center;

  vec::vm_MakeZero(centroid);

  // First figure out the total area of this polygon
  vm_GetPerp(&normal, &src[0], &src[1], &src[2]);
  total_area = (vm_GetMagnitudeFast(&normal) / 2);

  for (i = 2; i < nv - 1; i++) {
    vm_GetPerp(&normal, &src[0], &src[i], &src[i + 1]);
    area = (vm_GetMagnitudeFast(&normal) / 2);
    total_area += area;
  }

  // Now figure out how much weight each triangle represents to the overall
  // polygon
  vm_GetPerp(&normal, &src[0], &src[1], &src[2]);
  area = (vm_GetMagnitudeFast(&normal) / 2);

  // Get the center of the first polygon
  vec::vm_MakeZero(&tmp_center);
  for (i = 0; i < 3; i++) {
    tmp_center += src[i];
  }
  tmp_center /= 3;

  *centroid += (tmp_center * (area / total_area));

  // Now do the same for the rest
  for (i = 2; i < nv - 1; i++) {
    vm_GetPerp(&normal, &src[0], &src[i], &src[i + 1]);
    area = (vm_GetMagnitudeFast(&normal) / 2);

    vec::vm_MakeZero(&tmp_center);

    tmp_center += src[0];
    tmp_center += src[i];
    tmp_center += src[i + 1];

    tmp_center /= 3;

    *centroid += (tmp_center * (area / total_area));
  }

  return total_area;
}

//	creates a completely random, non-normalized vector with a range of values from -1023 to +1024 values)
void vm_MakeRandomVector(simd::float3 *vec) {
  vec->x = rand() - RAND_MAX / 2;
  vec->y = rand() - RAND_MAX / 2;
  vec->z = rand() - RAND_MAX / 2;
}

// Given a set of points, computes the minimum bounding sphere of those points
float vm_ComputeBoundingSphere(simd::float3 *center, simd::float3 *vecs, int num_verts) {
  // This algorithm is from Graphics Gems I.  There's a better algorithm in Graphics Gems III that
  // we should probably implement sometime.

  simd::float3 *min_x, *max_x, *min_y, *max_y, *min_z, *max_z, *vp;
  float dx, dy, dz;
  float rad, rad2;
  int i;

  // Initialize min, max vars
  min_x = max_x = min_y = max_y = min_z = max_z = &vecs[0];

  // First, find the points with the min & max x,y, & z coordinates
  for (i = 0, vp = vecs; i < num_verts; i++, vp++) {

    if (vp->x < min_x->x)
      min_x = vp;

    if (vp->x > max_x->x)
      max_x = vp;

    if (vp->y < min_y->y)
      min_y = vp;

    if (vp->y > max_y->y)
      max_y = vp;

    if (vp->z < min_z->z)
      min_z = vp;

    if (vp->z > max_z->z)
      max_z = vp;
  }

  // Calculate initial sphere

  dx = vm_VectorDistance(min_x, max_x);
  dy = vm_VectorDistance(min_y, max_y);
  dz = vm_VectorDistance(min_z, max_z);

  if (dx > dy)
    if (dx > dz) {
      *center = (*min_x + *max_x) / 2;
      rad = dx / 2;
    } else {
      *center = (*min_z + *max_z) / 2;
      rad = dz / 2;
    }
  else if (dy > dz) {
    *center = (*min_y + *max_y) / 2;
    rad = dy / 2;
  } else {
    *center = (*min_z + *max_z) / 2;
    rad = dz / 2;
  }

  // Go through all points and look for ones that don't fit
  rad2 = rad * rad;
  for (i = 0, vp = vecs; i < num_verts; i++, vp++) {
    simd::float3 delta;
    float t2;

    delta = *vp - *center;
    t2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

    // If point outside, make the sphere bigger
    if (t2 > rad2) {
      float t;

      t = sqrt(t2);
      rad = (rad + t) / 2;
      rad2 = rad * rad;
      *center += delta * (t - rad) / t;
    }
  }

  // We're done
  return rad;
}
