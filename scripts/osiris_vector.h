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

#ifndef OSIRIS_VECTOR_H
#define OSIRIS_VECTOR_H

#include "fix.h"
#include "vecmat_external.h"

const simd::float3 Zero_vector = {0.0f, 0.0f, 0.0f};

// Disable the "possible loss of data" warning
#pragma warning(disable : 4244)

// Used for debugging.  It is used in printf's so we do not have to write out the structure 3 times
// to print all the coordinates.
#define XYZ(v) (v)->x, (v)->y, (v)->z

extern const vec::matrix Identity_matrix;

// Given a matrix, makes it an identity matrix
extern void vm_MakeIdentity(vec::matrix *);

// Set a vector to {0,0,0}
extern void vm_MakeZero(simd::float3 *v);

// Set an angvec to {0,0,0}
extern void vm_MakeZero(vec::angvec *a);

// Rotates a vector thru a matrix
extern void vm_MatrixMulVector(simd::float3 *, simd::float3 *, vec::matrix *);

// Multiply a vector times the transpose of a matrix
void vm_VectorMulTMatrix(simd::float3 *result, simd::float3 *v, vec::matrix *m);

// Multiplies 2 3x3 matrixes, returning the result in first argument
extern void vm_MatrixMul(vec::matrix *, vec::matrix *, vec::matrix *);

// Multiply a matrix times the transpose of a matrix
void vm_MatrixMulTMatrix(vec::matrix *dest, vec::matrix *src0, vec::matrix *src1);

// Given a vector, returns the magnitude.  Uses sqrt so it's slow
extern float vm_GetMagnitude(simd::float3 *);

// Given a simd::float3, returns an approximation of the magnitude
extern float vm_GetMagnitudeFast(simd::float3 *);

// Returns the dot product of the two given vectors
extern float vm_DotProduct(simd::float3 *, simd::float3 *);

// Returns a perpendicular simd::float3 to the two given vectors
extern void vm_CrossProduct(simd::float3 *, simd::float3 *, simd::float3 *);

// Returns the difference between two vectors
extern void vm_SubVectors(simd::float3 *, const simd::float3 *, const simd::float3 *);

// Returns adds two vectors, returns result in first arg
extern void vm_AddVectors(simd::float3 *, simd::float3 *, simd::float3 *);

// Inits simd::float3 to 0,0,0
extern void vm_CenterVector(simd::float3 *);

// Given a simd::float3, divides second arg by simd::float3 components
extern void vm_AverageVector(simd::float3 *, int);

// Normalizes a simd::float3
// Returns the magnitude before normalization
extern float vm_VectorNormalize(simd::float3 *);

// Scales second arg simd::float3 by 3rd arg, placing result in first arg
extern void vm_ScaleVector(simd::float3 *, simd::float3 *, float);

// Scales all components of simd::float3 v by value s adds the result to p and stores result in simd::float3 d
extern void vm_ScaleAddVector(simd::float3 *d, simd::float3 *p, simd::float3 *v, float s);

// Divides second simd::float3 components by 3rd arg, placing result in first arg.  Useful for parametric lines
extern void vm_DivVector(simd::float3 *, simd::float3 *, float);

// Same as VectorNormalize, but uses approximation
extern float vm_VectorNormalizeFast(simd::float3 *);

// Clears a matrix to zero
extern void vm_ClearMatrix(vec::matrix *);

// Transposes a matrix in place
extern void vm_TransposeMatrix(vec::matrix *);

// Given 3 angles (p,h,b), makes a rotation matrix out of them
extern void vm_AnglesToMatrix(vec::matrix *, angle p, angle h, angle b);

// Ensure that a matrix is orthogonal
void vm_Orthogonalize(vec::matrix *m);

// Compute a matrix from one or two vectors.  At least one and at most two vectors must/can be specified.
// Parameters:	m - filled in with the orienation matrix
//					fvec,uvec,rvec - pointers to vectors that determine the matrix.
//						One or two of these must be specified, with the other(s) set to NULL.
void vm_VectorToMatrix(vec::matrix *m, simd::float3 *fvec, simd::float3 *uvec = NULL, simd::float3 *rvec = NULL);

// Computes a matrix from a simd::float3 and and angle of rotation around that simd::float3
// Parameters:	m - filled in with the computed matrix
//					v - the forward simd::float3 of the new matrix
//					a - the angle of rotation around the forward simd::float3
void vm_VectorAngleToMatrix(vec::matrix *m, simd::float3 *v, vec::angle a);

// Given an angle, places sin in 2nd arg, cos in 3rd.  Either can be null
extern void vm_SinCos(angle, float *, float *);

// Given x1,y1,x2,y2, returns the slope
extern float vm_GetSlope(float, float, float, float);

// Calculates the perpendicular simd::float3 given three points
// Parms:	n - the computed perp simd::float3 (filled in)
//			v0,v1,v2 - three clockwise vertices
void vm_GetPerp(simd::float3 *n, simd::float3 *a, simd::float3 *b, simd::float3 *c);

// Calculates the (normalized) surface normal give three points
// Parms:	n - the computed surface normal (filled in)
//			v0,v1,v2 - three clockwise vertices
// Returns the magnitude of the normal before it was normalized.
// The bigger this value, the better the normal.
float vm_GetNormal(simd::float3 *n, simd::float3 *v0, simd::float3 *v1, simd::float3 *v2);

#define vm_GetSurfaceNormal vm_GetNormal

// Gets the distances (magnitude) between two vectors. Slow.
extern float vm_VectorDistance(const simd::float3 *a, const simd::float3 *b);

// Gets the approx distances (magnitude) between two vectors. Faster.
extern float vm_VectorDistanceQuick(simd::float3 *a, simd::float3 *b);

// Computes a normalized direction simd::float3 between two points
// Parameters:	dest - filled in with the normalized direction simd::float3
//					start,end - the start and end points used to calculate the simd::float3
// Returns:		the distance between the two input points
float vm_GetNormalizedDir(simd::float3 *dest, simd::float3 *end, simd::float3 *start);

// Returns a normalized direction simd::float3 between two points
// Uses sloppier magnitude, less precise
float vm_GetNormalizedDirFast(simd::float3 *dest, simd::float3 *end, simd::float3 *start);

// extract angles from a matrix
vec::angvec *vm_ExtractAnglesFromMatrix(vec::angvec *a, vec::matrix *m);

//	returns the angle between two vectors and a forward simd::float3
angle vm_DeltaAngVec(simd::float3 *v0, simd::float3 *v1, simd::float3 *fvec);

//	returns the angle between two normalized vectors and a forward simd::float3
angle vm_DeltaAngVecNorm(simd::float3 *v0, simd::float3 *v1, simd::float3 *fvec);

// Computes the distance from a point to a plane.
// Parms:	checkp - the point to check
// Parms:	norm - the (normalized) surface normal of the plane
//				planep - a point on the plane
// Returns:	The signed distance from the plane; negative dist is on the back of the plane
float vm_DistToPlane(simd::float3 *checkp, simd::float3 *norm, simd::float3 *planep);

// returns the value of a determinant
float calc_det_value(vec::matrix *det);

void vm_MakeInverseMatrix(vec::matrix *dest);
void vm_SinCosToMatrix(vec::matrix *m, float sinp, float cosp, float sinb, float cosb, float sinh, float cosh);

// Gets the real center of a polygon
float vm_GetCentroid(simd::float3 *centroid, simd::float3 *src, int nv);

//	retrieves a random simd::float3 in values -RAND_MAX/2 to RAND_MAX/2
void vm_MakeRandomVector(simd::float3 *vec);

// Given a set of points, computes the minimum bounding sphere of those points
float vm_ComputeBoundingSphere(simd::float3 *center, simd::float3 *vecs, int num_verts);

// Gets the real center of a polygon, but uses fast magnitude calculation
// Returns the size of the passed in stuff
float vm_GetCentroidFast(simd::float3 *centroid, simd::float3 *src, int nv);

// Here are the C++ operator overloads -- they do as expected
//extern vec::matrix operator*(vec::matrix src0, vec::matrix src1);
//extern vec::matrix operator*=(vec::matrix &src0, vec::matrix src1);

#endif
