#pragma once


#include "../Src/correct_Eigen_include.h"

#include "box_for_GJK.h"
#include "sphere_for_GJK.h"
#include "cilinder_for_GJK.h"

using Eigen::Matrix3f;

Vector3f calculateDistanceByGJK(const OBB_GJK& box1, const OBB_GJK& box2, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
Vector3f calculateDistanceByGJK(const Sphere_GJK& sphere1, const Sphere_GJK& sphere2, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
Vector3f calculateDistanceByGJK(const Cilinder_GJK& cilinder1, const Cilinder_GJK& cilinder2, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
								
Vector3f calculateDistanceByGJK(const OBB_GJK& box, const Sphere_GJK& sphere, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
Vector3f calculateDistanceByGJK(const Sphere_GJK& sphere, const OBB_GJK& box, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
							
Vector3f calculateDistanceByGJK(const OBB_GJK& box, const Cilinder_GJK& cilinder, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
Vector3f calculateDistanceByGJK(const Cilinder_GJK& cilinder, const OBB_GJK& box, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
								
Vector3f calculateDistanceByGJK(const Sphere_GJK& sphere, const Cilinder_GJK& cilinder, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
Vector3f calculateDistanceByGJK(const Cilinder_GJK& cilinder, const Sphere_GJK& sphere, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2);
