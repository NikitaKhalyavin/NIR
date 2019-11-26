#include "GJK.h"

#define EPSILON 0.01f

class Simplex
{
private:
	Vector3f points[4];
	int length;
	const int maxLength = 4;
public:
	Simplex() : length(0) {}
	
	int size() const
	{
		return length;
	}	
	
	void add(const Vector3f & data)
	{
		if(length >= maxLength)
			return;
		points[length] = data;
		length++;
	}
	
	void remove(int index)
	{
		if(index >= length)
			return;
		length--;
		if(index != length)
		{
			points[index] = points[length];
		}
	}
	
	Vector3f& operator[] (const int index)
	{
		if(index >= length)
			return points[length - 1];	
		return points[index];
	}
	
};



Vector3f getNearestSimplexPoint(Simplex& simplex)
{
	switch (simplex.size())
	{
	case 1:
	{
		return simplex[0];
	}
	
	case 2:
	{
		Vector3f firstPoint = simplex[0];
		Vector3f secondPoint = simplex[1];

		Vector3f diff = firstPoint - secondPoint;
		Vector3f cross = firstPoint.cross(secondPoint);
		Vector3f direction = diff.cross(cross);
		direction /= sqrt(direction.dot(direction));
		float lineProj = firstPoint.dot(direction);
		Vector3f nearestLinePoint = direction * lineProj;

		Vector3f diff1 = nearestLinePoint - firstPoint;
		Vector3f diff2 = nearestLinePoint - secondPoint;

		if (diff1.dot(diff2) < 0)
		{
			return nearestLinePoint;
		}

		float dist1 = sqrt(firstPoint.dot(firstPoint));
		float dist2 = sqrt(secondPoint.dot(secondPoint));

		if (dist1 < dist2)
		{
			return firstPoint;
		}
		else
		{
			return secondPoint;
		}
	}
	case 3:
	{
		Vector3f firstPoint = simplex[0];
		Vector3f secondPoint = simplex[1];
		Vector3f thirdPoint = simplex[2];

		Vector3f diff1 = firstPoint - thirdPoint;
		Vector3f diff2 = secondPoint - thirdPoint;
		Vector3f normal = diff1.cross(diff2);
		float normalLength = sqrt(normal.dot(normal));
		normal /= normalLength;
		if (normal.dot(firstPoint) < 0)
		{
			normal *= -1;
		}

		float planeProj = thirdPoint.dot(normal);
		Vector3f nearestPlanePoint = normal * planeProj;

		float triangleArea = normalLength / 2;
		Vector3f firstDiff = firstPoint - nearestPlanePoint;
		Vector3f secondDiff = secondPoint - nearestPlanePoint;
		Vector3f thirdDiff = thirdPoint - nearestPlanePoint;

		Vector3f firstCross = secondDiff.cross(thirdDiff);
		Vector3f secondCross = firstDiff.cross(thirdDiff);
		Vector3f thirdCross = secondDiff.cross(firstDiff);

		float smallTriangle1Area = sqrt(firstCross.dot(firstCross)) / 2;
		float smallTriangle2Area = sqrt(secondCross.dot(secondCross)) / 2;
		float smallTriangle3Area = sqrt(thirdCross.dot(thirdCross)) / 2;
		float smallTrianglesSummArea = smallTriangle1Area + smallTriangle2Area + smallTriangle3Area;

		if (abs(triangleArea - smallTrianglesSummArea) < EPSILON)
		{
			return nearestPlanePoint;
		}

		Simplex subSimplex1 = simplex;
		subSimplex1.remove(0);
		Simplex subSimplex2 = simplex;
		subSimplex2.remove(1);
		Simplex subSimplex3 = simplex;
		subSimplex3.remove(2);

		Vector3f firstNearest = getNearestSimplexPoint(subSimplex1);
		Vector3f secondNearest = getNearestSimplexPoint(subSimplex2);
		Vector3f thirdNearest = getNearestSimplexPoint(subSimplex3);

		float firstNearestDistance = sqrt(firstNearest.dot(firstNearest));
		float secondNearestDistance = sqrt(secondNearest.dot(secondNearest));
		float thirdNearestDistance = sqrt(thirdNearest.dot(thirdNearest));

		if ((firstNearestDistance < secondNearestDistance) &&
			(firstNearestDistance < thirdNearestDistance))
		{
			return firstNearest;
		}

		else
		{
			if (secondNearestDistance < thirdNearestDistance)
			{
				return secondNearest;
			}
			else
			{
				return thirdNearest;
			}
		}
	}

	case 4:
	{
		
		Simplex subSimplex1 = simplex;
		subSimplex1.remove(0);
		Simplex subSimplex2 = simplex;
		subSimplex2.remove(1);
		Simplex subSimplex3 = simplex;
		subSimplex3.remove(2);
		Simplex subSimplex4 = simplex;
		subSimplex4.remove(3);

		Vector3f firstNearest = getNearestSimplexPoint(subSimplex1);
		Vector3f secondNearest = getNearestSimplexPoint(subSimplex2);
		Vector3f thirdNearest = getNearestSimplexPoint(subSimplex3);
		Vector3f fourthNearest = getNearestSimplexPoint(subSimplex4);

		float firstNearestDistance = sqrt(firstNearest.dot(firstNearest));
		float secondNearestDistance = sqrt(secondNearest.dot(secondNearest));
		float thirdNearestDistance = sqrt(thirdNearest.dot(thirdNearest));
		float fourthNearestDistance = sqrt(fourthNearest.dot(fourthNearest));

		if ((firstNearestDistance < secondNearestDistance) &&
			(firstNearestDistance < thirdNearestDistance) && 
			(firstNearestDistance < fourthNearestDistance))
		{
			return firstNearest;
		}
		else
		{
			if ((secondNearestDistance < thirdNearestDistance) && 
				(secondNearestDistance < fourthNearestDistance))
			{
				return secondNearest;
			}
			else
			{
				if (thirdNearestDistance < fourthNearestDistance)
				{
					return thirdNearest;
				}
				else
				{
					return fourthNearest;
				}
			}
		}
	}
	}
	
  return Vector3f(0, 0, 0);
}

static bool isInsideTriangle(const Vector3f& point, Simplex& triangle)
{
	Vector3f lineDirection[3];
	lineDirection[0] = triangle[2] - triangle[1];
	lineDirection[1] = triangle[2] - triangle[0];
	lineDirection[2] = triangle[1] - triangle[0];

	Vector3f planeNormal = lineDirection[1].cross(lineDirection[2]);
	planeNormal /= sqrt(planeNormal.dot(planeNormal));

	for (int i = 0; i < 3; i++)
	{
		Vector3f lineNormal = lineDirection[i].cross(planeNormal);
		int pointOnLineIndex = i + 1;
		if (pointOnLineIndex > 2)
		{
			pointOnLineIndex -= 3;
		}
		float zeroProj = triangle[pointOnLineIndex].dot(lineNormal);
		float checkingPointProj = point.dot(lineNormal);
		checkingPointProj -= zeroProj;
		float trianglePointProj = triangle[i].dot(planeNormal);
		trianglePointProj -= zeroProj;
		if (trianglePointProj * checkingPointProj < -EPSILON)
		{
			triangle.remove(i);
			return false;
		}
	}
	return true;
}


template <typename T1, typename T2>
static Vector3f getMaxExtremalPoint(const T1& volume1, const T2& volume2, const Affine3f& transform1,
	const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2, const Vector3f& direction)
{
	Vector3f extremalPoint1 = transform1 * (volume1.supportFunction(rotate1 * direction));
	Vector3f extremalPoint2 = transform2 * (volume2.supportFunction(rotate2 * (-direction)));
	Vector3f maxPoint = extremalPoint1 - extremalPoint2;
	return maxPoint;
}

template <typename T1, typename T2>
static bool evolveSimplex(Simplex& simplex, const T1& volume1, const T2& volume2,
	const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	static Vector3f direction;
	
	switch (simplex.size())
	{
	case 0:
	{
		direction = Vector3f(1, 0, 0);
		Vector3f extremalPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, direction);
		simplex.add(extremalPoint);
		break;
	}
	case 1:
	{
		direction *= -1;
		Vector3f extremalPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, direction);
		simplex.add(extremalPoint);
		break;
	}
	case 2:
	{
		Vector3f point1 = simplex[0];
		Vector3f point2 = simplex[1];
		if (point1.cross(point2).isZero())
		{
			//turn point1 on 90 degres in xOy plane
			direction(0) = -point1(1);
			direction(1) = point1(0);
			direction(2) = point1(2);
		}
		else
		{
			direction = point1.cross(point2);
		}
		Vector3f extremalPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, direction);
		simplex.add(extremalPoint);
		break;
	}
	case 3:
	{
		Vector3f point1 = simplex[0];
		Vector3f point2 = simplex[1];
		Vector3f point3 = simplex[2];

		Vector3f line13 = point1 - point3;
		Vector3f line23 = point2 - point3;
		direction = line13.cross(line23);
		if (point1.dot(direction) > 0)
		{
			direction *= -1;
		}

		Vector3f extremalPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, direction);
		simplex.add(extremalPoint);
		break;
	}
	case 4:
	{
		Vector3f point1 = simplex[0];
		Vector3f point2 = simplex[1];
		Vector3f point3 = simplex[2];
		Vector3f point4 = simplex[3];

		int deletingIndex = 0;
		float maxProj = -INFINITY;
		Vector3f maxProjNormal(0, 0, 0);
		for (int i = 0; i < 4; i++)
		{
			Vector3f planePoints[3];
			int index = 0;
			for (int j = 0; j < 4; j++)
			{
				if (j != i)
				{
					planePoints[index] = simplex[j];
					index++;
				}
			}
			Vector3f pointWithoutPlane = simplex[i];
			
			Vector3f line12 = planePoints[1] - planePoints[0];
			Vector3f line13 = planePoints[2] - planePoints[0];
			Vector3f planeNormal = line12.cross(line13);
			planeNormal /= sqrt(planeNormal.dot(planeNormal));

			Vector3f lineToPointWithoutPlane = pointWithoutPlane - planePoints[0];
			if (lineToPointWithoutPlane.dot(planeNormal) > 0)
			{
				//set plane normal orientation as outside
				planeNormal *= -1;
			}

			float proj = -planePoints[0].dot(planeNormal);
			if (proj > maxProj)
			{
				maxProj = proj;
				deletingIndex = i;
				maxProjNormal = planeNormal;
			}			
		}

		if (maxProj > 0)
		{
			simplex.remove(deletingIndex);

			Vector3f nearest = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
				rotate1, rotate2, maxProjNormal);
			
			float minDirectionDistance = nearest.dot(maxProjNormal);
			if (minDirectionDistance > 0)
			{
				return false;
			}
			
			float planeDistance = simplex[0].dot(maxProjNormal);
			if (abs( (planeDistance - minDirectionDistance) / minDirectionDistance) < EPSILON)
			{
				//if the nearest point in this direction is in the simplex's plane
				Vector3f nearestPlanePoint = -maxProjNormal * planeDistance;
				
				if (isInsideTriangle(nearestPlanePoint, simplex))
				{
					//end calculating
					return true;
				}
				return false;
				//return true;
			}

			return false;
			// origin is outside simplex with side of this plane
		}

		//if we are here, origin is inside the simplex
		//delete the whole simplex and add the simplex with the point zero
		int count = simplex.size();
		for (int i = 0; i < count; i++)
		{
			simplex.remove(0);
		}
		Vector3f zero(0, 0, 0);
		simplex.add(zero);
		return true;
	}
	}	
	return false;
}


template <typename T1, typename T2>
static Vector3f calculate(const T1& volume1, const T2& volume2, const Affine3f& transform1, 
	const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	Simplex simplex;
	bool exit = false;
	for (int i = 0; i < 50; i++)
	{
		exit = evolveSimplex(simplex, volume1, volume2, transform1, transform2, rotate1, rotate2);
		if (exit)
		{
			break;
		}
	}
	
	Vector3f point = getNearestSimplexPoint(simplex);
	return point;
}

Vector3f calculateDistanceByGJK(const OBB_GJK& box1, const OBB_GJK& box2, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(box1, box2, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const OBB_GJK& box, const Sphere_GJK& sphere, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(box, sphere, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const Sphere_GJK& sphere, const OBB_GJK& box, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(sphere, box, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const Sphere_GJK& sphere1, const Sphere_GJK& sphere2, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(sphere1, sphere2, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const OBB_GJK& box, const Cilinder_GJK& cilinder, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(box, cilinder, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const Cilinder_GJK& cilinder, const OBB_GJK& box, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(cilinder, box, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const Cilinder_GJK& cilinder1, const Cilinder_GJK& cilinder2, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(cilinder1, cilinder2, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const Sphere_GJK& sphere, const Cilinder_GJK& cilinder, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(sphere, cilinder, transform1, transform2, rotate1, rotate2);
}

Vector3f calculateDistanceByGJK(const Cilinder_GJK& cilinder, const Sphere_GJK& sphere, const Affine3f& transform1, const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	return calculate(cilinder, sphere, transform1, transform2, rotate1, rotate2);
}




