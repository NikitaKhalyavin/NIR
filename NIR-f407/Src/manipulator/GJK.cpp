#include "GJK.h"
#include "../time_measurement.h"
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
	
	void erase()
	{
		length = 0;
	}
	
};


bool isOriginInsideSimplex(Simplex& simplex)
{
	Vector3f point1 = simplex[0];
	Vector3f point2 = simplex[1];
	Vector3f point3 = simplex[2];
	Vector3f point4 = simplex[3];

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
        
        float pointProj = pointWithoutPlane.dot(planeNormal);
        float planeProj = planePoints[0].dot(planeNormal);
        
		if ( ( (pointProj > 0) && (planeProj > 0) ) || ( (pointProj < 0) && (planeProj < 0) ) )
		{
            //if the plain and the point are on the same side
			return false;
		}			
	}
	return true;
}


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

		Vector3f normalForTriangleOfPointsAndOrigin = firstPoint.cross(secondPoint);
		if(normalForTriangleOfPointsAndOrigin.isZero())
		{
			//simplex and origin are on the one line
			if(firstPoint.dot(secondPoint) < 0)
			{
				//points are in different sides of origin
				return Vector3f(0, 0, 0);
			}
			float dist1 = sqrt(firstPoint.dot(firstPoint));
			float dist2 = sqrt(secondPoint.dot(secondPoint));
			if(dist1 < dist2) 
			{
				return firstPoint;
			}
			return secondPoint;
		}
		
		Vector3f line = firstPoint - secondPoint;
		Vector3f direction = line.cross(normalForTriangleOfPointsAndOrigin);
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

		
		//get normalized plane's normal, directed from the origin
		Vector3f firstLine = secondPoint - thirdPoint;
		Vector3f secondLine = firstPoint - thirdPoint;
		Vector3f thirdLine = firstPoint - secondPoint;
		Vector3f normal = firstLine.cross(secondLine);
		float normalLength = sqrt(normal.dot(normal));
		normal /= normalLength;
		if (normal.dot(firstPoint) < 0)
		{
			normal *= -1;
		}

		//get nearest point of the plane
		float planeProj = thirdPoint.dot(normal);
		Vector3f nearestPlanePoint = normal * planeProj;

		//check if the nearest point is inside triangle
		Vector3f firstLineNormal = firstLine.cross(normal);
		float firstPointProjOnFirstLineNormal = (firstPoint - secondPoint).dot(firstLineNormal);
		if(firstPointProjOnFirstLineNormal > 0)
		{
			firstLineNormal *= -1;
		}
		float nearestProjOnFirstLineNormal = (nearestPlanePoint - secondPoint).dot(firstLineNormal);
		Vector3f secondLineNormal = secondLine.cross(normal);
		float secondPointProjOnSecondLineNormal = (secondPoint - thirdPoint).dot(secondLineNormal);
		if(secondPointProjOnSecondLineNormal > 0)
		{
			secondLineNormal *= -1;
		}
		float nearestProjOnSecondLineNormal = (nearestPlanePoint - thirdPoint).dot(secondLineNormal);
		Vector3f thirdLineNormal = thirdLine.cross(normal);
		float thirdPointProjOnThirdLineNormal = (thirdPoint - firstPoint).dot(thirdLineNormal);
		if(thirdPointProjOnThirdLineNormal > 0)
		{
			thirdLineNormal *= -1;
		}
		float nearestProjOnThirdLineNormal = (nearestPlanePoint - firstPoint).dot(thirdLineNormal);
		
		if( (nearestProjOnFirstLineNormal < 0) && (nearestProjOnSecondLineNormal < 0) && (nearestProjOnThirdLineNormal < 0) )
		{
			return nearestPlanePoint;
		}			
		
		
		//if the nearest plane point is outside triangle, find nearest point of borders
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
		if(isOriginInsideSimplex(simplex))
		{
			return Vector3f(0, 0, 0);
		}
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



template <typename T1, typename T2>
static Vector3f getMaxExtremalPoint(const T1& volume1, const T2& volume2, const Affine3f& transform1,
	const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2, const Vector3f& direction)
{
	Vector3f extremalPoint1 = transform1 * (volume1.supportFunction(rotate1 * (-direction)));
	Vector3f extremalPoint2 = transform2 * (volume2.supportFunction(rotate2 * direction));
	Vector3f maxPoint = -extremalPoint1 + extremalPoint2;
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
		//initialize the simplex by 1 point in constant direction
		direction = Vector3f(1, 0, 0);
		Vector3f nextPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, direction);

        //check if origin is inside simplex
        if(nextPoint.isZero())
		{
			simplex.add(nextPoint);
			return true;
		}
		simplex.add(nextPoint);
		
		break;
	}
	case 1:
	{
        //add second point
		Vector3f nearest = getNearestSimplexPoint(simplex);
        direction = -nearest;
		Vector3f nextPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, direction);

        //check if first point is the nearest
        float nearestLength = sqrt(nearest.dot(nearest));
		float nextPointProj = nextPoint.dot(nearest) / nearestLength;
		
		float differenceLength = nearestLength - nextPointProj;
		if(differenceLength < EPSILON)
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		simplex.add(nextPoint);
		break;
	}
	case 2:
	{
		Vector3f nearest = getNearestSimplexPoint(simplex);
		if(nearest.isZero())
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		
		Vector3f nextPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, -nearest);
		
		float nearestLength = sqrt(nearest.dot(nearest));
		float nextPointProj = nextPoint.dot(nearest) / nearestLength;
		
		float differenceLength = nearestLength - nextPointProj;
		if(differenceLength < EPSILON)
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		
		simplex.add(nextPoint);
		break;
	}
	case 3:
	{
        Vector3f nearest = getNearestSimplexPoint(simplex);
		if(nearest.isZero())
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		
		Vector3f nextPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, -nearest);
		
		float nearestLength = sqrt(nearest.dot(nearest));
		float nextPointProj = nextPoint.dot(nearest) / nearestLength;
		
		float differenceLength = nearestLength - nextPointProj;
		if(differenceLength < EPSILON)
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		
		simplex.add(nextPoint);
		break;
	}
	case 4:
	{
		Vector3f nearest = getNearestSimplexPoint(simplex);
		if(nearest.isZero())
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		
		Vector3f nextPoint = getMaxExtremalPoint(volume1, volume2, transform1, transform2, 
			rotate1, rotate2, -nearest);
		
		float nearestLength = sqrt(nearest.dot(nearest));
		float nextPointProj = nextPoint.dot(nearest) / nearestLength;
		
		float differenceLength = nearestLength - nextPointProj;
		if(differenceLength < EPSILON)
		{
			simplex.erase();
			simplex.add(nearest);
			return true;
		}
		
		int deletingIndex = 0;
		float maxProj = -INFINITY;
		for(int i = 0; i < 4; i++)
		{
			Vector3f simplexPoint = simplex[i];
			float projOnDirection = simplexPoint.dot(nearest);
			if(projOnDirection > maxProj)
			{
				maxProj = projOnDirection;
				deletingIndex = i;
			}
		}
		
		simplex.remove(deletingIndex);
		simplex.add(nextPoint);
		
		return false;
	
	}
	}	
	return false;
}


TimeStatisticCollector GJK_Statistic;

template <typename T1, typename T2>
static Vector3f calculate(const T1& volume1, const T2& volume2, const Affine3f& transform1, 
	const Affine3f& transform2, const Matrix3f& rotate1, const Matrix3f& rotate2)
{
	GJK_Statistic.startMeasurement();
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
	GJK_Statistic.stopMeasurement();
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




