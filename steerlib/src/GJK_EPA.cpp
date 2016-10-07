#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& shapeA, std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex) {
  Util::Vector direction = getInitDirection(shapeA, shapeB);
  if(direction.z == 0)
    direction.z = 1;
  simplex = initSimplex(shapeA, shapeB, direction);
  while(true) {
    if(getDotProduct(simplex.back(),direction) > 0) {
      if(checkForOrigin(simplex, direction)) {
        return true;
      }
      simplex.push_back(getMinkowskiDifferencePt(getFarthestPoint(shapeA, direction), getFarthestPoint(shapeB, direction*-1)));
    }
    else {
      return false;
    }
  }
}

void SteerLib::GJK_EPA::EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, std::vector<Util::Vector> simplex) {
  if(simplex.size() > 2) {
    while(true) {
      int index = 0;
      int insertIndex = 0;
      float finalDistance = 100000;
      Util::Vector finalPenetrationVector;
      while(index < simplex.size()) {
        int nextIndex = index+1;
        if(nextIndex >= simplex.size()) {
          break;
        }
        Util::Vector pointA = simplex[index];
        Util::Vector pointB = simplex[nextIndex];
        Util::Vector testEdge = pointB - pointA;
        Util::Vector originVectorA = pointA;
        Util::Vector testPenetrationVector = originVectorA*(getDotProduct(testEdge, testEdge)) - testEdge*(getDotProduct(testEdge, originVectorA));
        testPenetrationVector = testPenetrationVector/testPenetrationVector.length();
        float testDistance = getDotProduct(pointA, testPenetrationVector);
        if(testDistance < finalDistance) {
          finalDistance = testDistance;
          insertIndex = index;
          finalPenetrationVector = testPenetrationVector;
        }
        index++;
      }
      Util::Vector newEdgePoint = getMinkowskiDifferencePt(getFarthestPoint(shapeA, finalPenetrationVector), getFarthestPoint(shapeB, finalPenetrationVector*-1));
      float newEdgePointDistance = getDotProduct(newEdgePoint, finalPenetrationVector);
      if(newEdgePointDistance - finalDistance < .000001) {
        return_penetration_depth = newEdgePointDistance;
        return_penetration_vector = finalPenetrationVector;
        return;
      }
      else {
        simplex.insert(simplex.begin()+insertIndex+1, newEdgePoint);
      }
    }
  }
}

std::vector<Util::Vector> SteerLib::GJK_EPA::initSimplex(std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, Util::Vector& direction) {
  std::vector<Util::Vector> outputSimplex;
  outputSimplex.push_back(getMinkowskiDifferencePt(getFarthestPoint(shapeA, direction), getFarthestPoint(shapeB, direction*-1)));
  direction=direction*-1;
  outputSimplex.push_back(getMinkowskiDifferencePt(getFarthestPoint(shapeA, direction), getFarthestPoint(shapeB, direction*-1)));
  return outputSimplex;
}

Util::Vector SteerLib::GJK_EPA::getFarthestPoint(std::vector<Util::Vector> shape, Util::Vector direction) {
  Util::Vector farthestPoint = shape[0];
  int index = 1;
  while(index < shape.size()) {
    if(getDotProduct(shape[index],direction) > getDotProduct(farthestPoint,direction)) {
      if(shape[index].x == farthestPoint.x) {
        if(shape[index].z > farthestPoint.z || shape[index].z < farthestPoint.z)
          farthestPoint = shape[index];
      }
      else {
        farthestPoint = shape[index];
      }
    }
    index++;
  }
  return farthestPoint;
}

Util::Vector SteerLib::GJK_EPA::getMinkowskiDifferencePt(Util::Vector pointA, Util::Vector pointB) {
    return pointA - pointB;
}

Util::Vector SteerLib::GJK_EPA::getInitDirection(std::vector<Util::Vector>& shapeA, std::vector<Util::Vector>& shapeB) {
  float shapeALowestX = shapeA[0].x;
  float shapeAGreatestX = shapeA[0].x;
  float shapeALowestY = shapeA[0].z;
  float shapeAGreatestY = shapeA[0].z;
  float shapeALengthX = 0;
  float shapeALengthY = 0;

  float shapeBLowestX = shapeB[0].x;
  float shapeBGreatestX = shapeB[0].x;
  float shapeBLowestY = shapeB[0].z;
  float shapeBGreatestY = shapeB[0].z;
  float shapeBLengthX = 0;
  float shapeBLengthY = 0;

  int index = 1;
  while(index < shapeA.size()) {
    if(shapeA[index].x < shapeALowestX) {
      shapeALowestX = shapeA[index].x;
    }
    else if (shapeA[index].x > shapeAGreatestX) {
      shapeAGreatestX = shapeA[index].x;
    }
    if(shapeA[index].z < shapeALowestY) {
      shapeALowestY = shapeA[index].z;
    }
    else if(shapeA[index].z > shapeAGreatestY) {
      shapeAGreatestY = shapeA[index].z;
    }
    index++;
  }
  shapeALengthX = shapeAGreatestX - shapeALowestX;
  shapeALengthY = shapeAGreatestY - shapeALowestY;

  index = 1;
  while(index < shapeB.size()) {
    if(shapeB[index].x < shapeBLowestX) {
      shapeBLowestX = shapeB[index].x;
    }
    else if (shapeB[index].x > shapeBGreatestX) {
      shapeBGreatestX = shapeB[index].x;
    }
    if(shapeB[index].z < shapeBLowestY) {
      shapeBLowestY = shapeB[index].z;
    }
    else if(shapeB[index].z > shapeBGreatestY) {
      shapeBGreatestY = shapeB[index].z;
    }
    index++;
  }
  shapeBLengthX = shapeBGreatestX - shapeBLowestX;
  shapeBLengthY = shapeBGreatestY - shapeBLowestY;

  Util::Vector centerA((shapeALengthX/2) + shapeALowestX, 0.0f, (shapeALengthY/2) + shapeALowestY);
  Util::Vector centerB((shapeBLengthX/2) + shapeBLowestX, 0.0f, (shapeBLengthY/2) + shapeBLowestY);

  if(centerA.x > centerB.x) {
    std::vector<Util::Vector> temp = shapeA;
    shapeA = shapeB;
    shapeB = temp;
    return centerB-centerA;
  }
  return centerA-centerB;
}

bool SteerLib::GJK_EPA::checkForOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction) {
  if(simplex.size() < 3) {
    Util::Vector pointA = simplex[1];
    Util::Vector pointA0 = pointA * -1;
    Util::Vector pointB = simplex[0];
    Util::Vector lineAB = pointB - pointA;
    Util::Vector ABperpendicular =  pointA0*(getDotProduct(lineAB,lineAB)) - lineAB*(getDotProduct(pointA0,lineAB));
    direction = ABperpendicular;
    return false;
  }
  else {
    Util::Vector pointA = simplex.back();
    Util::Vector pointA0 = pointA * -1;
    Util::Vector pointB = simplex[simplex.size()-2];
    Util::Vector pointC = simplex[simplex.size()-3];
    Util::Vector lineAB = pointB - pointA;
    Util::Vector lineAC = pointC - pointA;
    Util::Vector ABperpendicular = lineAB*(getDotProduct(lineAB,lineAC)) - lineAC*(getDotProduct(lineAB,lineAB));
    Util::Vector ACperpendicular = lineAC*(getDotProduct(lineAC,lineAB)) - lineAB*(getDotProduct(lineAC,lineAC));
    if(getDotProduct(ABperpendicular,pointA0) > 0) {
      simplex.erase(simplex.begin());
      direction = ABperpendicular;
      return false;
    }
    else {
      if(getDotProduct(ACperpendicular,pointA0) > 0) {
        simplex.erase(simplex.begin()+1);
        direction = ACperpendicular;
        return false;
      }
      else {
        return true;
      }
    }
  }
}

float SteerLib::GJK_EPA::getDotProduct(Util::Vector a, Util::Vector b) {
  Util::Vector product(a.x*b.x, a.y*b.y, a.z*b.z);
  return product.x+product.y+product.z;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
  std::vector<Util::Vector> simplex;
  std::vector<Util::Vector> shapeA = _shapeA;
  std::vector<Util::Vector> shapeB = _shapeB;
  if(GJK(shapeA, shapeB, simplex)) {
    EPA(return_penetration_depth, return_penetration_vector, shapeA, shapeB, simplex);
    if(return_penetration_vector.x > 0) {
      return_penetration_vector.x = 1;
    }
    else if(return_penetration_vector.x < 0) {
      return_penetration_vector.x = -1;
    }
    if(return_penetration_vector.z > 0) {
      return_penetration_vector.z = 1;
    }
    else if(return_penetration_vector.z < 0) {
      return_penetration_vector.z = -1;
    }
    return true;
  }
	return false; // There is no collision
}
