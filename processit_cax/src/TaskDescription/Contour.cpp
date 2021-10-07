/**
 *  \file      Contour.cpp
 *  \brief     Contour and children classes
 *  \details   Geometric contour descriptions (linear, circle and spline) as used in TaskDefinition.h
 *  \copyright Fraunhofer IPA
 */

#include "TaskDescription/Contour.h"
#include "TaskDescription/Templates.h"

using namespace std;

// LinearContour
LinearContour* LinearContour::clone() const
{
  return new LinearContour(*this);
};

string LinearContour::GetContourType()
{
  return "LinearContour";
};

void LinearContour::SetPointOne(double x, double y, double z)
{
  PointOne[0] = x;
  PointOne[1] = y;
  PointOne[2] = z;
};

void LinearContour::SetPointTwo(double x, double y, double z)
{
  PointTwo[0] = x;
  PointTwo[1] = y;
  PointTwo[2] = z;
};

double LinearContour::GetContourLength()
{
  if (ContourLength == -1)
  {
    Vector3d v = PointTwo - PointOne;
    ContourLength = v.norm();
  }
  return ContourLength;
};

Vector3d LinearContour::GetContourPosition(double DistanceFromPointOne)
{
  Vector3d ContourPosition;
  // compute normalized direction vector (division of x/x returns weird values
  // when calling method from Delfoi)
  Vector3d DirectionVectorNormalized = PointTwo - PointOne;
  double DirectionVectorNorm = DirectionVectorNormalized.norm();
  DirectionVectorNormalized[0] = DirectionVectorNormalized[0] / DirectionVectorNorm;
  DirectionVectorNormalized[1] = DirectionVectorNormalized[1] / DirectionVectorNorm;
  DirectionVectorNormalized[2] = DirectionVectorNormalized[2] / DirectionVectorNorm;
  // ContourPosition = PointOne + DistanceFromPointOne * (PointTwo
  // -PointOne).normalized();
  ContourPosition = PointOne + DistanceFromPointOne * DirectionVectorNormalized;
  return ContourPosition;
};

Vector3d LinearContour::GetContourDirection(double DistanceFromPointOne)
{
  Vector3d ContourDirection;
  ContourDirection = PointTwo - PointOne;
  ContourDirection.normalize();
  return ContourDirection;
};

void LinearContour::InvertContour()
{
  Vector3d PointOneTemp = PointOne;
  Vector3d PointTwoTemp = PointTwo;
  PointOne = PointTwoTemp;
  PointTwo = PointOneTemp;
};

void LinearContour::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem5 = ParentXmlNode;
  int PointCount = 1;
  for (TiXmlElement* pElem6 = pElem5->FirstChildElement(); pElem6 != NULL; pElem6 = pElem6->NextSiblingElement())
  {
    string pVal6 = pElem6->Value();
    if (pVal6 == "Point")
    {
      double x;
      double y;
      double z;
      pElem6->QueryDoubleAttribute("x", &x);
      pElem6->QueryDoubleAttribute("y", &y);
      pElem6->QueryDoubleAttribute("z", &z);

      if (PointCount == 1)
      {
        SetPointOne(x, y, z);
      }
      if (PointCount == 2)
      {
        SetPointTwo(x, y, z);
      }
      PointCount++;
    }
  }
};

void LinearContour::WriteXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* XmlNode = new TiXmlElement("LinearContour");
  ParentXmlNode->LinkEndChild(XmlNode);

  TiXmlElement* XmlNodePointOne = new TiXmlElement("Point");
  XmlNode->LinkEndChild(XmlNodePointOne);
  XmlNodePointOne->SetDoubleAttribute("x", PointOne[0]);
  XmlNodePointOne->SetDoubleAttribute("y", PointOne[1]);
  XmlNodePointOne->SetDoubleAttribute("z", PointOne[2]);

  TiXmlElement* XmlNodePointTwo = new TiXmlElement("Point");
  XmlNode->LinkEndChild(XmlNodePointTwo);
  XmlNodePointTwo->SetDoubleAttribute("x", PointTwo[0]);
  XmlNodePointTwo->SetDoubleAttribute("y", PointTwo[1]);
  XmlNodePointTwo->SetDoubleAttribute("z", PointTwo[2]);
};

// CircleContour
CircleContour* CircleContour::clone() const
{
  return new CircleContour(*this);
};

string CircleContour::GetContourType()
{
  return "CircleContour";
};

void CircleContour::SetPositionCentre(double x, double y, double z)
{
  PositionCentre[0] = x;
  PositionCentre[1] = y;
  PositionCentre[2] = z;
};

void CircleContour::SetDirectionNormal(double x, double y, double z)
{
  DirectionNormal[0] = x;
  DirectionNormal[1] = y;
  DirectionNormal[2] = z;
  DirectionNormal.normalize();
};

void CircleContour::SetDirectionPlane(double x, double y, double z)
{
  DirectionPlane[0] = x;
  DirectionPlane[1] = y;
  DirectionPlane[2] = z;
  DirectionPlane.normalize();
};

void CircleContour::SetRadius(double r)
{
  Radius = r;
};
void CircleContour::SetAlphaStart(double as)
{
  AlphaStart = as;
};
void CircleContour::SetAlphaEnd(double ae)
{
  AlphaEnd = ae;
};

double CircleContour::GetContourLength()
{
  if (ContourLength == -1)
  {
    ContourLength = Radius * (AlphaEnd - AlphaStart);
  }
  return ContourLength;
};

Vector3d CircleContour::GetContourPosition(double DistanceFromPointOne)
{
  double t = DistanceFromPointOne / GetContourLength() * (AlphaEnd - AlphaStart) + AlphaStart;
  Vector3d yDir;
  Vector3d res;

  // compute the y direction:
  yDir = DirectionNormal.cross(DirectionPlane);

  // compute the curve point with Rodrigue's rotation formula (axis-angle convention)
  // with the special case of the rotation vector (DirectionNormal)
  // being orthogonal to the rotated vector (DirectionPlane)
  res[0] = PositionCentre[0] + Radius * (DirectionPlane[0] * cos(t) + yDir[0] * sin(t));
  res[1] = PositionCentre[1] + Radius * (DirectionPlane[1] * cos(t) + yDir[1] * sin(t));
  res[2] = PositionCentre[2] + Radius * (DirectionPlane[2] * cos(t) + yDir[2] * sin(t));

  return res;
};

Vector3d CircleContour::GetContourDirection(double DistanceFromPointOne)
{
  double t = DistanceFromPointOne / GetContourLength() * (AlphaEnd - AlphaStart) + AlphaStart;
  Vector3d yDir;
  Vector3d res;
  // compute the y direction
  yDir = DirectionNormal.cross(DirectionPlane);

  // compute the curve derivation (of equation in GetContourPosition)
  // at specific point (derivation of circle curve, see theoretical background document)
  res[0] = Radius * (yDir[0] * cos(t) - DirectionPlane[0] * sin(t));
  res[1] = Radius * (yDir[1] * cos(t) - DirectionPlane[1] * sin(t));
  res[2] = Radius * (yDir[2] * cos(t) - DirectionPlane[2] * sin(t));

  res.normalize();
  return res;
};

void CircleContour::InvertContour()
{
  double AlphaStartTemp = AlphaStart;
  double AlphaEndTemp = AlphaEnd;
  AlphaStart = AlphaEndTemp;
  AlphaEnd = AlphaStartTemp;
};

void CircleContour::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem5 = ParentXmlNode;
  int DirectionCountCircleContour = 1;
  for (TiXmlElement* pElem6 = pElem5->FirstChildElement(); pElem6 != NULL; pElem6 = pElem6->NextSiblingElement())
  {
    string pVal6 = pElem6->Value();
    if (pVal6 == "Point")
    {
      double x;
      double y;
      double z;
      pElem6->QueryDoubleAttribute("x", &x);
      pElem6->QueryDoubleAttribute("y", &y);
      pElem6->QueryDoubleAttribute("z", &z);
      SetPositionCentre(x, y, z);
    }
    if (pVal6 == "Direction")
    {
      double x;
      double y;
      double z;
      pElem6->QueryDoubleAttribute("x", &x);
      pElem6->QueryDoubleAttribute("y", &y);
      pElem6->QueryDoubleAttribute("z", &z);

      if (DirectionCountCircleContour == 1)
      {
        SetDirectionNormal(x, y, z);
      }
      if (DirectionCountCircleContour == 2)
      {
        SetDirectionPlane(x, y, z);
      }
      DirectionCountCircleContour++;
    }
    if (pVal6 == "Radius")
    {
      double i_radius;
      pElem6->QueryDoubleAttribute("Radius", &i_radius);
      SetRadius(i_radius);
    }
    if (pVal6 == "Alpha")
    {
      double i_alphaStart;
      double i_alphaEnd;
      pElem6->QueryDoubleAttribute("AlphaStart", &i_alphaStart);
      pElem6->QueryDoubleAttribute("AlphaEnd", &i_alphaEnd);
      SetAlphaStart(i_alphaStart);
      SetAlphaEnd(i_alphaEnd);
    }
  }
};

void CircleContour::WriteXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* XmlNode = new TiXmlElement("CircleContour");
  ParentXmlNode->LinkEndChild(XmlNode);

  TiXmlElement* XmlNodePositionCentre = new TiXmlElement("Point");
  XmlNode->LinkEndChild(XmlNodePositionCentre);
  XmlNodePositionCentre->SetDoubleAttribute("x", PositionCentre[0]);
  XmlNodePositionCentre->SetDoubleAttribute("y", PositionCentre[1]);
  XmlNodePositionCentre->SetDoubleAttribute("z", PositionCentre[2]);

  TiXmlElement* XmlNodeDirectionNormal = new TiXmlElement("Direction");
  XmlNode->LinkEndChild(XmlNodeDirectionNormal);
  XmlNodeDirectionNormal->SetDoubleAttribute("x", DirectionNormal[0]);
  XmlNodeDirectionNormal->SetDoubleAttribute("y", DirectionNormal[1]);
  XmlNodeDirectionNormal->SetDoubleAttribute("z", DirectionNormal[2]);

  TiXmlElement* XmlNodeDirectionPlane = new TiXmlElement("Direction");
  XmlNode->LinkEndChild(XmlNodeDirectionPlane);
  XmlNodeDirectionPlane->SetDoubleAttribute("x", DirectionPlane[0]);
  XmlNodeDirectionPlane->SetDoubleAttribute("y", DirectionPlane[1]);
  XmlNodeDirectionPlane->SetDoubleAttribute("z", DirectionPlane[2]);

  TiXmlElement* XmlNodeRadius = new TiXmlElement("Radius");
  XmlNode->LinkEndChild(XmlNodeRadius);
  XmlNodeRadius->SetDoubleAttribute("Radius", Radius);

  TiXmlElement* XmlNodeAlpha = new TiXmlElement("Alpha");
  XmlNode->LinkEndChild(XmlNodeAlpha);
  XmlNodeAlpha->SetDoubleAttribute("AlphaStart", AlphaStart);
  XmlNodeAlpha->SetDoubleAttribute("AlphaEnd", AlphaEnd);
};

// SplineContour
SplineContour* SplineContour::clone() const
{
  return new SplineContour(*this);
};

string SplineContour::GetContourType()
{
  return "SplineContour";
};

void SplineContour::SetNodeVector(double i_NodeVector)
{
  List_NodeVector.push_back(i_NodeVector);
};

void SplineContour::SetPoleVector(double i_PoleVectorX, double i_PoleVectorY, double i_PoleVectorZ)
{
  Vector3d i_PoleVector(i_PoleVectorX, i_PoleVectorY, i_PoleVectorZ);
  List_PoleVector.push_back(i_PoleVector);
};

list<double> SplineContour::GetListNodeVector()
{
  return List_NodeVector;
};
list<Vector3d> SplineContour::GetListPoleVector()
{
  return List_PoleVector;
};

double SplineContour::GetContourLength()
{
  if (ContourLength == -1)
  {
    ContourLength = GetContourLengthByParameter(1);
  }
  return ContourLength;
};

Vector3d SplineContour::GetContourPosition(double DistanceFromPointOne)
{
  if (ContourLength == -1)
  {
    ContourLength = GetContourLengthByParameter(1);
  }
  double t = DistanceFromPointOne / ContourLength;
  Vector3d ContourPosition = GetPositionByParameter(t, List_NodeVector, List_PoleVector);
  return ContourPosition;
};

Vector3d SplineContour::GetContourDirection(double DistanceFromPointOne)
{
  if (ContourLength == -1)
  {
    ContourLength = GetContourLengthByParameter(1);
  }
  double t = DistanceFromPointOne / ContourLength;
  Vector3d ContourDirection = GetDirectionByParameter(t, List_NodeVector, List_PoleVector);
  ContourDirection.normalize();
  return ContourDirection;
};

void SplineContour::InvertContour(){};

Vector3d SplineContour::GetPositionByParameter(double t, list<double> i_List_NodeVector,
                                               list<Vector3d> i_List_PoleVector)
{
  Vector3d Position;

  // calculate degree
  int p = i_List_NodeVector.size() - i_List_PoleVector.size() - 1;
  // Estimate the index r for the interval t_r <= t < t_(r+1)
  int r = i_List_PoleVector.size() - 1;
  for (int knotNr = 0; knotNr < i_List_NodeVector.size() - 1; knotNr++)
  {
    list<double>::iterator i = i_List_NodeVector.begin();
    std::advance(i, knotNr);
    double Node;
    Node = *i;

    list<double>::iterator j = i_List_NodeVector.begin();
    std::advance(j, knotNr + 1);
    double Node1;
    Node1 = *j;

    if (Node <= t && Node1 > t)
    {
      r = knotNr;
      break;
    }
  }

  // Compute the curve point
  return deBoor(p, p, r, t, i_List_NodeVector, i_List_PoleVector);
};

Vector3d SplineContour::deBoor(int j, int degree, int i, double t, list<double> i_List_NodeVector,
                               list<Vector3d> i_List_PoleVector)
{
  if (j == 0)
  {
    list<Vector3d>::iterator iter = i_List_PoleVector.begin();
    std::advance(iter, i);
    Vector3d Pole;
    Pole = *iter;
    return Pole;
  }

  list<double>::iterator iterA = i_List_NodeVector.begin();
  std::advance(iterA, i);
  double NodeA;
  NodeA = *iterA;

  list<double>::iterator iterB = i_List_NodeVector.begin();
  std::advance(iterB, i + degree + 1 - j);
  double NodeB;
  NodeB = *iterB;

  double alpha = (t - NodeA) / (NodeB - NodeA);

  Vector3d deBoor1 = deBoor(j - 1, degree, i - 1, t, i_List_NodeVector, i_List_PoleVector);
  Vector3d deBoor2 = deBoor(j - 1, degree, i, t, i_List_NodeVector, i_List_PoleVector);

  double x = (deBoor1.x() * (1 - alpha) + deBoor2.x() * alpha);
  double y = (deBoor1.y() * (1 - alpha) + deBoor2.y() * alpha);
  double z = (deBoor1.z() * (1 - alpha) + deBoor2.z() * alpha);

  return Vector3d(x, y, z);
};

Vector3d SplineContour::GetDirectionByParameter(double t, list<double> i_List_NodeVector,
                                                list<Vector3d> i_List_PoleVector)
{
  // calculate degree
  int degree = i_List_NodeVector.size() - i_List_PoleVector.size() - 1;

  // compute new control points of the derivation curve
  list<Vector3d> i_List_PoleVector_Derivation;

  for (int i = 1; i < i_List_PoleVector.size(); i++)
  {
    double NodeA = GetListElement(i_List_NodeVector, i + degree);
    double NodeB = GetListElement(i_List_NodeVector, i);
    Vector3d PoleA = GetListElement(i_List_PoleVector, i);
    Vector3d PoleB = GetListElement(i_List_PoleVector, i - 1);

    double factor = (double)degree / (NodeA - NodeB);

    double x = factor * (PoleA.x() - PoleB.x());
    double y = factor * (PoleA.y() - PoleB.y());
    double z = factor * (PoleA.z() - PoleB.z());

    Vector3d derivationPoint(x, y, z);
    i_List_PoleVector_Derivation.push_back(derivationPoint);
  }

  // reduce the knots vector
  int sizeTemp = i_List_NodeVector.size();
  list<double>::iterator it = i_List_NodeVector.end();
  it--;
  i_List_NodeVector.erase(it);
  sizeTemp = i_List_NodeVector.size();
  i_List_NodeVector.erase(i_List_NodeVector.begin());
  sizeTemp = i_List_NodeVector.size();

  return GetPositionByParameter(t, i_List_NodeVector, i_List_PoleVector_Derivation);
};

double SplineContour::GetContourLengthByParameter(double t)
{
  double sum = 0, last_sum = 0, delta;
  // Compute the length of the spline curve, increasing the
  // number of sample points (pointCount) until the result
  // converges
  int pointCount = 100;
  do
  {
    sum = 0;
    // add up derivatives at each point p
    // val = Derivative(p).x()^2+Derivative(p).y()^2+Derivative(p).z()^2;
    for (int pointNr = 0; pointNr <= pointCount; pointNr++)
    {
      double p = t * (double)pointNr / pointCount;
      Vector3d DirTemp = GetDirectionByParameter(p, List_NodeVector, List_PoleVector);
      double val = DirTemp.norm();
      if (pointNr == 0 || pointNr == pointCount)
      {
        val *= 0.5;
      }
      sum += val;
    }
    sum *= t / pointCount;
    delta = sqrt((sum - last_sum) * (sum - last_sum));
    pointCount += 20;
    last_sum = sum;
  } while (delta > 1e-4);

  return sum;
};

double SplineContour::GetParameterOfLength(double i_Length)
{
  // ToDo: Implement this function with efficient algorithm
  double t;
  // double ResTreshold = 0.0001;
  // double Res1, Res2 = 0;
  // do
  //{
  //	GetContourLengthByParameter(t);
  //	if (Res2 < ResTreshold)
  //	{
  //		break;
  //	}

  //	if (Res1 < Res2)
  //	{
  //		Length1 = Length1;
  //		Length2 = Length1 +LengthDelta;
  //	}

  //	if (Res1 >= Res2)
  //	{
  //		Length1 = Length1 + LengthDelta ;
  //		Length2 = Length2;
  //	}

  //	LengthDelta = LengthDelta * 0.5;

  //} while (Iter < IterMax);
  return t;
};

void SplineContour::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem5 = ParentXmlNode;
  for (TiXmlElement* pElem6 = pElem5->FirstChildElement(); pElem6 != NULL; pElem6 = pElem6->NextSiblingElement())
  {
    string pVal6 = pElem6->Value();
    if (pVal6 == "NodeVector")
    {
      for (TiXmlElement* pElem7 = pElem6->FirstChildElement(); pElem7 != NULL; pElem7 = pElem7->NextSiblingElement())
      {
        string pVal7 = pElem7->Value();
        if (pVal7 == "NodeElement")
        {
          double t;
          pElem7->QueryDoubleAttribute("t", &t);
          SetNodeVector(t);
        }
      }
    }
    else if (pVal6 == "PoleVector")
    {
      for (TiXmlElement* pElem7 = pElem6->FirstChildElement(); pElem7 != NULL; pElem7 = pElem7->NextSiblingElement())
      {
        string pVal7 = pElem7->Value();
        if (pVal7 == "PoleElement")
        {
          double x;
          double y;
          double z;
          pElem7->QueryDoubleAttribute("x", &x);
          pElem7->QueryDoubleAttribute("y", &y);
          pElem7->QueryDoubleAttribute("z", &z);
          SetPoleVector(x, y, z);
        }
      }
    }
  }
};

void SplineContour::WriteXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* XmlNode = new TiXmlElement("SplineContour");
  ParentXmlNode->LinkEndChild(XmlNode);

  TiXmlElement* XmlNodeNodeVector = new TiXmlElement("NodeVector");
  XmlNode->LinkEndChild(XmlNodeNodeVector);

  // loop for all nodes
  for (int j = 0; j < List_NodeVector.size(); j++)
  {
    TiXmlElement* XmlNodeNodeElement = new TiXmlElement("NodeElement");
    XmlNodeNodeVector->LinkEndChild(XmlNodeNodeElement);
    list<double>::iterator i = List_NodeVector.begin();
    std::advance(i, j);
    double i_nodeVector;
    i_nodeVector = *i;
    XmlNodeNodeElement->SetDoubleAttribute("t", i_nodeVector);
  }

  TiXmlElement* XmlNodePoleVector = new TiXmlElement("PoleVector");
  XmlNode->LinkEndChild(XmlNodePoleVector);

  // loop for all poles
  for (int j = 0; j < List_PoleVector.size(); j++)
  {
    TiXmlElement* XmlNodePoleElement = new TiXmlElement("PoleElement");
    XmlNodePoleVector->LinkEndChild(XmlNodePoleElement);
    list<Vector3d>::iterator i = List_PoleVector.begin();
    std::advance(i, j);
    Vector3d i_poleVector;
    i_poleVector = *i;
    XmlNodePoleElement->SetDoubleAttribute("x", i_poleVector[0]);
    XmlNodePoleElement->SetDoubleAttribute("y", i_poleVector[1]);
    XmlNodePoleElement->SetDoubleAttribute("z", i_poleVector[2]);
  }
};
