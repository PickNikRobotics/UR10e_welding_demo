/**
 *  \file      Contour.h
 *  \brief     Contour and children classes
 *  \details   Geometric contour descriptions (linear, circle and spline) as used in TaskDefinition.h
 *  \copyright Fraunhofer IPA
 */

#pragma once
#include <Eigen/Eigen>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>

// #include "EigenLib.h"
#include "tinyxml.h"

using namespace std;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

class Contour  // Possible access even for unknown contour type
{
public:
  virtual double GetContourLength() = 0;
  virtual Contour* clone() const = 0;
  virtual string GetContourType() = 0;
  virtual Vector3d GetContourPosition(double DistanceFromPointOne) = 0;
  virtual Vector3d GetContourDirection(double DistanceFromPointOne) = 0;
  virtual void InvertContour() = 0;
  virtual void WriteXML(TiXmlElement* ParentXmlNode) = 0;
};

class LinearContour : public Contour
{
public:
  LinearContour() : ContourLength(-1){};
  virtual ~LinearContour() = default;
  virtual LinearContour* clone() const;
  string GetContourType();
  void SetPointOne(double x, double y, double z);
  void SetPointTwo(double x, double y, double z);
  double GetContourLength();
  Vector3d GetContourPosition(double DistanceFromPointOne);
  Vector3d GetContourDirection(double DistanceFromPointOne);
  void InvertContour();
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode);

public:
  Vector3d PointOne;
  Vector3d PointTwo;
  double ContourLength;
};

class CircleContour : public Contour
{
public:
  CircleContour() : ContourLength(-1){};
  virtual ~CircleContour() = default;
  virtual CircleContour* clone() const;
  string GetContourType();
  void SetPositionCentre(double x, double y, double z);
  void SetDirectionNormal(double x, double y, double z);
  void SetDirectionPlane(double x, double y, double z);
  void SetRadius(double r);
  void SetAlphaStart(double as);
  void SetAlphaEnd(double ae);
  double GetContourLength();
  Vector3d GetContourPosition(double DistanceFromPointOne);
  Vector3d GetContourDirection(double DistanceFromPointOne);
  void InvertContour();
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode);

public:
  Vector3d PositionCentre;
  Vector3d DirectionNormal;
  Vector3d DirectionPlane;
  double Radius;
  double AlphaStart;
  double AlphaEnd;
  double ContourLength;
};

class SplineContour : public Contour
{
public:
  SplineContour() : ContourLength(-1){};
  virtual ~SplineContour() = default;
  virtual SplineContour* clone() const;
  void SetNodeVector(double i_NodeVector);
  void SetPoleVector(double i_PoleVectorX, double i_PoleVectorY, double i_PoleVectorZ);
  list<double> GetListNodeVector();
  list<Vector3d> GetListPoleVector();
  string GetContourType();
  double GetContourLength();
  Vector3d GetContourPosition(double DistanceFromPointOne);
  Vector3d GetContourDirection(double DistanceFromPointOne);
  void InvertContour();
  double GetParameterOfLength(double i_Length);  // ToDo: Implement this function
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode);

private:
  double GetContourLengthByParameter(double t);
  Vector3d GetPositionByParameter(double t, list<double> i_List_NodeVector, list<Vector3d> i_List_PoleVector);
  Vector3d GetDirectionByParameter(double t, list<double> i_List_NodeVector, list<Vector3d> i_List_PoleVector);
  Vector3d deBoor(int j, int degree, int i, double t, list<double> i_List_NodeVector, list<Vector3d> i_List_PoleVector);
  list<double> List_NodeVector;
  list<Vector3d> List_PoleVector;
  double ContourLength;
};
