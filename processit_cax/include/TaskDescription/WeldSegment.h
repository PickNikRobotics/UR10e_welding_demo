/**
 *  \file      WeldSegment.h
 *  \brief     WeldSegment and children classes (FilletWeld, ButtWeld...)
 *  \details   Description of weld segments as used in TaskDefinition.h. FilletWeldSegment
         and SingleBevelButtWeldSegment are children classes of abstract WeldSegment class.
         Position and direction segments define the different contours of weld
         segments. E.g. a fillet weld has one position segment on the weld seam and
         one on the reference wall part, as well as a direction segment for wall and
         base part.
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
#include "Contour.h"

using namespace std;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

class PositionSegment
{
public:
  PositionSegment() : m_pContour(NULL){};  // Constructor with list for initialization
  ~PositionSegment()
  {
    if (m_pContour)
      delete m_pContour;  // if the position element does no longer exist, also the contour element will be deleted
  };
  PositionSegment(const PositionSegment& other);       // Copy Constructor
  PositionSegment& operator=(const PositionSegment&);  // Assignment Operator
  string GetContourType();
  void AddContour(Contour* pContour);
  Contour* GetContour();
  double GetLengthOfPositionSegment();
  Vector3d GetPosition(double length);
  Vector3d GetDirection(double length);

  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode, PositionSegment* i_PositionSegment);

public:
  Contour* m_pContour;
};

class DirectionSegment
{
public:
  DirectionSegment() : m_pContour(NULL){};
  ~DirectionSegment()
  {
    if (m_pContour)
      delete m_pContour;
  };
  DirectionSegment(const DirectionSegment& other);
  DirectionSegment& operator=(const DirectionSegment&);
  void AddContour(Contour* pContour);
  Vector3d GetDirection(double length);
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode, DirectionSegment* i_DirectionSegment);

public:
  Contour* m_pContour;
};

// parent class for FilletWeldSegment and SingleBevelButtWeldSegment
class WeldSegment
{
public:
  virtual ~WeldSegment() = default;
  virtual PositionSegment GetPositionSegment() = 0;
  virtual Vector3d GetPosition(double length) = 0;
  virtual Vector3d GetDirectionPartA(double length) = 0;
  virtual Vector3d GetDirectionPartB(double length) = 0;
  // directions part A and B are defined so that A.cross(B)=direction of seam
  virtual Vector3d GetDirectionSeam(double length) = 0;
  virtual double GetGapLength(double length) = 0;
  virtual double GetLengthOfPositionSegment() = 0;
  virtual void Invert() = 0;
  void SetWeldSegmentID(int i_WeldSegmentID);
  int GetWeldSegmentID();
  void SetDirectionFlag(int i_DirectionFlag);
  int GetDirectionFlag();
  virtual MatrixXd GetManufacturingCoordinateSystem(double length) = 0;
  virtual void ReadXML(TiXmlElement* ParentXmlNode) = 0;
  virtual void WriteXML(TiXmlElement* ParentXmlNode, string segmentType) = 0;

protected:
  int WeldSegmentID = -1;
  int DirectionFlag = 0;
};

class FilletWeldSegment : public WeldSegment
{
public:
  ~FilletWeldSegment() = default;
  void AddPositionSegment(PositionSegment* P);
  void AddPositionSegmentRef(PositionSegment* PR);
  void AddDirectionDirectionBaseSegment(DirectionSegment* DSB);
  void AddDirectionDirectionWallSegment(DirectionSegment* DSW);
  PositionSegment GetPositionSegment();
  PositionSegment GetPositionSegmentRef();
  DirectionSegment GetDirectionBaseSegment();
  DirectionSegment GetDirectionWallSegment();
  Vector3d GetPosition(double length);
  Vector3d GetDirectionPartA(double length);
  Vector3d GetDirectionWall(double length);
  Vector3d GetDirectionPartB(double length);
  Vector3d GetDirectionBase(double length);
  Vector3d GetDirectionSeam(double length);
  double GetGapLength(double length);
  double GetLengthOfPositionSegment();
  void Invert();
  MatrixXd GetManufacturingCoordinateSystem(double length);
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode, string segmentType);

private:
  PositionSegment FilletWeldPositionSegment;
  PositionSegment FilletWeldPositionSegmentRef;
  DirectionSegment FilletWeldDirectionSegmentBase;
  DirectionSegment FilletWeldDirectionSegmentWall;
};

class SingleBevelButtWeldSegment : public WeldSegment
{
public:
  ~SingleBevelButtWeldSegment() = default;
  void AddBevelSegment(FilletWeldSegment* BS);
  void AddFlatSegment(FilletWeldSegment* FS);
  PositionSegment GetPositionSegment();
  Vector3d GetPosition(double length);
  Vector3d GetPositionBevel(double length);
  Vector3d GetPositionFlat(double length);
  Vector3d
  GetDirectionPartA(double length);  // directions part A and B are defined so that A.cross(B)=direction of seam
  Vector3d GetDirectionPartB(double length);
  Vector3d GetDirectionBevel(double length);
  Vector3d GetDirectionFlat(double length);
  Vector3d GetDirectionSeam(double length);
  double GetGapLength(double length);
  double GetLengthOfPositionSegment();
  void Invert();
  MatrixXd GetManufacturingCoordinateSystem(double length);
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode, string segmentType);

private:
  FilletWeldSegment WeldSegmentBevel;
  FilletWeldSegment WeldSegmentFlat;
};
