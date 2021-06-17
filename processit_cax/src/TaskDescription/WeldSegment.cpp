/**
 *  \file      WeldSegment.cpp
 *  \brief     WeldSegment and children classes (FilletWeld, ButtWeld...)
 *  \details   Description of weld segments as used in TaskDefinition.h. FilletWeldSegment
         and SingleBevelButtWeldSegment are children classes of abstract WeldSegment class.
         Position and direction segments define the different contours of weld
         segments. E.g. a fillet weld has one position segment on the weld seam and
         one on the reference wall part, as well as a direction segment for wall and
         base part.
 *  \copyright Fraunhofer IPA
 */
#include "TaskDescription/WeldSegment.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PositionSegment::PositionSegment(const PositionSegment& other) : m_pContour(NULL)
{
  if (other.m_pContour)
  {
    this->m_pContour = other.m_pContour->clone();
  }
};
PositionSegment& PositionSegment::operator=(const PositionSegment& other)
{
  if (this->m_pContour)  // if the position element already has a contour
  {
    delete m_pContour;
    m_pContour = NULL;
  }

  if (other.m_pContour)
  {
    this->m_pContour = other.m_pContour->clone();
  }
  return *this;
};
string PositionSegment::GetContourType()
{
  return m_pContour->GetContourType();
};
void PositionSegment::AddContour(Contour* pContour)
{
  m_pContour = pContour;
};
Contour* PositionSegment::GetContour()
{
  return m_pContour;
};
double PositionSegment::GetLengthOfPositionSegment()
{
  double length;
  length = m_pContour->GetContourLength();
  return length;
};
Vector3d PositionSegment::GetPosition(double length)
{
  Vector3d Pos;
  Pos = m_pContour->GetContourPosition(length);
  return Pos;
};
Vector3d PositionSegment::GetDirection(double length)
{
  Vector3d Pos;
  Pos = m_pContour->GetContourDirection(length);
  return Pos;
};
void PositionSegment::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem4 = ParentXmlNode;
  // Check type of contour within child element
  TiXmlElement* pElem5 = pElem4->FirstChildElement();
  string pVal5 = pElem5->Value();
  if (pVal5 == "LinearContour")
  {
    LinearContour* i_LinearContour = new LinearContour();
    i_LinearContour->ReadXML(pElem5);
    AddContour(i_LinearContour);
  }
  else if (pVal5 == "CircleContour")
  {
    CircleContour* i_CircleContour = new CircleContour();
    i_CircleContour->ReadXML(pElem5);
    AddContour(i_CircleContour);
  }
  else if (pVal5 == "SplineContour")
  {
    SplineContour* i_SplineContour = new SplineContour();
    i_SplineContour->ReadXML(pElem5);
    AddContour(i_SplineContour);
  }
};
void PositionSegment::WriteXML(TiXmlElement* ParentXmlNode, PositionSegment* i_PositionSegment)
{
  TiXmlElement* XmlNode = new TiXmlElement("PositionSegment");
  ParentXmlNode->LinkEndChild(XmlNode);
  i_PositionSegment->m_pContour->WriteXML(XmlNode);
}

DirectionSegment::DirectionSegment(const DirectionSegment& other) : m_pContour(NULL)
{
  if (other.m_pContour)
  {
    this->m_pContour = other.m_pContour->clone();
  }
};
DirectionSegment& DirectionSegment::operator=(const DirectionSegment& other)
{
  if (this->m_pContour)  // if the direction element already has a contour
  {
    delete m_pContour;
    m_pContour = NULL;
  }

  if (other.m_pContour)
  {
    this->m_pContour = other.m_pContour->clone();
  }
  return *this;
};
Vector3d DirectionSegment::GetDirection(double length)
{
  Vector3d Dir;
  Dir = m_pContour->GetContourDirection(length);
  return Dir;
};
void DirectionSegment::AddContour(Contour* pContour)
{
  m_pContour = pContour;
};
void DirectionSegment::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem4 = ParentXmlNode;
  // Check type of contour within child element
  TiXmlElement* pElem5 = pElem4->FirstChildElement();
  string pVal5 = pElem5->Value();
  if (pVal5 == "LinearContour")
  {
    LinearContour* i_LinearContour = new LinearContour();
    i_LinearContour->ReadXML(pElem5);
    AddContour(i_LinearContour);
  }
  else if (pVal5 == "CircleContour")
  {
    CircleContour* i_CircleContour = new CircleContour();
    i_CircleContour->ReadXML(pElem5);
    AddContour(i_CircleContour);
  }
  else if (pVal5 == "SplineContour")
  {
    SplineContour* i_SplineContour = new SplineContour();
    i_SplineContour->ReadXML(pElem5);
    AddContour(i_SplineContour);
  }
};
void DirectionSegment::WriteXML(TiXmlElement* ParentXmlNode, DirectionSegment* i_DirectionSegment)
{
  TiXmlElement* XmlNode = new TiXmlElement("DirectionSegment");
  ParentXmlNode->LinkEndChild(XmlNode);
  i_DirectionSegment->m_pContour->WriteXML(XmlNode);
}

void WeldSegment::SetWeldSegmentID(int i_WeldSegmentID)
{
  WeldSegmentID = i_WeldSegmentID;
};
int WeldSegment::GetWeldSegmentID()
{
  return WeldSegmentID;
};
void WeldSegment::SetDirectionFlag(int i_DirectionFlag)
{
  DirectionFlag = i_DirectionFlag;
};
int WeldSegment::GetDirectionFlag()
{
  return DirectionFlag;
};

void FilletWeldSegment::AddPositionSegment(PositionSegment* P)
{
  FilletWeldPositionSegment = *P;
};
void FilletWeldSegment::AddPositionSegmentRef(PositionSegment* PR)
{
  FilletWeldPositionSegmentRef = *PR;
};
void FilletWeldSegment::AddDirectionDirectionBaseSegment(DirectionSegment* DSB)
{
  FilletWeldDirectionSegmentBase = *DSB;
};
void FilletWeldSegment::AddDirectionDirectionWallSegment(DirectionSegment* DSW)
{
  FilletWeldDirectionSegmentWall = *DSW;
};
PositionSegment FilletWeldSegment::GetPositionSegment()
{
  return FilletWeldPositionSegment;
};
PositionSegment FilletWeldSegment::GetPositionSegmentRef()
{
  return FilletWeldPositionSegmentRef;
};
DirectionSegment FilletWeldSegment::GetDirectionBaseSegment()
{
  return FilletWeldDirectionSegmentBase;
};
DirectionSegment FilletWeldSegment::GetDirectionWallSegment()
{
  return FilletWeldDirectionSegmentWall;
};
Vector3d FilletWeldSegment::GetPosition(double length)
{
  Vector3d Position;
  if (DirectionFlag == 0)
  {
    Position = FilletWeldPositionSegment.GetPosition(length);
  }
  else if (DirectionFlag == 1)
  {
    Position = FilletWeldPositionSegment.GetPosition(FilletWeldPositionSegment.GetLengthOfPositionSegment() - length);
  }
  return Position;
};
Vector3d FilletWeldSegment::GetDirectionPartA(double length)
// For fillet welds part A of the weld is the wall part
{
  return GetDirectionWall(length);
}
Vector3d FilletWeldSegment::GetDirectionWall(double length)
{
  Vector3d DirectionWall;
  if (DirectionFlag == 0)
  {
    DirectionWall = FilletWeldDirectionSegmentWall.GetDirection(length);
  }
  else if (DirectionFlag == 1)
  {
    DirectionWall =
        FilletWeldDirectionSegmentWall.GetDirection(FilletWeldPositionSegment.GetLengthOfPositionSegment() - length);
  }
  return DirectionWall;
};
Vector3d FilletWeldSegment::GetDirectionPartB(double length)
// For fillet welds part B of the weld is the base part
{
  return GetDirectionBase(length);
};
Vector3d FilletWeldSegment::GetDirectionBase(double length)
{
  Vector3d DirectionBase;
  if (DirectionFlag == 0)
  {
    DirectionBase = FilletWeldDirectionSegmentBase.GetDirection(length);
  }
  else if (DirectionFlag == 1)
  {
    DirectionBase =
        FilletWeldDirectionSegmentBase.GetDirection(FilletWeldPositionSegment.GetLengthOfPositionSegment() - length);
  }
  return DirectionBase;
};
Vector3d FilletWeldSegment::GetDirectionSeam(double length)
{
  Vector3d VecB = GetDirectionBase(length);
  Vector3d VecW = GetDirectionWall(length);
  return VecW.cross(VecB).normalized();
};
double FilletWeldSegment::GetGapLength(double length)
{
  double ResTreshold = 0.0001;
  double GapLengthPrecision = 0.0001;
  double LengthOffset = 0;  // If P2 lies on virtual continuation of FilletWeldPositionSegmentRef
  int IterMax = 1000;
  int Iter = 0;
  double GapLength, PositionSegmentRefLength, Res1, Res2, Res3, Length1, Length2, LengthDelta;
  Vector3d P1, P2, D12, DirectionBase, DirectionWall, DirectionSeam;

  P1 = FilletWeldPositionSegment.GetPosition(length);
  PositionSegmentRefLength = FilletWeldPositionSegmentRef.GetLengthOfPositionSegment();

  Length1 = 0 - LengthOffset;
  Length2 = PositionSegmentRefLength + LengthOffset;
  LengthDelta = (Length2 - Length1) * 0.5;

  DirectionBase = FilletWeldDirectionSegmentBase.GetDirection(length);
  DirectionWall = FilletWeldDirectionSegmentWall.GetDirection(length);
  DirectionSeam = DirectionBase.cross(DirectionWall);

  do
  {
    P2 = FilletWeldPositionSegmentRef.GetPosition(Length1);
    D12 = P2 - P1;
    Res1 = abs(DirectionSeam.dot(D12));
    GapLength = D12.norm();
    double GapLength1 = GapLength;

    if (Res1 < ResTreshold)
    {
      break;
    }

    P2 = FilletWeldPositionSegmentRef.GetPosition(Length2);
    D12 = P2 - P1;
    Res2 = abs(DirectionSeam.dot(D12));
    GapLength = D12.norm();
    double GapLength2 = GapLength;

    if (Res2 < ResTreshold)
    {
      break;
    }

    double deltaGapLength = abs(GapLength2 - GapLength1);
    if (deltaGapLength < GapLengthPrecision)
    {
      break;
    }

    if (Res1 < Res2)
    {
      Length1 = Length1;
      Length2 = Length1 + LengthDelta;
    }

    if (Res1 >= Res2)
    {
      Length1 = Length1 + LengthDelta;
      Length2 = Length2;
    }

    LengthDelta = LengthDelta * 0.5;

  } while (Iter < IterMax);

  double prec = 0.0001;
  GapLength = (double)(floor(GapLength * (1.0 / prec) + 0.5) / (1.0 / prec));
  return GapLength;
};
double FilletWeldSegment::GetLengthOfPositionSegment()
{
  return FilletWeldPositionSegment.GetLengthOfPositionSegment();
};
void FilletWeldSegment::Invert()
{
  if (DirectionFlag == 0)
  {
    DirectionFlag = 1;
  }
  else if (DirectionFlag == 1)
  {
    DirectionFlag = 0;
  }
};
MatrixXd FilletWeldSegment::GetManufacturingCoordinateSystem(double length)
// Definition of 4x4-Matrix out of vWall and vBase
// Definition of coordinate system according to Delmia (DelmiaV5 Online-Help):
// y-axis is the cross product of base normal and wall normal. It is along the seam (reversed direction).
// z-axis is given by the bisector of the wall normal and base normal with the sense of direction reversed.
// So z-direction points towards the seam along the line given by the bisector of wall normal and base normal.
// x-axis is the cross product of y-axis and z-axis that are as defined in the above two statements.
{
  Vector3d vPos = GetPosition(length);
  Vector3d vBase = GetDirectionBase(length);
  Vector3d vWall = GetDirectionWall(length);
  Vector3d xAxis = vWall.normalized();
  Vector3d yAxis = vBase.cross(vWall).normalized();
  Vector3d zAxis = xAxis.cross(yAxis).normalized();

  // Initial Trafo
  MatrixXd TrafoIni(4, 4);
  TrafoIni(0, 0) = xAxis.x();
  TrafoIni(1, 0) = xAxis.y();
  TrafoIni(2, 0) = xAxis.z();
  TrafoIni(3, 0) = 0;

  TrafoIni(0, 1) = yAxis.x();
  TrafoIni(1, 1) = yAxis.y();
  TrafoIni(2, 1) = yAxis.z();
  TrafoIni(3, 1) = 0;

  TrafoIni(0, 2) = zAxis.x();
  TrafoIni(1, 2) = zAxis.y();
  TrafoIni(2, 2) = zAxis.z();
  TrafoIni(3, 2) = 0;

  TrafoIni(0, 3) = vPos.x();
  TrafoIni(1, 3) = vPos.y();
  TrafoIni(2, 3) = vPos.z();
  TrafoIni(3, 3) = 1;

  // Angle between base vector and wall vector (choose smallest vector as we have fillet weld tasks)
  double angleBaseWall = acos(vBase.dot(vWall));
  // double angleBaseWall = GetSmallestAngle(WeldTaskID, DistanceFromStartPoint);
  double angleRot = M_PI * 3 / 2 - angleBaseWall / 2;

  // Build rotation matrix (rotating around y-axis)
  MatrixXd TrafoRot(4, 4);
  TrafoRot(0, 0) = cos(angleRot);
  TrafoRot(1, 0) = 0;
  TrafoRot(2, 0) = -sin(angleRot);
  TrafoRot(3, 0) = 0;

  TrafoRot(0, 1) = 0;
  TrafoRot(1, 1) = 1;
  TrafoRot(2, 1) = 0;
  TrafoRot(3, 1) = 0;

  TrafoRot(0, 2) = sin(angleRot);
  TrafoRot(1, 2) = 0;
  TrafoRot(2, 2) = cos(angleRot);
  TrafoRot(3, 2) = 0;

  TrafoRot(0, 3) = 0;
  TrafoRot(1, 3) = 0;
  TrafoRot(2, 3) = 0;
  TrafoRot(3, 3) = 1;

  // Rotate initial transformation with rotation matrix
  MatrixXd ManufacturingCoordinateSystem;
  ManufacturingCoordinateSystem = TrafoIni * TrafoRot;
  return ManufacturingCoordinateSystem;
};
void FilletWeldSegment::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem3 = ParentXmlNode;
  int i_FilletWeldSegmentID;
  pElem3->QueryIntAttribute("ID", &i_FilletWeldSegmentID);
  SetWeldSegmentID(i_FilletWeldSegmentID);
  int i_DirectionFlag;
  pElem3->QueryIntAttribute("DirectionFlag", &i_DirectionFlag);
  SetDirectionFlag(i_DirectionFlag);
  int PosSegCount = 0;
  int DirSegCount = 0;

  // Go through all elements of FilletWeldSegment
  for (TiXmlElement* pElem4 = pElem3->FirstChildElement(); pElem4 != NULL; pElem4 = pElem4->NextSiblingElement())
  {
    PositionSegment i_PositionSegment;
    DirectionSegment i_DirectionSegmentBase;
    DirectionSegment i_DirectionSegmentWall;

    // Check for type of segment
    string pVal4 = pElem4->Value();
    if (pVal4 == "PositionSegment")
    {
      if (PosSegCount == 0)
      {
        i_PositionSegment.ReadXML(pElem4);
        AddPositionSegment(&i_PositionSegment);
        PosSegCount++;
      }
      else if (PosSegCount == 1)
      {
        i_PositionSegment.ReadXML(pElem4);
        AddPositionSegmentRef(&i_PositionSegment);
      }
    }

    else if (pVal4 == "DirectionSegment")
    {
      if (DirSegCount == 0)
      {
        i_DirectionSegmentBase.ReadXML(pElem4);
        AddDirectionDirectionBaseSegment(&i_DirectionSegmentBase);
        DirSegCount++;
      }
      else if (DirSegCount == 1)
      {
        i_DirectionSegmentWall.ReadXML(pElem4);
        AddDirectionDirectionWallSegment(&i_DirectionSegmentWall);
      }
    }
  }
};
void FilletWeldSegment::WriteXML(TiXmlElement* ParentXmlNode, string segmentType)
// segmentType can be FilletWeldSegment, BevelSegment or FlatSegment
{
  TiXmlElement* XmlNode = new TiXmlElement(segmentType.c_str());
  ParentXmlNode->LinkEndChild(XmlNode);
  XmlNode->SetAttribute("ID", GetWeldSegmentID());
  XmlNode->SetAttribute("DirectionFlag", GetDirectionFlag());
  PositionSegment i_PositionSegment = this->GetPositionSegment();
  i_PositionSegment.WriteXML(XmlNode, &(i_PositionSegment));
  PositionSegment i_PositionSegmentRef = this->GetPositionSegmentRef();
  i_PositionSegmentRef.WriteXML(XmlNode, &(i_PositionSegmentRef));
  DirectionSegment i_DirectionBaseSegment = this->GetDirectionBaseSegment();
  i_DirectionBaseSegment.WriteXML(XmlNode, &(i_DirectionBaseSegment));
  DirectionSegment i_DirectionWallSegment = this->GetDirectionWallSegment();
  i_DirectionWallSegment.WriteXML(XmlNode, &(i_DirectionWallSegment));
};

void SingleBevelButtWeldSegment::AddBevelSegment(FilletWeldSegment* BS)
{
  WeldSegmentBevel = *BS;
}

void SingleBevelButtWeldSegment::AddFlatSegment(FilletWeldSegment* FS)
{
  WeldSegmentFlat = *FS;
}

PositionSegment SingleBevelButtWeldSegment::GetPositionSegment()
// default position segment is flat side of single bevel butt weld
{
  return WeldSegmentFlat.GetPositionSegment();
}

Vector3d SingleBevelButtWeldSegment::GetPosition(double length)
// default seam position is on flat side of single bevel butt weld
{
  return GetPositionFlat(length);
}

Vector3d SingleBevelButtWeldSegment::GetPositionBevel(double length)
{
  return WeldSegmentBevel.GetPosition(length);
}

Vector3d SingleBevelButtWeldSegment::GetPositionFlat(double length)
{
  return WeldSegmentFlat.GetPosition(length);
}

Vector3d SingleBevelButtWeldSegment::GetDirectionPartA(double length)
{
  return GetDirectionBevel(length);
}

Vector3d SingleBevelButtWeldSegment::GetDirectionPartB(double length)
{
  return GetDirectionFlat(length);
}

Vector3d SingleBevelButtWeldSegment::GetDirectionBevel(double length)
{
  return WeldSegmentBevel.GetDirectionWall(length);
}

Vector3d SingleBevelButtWeldSegment::GetDirectionFlat(double length)
{
  return WeldSegmentFlat.GetDirectionWall(length);
}

Vector3d SingleBevelButtWeldSegment::GetDirectionSeam(double length)
{
  Vector3d VecBevel = GetDirectionBevel(length);
  Vector3d VecFlat = GetDirectionFlat(length);
  return VecBevel.cross(VecFlat).normalized();
}

double SingleBevelButtWeldSegment::GetGapLength(double length)
{
  Vector3d VecGap = WeldSegmentBevel.GetPosition(length) - WeldSegmentFlat.GetPosition(length);
  return VecGap.norm();
}

double SingleBevelButtWeldSegment::GetLengthOfPositionSegment()
{
  return GetPositionSegment().GetLengthOfPositionSegment();
}

void SingleBevelButtWeldSegment::Invert()
{
  WeldSegmentBevel.Invert();
  WeldSegmentFlat.Invert();
}

MatrixXd SingleBevelButtWeldSegment::GetManufacturingCoordinateSystem(double length)
// Sets default coordinate system in the middle of the joint between beveled
// and flat side.
{
  // Position of coordinate frame is by default half way between bevel
  // and flat part of butt joint
  Vector3d posF = WeldSegmentFlat.GetPosition(length);
  Vector3d posB = WeldSegmentBevel.GetPosition(length);
  Vector3d translation = posF + 0.5 * (posB - posF);

  // y-Vector is the reversed seam direction
  // E.g. cross product of flat wall and flat base
  Vector3d vecFlatBase = WeldSegmentFlat.GetDirectionBase(length);
  Vector3d vecFlatWall = WeldSegmentFlat.GetDirectionWall(length);
  Vector3d yAxis = vecFlatWall.cross(vecFlatBase);

  // z-Vector points into the seam at the bisector between bevel
  // and flat surfaces of butt joint
  Vector3d vecBevelWall = WeldSegmentBevel.GetDirectionWall(length);
  double angleSeam = acos(vecFlatWall.dot(vecBevelWall));
  double angleZ = M_PI + 0.5 * angleSeam;
  Vector3d zAxis = vecFlatWall;
  // rotate wall vector of flat joint side around y-Vector
  Matrix3d rotZ;
  rotZ = AngleAxisd(angleZ, yAxis);
  zAxis = rotZ * zAxis;

  // x-Vector is cross product of y- and z-Axis
  Vector3d xAxis = yAxis.cross(zAxis);

  // set manufacturing coordinate frame
  MatrixXd MCS(4, 4);
  MCS(0, 0) = xAxis.x();
  MCS(1, 0) = xAxis.y();
  MCS(2, 0) = xAxis.z();
  MCS(3, 0) = 0;

  MCS(0, 1) = yAxis.x();
  MCS(1, 1) = yAxis.y();
  MCS(2, 1) = yAxis.z();
  MCS(3, 1) = 0;

  MCS(0, 2) = zAxis.x();
  MCS(1, 2) = zAxis.y();
  MCS(2, 2) = zAxis.z();
  MCS(3, 2) = 0;

  MCS(0, 3) = translation.x();
  MCS(1, 3) = translation.y();
  MCS(2, 3) = translation.z();
  MCS(3, 3) = 1;

  return MCS;
}

void SingleBevelButtWeldSegment::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* parentNode = ParentXmlNode;
  int i_ButtWeldSegmentID;
  parentNode->QueryIntAttribute("ID", &i_ButtWeldSegmentID);
  SetWeldSegmentID(i_ButtWeldSegmentID);
  int i_DirectionFlag;
  parentNode->QueryIntAttribute("DirectionFlag", &i_DirectionFlag);
  SetDirectionFlag(i_DirectionFlag);

  // Go through all elements of SingleBevelButtWeldSegment
  for (TiXmlElement* childNode = parentNode->FirstChildElement(); childNode != NULL;
       childNode = childNode->NextSiblingElement())
  {
    FilletWeldSegment i_FilletWeldSegment;

    // Check for type of segment
    string pVal = childNode->Value();
    if (pVal == "BevelSegment")
    {
      i_FilletWeldSegment.ReadXML(childNode);
      AddBevelSegment(&i_FilletWeldSegment);
    }
    else if (pVal == "FlatSegment")
    {
      i_FilletWeldSegment.ReadXML(childNode);
      AddFlatSegment(&i_FilletWeldSegment);
    }
  }
}

void SingleBevelButtWeldSegment::WriteXML(TiXmlElement* ParentXmlNode, string segmentType)
{
  if (segmentType != "SingleBevelButtWeldSegment")
  {
    throw logic_error("Wrong type of seam specified. Should be single bevel butt weld.");
  }
  TiXmlElement* XmlNode = new TiXmlElement("SingleBevelButtWeldSegment");
  ParentXmlNode->LinkEndChild(XmlNode);
  XmlNode->SetAttribute("ID", GetWeldSegmentID());
  XmlNode->SetAttribute("DirectionFlag", GetDirectionFlag());
  WeldSegmentBevel.WriteXML(XmlNode, "BevelSegment");
  WeldSegmentFlat.WriteXML(XmlNode, "FlatSegment");
}
