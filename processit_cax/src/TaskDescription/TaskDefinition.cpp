/**
 *  \file      TaskDefintion.cpp
 *  \brief     Weld Task Definition
 *  \details   WeldTaskDefintion - Builds on top of the TaskDefintion
 *  \copyright Fraunhofer IPA
 */

#include "TaskDescription/TaskDefinition.h"
#include "TaskDescription/Templates.h"
#include "math.h"
#include <fstream>
#include <iostream>
#include <sstream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

void SubTask::SetSubTaskID(int aID)
{
  this->SubTaskID = aID;
};

void SubTask::SetStartLength(double aLength)
{
  this->StartLength = aLength;
};

void SubTask::SetEndLength(double aLength)
{
  this->EndLength = aLength;
};

void SubTask::SetDirectionFlag(bool aFlag)
{
  this->IsReversed = aFlag;
};

void SubTask::SetRotWire(double aR)
{
  this->RotWire = aR;
};

void SubTask::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* pElem3 = ParentXmlNode;
  int aSubTaskID, aIntFlag;
  pElem3->QueryIntAttribute("ID", &aSubTaskID);
  pElem3->QueryIntAttribute("IsReversed", &aIntFlag);
  bool aFlag = aIntFlag;
  this->SetSubTaskID(aSubTaskID);
  this->SetDirectionFlag(aFlag);
  double SL, EL, aR;
  pElem3->QueryDoubleAttribute("StartLength", &SL);
  pElem3->QueryDoubleAttribute("EndLength", &EL);
  pElem3->QueryDoubleAttribute("RotWire", &aR);
  this->SetStartLength(SL);
  this->SetEndLength(EL);
  this->SetRotWire(aR);
};

void SubTask::WriteXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* XmlNode = new TiXmlElement("SubTask");
  ParentXmlNode->LinkEndChild(XmlNode);
  XmlNode->SetAttribute("ID", this->SubTaskID);
  XmlNode->SetDoubleAttribute("StartLength", this->StartLength);
  XmlNode->SetDoubleAttribute("EndLength", this->EndLength);
  XmlNode->SetAttribute("IsReversed", this->IsReversed);
  XmlNode->SetDoubleAttribute("RotWire", this->RotWire);
};

void EdgeTask::SetTaskID(int i_TaskID)
{
  TaskID = i_TaskID;
};

int EdgeTask::GetTaskID()
{
  return TaskID;
};

void EdgeTask::SetState(int i_State)
{
  State = i_State;
};
int EdgeTask::GetState()
{
  return State;
};

void WeldTask::AddWeldSegment(shared_ptr<WeldSegment> WS)
{
  List_WeldSegment.push_back(WS);
};
void WeldTask::SetList_WeldSegment(list<shared_ptr<WeldSegment>> i_List_WeldSegment)
{
  List_WeldSegment.clear();
  List_WeldSegment = i_List_WeldSegment;
};
list<shared_ptr<WeldSegment>> WeldTask::GetList_WeldSegment()
{
  return List_WeldSegment;
};
int WeldTask::GetNumberOfWeldSegments()
{
  return List_WeldSegment.size();
};
shared_ptr<WeldSegment> WeldTask::GetWeldSegment(int numberOfWeldSegment)
{
  list<shared_ptr<WeldSegment>>::iterator i = List_WeldSegment.begin();
  advance(i, numberOfWeldSegment);
  shared_ptr<WeldSegment> WS;
  WS = *i;
  return WS;
};
double WeldTask::GetTaskContourLength()
{
  if (TaskLength = -1)
  {
    TaskLength = 0;
    for (int j = 0; j < List_WeldSegment.size(); j++)
    {
      list<shared_ptr<WeldSegment>>::iterator i = List_WeldSegment.begin();
      advance(i, j);
      shared_ptr<WeldSegment> i_WeldSegment = *i;
      TaskLength += i_WeldSegment->GetLengthOfPositionSegment();
    }
  }
  return TaskLength;
};

double WeldTask::GetWeldSegmentLength(int numberOfWeldSegment)
{
  list<shared_ptr<WeldSegment>>::iterator i = List_WeldSegment.begin();
  advance(i, numberOfWeldSegment);
  shared_ptr<WeldSegment> i_WeldSegment = *i;
  return i_WeldSegment->GetLengthOfPositionSegment();
};

shared_ptr<WeldSegment> WeldTask::GetSegmentFromStartPoint(double& DistanceFromStartPoint)
{
  // Check in which WeldSegment the DistanceFromStartPoint (of the weld task)
  // is located and return distance from start point of that WeldSegment (passed by reference)
  double i_ContourLengthSum = 0;
  double i_DistanceFromStartPointOfActualSegment = 0;
  list<shared_ptr<WeldSegment>>::iterator iter = List_WeldSegment.begin();
  for (int j = 0; j < List_WeldSegment.size(); j++)
  {
    advance(iter, j);
    shared_ptr<WeldSegment> i_WeldSegment = *iter;
    double Length_temp = i_WeldSegment->GetLengthOfPositionSegment();
    if (DistanceFromStartPoint >= i_ContourLengthSum && DistanceFromStartPoint <= (i_ContourLengthSum + Length_temp))
    {
      i_DistanceFromStartPointOfActualSegment = DistanceFromStartPoint - i_ContourLengthSum;
      break;
    }
    i_ContourLengthSum += Length_temp;
    iter = List_WeldSegment.begin();
  }
  DistanceFromStartPoint = i_DistanceFromStartPointOfActualSegment;
  return *iter;
};

Vector3d WeldTask::GetPosition(double DistanceFromStartPoint)
{
  // Check in which WeldSegment questioned position is located according to DistanceFromStartPoint
  double DistanceFromStartPointOfActualSegment = DistanceFromStartPoint;
  shared_ptr<WeldSegment> i_WeldSegment = GetSegmentFromStartPoint(DistanceFromStartPointOfActualSegment);
  Vector3d Position = i_WeldSegment->GetPosition(DistanceFromStartPointOfActualSegment);
  return Position;
};

double WeldTask::GetGapLength(double DistanceFromStartPoint)
{
  // Check in which WeldSegment questioned position is located according to DistanceFromStartPoint
  double DistanceFromStartPointOfActualSegment = DistanceFromStartPoint;
  shared_ptr<WeldSegment> i_WeldSegment = GetSegmentFromStartPoint(DistanceFromStartPointOfActualSegment);
  double GapLength = i_WeldSegment->GetGapLength(DistanceFromStartPointOfActualSegment);
  return GapLength;
};

Vector3d WeldTask::GetDirectionPartA(double DistanceFromStartPoint)
// Part A for fillet welds == wall part, for single bevel butt welds == bevel part
{
  // Check in which WeldSegment questioned position is located according to DistanceFromStartPoint
  double DistanceFromStartPointOfActualSegment = DistanceFromStartPoint;
  shared_ptr<WeldSegment> i_WeldSegment = GetSegmentFromStartPoint(DistanceFromStartPointOfActualSegment);
  Vector3d DirectionPartA = i_WeldSegment->GetDirectionPartA(DistanceFromStartPointOfActualSegment);
  return DirectionPartA;
};

Vector3d WeldTask::GetDirectionPartB(double DistanceFromStartPoint)
// Part B for fillet welds == base part, for single bevel butt welds == flat part
{
  // Check in which WeldSegment questioned position is located according to DistanceFromStartPoint
  double DistanceFromStartPointOfActualSegment = DistanceFromStartPoint;
  shared_ptr<WeldSegment> i_WeldSegment = GetSegmentFromStartPoint(DistanceFromStartPointOfActualSegment);
  Vector3d DirectionPartB = i_WeldSegment->GetDirectionPartB(DistanceFromStartPointOfActualSegment);
  return DirectionPartB;
};

Vector3d WeldTask::GetDirectionSeam(double DistanceFromStartPoint)
{
  // Check in which WeldSegment questioned position is located according to DistanceFromStartPoint
  double DistanceFromStartPointOfActualSegment = DistanceFromStartPoint;
  shared_ptr<WeldSegment> i_WeldSegment = GetSegmentFromStartPoint(DistanceFromStartPointOfActualSegment);
  Vector3d DirectionSeam = i_WeldSegment->GetDirectionSeam(DistanceFromStartPointOfActualSegment);
  return DirectionSeam;
}

void WeldTask::InvertWeldTask()
{
  list<shared_ptr<WeldSegment>> i_List_WeldSegment;
  for (int j = 0; j < i_List_WeldSegment.size(); j++)
  {
    shared_ptr<WeldSegment> i_WeldSegment;
    list<shared_ptr<WeldSegment>>::iterator i = List_WeldSegment.begin();
    advance(i, j);
    i_WeldSegment = *i;
    i_WeldSegment->Invert();
    i_List_WeldSegment.push_front(i_WeldSegment);
  }
  SetList_WeldSegment(i_List_WeldSegment);
};

MatrixXd WeldTask::GetManufacturingCoordinateSystem(double DistanceFromStartPoint)
{
  // Check in which WeldSegment questioned position is located according to DistanceFromStartPoint
  double DistanceFromStartPointOfActualSegment = DistanceFromStartPoint;
  shared_ptr<WeldSegment> i_WeldSegment = GetSegmentFromStartPoint(DistanceFromStartPointOfActualSegment);
  MatrixXd MFCS = i_WeldSegment->GetManufacturingCoordinateSystem(DistanceFromStartPointOfActualSegment);
  return MFCS;
};

void WeldTask::ReadXML(TiXmlElement* ParentXmlNode)
{
  TiXmlElement* parentNode = ParentXmlNode;
  parentNode->QueryIntAttribute("ID", &TaskID);
  parentNode->QueryIntAttribute("State", &State);
  // Go through all WeldSegments
  for (TiXmlElement* childNode = parentNode->FirstChildElement(); childNode != NULL;
       childNode = childNode->NextSiblingElement())
  {
    // Check for type of segment
    string pVal = childNode->Value();
    if (pVal == "FilletWeldSegment")
    {
      FilletWeldSegment i_WeldConSeg;
      i_WeldConSeg.ReadXML(childNode);
      shared_ptr<FilletWeldSegment> i_WeldConSeg_Ptr = make_shared<FilletWeldSegment>();
      *i_WeldConSeg_Ptr = i_WeldConSeg;
      AddWeldSegment(i_WeldConSeg_Ptr);
    }
    else if (pVal == "SingleBevelButtWeldSegment")
    {
      SingleBevelButtWeldSegment i_ButtWeldSeg;
      i_ButtWeldSeg.ReadXML(childNode);
      shared_ptr<SingleBevelButtWeldSegment> i_ButtWeldSeg_Ptr = make_shared<SingleBevelButtWeldSegment>();
      *i_ButtWeldSeg_Ptr = i_ButtWeldSeg;
      AddWeldSegment(i_ButtWeldSeg_Ptr);
    }
    else if (pVal == "SubTask")
    {
      SubTask i_SubTask;
      i_SubTask.ReadXML(childNode);
      this->List_SubTask.push_back(i_SubTask);
    }
  }
};

void WeldTask::WriteXML(TiXmlElement* ParentXmlNode, int j)
{
  shared_ptr<WeldSegment> i_WeldSegment;
  TiXmlElement* XmlNodeWeldTask = new TiXmlElement("WeldTask");
  ParentXmlNode->LinkEndChild(XmlNodeWeldTask);
  XmlNodeWeldTask->SetAttribute("ID", TaskID);
  XmlNodeWeldTask->SetAttribute("State", State);

  // loop through all WeldSegments
  for (int i = 0; i < List_WeldSegment.size(); i++)
  {
    i_WeldSegment = GetWeldSegment(i);
    // check segment type during run time
    if (dynamic_cast<FilletWeldSegment*>(i_WeldSegment.get()) != nullptr)
    {
      i_WeldSegment->WriteXML(XmlNodeWeldTask, "FilletWeldSegment");
    }
    else if (dynamic_cast<SingleBevelButtWeldSegment*>(i_WeldSegment.get()) != nullptr)
    {
      i_WeldSegment->WriteXML(XmlNodeWeldTask, "SingleBevelButtWeldSegment");
    }
  }
  for (int i = 0; i < this->List_SubTask.size(); i++)
  {
    SubTask aSubTask;
    aSubTask = this->GetSubTask(i);
    aSubTask.WriteXML(XmlNodeWeldTask);
  }
}

void WeldTask::AddSubTask(int SubTaskID, double StartLength, double EndLength, bool IsReversed, double RotWire)
{
  SubTask aSubTask(SubTaskID, StartLength, EndLength, IsReversed, RotWire);
  this->List_SubTask.push_back(aSubTask);
};

void WeldTask::SetSubTaskDefault()
{
  this->List_SubTask.clear();
  int SubTaskID = 0;
  double StartLength = 0.;
  double EndLength = this->GetTaskContourLength();
  this->AddSubTask(SubTaskID, StartLength, EndLength, 0, 0.);
};

SubTask WeldTask::GetSubTask(int SubTaskID)
{
  list<SubTask>::iterator iter = this->List_SubTask.begin();
  advance(iter, SubTaskID);
  SubTask aSubTask = *iter;
  return aSubTask;
};

int WeldTask::GetNumberOfSubTasks()
{
  int Num = this->List_SubTask.size();
  return Num;
};

void WeldTask::DeleteSubTask(int SubTaskID)
{
  int SubTasksNum = this->List_SubTask.size();
  list<SubTask>::iterator iter = this->List_SubTask.begin();
  advance(iter, SubTaskID);
  List_SubTask.erase(iter);
  if (SubTaskID < SubTasksNum - 1)
  {
    for (int i = SubTaskID; i < SubTasksNum - 1; i++)
    {
      iter = this->List_SubTask.begin();
      advance(iter, i);
      iter->SetSubTaskID(i);
    }
  }
};

double WeldTask::GetStartLengthOfSubTask(int SubTaskID)
{
  list<SubTask>::iterator iter = this->List_SubTask.begin();
  advance(iter, SubTaskID);
  double Length = iter->StartLength;
  return Length;
};

double WeldTask::GetEndLengthOfSubTask(int SubTaskID)
{
  list<SubTask>::iterator iter = this->List_SubTask.begin();
  advance(iter, SubTaskID);
  double Length = iter->EndLength;
  return Length;
};

bool WeldTask::GetReversedFlagOfSubTask(int SubTaskID)
{
  list<SubTask>::iterator iter = this->List_SubTask.begin();
  advance(iter, SubTaskID);
  bool Flag = iter->IsReversed;
  return Flag;
};

double WeldTask::GetRotWireOfSubTask(int SubTaskID)
{
  list<SubTask>::iterator iter = this->List_SubTask.begin();
  advance(iter, SubTaskID);
  double Value = iter->RotWire;
  return Value;
};

void TaskList::AddWeldTask(WeldTask* i_WeldTask)
{
  List_WeldTask.push_back(*i_WeldTask);
};

int TaskList::GetNumberOfWeldTasks()
{
  return List_WeldTask.size();
};

int TaskList::GetNumberOfWeldSegments(int WeldTaskID)
{
  WeldTask i_WeldTask;
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  i_WeldTask = *i;
  return i_WeldTask.GetNumberOfWeldSegments();
};

double TaskList::GetLengthOfWeldTask(int WeldTaskID)
{
  WeldTask i_WeldTask;
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  i_WeldTask = *i;
  return i_WeldTask.GetTaskContourLength();
};

double TaskList::GetWeldSegmentLength(int WeldTaskID, int FilletWeldSegmentID)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetWeldSegmentLength(FilletWeldSegmentID);
};

double TaskList::GetSmallestAngle(int WeldTaskID, double DistanceFromStartPoint)
// Compute between welding partners e.g. between base and wall part
// of fillet weld or between top and bottom part of vertical butt weld
{
  double angle;
  Vector3d vPartB = GetDirectionPartBOfWeldTask(WeldTaskID, DistanceFromStartPoint);
  Vector3d vPartA = GetDirectionPartAOfWeldTask(WeldTaskID, DistanceFromStartPoint);
  angle = acos(vPartB.dot(vPartA));
  return angle;
};

MatrixXd TaskList::GetManufacturingCoordinateSystem(int WeldTaskID, double DistanceFromStartPoint)
// Definition of 4x4-Matrix out of vWall and vBase
// Definition of coordinate system according to Delmia (DelmiaV5 Online-Help):
// y-axis is the cross product of base normal and wall normal. It is along the seam (reversed direction).
// z-axis is given by the bisector of the wall normal and base normal with the sense of direction reversed.
// So z-direction points towards the seam along the line given by the bisector of wall normal and base normal.
// x-axis is the cross product of y-axis and z-axis that are as defined in the above two statements.
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetManufacturingCoordinateSystem(DistanceFromStartPoint);
};

Vector3d TaskList::GetPositionOfWeldTask(int WeldTaskID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetPosition(DistanceFromStartPoint);
};

Vector3d TaskList::GetDirectionPartAOfWeldTask(int WeldTaskID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetDirectionPartA(DistanceFromStartPoint);
};

Vector3d TaskList::GetDirectionPartBOfWeldTask(int WeldTaskID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetDirectionPartB(DistanceFromStartPoint);
};

Vector3d TaskList::GetDirectionSeamOfWeldTask(int WeldTaskID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetDirectionSeam(DistanceFromStartPoint);
};

double TaskList::GetGapLengthOfWeldTask(int WeldTaskID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  return i->GetGapLength(DistanceFromStartPoint);
};

Vector3d TaskList::GetPositionOfWeldSegment(int WeldTaskID, int WeldSegmentID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  list<shared_ptr<WeldSegment>> i_List_WeldSegment = i->GetList_WeldSegment();
  list<shared_ptr<WeldSegment>>::iterator j = i_List_WeldSegment.begin();
  advance(j, WeldSegmentID);
  shared_ptr<WeldSegment> i_WeldSegment;
  i_WeldSegment = *j;
  return i_WeldSegment->GetPosition(DistanceFromStartPoint);
};

Vector3d TaskList::GetDirectionPartAOfWeldSegment(int WeldTaskID, int WeldSegmentID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  list<shared_ptr<WeldSegment>> i_List_WeldSegment = i->GetList_WeldSegment();
  list<shared_ptr<WeldSegment>>::iterator j = i_List_WeldSegment.begin();
  advance(j, WeldSegmentID);
  shared_ptr<WeldSegment> i_WeldSegment;
  i_WeldSegment = *j;
  return i_WeldSegment->GetDirectionPartA(DistanceFromStartPoint);
};

Vector3d TaskList::GetDirectionPartBOfWeldSegment(int WeldTaskID, int WeldSegmentID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  list<shared_ptr<WeldSegment>> i_List_WeldSegment = i->GetList_WeldSegment();
  list<shared_ptr<WeldSegment>>::iterator j = i_List_WeldSegment.begin();
  advance(j, WeldSegmentID);
  shared_ptr<WeldSegment> i_WeldSegment;
  i_WeldSegment = *j;
  return i_WeldSegment->GetDirectionPartB(DistanceFromStartPoint);
};

double TaskList::GetGapLengthOfWeldSegment(int WeldTaskID, int WeldSegmentID, double DistanceFromStartPoint)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  list<shared_ptr<WeldSegment>> i_List_WeldSegment = i->GetList_WeldSegment();
  list<shared_ptr<WeldSegment>>::iterator j = i_List_WeldSegment.begin();
  advance(j, WeldSegmentID);
  shared_ptr<WeldSegment> i_WeldSegment;
  i_WeldSegment = *j;
  return i_WeldSegment->GetGapLength(DistanceFromStartPoint);
};

string TaskList::GetContourTypeOfPositionSegment(int WeldTaskID, int WeldSegmentID)
{
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  list<shared_ptr<WeldSegment>> i_List_WeldSegment = i->GetList_WeldSegment();
  list<shared_ptr<WeldSegment>>::iterator j = i_List_WeldSegment.begin();
  advance(j, WeldSegmentID);
  shared_ptr<WeldSegment> i_WeldSegment;
  i_WeldSegment = *j;
  PositionSegment i_PositionSegment = i_WeldSegment->GetPositionSegment();
  return i_PositionSegment.GetContourType();
};
double round(double d, double prec)
{
  return (double)(floor(d * (1.0 / prec) + 0.5) / (1.0 / prec));
};
void TaskList::GroupWeldTasks()
{
  double ToleranceDirections = 1;
  double TolerancePositions = 0.5;
  for (int Count = 0; Count < 2; Count++)
  {
    // Case1: Compare end point of vector1 with start point of vector2
    int List_WeldTaskSize = List_WeldTask.size();
    for (int i = 0; i < List_WeldTask.size(); i++)
    {
      bool Group = false;

      int temp;
      int count = 0;

      for (int j = 0; j < List_WeldTask.size(); j++)
      {
        if (i < List_WeldTask.size() && i != j)
        {
          temp = List_WeldTask.size();
          list<WeldTask> List_WeldTaskNew;
          List_WeldTaskNew.clear();
          list<WeldTask>::iterator iter1 = List_WeldTask.begin();
          advance(iter1, i);
          WeldTask i_WeldTask1;
          i_WeldTask1 = *iter1;
          Vector3d vec1 = i_WeldTask1.GetPosition(i_WeldTask1.GetTaskContourLength());

          // Derivative of position segment at end point of WeldTask1
          shared_ptr<WeldSegment> i_WeldSegment1;
          i_WeldSegment1 = i_WeldTask1.GetWeldSegment(i_WeldTask1.GetNumberOfWeldSegments() - 1);
          int i_DirectionFlag1 = i_WeldSegment1->GetDirectionFlag();
          PositionSegment i_PositionSegment1 = i_WeldSegment1->GetPositionSegment();
          string i_ContourType1 = i_PositionSegment1.GetContourType();
          Contour* i_Contour1 = i_PositionSegment1.GetContour();
          double i_ContourLength1 = i_Contour1->GetContourLength();
          Vector3d vec1Dir, vec1partB, vec1partA;
          if (i_DirectionFlag1 == 0)
          {
            vec1Dir = i_Contour1->GetContourDirection(i_ContourLength1);
          }
          else if (i_DirectionFlag1 == 1)
          {
            vec1Dir = -(i_Contour1->GetContourDirection(0));
          }
          vec1partB = i_WeldSegment1->GetDirectionPartB(i_ContourLength1);
          vec1partA = i_WeldSegment1->GetDirectionPartA(i_ContourLength1);

          list<WeldTask>::iterator iter2 = List_WeldTask.begin();
          advance(iter2, j);
          WeldTask i_WeldTask2;
          i_WeldTask2 = *iter2;
          Vector3d vec2 = i_WeldTask2.GetPosition(0);

          // Derivative of position segment at start point of WeldTask2
          shared_ptr<WeldSegment> i_WeldSegment2;
          i_WeldSegment2 = i_WeldTask2.GetWeldSegment(0);
          int i_DirectionFlag2 = i_WeldSegment2->GetDirectionFlag();
          PositionSegment i_PositionSegment2 = i_WeldSegment2->GetPositionSegment();
          string i_ContourType2 = i_PositionSegment2.GetContourType();
          Contour* i_Contour2 = i_PositionSegment2.GetContour();
          double i_ContourLength2 = i_Contour2->GetContourLength();
          Vector3d vec2Dir, vec2partB, vec2partA;
          if (i_DirectionFlag2 == 0)
          {
            vec2Dir = i_Contour2->GetContourDirection(0);
          }
          else if (i_DirectionFlag2 == 1)
          {
            vec2Dir = -(i_Contour2->GetContourDirection(i_ContourLength2));
          }
          vec2partB = i_WeldSegment2->GetDirectionPartB(0);
          vec2partA = i_WeldSegment2->GetDirectionPartA(0);

          bool compDir12 = CompareDirections(vec1Dir, vec2Dir, ToleranceDirections);

          // if (ComparePositions(vec1, vec2, TolerancePositions) == true &&  CompareDirections(vec1Dir, vec2Dir,
          // ToleranceDirections)==true)
          if (ComparePositions(vec1, vec2, TolerancePositions) == true)
          {
            Group = true;
            compDir12 = CompareDirections(vec1Dir, vec2Dir, ToleranceDirections);
            for (int l = 0; l < j; l++)
            {
              if (l != i)
              {
                list<WeldTask>::iterator iter3 = List_WeldTask.begin();
                advance(iter3, l);
                WeldTask i_WeldTask3;
                i_WeldTask3 = *iter3;
                List_WeldTaskNew.push_back(i_WeldTask3);
              }
            }

            WeldTask i_WeldTask12;

            for (int k = 0; k < i_WeldTask1.GetNumberOfWeldSegments(); k++)
            {
              list<shared_ptr<WeldSegment>> i_List_WeldSegment = i_WeldTask1.GetList_WeldSegment();
              list<shared_ptr<WeldSegment>>::iterator iterCont1 = i_List_WeldSegment.begin();
              advance(iterCont1, k);
              shared_ptr<WeldSegment> i_WeldSegment;
              i_WeldSegment = *iterCont1;
              i_WeldTask12.AddWeldSegment(i_WeldSegment);
            }
            for (int k = 0; k < i_WeldTask2.GetNumberOfWeldSegments(); k++)
            {
              list<shared_ptr<WeldSegment>> i_List_WeldSegment = i_WeldTask2.GetList_WeldSegment();
              list<shared_ptr<WeldSegment>>::iterator iterCont2 = i_List_WeldSegment.begin();
              advance(iterCont2, k);
              shared_ptr<WeldSegment> i_WeldSegment;
              i_WeldSegment = *iterCont2;
              i_WeldTask12.AddWeldSegment(i_WeldSegment);
            }
            List_WeldTaskNew.push_back(i_WeldTask12);

            for (int m = j + 1; m < List_WeldTask.size(); m++)
            {
              if (m != i)
              {
                list<WeldTask>::iterator iter3 = List_WeldTask.begin();
                advance(iter3, m);
                WeldTask i_WeldTask3;
                i_WeldTask3 = *iter3;
                List_WeldTaskNew.push_back(i_WeldTask3);
              }
            }
            List_WeldTask = List_WeldTaskNew;
            j = -1;
            WriteXML("D:\\Software\\WeldRob3D\\20150827_"
                     "WeldRob3D\\AppOccTest\\TaskDefinition_OCC_"
                     "Iterations.xml");
          }
        }
      }
      if (Group == true)
      {
        i = -1;
        count++;
      }
    }

    // Case2: Compare end point of vector1 with end point of vector2
    List_WeldTaskSize = List_WeldTask.size();
    for (int i = 0; i < List_WeldTask.size(); i++)
    {
      bool Group = false;

      int temp;
      int count = 0;

      for (int j = 0; j < List_WeldTask.size(); j++)
      {
        if (i < List_WeldTask.size() && i != j)
        {
          temp = List_WeldTask.size();
          list<WeldTask> List_WeldTaskNew;
          List_WeldTaskNew.clear();
          list<WeldTask>::iterator iter1 = List_WeldTask.begin();
          advance(iter1, i);
          WeldTask i_WeldTask1;
          i_WeldTask1 = *iter1;
          Vector3d vec1 = i_WeldTask1.GetPosition(i_WeldTask1.GetTaskContourLength());

          // Derivative of position segment at end point of WeldTask1
          shared_ptr<WeldSegment> i_WeldSegment1;
          i_WeldSegment1 = i_WeldTask1.GetWeldSegment(i_WeldTask1.GetNumberOfWeldSegments() - 1);
          int i_DirectionFlag1 = i_WeldSegment1->GetDirectionFlag();
          PositionSegment i_PositionSegment1 = i_WeldSegment1->GetPositionSegment();
          string i_ContourType1 = i_PositionSegment1.GetContourType();
          Contour* i_Contour1 = i_PositionSegment1.GetContour();
          double i_ContourLength1 = i_Contour1->GetContourLength();
          Vector3d vec1Dir;
          if (i_DirectionFlag1 == 0)
          {
            vec1Dir = i_Contour1->GetContourDirection(i_ContourLength1);
          }
          else if (i_DirectionFlag1 == 1)
          {
            vec1Dir = -(i_Contour1->GetContourDirection(0));
          }

          list<WeldTask>::iterator iter2 = List_WeldTask.begin();
          advance(iter2, j);
          WeldTask i_WeldTask2;
          i_WeldTask2 = *iter2;
          Vector3d vec2 = i_WeldTask2.GetPosition(i_WeldTask2.GetTaskContourLength());

          // Derivative of position segment at end point of WeldTask2
          shared_ptr<WeldSegment> i_WeldSegment2;
          i_WeldSegment2 = i_WeldTask2.GetWeldSegment(i_WeldTask2.GetNumberOfWeldSegments() - 1);
          int i_DirectionFlag2 = i_WeldSegment2->GetDirectionFlag();
          PositionSegment i_PositionSegment2 = i_WeldSegment2->GetPositionSegment();
          string i_ContourType2 = i_PositionSegment2.GetContourType();
          Contour* i_Contour2 = i_PositionSegment2.GetContour();
          double i_ContourLength2 = i_Contour2->GetContourLength();
          Vector3d vec2Dir;
          if (i_DirectionFlag2 == 0)
          {
            vec2Dir = -i_Contour2->GetContourDirection(i_ContourLength2);
          }
          else if (i_DirectionFlag2 == 1)
          {
            vec2Dir = i_Contour2->GetContourDirection(0);
          }

          bool tempB = CompareDirections(vec1Dir, vec2Dir, ToleranceDirections);

          // if (ComparePositions(vec1, vec2, TolerancePositions) == true &&  CompareDirections(vec1Dir, vec2Dir,
          // ToleranceDirections)==true)
          if (ComparePositions(vec1, vec2, TolerancePositions) == true)
          {
            Group = true;
            for (int l = 0; l < j; l++)
            {
              if (l != i)
              {
                list<WeldTask>::iterator iter3 = List_WeldTask.begin();
                advance(iter3, l);
                WeldTask i_WeldTask3;
                i_WeldTask3 = *iter3;
                List_WeldTaskNew.push_back(i_WeldTask3);
              }
            }

            WeldTask i_WeldTask12;
            for (int k = 0; k < i_WeldTask1.GetNumberOfWeldSegments(); k++)
            {
              list<shared_ptr<WeldSegment>> i_List_WeldSegment = i_WeldTask1.GetList_WeldSegment();
              list<shared_ptr<WeldSegment>>::iterator iterCont1 = i_List_WeldSegment.begin();
              advance(iterCont1, k);
              shared_ptr<WeldSegment> i_WeldSegment;
              i_WeldSegment = *iterCont1;
              i_WeldTask12.AddWeldSegment(i_WeldSegment);
            }
            i_WeldTask2.InvertWeldTask();
            for (int k = 0; k < i_WeldTask2.GetNumberOfWeldSegments(); k++)
            {
              list<shared_ptr<WeldSegment>> i_List_WeldSegment = i_WeldTask2.GetList_WeldSegment();
              list<shared_ptr<WeldSegment>>::iterator iterCont2 = i_List_WeldSegment.begin();
              advance(iterCont2, k);
              shared_ptr<WeldSegment> i_WeldSegment;
              i_WeldSegment = *iterCont2;
              i_WeldTask12.AddWeldSegment(i_WeldSegment);
            }
            List_WeldTaskNew.push_back(i_WeldTask12);

            for (int m = j + 1; m < List_WeldTask.size(); m++)
            {
              if (m != i)
              {
                list<WeldTask>::iterator iter3 = List_WeldTask.begin();
                advance(iter3, m);
                WeldTask i_WeldTask3;
                i_WeldTask3 = *iter3;
                List_WeldTaskNew.push_back(i_WeldTask3);
              }
            }
            List_WeldTask = List_WeldTaskNew;
            j = -1;
            WriteXML("D:\\Software\\WeldRob3D\\20150827_"
                     "WeldRob3D\\AppOccTest\\TaskDefinition_OCC_"
                     "Iterations.xml");
          }
        }
      }
      if (Group == true)
      {
        i = -1;
        count++;
      }
    }
    // Case3: Compare start point of vector1 with start point of vector2
    List_WeldTaskSize = List_WeldTask.size();
    for (int i = 0; i < List_WeldTask.size(); i++)
    {
      bool Group = false;

      int temp;
      int count = 0;

      for (int j = 0; j < List_WeldTask.size(); j++)
      {
        if (i < List_WeldTask.size() && i != j)
        {
          temp = List_WeldTask.size();
          list<WeldTask> List_WeldTaskNew;
          List_WeldTaskNew.clear();
          list<WeldTask>::iterator iter1 = List_WeldTask.begin();
          advance(iter1, i);
          WeldTask i_WeldTask1;
          i_WeldTask1 = *iter1;
          Vector3d vec1 = i_WeldTask1.GetPosition(0);

          // Derivative of position segment at end point of WeldTask1
          shared_ptr<WeldSegment> i_WeldSegment1;
          i_WeldSegment1 = i_WeldTask1.GetWeldSegment(0);
          int i_DirectionFlag1 = i_WeldSegment1->GetDirectionFlag();
          PositionSegment i_PositionSegment1 = i_WeldSegment1->GetPositionSegment();
          string i_ContourType1 = i_PositionSegment1.GetContourType();
          Contour* i_Contour1 = i_PositionSegment1.GetContour();
          double i_ContourLength1 = i_Contour1->GetContourLength();
          Vector3d vec1Dir;
          if (i_DirectionFlag1 == 0)
          {
            vec1Dir = i_Contour1->GetContourDirection(0);
          }
          else if (i_DirectionFlag1 == 1)
          {
            vec1Dir = -(i_Contour1->GetContourDirection(i_ContourLength1));
          }

          list<WeldTask>::iterator iter2 = List_WeldTask.begin();
          advance(iter2, j);
          WeldTask i_WeldTask2;
          i_WeldTask2 = *iter2;
          Vector3d vec2 = i_WeldTask2.GetPosition(0);

          // Derivative of position segment at end point of WeldTask2
          shared_ptr<WeldSegment> i_WeldSegment2;
          i_WeldSegment2 = i_WeldTask2.GetWeldSegment(0);
          int i_DirectionFlag2 = i_WeldSegment2->GetDirectionFlag();
          PositionSegment i_PositionSegment2 = i_WeldSegment2->GetPositionSegment();
          string i_ContourType2 = i_PositionSegment2.GetContourType();
          Contour* i_Contour2 = i_PositionSegment2.GetContour();
          double i_ContourLength2 = i_Contour2->GetContourLength();
          Vector3d vec2Dir;
          if (i_DirectionFlag2 == 0)
          {
            vec2Dir = -i_Contour2->GetContourDirection(0);
          }
          else if (i_DirectionFlag2 == 1)
          {
            vec2Dir = i_Contour2->GetContourDirection(i_ContourLength2);
          }

          bool tempB = CompareDirections(vec1Dir, vec2Dir, ToleranceDirections);

          // if (ComparePositions(vec1, vec2, TolerancePositions) == true &&  CompareDirections(vec1Dir, vec2Dir,
          // ToleranceDirections)==true)
          if (ComparePositions(vec1, vec2, TolerancePositions) == true)
          {
            Group = true;
            for (int l = 0; l < j; l++)
            {
              if (l != i)
              {
                list<WeldTask>::iterator iter3 = List_WeldTask.begin();
                advance(iter3, l);
                WeldTask i_WeldTask3;
                i_WeldTask3 = *iter3;
                List_WeldTaskNew.push_back(i_WeldTask3);
              }
            }

            WeldTask i_WeldTask12;
            for (int k = 0; k < i_WeldTask1.GetNumberOfWeldSegments(); k++)
            {
              list<shared_ptr<WeldSegment>> i_List_WeldSegment = i_WeldTask1.GetList_WeldSegment();
              list<shared_ptr<WeldSegment>>::iterator iterCont1 = i_List_WeldSegment.begin();
              advance(iterCont1, k);
              shared_ptr<WeldSegment> i_WeldSegment;
              i_WeldSegment = *iterCont1;
              i_WeldTask12.AddWeldSegment(i_WeldSegment);
            }
            i_WeldTask2.InvertWeldTask();
            for (int k = 0; k < i_WeldTask2.GetNumberOfWeldSegments(); k++)
            {
              list<shared_ptr<WeldSegment>> i_List_WeldSegment = i_WeldTask2.GetList_WeldSegment();
              list<shared_ptr<WeldSegment>>::iterator iterCont2 = i_List_WeldSegment.begin();
              advance(iterCont2, k);
              shared_ptr<WeldSegment> i_WeldSegment;
              i_WeldSegment = *iterCont2;
              i_WeldTask12.AddWeldSegment(i_WeldSegment);
            }
            List_WeldTaskNew.push_back(i_WeldTask12);

            for (int m = j + 1; m < List_WeldTask.size(); m++)
            {
              if (m != i)
              {
                list<WeldTask>::iterator iter3 = List_WeldTask.begin();
                advance(iter3, m);
                WeldTask i_WeldTask3;
                i_WeldTask3 = *iter3;
                List_WeldTaskNew.push_back(i_WeldTask3);
              }
            }
            List_WeldTask = List_WeldTaskNew;
            j = -1;
            WriteXML("D:\\Software\\WeldRob3D\\20150827_"
                     "WeldRob3D\\AppOccTest\\TaskDefinition_OCC_"
                     "Iterations.xml");
          }
        }
      }
      if (Group == true)
      {
        i = -1;
        count++;
      }
    }
  }
  // Renew IDs
  list<WeldTask> List_WeldTaskNew;
  List_WeldTaskNew.clear();
  int WeldSegmentIDNew = 0;
  for (int i = 0; i < List_WeldTask.size(); i++)
  {
    // Renew IDs of WeldTasks
    list<WeldTask>::iterator iter1 = List_WeldTask.begin();
    advance(iter1, i);
    WeldTask i_WeldTask;
    i_WeldTask = *iter1;
    i_WeldTask.SetTaskID(i);

    /*//Renew IDs of WeldTasks
    list<FilletWeldSegment> List_FilletWeld
        = i_WeldTask.GetList_FilletWeld();
    list<FilletWeldSegment> List_FilletWeldNew;
    List_FilletWeldNew.clear();
    for (int j = 0; j < List_FilletWeld.size(); j++) {
        list<FilletWeldSegment>::iterator iter2
            = List_FilletWeld.begin();
        advance(iter2, j);
        FilletWeldSegment i_FilletWeldSegment;
        i_FilletWeldSegment = *iter2;
        i_FilletWeldSegment.SetWeldSegmentID(
            FilletWeldSegmentIDNew);
        List_FilletWeldNew.push_back(i_FilletWeldSegment);
        FilletWeldSegmentIDNew++;
    }
    i_WeldTask.SetList_FilletWeld(List_FilletWeldNew);*/
    // Renew IDs of WeldTasks
    list<shared_ptr<WeldSegment>> List_WeldSegment = i_WeldTask.GetList_WeldSegment();
    list<shared_ptr<WeldSegment>> List_WeldSegmentNew;
    List_WeldSegmentNew.clear();
    for (int j = 0; j < List_WeldSegment.size(); j++)
    {
      list<shared_ptr<WeldSegment>>::iterator iter2 = List_WeldSegment.begin();
      advance(iter2, j);
      shared_ptr<WeldSegment> i_WeldSegment;
      i_WeldSegment = *iter2;
      i_WeldSegment->SetWeldSegmentID(WeldSegmentIDNew);
      List_WeldSegmentNew.push_back(i_WeldSegment);
      WeldSegmentIDNew++;
    }
    i_WeldTask.SetList_WeldSegment(List_WeldSegmentNew);
    List_WeldTaskNew.push_back(i_WeldTask);
  }
  List_WeldTask = List_WeldTaskNew;
};

bool TaskList::ComparePositions(Vector3d Vector1, Vector3d Vector2, double Tolerance)
{
  Vector3d VectorTemp = Vector1 - Vector2;
  double s = VectorTemp.norm();
  if (s < Tolerance)
  {
    return true;
  }
  else
  {
    return false;
  }
};
bool TaskList::CompareDirections(Vector3d Vector1, Vector3d Vector2, double Tolerance)
{
  // Tolerance in degree
  double Dot_Tolerance = 0.000001;
  double i_Dot = Vector1.dot(Vector2);
  if (i_Dot > 1 && (i_Dot - 1) < Dot_Tolerance)
  {
    i_Dot = 1;
  }
  double angle = abs(acos(i_Dot) * 360 / (2 * M_PI));
  if (angle < Tolerance)
  {
    return true;
  }
  else
  {
    return false;
  }
};

void TaskList::WriteXML(string i_filename)
{
  TiXmlDocument doc;
  TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
  doc.LinkEndChild(decl);
  TiXmlElement* root = new TiXmlElement("Taskdefinition");
  doc.LinkEndChild(root);
  TiXmlElement* XmlNodeTasks = new TiXmlElement("Tasks");
  root->LinkEndChild(XmlNodeTasks);
  // loop for all WeldTask
  for (int j = 0; j < List_WeldTask.size(); j++)
  {
    list<WeldTask>::iterator i = List_WeldTask.begin();
    advance(i, j);
    WeldTask i_WeldTask;
    i_WeldTask = *i;
    i_WeldTask.WriteXML(XmlNodeTasks, j);
  }

  TiXmlElement* XmlNodeTaskSequence = new TiXmlElement("TaskSequence");
  root->LinkEndChild(XmlNodeTaskSequence);

  // int TaskIDtest =2;

  // const char temp = intToXmlChar(TaskSequence.size());

  for (int k = 0; k < TaskSequence.size(); k++)
  {
    list<TaskID>::iterator iter = TaskSequence.begin();
    advance(iter, k);
    TiXmlElement* XmlNodeID = new TiXmlElement("Task");
    XmlNodeTaskSequence->LinkEndChild(XmlNodeID);
    XmlNodeID->SetAttribute("WeldTaskID", iter->WeldTaskID);
    XmlNodeID->SetAttribute("SubTaskID", iter->SubTaskID);
  }
  const char* i_filename_char = i_filename.c_str();
  doc.SaveFile(i_filename_char);
};

void TaskList::ReadXML(string i_filename)
{
  const char* i_filename_char = i_filename.c_str();
  TiXmlDocument doc(i_filename_char);
  bool loadOkay = doc.LoadFile();
  if (loadOkay)
  {
    printf("\n%s:\n", i_filename_char);
  }
  else
  {
    printf("Failed to load file \"%s\"\n", i_filename_char);
  }

  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);
  pElem = doc.FirstChildElement();

  if (pElem == NULL)
  {
    cerr << "Failed to load file: No root element." << endl;
    doc.Clear();
    return;
  }

  // Go through Tasks
  TiXmlElement* pElem1 = pElem->FirstChildElement("Tasks");
  {
    // Go through all WeldTask
    for (TiXmlElement* pElem2 = pElem1->FirstChildElement("WeldTask"); pElem2 != NULL;
         pElem2 = pElem2->NextSiblingElement("WeldTask"))
    {
      WeldTask i_WeldTask;
      i_WeldTask.ReadXML(pElem2);
      /// WeldTask *i_WeldTask_Ptr = &i_WeldTask;
      AddWeldTask(&i_WeldTask);
    }
  }

  // Read TaskSequence
  TiXmlElement* pElemSquence = pElem->FirstChildElement("TaskSequence");
  {
    TaskSequence.clear();
    for (TiXmlElement* pElemAT = pElemSquence->FirstChildElement(); pElemAT != NULL;
         pElemAT = pElemAT->NextSiblingElement())
    {
      int ID1, ID2;
      pElemAT->QueryIntAttribute("WeldTaskID", &ID1);
      pElemAT->QueryIntAttribute("SubTaskID", &ID2);
      TaskID taskID;
      taskID.WeldTaskID = ID1;
      taskID.SubTaskID = ID2;
      TaskSequence.push_back(taskID);
    }
  }
};

void TaskList::SetTaskSequenceDefault()
{
  TaskSequence.clear();
  for (int j = 0; j < List_WeldTask.size(); j++)
  {
    list<WeldTask>::iterator i = List_WeldTask.begin();
    advance(i, j);
    WeldTask i_WeldTask;
    i_WeldTask = *i;
    int ID1 = i_WeldTask.TaskID;
    int NumOfSubTasks = i->GetNumberOfSubTasks();
    for (int k = 0; k < NumOfSubTasks; k++)
    {
      list<SubTask>::iterator iterSubTask = i->List_SubTask.begin();
      advance(iterSubTask, k);
      TaskID taskID;
      taskID.WeldTaskID = ID1;
      taskID.SubTaskID = iterSubTask->SubTaskID;
      TaskSequence.push_back(taskID);
    }
  }
};

void TaskList::AddTaskSequenceElement(int WeldTaskID, int SubTaskID)
{
  if (WeldTaskID < this->GetNumberOfWeldTasks() && SubTaskID < this->GetNumberOfSubTasks(WeldTaskID))
  {
    TaskID taskID;
    taskID.WeldTaskID = WeldTaskID;
    taskID.SubTaskID = SubTaskID;
    this->TaskSequence.push_back(taskID);
  }
};

void TaskList::GetTaskSequenceElement(int ElementNumber, int& WeldTaskID, int& SubTaskID)
{
  if (ElementNumber < this->TaskSequence.size())
  {
    list<TaskID>::iterator iter = this->TaskSequence.begin();
    advance(iter, ElementNumber);
    WeldTaskID = iter->WeldTaskID;
    SubTaskID = iter->SubTaskID;
  }
  else
  {
    WeldTaskID = -1;
    SubTaskID = -1;
  }
}

int TaskList::GetNumberOfTaskSequenceElement()
{
  return this->TaskSequence.size();
}

int TaskList::GetTaskSequenceWeldTaskID(int ElementNumber)
{
  int ID = -1;
  if (ElementNumber < this->TaskSequence.size())
  {
    list<TaskID>::iterator iter = this->TaskSequence.begin();
    advance(iter, ElementNumber);
    ID = iter->WeldTaskID;
  }
  return ID;
}

int TaskList::GetTaskSequenceSubTaskID(int ElementNumber)
{
  int ID = -1;
  if (ElementNumber < this->TaskSequence.size())
  {
    list<TaskID>::iterator iter = this->TaskSequence.begin();
    advance(iter, ElementNumber);
    ID = iter->SubTaskID;
  }
  return ID;
}

void TaskList::SetSubTaskOfWeldTasksDefault()
{
  for (int i = 0; i < this->List_WeldTask.size(); i++)
  {
    list<WeldTask>::iterator iter = this->List_WeldTask.begin();
    advance(iter, i);
    iter->SetSubTaskDefault();
  }
};

void TaskList::AddSubTaskInWeldTask(int WeldTaskID, int SubTaskID, double StartLength, double EndLength,
                                    bool IsReversed, double RotWire)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  iter->AddSubTask(SubTaskID, StartLength, EndLength, IsReversed, RotWire);
};

void TaskList::DeleteSubTaskInWeldTask(int WeldTaskID, int SubTaskID)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  iter->DeleteSubTask(SubTaskID);
};

void TaskList::DeleteAllSubTaskInWeldTask(int WeldTaskID)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  iter->List_SubTask.clear();
};

int TaskList::GetNumberOfSubTasks(int WeldTaskID)
{
  int Num;
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  Num = iter->GetNumberOfSubTasks();
  return Num;
};

Vector3d TaskList::GetPositionOfSubTask(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  bool IsReversed = iter->GetReversedFlagOfSubTask(SubTaskID);
  double ForwardLength;
  if (IsReversed)
    ForwardLength = iter->GetEndLengthOfSubTask(SubTaskID);
  else
    ForwardLength = iter->GetStartLengthOfSubTask(SubTaskID);
  double SumLength = ForwardLength + DistanceFromSubTaskStartPoint;
  double FullLengthOfTask = iter->GetTaskContourLength();
  SumLength = (SumLength > FullLengthOfTask) ? FullLengthOfTask : SumLength;
  Vector3d Position = this->GetPositionOfWeldTask(WeldTaskID, SumLength);
  return Position;
};

double TaskList::GetSmallestAngleOfSubTask(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  bool IsReversed = iter->GetReversedFlagOfSubTask(SubTaskID);
  double ForwardLength;
  if (IsReversed)
    ForwardLength = iter->GetEndLengthOfSubTask(SubTaskID);
  else
    ForwardLength = iter->GetStartLengthOfSubTask(SubTaskID);
  double SumLength = ForwardLength + DistanceFromSubTaskStartPoint;
  double FullLengthOfTask = iter->GetTaskContourLength();
  SumLength = (SumLength > FullLengthOfTask) ? FullLengthOfTask : SumLength;
  double Angle = this->GetSmallestAngle(WeldTaskID, SumLength);
  return Angle;
};

MatrixXd TaskList::GetManufacturingCoordinateSystemOfSubTask(int WeldTaskID, int SubTaskID,
                                                             double DistanceFromSubTaskStartPoint)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  bool IsReversed = iter->GetReversedFlagOfSubTask(SubTaskID);
  double ForwardLength;
  if (IsReversed)
    ForwardLength = iter->GetEndLengthOfSubTask(SubTaskID);
  else
    ForwardLength = iter->GetStartLengthOfSubTask(SubTaskID);
  double SumLength = ForwardLength + DistanceFromSubTaskStartPoint;
  double FullLengthOfTask = iter->GetTaskContourLength();
  SumLength = (SumLength > FullLengthOfTask) ? FullLengthOfTask : SumLength;
  MatrixXd CS = this->GetManufacturingCoordinateSystem(WeldTaskID, SumLength);
  return CS;
};

double TaskList::GetGapLengthOfSubTask(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  bool IsReversed = iter->GetReversedFlagOfSubTask(SubTaskID);
  double ForwardLength;
  if (IsReversed)
    ForwardLength = iter->GetEndLengthOfSubTask(SubTaskID);
  else
    ForwardLength = iter->GetStartLengthOfSubTask(SubTaskID);
  double SumLength = ForwardLength + DistanceFromSubTaskStartPoint;
  double FullLengthOfTask = iter->GetTaskContourLength();
  SumLength = (SumLength > FullLengthOfTask) ? FullLengthOfTask : SumLength;
  double Length = this->GetGapLengthOfWeldTask(WeldTaskID, SumLength);
  return Length;
};

double TaskList::GetStartLengthOfSubTask(int WeldTaskID, int SubTaskID)
{
  double L;
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  L = iter->GetStartLengthOfSubTask(SubTaskID);
  return L;
}

double TaskList::GetlengthOfSubTask(int WeldTaskID, int SubTaskID)
{
  double L, LStart, LEnd;
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  if (iter->GetReversedFlagOfSubTask(SubTaskID))
  {
    LStart = iter->GetEndLengthOfSubTask(SubTaskID);
    LEnd = iter->GetStartLengthOfSubTask(SubTaskID);
  }
  else
  {
    LEnd = iter->GetEndLengthOfSubTask(SubTaskID);
    LStart = iter->GetStartLengthOfSubTask(SubTaskID);
  }
  if (LEnd <= LStart)
    L = iter->GetTaskContourLength() - LStart + LEnd;
  else
    L = LEnd - LStart;
  return L;
}

double TaskList::GetEndLengthOfSubTask(int WeldTaskID, int SubTaskID)
{
  double L;
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  L = iter->GetEndLengthOfSubTask(SubTaskID);
  return L;
}

bool TaskList::GetReversedFlagOfSubtask(int WeldTaskID, int SubTaskID)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  bool flag = iter->GetReversedFlagOfSubTask(SubTaskID);
  return flag;
}

int TaskList::GetWeldSegmentIDOfSubTaskFromStartPoint(int WeldTaskID, int SubTaskID,
                                                      double DistanceFromSubTaskStartPoint)
{
  list<WeldTask>::iterator iter = this->List_WeldTask.begin();
  advance(iter, WeldTaskID);
  double ForwardLength = iter->GetStartLengthOfSubTask(SubTaskID);
  double SumLength = ForwardLength + DistanceFromSubTaskStartPoint;
  double FullLengthOfTask = iter->GetTaskContourLength();
  if (SumLength > FullLengthOfTask)
    SumLength = SumLength - FullLengthOfTask;
  double tempLength = 0;
  int NumContour = iter->GetNumberOfWeldSegments();
  int ID = NumContour;
  for (int i = 0; i < NumContour; i++)
  {
    tempLength += iter->GetWeldSegmentLength(i);
    if (tempLength >= SumLength)
    {
      ID = i;
      break;
    }
  }
  return ID;
};

/* discretize the subtask with a given max DiscretizationLength
   e.g. for WeldTask (FilletWeld0 (ID=0, length=100, type:linear); FilletWeld1 (ID=1, length:15, type:cycle); FilletWeld2
   (ID=2, length:8, type:spline)), which has SubTask(StartLength=50, EndLength=128), given MaxDiscretizationLength == 5,
   return the discrete results in a vector: [50, -0, 5, 5, 5, -1, 4, 4, -2], the negative number is the ID of FilletWeld */
vector<double> TaskList::GetDiscretizationLengthListOfSubTask(int WeldTaskID, int SubTaskID,
                                                              double MaxDiscretizationLength)
{
  vector<double> LengthList;
  double StartLength = this->GetStartLengthOfSubTask(WeldTaskID, SubTaskID);
  double EndLength = this->GetEndLengthOfSubTask(WeldTaskID, SubTaskID);

  // for case open seam
  if (StartLength < EndLength)
  {
    double SubTaskLength = this->GetlengthOfSubTask(WeldTaskID, SubTaskID);
    int StartID = this->GetWeldSegmentIDOfSubTaskFromStartPoint(
        WeldTaskID, SubTaskID,
        0.1);  // 0.1 not 0, for avoiding intersection point between two FilletWeld
    int EndID =
        this->GetWeldSegmentIDOfSubTaskFromStartPoint(WeldTaskID, SubTaskID, SubTaskLength - 0.1);  // same as above
    double Discri = 0;
    int Num = 0;
    string type;

    // for case: SubTask only distributes in one FilletWeld
    if (StartID == EndID)
    {
      type = this->GetContourTypeOfPositionSegment(WeldTaskID, StartID);
      if (type == "LinearContour")
      {
        LengthList.push_back(SubTaskLength);
      }
      else
      {
        // make DiscritizationLength <= MaxDiscretizationLength
        Num = ceil(SubTaskLength / MaxDiscretizationLength);
        Discri = SubTaskLength / Num;
        for (int i = 0; i < Num; i++)
        {
          LengthList.push_back(Discri);
        }
      }
      LengthList.push_back(-StartID);
      return LengthList;
    }

    // for case: SubTask only distributes in several FilletWelds
    double tempLength = 0;
    for (int ID = 0; ID <= StartID; ID++)
    {
      tempLength += this->GetWeldSegmentLength(WeldTaskID, ID);
    }

    double FirstLength = tempLength - StartLength;
    type = this->GetContourTypeOfPositionSegment(WeldTaskID, StartID);
    if (type == "LinearContour")
    {
      LengthList.push_back(FirstLength);
    }
    else
    {
      Num = ceil(FirstLength / MaxDiscretizationLength);
      Discri = FirstLength / Num;
      for (int i = 0; i < Num; i++)
      {
        LengthList.push_back(Discri);
      }
    }
    LengthList.push_back(-StartID);

    for (int ID = StartID + 1; ID < EndID; ID++)
    {
      type = this->GetContourTypeOfPositionSegment(WeldTaskID, ID);
      double ContourLength = this->GetWeldSegmentLength(WeldTaskID, ID);
      tempLength += ContourLength;
      if (type == "LinearContour")
        LengthList.push_back(ContourLength);
      else
      {
        Num = ceil(ContourLength / MaxDiscretizationLength);
        Discri = ContourLength / Num;
        for (int i = 0; i < Num; i++)
        {
          LengthList.push_back(Discri);
        }
      }
      LengthList.push_back(-ID);
    }

    double LastLength = StartLength + SubTaskLength - tempLength;
    type = this->GetContourTypeOfPositionSegment(WeldTaskID, EndID);
    if (type == "LinearContour")
      LengthList.push_back(LastLength);
    else
    {
      Num = ceil(LastLength / MaxDiscretizationLength);
      Discri = LastLength / Num;
      for (int i = 0; i < Num; i++)
      {
        LengthList.push_back(Discri);
      }
    }
    LengthList.push_back(-EndID);
    return LengthList;
  }
  // for case closed seam
  else
  {
    double SubTaskLength = this->GetlengthOfSubTask(WeldTaskID, SubTaskID);
    int StartID = this->GetWeldSegmentIDOfSubTaskFromStartPoint(WeldTaskID, SubTaskID, 0.);
    int LastID = this->GetNumberOfWeldSegments(WeldTaskID) - 1;
    int EndID = this->GetWeldSegmentIDOfSubTaskFromStartPoint(WeldTaskID, SubTaskID, SubTaskLength);
    double Discri = 0;
    int Num = 0;
    string type;

    double tempLength = 0;
    for (int ID = 0; ID <= StartID; ID++)
    {
      tempLength += this->GetWeldSegmentLength(WeldTaskID, ID);
    }

    double FirstLength = tempLength - StartLength;
    type = this->GetContourTypeOfPositionSegment(WeldTaskID, StartID);
    if (type == "LinearContour")
      LengthList.push_back(FirstLength);
    else
    {
      Num = ceil(FirstLength / MaxDiscretizationLength);
      Discri = FirstLength / Num;
      for (int i = 0; i < Num; i++)
      {
        LengthList.push_back(Discri);
      }
    }
    LengthList.push_back(-StartID);

    for (int ID = StartID + 1; ID <= LastID; ID++)
    {
      type = this->GetContourTypeOfPositionSegment(WeldTaskID, ID);
      double ContourLength = this->GetWeldSegmentLength(WeldTaskID, ID);
      if (type == "LinearContour")
        LengthList.push_back(ContourLength);
      else
      {
        Num = ceil(ContourLength / MaxDiscretizationLength);
        Discri = ContourLength / Num;
        for (int i = 0; i < Num; i++)
        {
          LengthList.push_back(Discri);
        }
      }
      LengthList.push_back(-ID);
    }

    tempLength = 0;
    for (int ID = 0; ID < EndID; ID++)
    {
      type = this->GetContourTypeOfPositionSegment(WeldTaskID, ID);
      double ContourLength = this->GetWeldSegmentLength(WeldTaskID, ID);
      tempLength += ContourLength;
      if (type == "LinearContour")
        LengthList.push_back(ContourLength);
      else
      {
        Num = ceil(ContourLength / MaxDiscretizationLength);
        Discri = ContourLength / Num;
        for (int i = 0; i < Num; i++)
        {
          LengthList.push_back(Discri);
        }
      }
      LengthList.push_back(-ID);
    }

    double LastLength = EndLength - tempLength;
    type = this->GetContourTypeOfPositionSegment(WeldTaskID, EndID);
    if (type == "LinearContour")
      LengthList.push_back(LastLength);
    else
    {
      Num = ceil(LastLength / MaxDiscretizationLength);
      Discri = LastLength / Num;
      for (int i = 0; i < Num; i++)
      {
        LengthList.push_back(Discri);
      }
    }
    LengthList.push_back(-EndID);
    return LengthList;
  }
};

int TaskList::GetTaskState(int WeldTaskID)
{
  WeldTask i_WeldTask;
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  i_WeldTask = *i;
  return i_WeldTask.GetState();
}

void TaskList::SetTaskState(int WeldTaskID, int TaskState)
{
  WeldTask i_WeldTask;
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  i_WeldTask = *i;
  i_WeldTask.SetState(TaskState);
  List_WeldTask.insert(i, i_WeldTask);
  List_WeldTask.erase(i);
}

/*	Check whether a point on the WeldTask with a distance threshold,
  return the length from the corresponding point to the start point
  of the WeldTask, if the point is not on it, return -1.
  Algorithms: sample points of the WeldTask (e.g. every 0.1 mm),
  calculate its distance to the given point, and then check if it
  less than the threshold
*/
// Version A: adjust the length using the orientation of the point (for intersection point)
bool TaskList::IsPointOnWeldTask(int WeldTaskID, Vector3d Point, Vector3d Orientation, double PositionTreshold,
                                 double& Length)
{
  double FullLength = GetLengthOfWeldTask(WeldTaskID);
  double Precision = 0.1;
  Length = 0;
  double TempLength = Length + Precision;

  Vector3d P0 = Point;
  Vector3d P1 = GetPositionOfWeldTask(WeldTaskID, Length);
  Vector3d D01 = P0 - P1;
  double Distance = D01.norm();

  // exist sampling error, threshold must be larger then precision, for the nominal case, threshold == precision
  PositionTreshold = (Precision > PositionTreshold) ? Precision : PositionTreshold;

  // go through the sampling points
  while (TempLength < (FullLength + Precision))
  {
    if (TempLength > FullLength)
      TempLength = FullLength;
    P1 = GetPositionOfWeldTask(WeldTaskID, TempLength);
    D01 = P0 - P1;
    double Temp = Distance;
    Distance = (Distance < D01.norm()) ? Distance : D01.norm();
    Length = (Distance != Temp) ? TempLength : Length;
    TempLength += Precision;
  }
  Length = (Distance <= PositionTreshold) ? Length : -1;

  // if the given point is on the WeldTask, try to get exacter returned length in a range [Length - precision, Length + precision]
  double angle = 90;
  double OldLength = Length;
  if (OldLength != -1)
  {
    // go through sampling points with distance 0.01 mm
    for (int i = 0; i <= ceil(Precision / 0.01); i++)
    {
      // a range [Length-precision, 0]
      TempLength = OldLength - i * 0.01;
      // guarantee TempLength in a range [0, FullLength]
      if (TempLength > FullLength)
        TempLength = TempLength - FullLength;
      if (TempLength < 0)
        TempLength = FullLength + TempLength;

      // get orientation of sampling points (Orientation = Wall X Base)
      Vector3d Dir = GetDirectionSeamOfWeldTask(WeldTaskID, TempLength);

      // look for the sampling point, which has the minimal difference with the given orientation
      double dot = abs(Dir.dot(Orientation) / (Dir.norm() * Orientation.norm()));
      dot = (dot > 1) ? 1 : dot;
      double degree = acos(dot) * 180 / M_PI;
      double Temp = angle;
      angle = (degree < angle) ? degree : angle;
      Length = (Temp != angle) ? TempLength : Length;

      // a range (0, Length+precision]
      if (i != 0)
      {
        TempLength = OldLength + i * 0.01;
        if (TempLength > FullLength)
          TempLength = TempLength - FullLength;
        if (TempLength < 0)
          TempLength = FullLength + TempLength;
        Dir = GetDirectionSeamOfWeldTask(WeldTaskID, TempLength);
        dot = abs(Dir.dot(Orientation) / (Dir.norm() * Orientation.norm()));
        dot = (dot > 1) ? 1 : dot;
        degree = acos(dot) * 180 / M_PI;
        double Temp = angle;
        angle = (degree < angle) ? degree : angle;
        Length = (Temp != angle) ? TempLength : Length;
      }
    }
  }
  bool isPointOnWeldTask = true;
  if (Length == -1)
    isPointOnWeldTask = false;
  return isPointOnWeldTask;
};

// Version B: without taking point orientation into account
// used to match / compare nominal and deformed CAD TaskDescriptions
bool TaskList::IsPointOnWeldTask(int WeldTaskID, Vector3d Point, double PositionTreshold, double& Length)
{
  double FullLength = GetLengthOfWeldTask(WeldTaskID);
  double Precision = 0.1;  // discretization step
  Length = 0;
  double TempLength = Length + Precision;

  Vector3d P0 = Point;
  Vector3d P1 = GetPositionOfWeldTask(WeldTaskID, Length);
  Vector3d D01 = P0 - P1;
  double Distance = D01.norm();

  PositionTreshold = (Precision > PositionTreshold) ? Precision : PositionTreshold;

  // iterate along seam with iteration step Precision, running variable is TempLength
  // compute distance of Point to current seam position
  while (TempLength < (FullLength + Precision))
  {
    if (TempLength > FullLength)  // last point tested is the end point of seam
      TempLength = FullLength;
    P1 = GetPositionOfWeldTask(WeldTaskID, TempLength);
    D01 = P0 - P1;
    double OldDistance = Distance;
    // Compute distance between closest point of the seam
    // if (Distance < D01.norm()) Distance = D01.norm()
    Distance = (Distance < D01.norm()) ? Distance : D01.norm();
    // if (Distance == OldDistance) Length = TempLength
    Length = (Distance == OldDistance) ? Length : TempLength;
    TempLength += Precision;
  }
  // if minimum distance between Point and seam is too great, assume mismatch
  // set Length == -1 for "no correspondence" between seams
  Length = (Distance <= PositionTreshold) ? Length : -1;
  bool isPointOnWeldTask = true;
  if (Length == -1)
    isPointOnWeldTask = false;
  return isPointOnWeldTask;
};

void TaskList::InvertWeldTask(int WeldTaskID)
{
  WeldTask i_WeldTask;
  list<WeldTask>::iterator i = List_WeldTask.begin();
  advance(i, WeldTaskID);
  i_WeldTask = *i;
  i_WeldTask.InvertWeldTask();
  List_WeldTask.insert(i, i_WeldTask);
  List_WeldTask.erase(i);
};

int TaskList::GetDirectionFlagOfWeldSegment(int WeldTaskID, int WeldSegmentID)
{
  int DirectionFlag;

  WeldTask i_WeldTask;
  list<WeldTask>::iterator iter1 = List_WeldTask.begin();
  advance(iter1, WeldTaskID);
  i_WeldTask = *iter1;

  shared_ptr<WeldSegment> i_WeldSegment;
  list<shared_ptr<WeldSegment>> List_WeldSegment = i_WeldTask.List_WeldSegment;
  list<shared_ptr<WeldSegment>>::iterator iter2 = List_WeldSegment.begin();
  advance(iter2, WeldSegmentID);
  i_WeldSegment = *iter2;

  DirectionFlag = i_WeldSegment->GetDirectionFlag();
  return DirectionFlag;
};

shared_ptr<WeldSegment> TaskList::GetWeldSegment(int WeldTaskID, int WeldSegmentID)
{
  WeldTask i_WeldTask;
  list<WeldTask>::iterator iter1 = List_WeldTask.begin();
  advance(iter1, WeldTaskID);
  i_WeldTask = *iter1;
  return iter1->GetWeldSegment(WeldSegmentID);
};

TaskList IdentifyWeldSegmentTypes(list<shared_ptr<FilletWeldSegment>> List_FilletWelds)
{
  list<WeldTask> list_WeldTasks;
  TaskList identifiedTaskList;
  double tolerance = 0.05;

  // loop through all fillet weld segments
  list<shared_ptr<FilletWeldSegment>>::iterator iter_FilletWeld;
  int IDbuttWeld = 0;
  int IDfilletWeld = 0;
  for (int j = 0; j < List_FilletWelds.size(); j++)
  {
    iter_FilletWeld = List_FilletWelds.begin();
    advance(iter_FilletWeld, j);
    shared_ptr<FilletWeldSegment> filletWeld_A = make_shared<FilletWeldSegment>();
    filletWeld_A = *iter_FilletWeld;

    // compare fillet weld with all other fillet welds and check if
    // segment A and B are SingleBevelButtWeld:
    for (int i = j + 1; i < List_FilletWelds.size(); i++)
    {
      bool is_buttWeldCandidate = false;
      shared_ptr<SingleBevelButtWeldSegment> i_ButtWeldSegment = make_shared<SingleBevelButtWeldSegment>();
      iter_FilletWeld = List_FilletWelds.begin();
      advance(iter_FilletWeld, i);
      shared_ptr<FilletWeldSegment> filletWeld_B = make_shared<FilletWeldSegment>();
      filletWeld_B = *iter_FilletWeld;

      // check if length of segment A and B are similar
      double lengthA = filletWeld_A->GetLengthOfPositionSegment();
      double lengthB = filletWeld_B->GetLengthOfPositionSegment();
      if (lengthA - lengthB > tolerance * lengthA)
        continue;
      else
      {
        // check if base direction vectors oppose each other on the vector AB between the seams:
        // --vec_A_base--> <--vec_B_base--
        // -------vec_AB----------------->
        Vector3d pos_A = filletWeld_A->GetPosition(0);              // start point of A
        Vector3d pos_B_start = filletWeld_B->GetPosition(0);        // start point of B
        Vector3d pos_B_end = filletWeld_B->GetPosition(lengthB);    // end point of B
        Vector3d vecAB_start = (pos_B_start - pos_A).normalized();  // vector between seams
        Vector3d vecAB_end = (pos_B_end - pos_A).normalized();      // vector between seams
        Vector3d vec_A_base = filletWeld_A->GetDirectionPartB(0);   // direction vector of base part of seam A
        Vector3d vec_B_base_start =
            filletWeld_A->GetDirectionPartB(0);  // direction vector of base part of seam B (start point)
        Vector3d vec_B_base_end = filletWeld_B->GetDirectionPartB(lengthB);
        Vector3d vec_A_wall = filletWeld_A->GetDirectionPartA(0);  // direction vector of wall part of seam A
        Vector3d vec_B_wall_start =
            filletWeld_B->GetDirectionPartA(0);  // direction vector of wall part of seam B (start point)
        Vector3d vec_B_wall_end = filletWeld_B->GetDirectionPartA(lengthB);
        if (vecAB_start.isApprox(vec_A_base, tolerance) && vecAB_start.isApprox(-1 * vec_B_base_start, tolerance))
        {
          // check for similar wall directions (should be less than 180deg apart)
          if (acos(vec_A_wall.dot(vec_B_wall_start)) <= M_PI)
            is_buttWeldCandidate = true;
        }
        else if (vecAB_end.isApprox(vec_A_base, tolerance) && vecAB_end.isApprox(-1 * vec_B_base_end, tolerance))
        {
          // check for similar wall directions (should be less than 180deg apart)
          if (acos(vec_A_wall.dot(vec_B_wall_end)) <= M_PI)
          {
            is_buttWeldCandidate = true;
            // invert seam B if it matches to A back to front
            filletWeld_B->Invert();
          }
        }
        if (is_buttWeldCandidate)
        {
          double angle_A = acos(vec_A_base.dot(vec_A_wall));
          double angle_B = acos(vec_B_base_start.dot(vec_B_wall_start));
          if ((angle_A - 0.5 * M_PI) <= tolerance && (angle_B > 0.5 * M_PI + tolerance))
          {
            // A is flat side, B is bevel side of single bevel butt joint
            i_ButtWeldSegment->AddFlatSegment(filletWeld_A.get());
            i_ButtWeldSegment->AddBevelSegment(filletWeld_B.get());
          }
          else if ((angle_B - 0.5 * M_PI) <= tolerance && (angle_A > 0.5 * M_PI + tolerance))
          {
            // B is flat side, A is bevel side of single bevel butt joint
            i_ButtWeldSegment->AddFlatSegment(filletWeld_B.get());
            i_ButtWeldSegment->AddBevelSegment(filletWeld_A.get());
          }
          else
            is_buttWeldCandidate = false;
        }
      }
      // add single bevel butt weld or fillet weld to task list
      if (is_buttWeldCandidate)
      {
        i_ButtWeldSegment->SetWeldSegmentID(IDbuttWeld);
        IDbuttWeld++;
        WeldTask newWeldTask;
        newWeldTask.AddWeldSegment(i_ButtWeldSegment);
        identifiedTaskList.AddWeldTask(&newWeldTask);
        // mark fillet welds that are part of a butt weld
        filletWeld_A->SetWeldSegmentID(-10);
        filletWeld_B->SetWeldSegmentID(-10);
        break;
      }
    }
  }
  // add all single fillet welds to task list
  for (int j = 0; j < List_FilletWelds.size(); j++)
  {
    iter_FilletWeld = List_FilletWelds.begin();
    advance(iter_FilletWeld, j);
    shared_ptr<FilletWeldSegment> filletWeld_A = make_shared<FilletWeldSegment>();
    filletWeld_A = *iter_FilletWeld;
    if (filletWeld_A->GetWeldSegmentID() != -10)
    {
      filletWeld_A->SetWeldSegmentID(IDfilletWeld);
      IDfilletWeld++;
      WeldTask newWeldTask;
      newWeldTask.AddWeldSegment(filletWeld_A);
      identifiedTaskList.AddWeldTask(&newWeldTask);
    }
  }
  return identifiedTaskList;
}
