/**
 *  \file      TaskDefintion.h
 *  \brief     Weld Task Definition
 *  \details   WeldTaskDefintion - Builds on top of the TaskDefintion
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
#include "WeldSegment.h"
#include <fstream>
#include <iostream>
#include <list>
#include <math.h>
#include <sstream>
#include <vector>
#include <memory>

using namespace std;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

class SubTask
{
public:
  SubTask() : SubTaskID(-1), StartLength(-1), EndLength(-1), IsReversed(0), RotWire(0.){};
  SubTask(int aID, double aSL, double aEL, bool aF, double aR)
    : SubTaskID(aID), StartLength(aSL), EndLength(aEL), IsReversed(aF), RotWire(aR){};
  ~SubTask() = default;
  void SetSubTaskID(int aID);
  void SetStartLength(double aLength);
  void SetEndLength(double aLength);
  void SetDirectionFlag(bool aFlag);
  void SetRotWire(double aR);
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode);

public:
  int SubTaskID;
  double StartLength;
  double EndLength;
  bool IsReversed;
  double RotWire;
};

class Task
{
public:
  virtual ~Task() = default;
};

class EdgeTask : public Task
{
public:
  virtual ~EdgeTask() = default;
  virtual int GetNumberOfWeldSegments() = 0;
  virtual double GetTaskContourLength() = 0;
  virtual double GetWeldSegmentLength(int numberOfWeldSegment) = 0;
  void SetTaskID(int i_TaskID);
  int GetTaskID();
  void SetState(int i_State);
  int GetState();

public:
  int State = -1;  // 0 for inactive task, 1 for active task
  int TaskID = -1;
};

class WeldTask : public EdgeTask
{
public:
  ~WeldTask() = default;
  void AddWeldSegment(shared_ptr<WeldSegment> WS);
  void SetList_WeldSegment(list<shared_ptr<WeldSegment>> i_List_WeldSegment);
  list<shared_ptr<WeldSegment>> GetList_WeldSegment();
  int GetNumberOfWeldSegments();
  shared_ptr<WeldSegment> GetWeldSegment(int numberOfWeldSegment);
  double GetTaskContourLength();
  double GetWeldSegmentLength(int numberOfWeldSegment);
  Vector3d GetPosition(double DistanceFromStartPoint);
  Vector3d GetDirectionPartA(double DistanceFromStartPoint);
  Vector3d GetDirectionPartB(double DistanceFromStartPoint);
  Vector3d GetDirectionSeam(double DistanceFromStartPoint);
  double GetGapLength(double DistanceFromStartPoint);
  void InvertWeldTask();
  MatrixXd GetManufacturingCoordinateSystem(double DistanceFromStartPoint);
  void ReadXML(TiXmlElement* ParentXmlNode);
  void WriteXML(TiXmlElement* ParentXmlNode, int j);

  // SubTask methods
  void AddSubTask(int SubTaskID, double StartLength, double EndLength, bool IsReversed, double RotWire);
  void DeleteSubTask(int SubTaskID);
  void SetSubTaskDefault();
  SubTask GetSubTask(int SubTaskID);
  int GetNumberOfSubTasks();
  double GetStartLengthOfSubTask(int SubTaskID);
  double GetEndLengthOfSubTask(int SubTaskID);
  bool GetReversedFlagOfSubTask(int SubTaskID);
  double GetRotWireOfSubTask(int SubTaskID);

public:
  double TaskLength = -1.0;
  list<shared_ptr<WeldSegment>> List_WeldSegment;
  list<SubTask> List_SubTask;

private:
  shared_ptr<WeldSegment> GetSegmentFromStartPoint(double& DistanceFromStartPoint);
};

class TaskID
{
public:
  ~TaskID() = default;

public:
  int WeldTaskID = -1;
  int SubTaskID = -1;
};

class TaskList
{
public:
  ~TaskList() = default;
  void AddWeldTask(WeldTask* i_WeldTask);
  int GetNumberOfWeldTasks();
  int GetNumberOfWeldSegments(int WeldTaskID);
  double GetLengthOfWeldTask(int WeldTaskID);
  double GetWeldSegmentLength(int WeldTaskID, int FilletWeldSegmentID);
  double GetSmallestAngle(int WeldTaskID, double DistanceFromStartPoint);
  MatrixXd GetManufacturingCoordinateSystem(int WeldTaskID, double DistanceFromStartPoint);
  Vector3d GetPositionOfWeldTask(int WeldTaskID, double DistanceFromStartPoint);
  Vector3d GetDirectionPartAOfWeldTask(int WeldTaskID, double DistanceFromStartPoint);
  Vector3d GetDirectionPartBOfWeldTask(int WeldTaskID, double DistanceFromStartPoint);
  Vector3d GetDirectionSeamOfWeldTask(int WeldTaskID, double DistanceFromStartPoint);
  double GetGapLengthOfWeldTask(int WeldTaskID, double DistanceFromStartPoint);
  Vector3d GetPositionOfWeldSegment(int WeldTaskID, int FilletWeldSegmentID, double DistanceFromStartPoint);
  Vector3d GetDirectionPartAOfWeldSegment(int WeldTaskID, int FilletWeldSegmentID, double DistanceFromStartPoint);
  Vector3d GetDirectionPartBOfWeldSegment(int WeldTaskID, int FilletWeldSegmentID, double DistanceFromStartPoint);
  double GetGapLengthOfWeldSegment(int WeldTaskID, int FilletWeldSegmentID, double DistanceFromStartPoint);
  string GetContourTypeOfPositionSegment(int WeldTaskID, int FilletWeldSegmentID);
  void GroupWeldTasks();
  bool ComparePositions(Vector3d Vector1, Vector3d Vector2, double Tolerance);
  bool CompareDirections(Vector3d Vector1, Vector3d Vector2, double Tolerance);
  void SetTaskSequenceDefault();
  void ReadXML(string i_filename);
  void WriteXML(string i_filename);

  // SubTask methods
  void AddSubTaskInWeldTask(int WeldTaskID, int SubTaskID, double StartLength, double EndLength, bool IsReversed,
                            double RotWire);
  void DeleteSubTaskInWeldTask(int WeldTaskID, int SubTaskID);
  void DeleteAllSubTaskInWeldTask(int WeldTaskID);
  int GetNumberOfSubTasks(int WeldTaskID);
  double GetStartLengthOfSubTask(int WeldTaskID, int SubTaskID);
  double GetlengthOfSubTask(int WeldTaskID, int SubTaskID);
  double GetEndLengthOfSubTask(int WeldTaskID, int SubTaskID);
  bool GetReversedFlagOfSubtask(int WeldTaskID, int SubTaskID);
  double GetSmallestAngleOfSubTask(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint);
  MatrixXd GetManufacturingCoordinateSystemOfSubTask(int WeldTaskID, int SubTaskID,
                                                     double DistanceFromSubTaskStartPoint);
  Vector3d GetPositionOfSubTask(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint);
  double GetGapLengthOfSubTask(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint);
  void SetSubTaskOfWeldTasksDefault();  // add one SubTask for each WeldTask
  int GetWeldSegmentIDOfSubTaskFromStartPoint(int WeldTaskID, int SubTaskID, double DistanceFromSubTaskStartPoint);
  vector<double> GetDiscretizationLengthListOfSubTask(int WeldTaskID, int SubTaskID, double MaxDiscretizationLength);

  // TaskSequence methods
  void AddTaskSequenceElement(int WeldTaskID, int SubTaskID);
  void GetTaskSequenceElement(int ElementNumber, int& WeldTaskID, int& SubTaskID);
  int GetTaskSequenceWeldTaskID(int ElementNumber);
  int GetTaskSequenceSubTaskID(int ElementNumber);
  int GetNumberOfTaskSequenceElement();

  // State methods
  int GetTaskState(int WeldTaskID);
  void SetTaskState(int WeldTaskID, int TaskState);

  /* Check whether a point on the WeldTask (for SplineSegment this runs very slow)
  and return the length from the corresponding point to the start point,if the point is not on it, return -1 */
  bool IsPointOnWeldTask(int WeldTaskID, Vector3d Point, Vector3d Orientation, double PositonTreshold, double& Length);
  bool IsPointOnWeldTask(int WeldTaskID, Vector3d Point, double PositonTreshold, double& Length);

  // Invert the WeldTask, given the Task ID
  void InvertWeldTask(int WeldTaskID);

  // Given the TaskID and WeldSegmentID, get the DirectionFlag of the the WeldSegment
  int GetDirectionFlagOfWeldSegment(int WeldTaskID, int WeldSegmentID);

  shared_ptr<WeldSegment> GetWeldSegment(int WeldTaskID, int WeldSegmentID);

public:
  list<WeldTask> List_WeldTask;
  list<TaskID> TaskSequence;
};

TaskList IdentifyWeldSegmentTypes(list<shared_ptr<FilletWeldSegment>> List_FilletWelds);
