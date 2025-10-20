# Sequence Diagrams - Vision-Based Pick and Place System

## Document Control

| **Item** | **Details** |
|----------|-------------|
| **Document Title** | Sequence Diagrams |
| **Version** | 1.0 |
| **Date** | 2025-10-18 |
| **Status** | Draft |
| **Author(s)** | System Architect |

---

## 1. Introduction

This document provides sequence diagrams showing time-ordered interactions between system components. All diagrams use ASCII art and can be rendered with Mermaid or PlantUML.

---

## 2. End-to-End Pick-Place Sequence

### 2.1 Complete Workflow (Nominal Path)

```
User    Task         Vision      Grasp       MoveIt2     ros2_control   Robot    Gripper
 │      Orchestrator  Pipeline   Planner                 Manager        Arm
 │          │            │          │            │            │           │         │
 ├─Start───▶│            │          │            │            │           │         │
 │          │            │          │            │            │           │         │
 │          │─ScanReq───▶│          │            │            │           │         │
 │          │            │          │            │            │           │         │
 │          │            │─Capture──▶Camera      │            │           │         │
 │          │            │          │            │            │           │         │
 │          │            │◀─Image───│            │            │           │         │
 │          │            │          │            │            │           │         │
 │          │            │─RunYOLO──│            │            │           │         │
 │          │            │          │            │            │           │         │
 │          │            │─EstPose──│            │            │           │         │
 │          │            │          │            │            │           │         │
 │          │◀─ObjPoses──│          │            │            │           │         │
 │          │            │          │            │            │           │         │
 │          │─GraspReq──────────────▶│            │            │           │         │
 │          │          (pose,cloud)  │            │            │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │─Sample─────│            │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │─CheckColl──│            │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │─RankGrasps─│            │           │         │
 │          │            │            │            │            │           │         │
 │          │◀─GraspPose─────────────│            │            │           │         │
 │          │            │            │            │            │           │         │
 │          │─PlanPickReq────────────────────────▶│            │           │         │
 │          │            │        (target_pose)   │            │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │─SolveIK────│           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │─PlanPath───│           │         │
 │          │            │            │      (RRT*)│            │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │─GenTraj────│           │         │
 │          │            │            │            │            │           │         │
 │          │◀─Trajectory────────────────────────│            │           │         │
 │          │            │            │            │            │           │         │
 │          │─ExecPickReq────────────────────────────────────▶│           │         │
 │          │            │            │       (trajectory)     │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │            │─MoveToPre─▶         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │            │─MoveToGrsp▶         │
 │          │            │            │            │            │           │         │
 │          │─CloseGripper───────────────────────────────────────────────────────────▶│
 │          │            │            │            │            │           │         │
 │          │            │            │            │            │           │         │
 │          │◀─GraspForce────────────────────────────────────────────────────────────│
 │          │         (F/T sensor: 20N)            │            │           │         │
 │          │            │            │            │            │           │         │
 │          │─CheckGrasp─│            │            │            │           │         │
 │          │            │            │            │            │           │         │
 │          ├─Success────│            │            │            │           │         │
 │          │            │            │            │            │           │         │
 │          │─PlanPlaceReq───────────────────────▶│            │           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │─PlanPath───│           │         │
 │          │            │            │            │            │           │         │
 │          │◀─Trajectory────────────────────────│            │           │         │
 │          │            │            │            │            │           │         │
 │          │─ExecPlaceReq───────────────────────────────────▶│           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │            │─MoveToTarget▶       │
 │          │            │            │            │            │           │         │
 │          │─OpenGripper────────────────────────────────────────────────────────────▶│
 │          │            │            │            │            │           │         │
 │          │            │            │            │            │─Retract───▶         │
 │          │            │            │            │            │           │         │
 │          │─ReturnHome─────────────────────────────────────▶│           │         │
 │          │            │            │            │            │           │         │
 │          │            │            │            │            │─MoveHome──▶         │
 │          │            │            │            │            │           │         │
 │◀─Complete─│            │            │            │            │           │         │
 │          │            │            │            │            │           │         │
```

**Timing:**
- Total cycle time: ~2 seconds
- Vision (Scan → ObjPoses): 150ms
- Grasp planning: 200ms
- Motion planning (pick): 300ms
- Execution (pick): 800ms
- Motion planning (place): 250ms
- Execution (place): 300ms

---

## 3. Vision Pipeline Sequence

### 3.1 Object Detection & Pose Estimation

```
Task         Camera      Image       Object      Pose         TF2
Orchestrator Driver      Processor   Detector    Estimator
 │              │            │           │           │          │
 │─ScanRequest─▶│            │           │           │          │
 │              │            │           │           │          │
 │              │─Trigger───▶│           │           │          │
 │              │            │           │           │          │
 │              │◀─RGB+Depth─│           │           │          │
 │              │            │           │           │          │
 │              │─PublishRGB─────────────▶           │          │
 │              │  (topic)   │           │           │          │
 │              │            │           │           │          │
 │              │─PublishDepth──────────▶│           │          │
 │              │  (topic)   │           │           │          │
 │              │            │           │           │          │
 │              │            │           │─Preprocess│          │
 │              │            │           │ (resize)  │          │
 │              │            │           │           │          │
 │              │            │           │─RunYOLO───│          │
 │              │            │      (TensorRT)       │          │
 │              │            │           │           │          │
 │              │            │           │─NMS───────│          │
 │              │            │           │           │          │
 │              │            │           │─Filter────│          │
 │              │            │       (conf>0.7)      │          │
 │              │            │           │           │          │
 │              │            │           │─Publish───────────────▶Task
 │              │            │           │Detections │          │
 │              │            │           │           │          │
 │              │            │           │           │──────────▶│
 │              │            │           │      (for each det)   │
 │              │            │           │           │          │
 │              │            │           │           │─ExtractROI
 │              │            │           │           │          │
 │              │            │           │           │─Deproject─│
 │              │            │           │      (depth→3D)      │
 │              │            │           │           │          │
 │              │            │           │           │─EstPose───│
 │              │            │           │         (PnP/ICP)    │
 │              │            │           │           │          │
 │              │            │           │           │─LookupTF──▶│
 │              │            │           │   (camera→base)      │
 │              │            │           │           │          │
 │              │            │           │           │◀─Transform│
 │              │            │           │           │          │
 │              │            │           │           │─Publish───▶Task
 │              │            │           │          Poses       │
 │              │            │           │          (base frame)│
 │◀─ObjPoses──────────────────────────────────────────────────────
```

---

## 4. Grasp Planning Sequence

### 4.1 Grasp Synthesis & Collision Checking

```
Task         Grasp       Grasp        Collision    Grasp
Orchestrator Planner     Sampler      Checker      Ranker
 │              │            │             │           │
 │─GraspRequest─▶│            │             │           │
 │  (pose,cloud)│            │             │           │
 │              │            │             │           │
 │              │─Sample────▶│             │           │
 │              │            │             │           │
 │              │            │─GenCandidates           │
 │              │            │  (N=50)     │           │
 │              │            │             │           │
 │              │◀─Grasps────│             │           │
 │              │            │             │           │
 │              │─ForEach────────────────▶│           │
 │              │   Grasp    │             │           │
 │              │            │             │           │
 │              │            │             │─CheckGripper
 │              │            │             │ -Object   │
 │              │            │             │           │
 │              │            │             │─CheckGripper
 │              │            │             │ -Scene    │
 │              │            │             │           │
 │              │◀─CollFree──────────────│           │
 │              │   (boolean)│             │           │
 │              │            │             │           │
 │              │─IfCollFree─────────────────────────▶│
 │              │            │             │           │
 │              │            │             │           │─CompQuality
 │              │            │             │      (force closure)
 │              │            │             │           │
 │              │◀─Quality───────────────────────────│
 │              │            │             │           │
 │              │─AddToValid─│             │           │
 │              │  List      │             │           │
 │              │            │             │           │
 │              │─EndLoop────│             │           │
 │              │            │             │           │
 │              │─Sort───────────────────────────────▶│
 │              │  ByQuality │             │           │
 │              │            │             │           │
 │              │◀─RankedList────────────────────────│
 │              │            │             │           │
 │              │─SelectTop──│             │           │
 │              │            │             │           │
 │◀─GraspPose───│            │             │           │
 │   +Quality   │            │             │           │
```

---

## 5. Motion Planning Sequence (MoveIt2)

### 5.1 Path Planning with Collision Checking

```
Task         MoveIt      Planning    IK          OMPL        Trajectory
Orchestrator MoveGroup   Scene       Solver      Planner     Generator
 │              │            │           │           │            │
 │─PlanRequest──▶│            │           │           │            │
 │ (target_pose)│            │           │           │            │
 │              │            │           │           │            │
 │              │─UpdateScene─▶           │           │            │
 │              │            │           │           │            │
 │              │            │─AddObstacles          │            │
 │              │       (point cloud)    │           │            │
 │              │            │           │           │            │
 │              │◀─SceneReady─           │           │            │
 │              │            │           │           │            │
 │              │─SolveIK────────────────▶           │            │
 │              │            │           │           │            │
 │              │            │           │─ComputeIK─│            │
 │              │            │      (KDL/TRAC-IK)    │            │
 │              │            │           │           │            │
 │              │◀─JointAngles───────────│           │            │
 │              │            │           │           │            │
 │              │─ValidateGoal           │           │            │
 │              │            │           │           │            │
 │              │─PlanPath───────────────────────────▶            │
 │              │            │      (start, goal)    │            │
 │              │            │           │           │            │
 │              │            │           │           │─RunRRT*────│
 │              │            │           │     (5 sec timeout)    │
 │              │            │           │           │            │
 │              │            │           │           │─CheckColl──▶Scene
 │              │            │           │   (repeated)           │
 │              │            │           │           │            │
 │              │            │           │           │◀─CollFree──│
 │              │            │           │           │            │
 │              │◀─Path──────────────────────────────│            │
 │              │            │      (joint configs)  │            │
 │              │            │           │           │            │
 │              │─GenTrajectory──────────────────────────────────▶│
 │              │            │           │           │            │
 │              │            │           │           │            │─TimeParam
 │              │            │           │           │       (parabolic)
 │              │            │           │           │            │
 │              │            │           │           │            │─ApplyLimits
 │              │            │           │     (vel, accel)       │
 │              │            │           │           │            │
 │              │◀─Trajectory────────────────────────────────────│
 │              │            │           │           │            │
 │◀─Trajectory──│            │           │           │            │
 │   (ready)    │            │           │           │            │
```

---

## 6. Trajectory Execution Sequence

### 6.1 ros2_control Execution Loop

```
MoveIt      Controller   Trajectory  PID         Hardware    Motor
MoveGroup   Manager      Follower    Controller  Interface   Driver
 │              │            │           │           │           │
 │─ExecAction──▶│            │           │           │           │
 │ (trajectory) │            │           │           │           │
 │              │            │           │           │           │
 │              │─LoadTraj───▶           │           │           │
 │              │            │           │           │           │
 │              │            │─Start─────│           │           │
 │              │            │  Loop     │           │           │
 │              │            │  @1kHz    │           │           │
 │              │            │           │           │           │
 │              │            │─Interpolate           │           │
 │              │            │ Setpoint  │           │           │
 │              │            │  (t=now)  │           │           │
 │              │            │           │           │           │
 │              │            │─SendSetpoint──────────▶           │
 │              │            │           │           │           │
 │              │            │           │─CompPID───│           │
 │              │            │     (error = sp-fb)   │           │
 │              │            │           │           │           │
 │              │            │           │─AddFF─────│           │
 │              │            │      (gravity comp)   │           │
 │              │            │           │           │           │
 │              │            │           │─Output────────────────▶│
 │              │            │         (torque cmd)  │           │
 │              │            │           │           │           │
 │              │            │           │           │─ReadEnc───▶│
 │              │            │           │     (position)        │
 │              │            │           │           │           │
 │              │            │           │◀─Feedback─────────────│
 │              │            │           │           │           │
 │              │            │◀─JointStates──────────│           │
 │              │            │           │           │           │
 │              │◀─Feedback──│           │           │           │
 │   (progress) │            │           │           │           │
 │              │            │           │           │           │
 │              │            │─CheckDone─│           │           │
 │              │            │           │           │           │
 │              │            │─IfDone────────────────▶           │
 │              │            │  Stop     │           │           │
 │              │            │  Loop     │           │           │
 │              │            │           │           │           │
 │◀─Result──────│            │           │           │           │
 │  (success)   │            │           │           │           │
```

**Loop Timing:**
- Control frequency: 1000 Hz (1ms period)
- Setpoint interpolation: <50μs
- PID computation: <100μs
- EtherCAT communication: <200μs
- Total loop time: <1ms (with margin for jitter)

---

## 7. Error Recovery Sequence

### 7.1 Grasp Failure → Retry

```
Task         F/T         Error       Grasp       MoveIt2     ros2_control
Orchestrator Sensor      Detector    Planner                 Manager
 │              │            │           │           │            │
 │─ExecPick────────────────────────────────────────▶            │
 │              │            │           │           │            │
 │              │            │           │           │──MoveToPre──▶Robot
 │              │            │           │           │            │
 │              │            │           │           │──MoveToGrsp─▶Robot
 │              │            │           │           │            │
 │─CloseGripper────────────────────────────────────────────────────▶Gripper
 │              │            │           │           │            │
 │              │◀─ForceReading──────────────────────────────────│
 │              │   (5N, low!)          │           │            │
 │              │            │           │           │            │
 │              │─DetectDrop─▶           │           │            │
 │              │            │           │           │            │
 │              │            │─RaiseFault            │            │
 │              │            │           │           │            │
 │◀─ErrorEvent──────────────│           │           │            │
 │  (GRASP_FAIL)            │           │           │            │
 │              │            │           │           │            │
 │─LogError─────│            │           │           │            │
 │              │            │           │           │            │
 │─CheckRetry───│            │           │           │            │
 │  Count       │            │           │           │            │
 │              │            │           │           │            │
 │─IfRetry<Max─│            │           │           │            │
 │              │            │           │           │            │
 │─Retract──────────────────────────────────────────▶            │
 │              │            │           │           │            │
 │              │            │           │           │──MoveBack───▶Robot
 │              │            │           │           │            │
 │─ReplanGrasp─────────────────────────▶│           │            │
 │  (increase   │            │           │           │            │
 │   force)     │            │           │           │            │
 │              │            │           │           │            │
 │              │            │           │─AdjustForce           │
 │              │            │           │  (50%→100%)           │
 │              │            │           │           │            │
 │◀─NewGrasp────────────────────────────│           │            │
 │              │            │           │           │            │
 │─RetryPick────────────────────────────────────────▶            │
 │              │            │           │           │            │
 │              │            │           │           │──Execute────▶Robot
 │              │            │           │           │            │
 │─CloseGripper────────────────────────────────────────────────────▶Gripper
 │              │            │           │           │            │
 │              │◀─ForceReading──────────────────────────────────│
 │              │   (20N, OK!)          │           │            │
 │              │            │           │           │            │
 │◀─Success─────│            │           │           │            │
 │              │            │           │           │            │
 │─Continue─────│            │           │           │            │
 │  (place)     │            │           │           │            │
```

---

## 8. Calibration Sequence

### 8.1 Hand-Eye Calibration

```
Calib       Robot       Camera      Detection   Calibration
Wizard      Controller  Driver      Node        Solver
 │              │           │           │             │
 │─Start────────│           │           │             │
 │              │           │           │             │
 │─MoveToPos1──▶           │           │             │
 │              │           │           │             │
 │              │─Execute───▶           │             │
 │              │           │           │             │
 │◀─AtPosition──│           │           │             │
 │              │           │           │             │
 │─CaptureImg──────────────▶           │             │
 │              │           │           │             │
 │              │           │─Trigger───▶             │
 │              │           │           │             │
 │              │           │◀─Image────│             │
 │              │           │           │             │
 │──────────────────────────────────────▶             │
 │              │           │ Detect    │             │
 │              │           │ Checkerboard            │
 │              │           │           │             │
 │◀─Corners─────────────────────────────│             │
 │              │           │           │             │
 │─ReadRobotPose▶           │           │             │
 │              │           │           │             │
 │◀─TcpPose─────│           │           │             │
 │              │           │           │             │
 │─StoreData────│           │           │             │
 │ (corners,    │           │           │             │
 │  robot_pose) │           │           │             │
 │              │           │           │             │
 │─RepeatFor────│           │           │             │
 │  Pos2-5      │           │           │             │
 │              │           │           │             │
 │ (loop 4 more times)      │           │             │
 │              │           │           │             │
 │─AllDataCollected         │           │             │
 │              │           │           │             │
 │─SolveCalib───────────────────────────────────────▶│
 │              │           │      (AX=XB)            │
 │              │           │           │             │
 │              │           │           │             │─ComputeTF
 │              │           │           │       (camera→base)
 │              │           │           │             │
 │◀─Transformation──────────────────────────────────│
 │   Matrix     │           │           │             │
 │              │           │           │             │
 │─Validate─────│           │           │             │
 │              │           │           │             │
 │─PlaceObject──│           │           │             │
 │              │           │           │             │
 │─DetectObject─────────────────────────▶             │
 │              │           │           │             │
 │◀─Pos(camera)─────────────────────────│             │
 │              │           │           │             │
 │─TransformTo──────────────────────────────────────▶│
 │  BaseFrame   │           │   (using TF)            │
 │              │           │           │             │
 │◀─Pos(base)───────────────────────────────────────│
 │              │           │           │             │
 │─MeasureActual▶           │           │             │
 │  (CMM/ruler) │           │           │             │
 │              │           │           │             │
 │◀─ActualPos───│           │           │             │
 │              │           │           │             │
 │─CompError────│           │           │             │
 │ (predicted-  │           │           │             │
 │  actual)     │           │           │             │
 │              │           │           │             │
 │─IfError<5mm─│           │           │             │
 │              │           │           │             │
 │─SaveCalib────────────────────────────────────────▶│
 │              │      (to YAML file)   │             │
 │              │           │           │             │
 │◀─Success─────│           │           │             │
```

---

## 9. System Startup Sequence

### 9.1 Boot & Initialization

```
User    Init        ROS2        Vision      MoveIt2     ros2_control  Robot
        Script      Daemon      Nodes       Nodes       Manager       Hardware
 │         │           │            │           │            │            │
 │─PowerOn─▶           │            │           │            │            │
 │         │           │            │           │            │            │
 │         │─StartROS2 ▶            │           │            │            │
 │         │           │            │           │            │            │
 │         │           │─LaunchCore─│           │            │            │
 │         │           │            │           │            │            │
 │         │           │◀─CoreReady─│           │            │            │
 │         │           │            │           │            │            │
 │         │─LaunchVision───────────▶           │            │            │
 │         │           │            │           │            │            │
 │         │           │            │─InitCamera│            │            │
 │         │           │            │           │            │            │
 │         │           │            │◀─CamReady─│            │            │
 │         │           │            │           │            │            │
 │         │           │            │─LoadModel─│            │            │
 │         │           │        (YOLOv8)        │            │            │
 │         │           │            │           │            │            │
 │         │           │            │◀─ModelReady           │            │
 │         │           │            │           │            │            │
 │         │─LaunchMoveIt───────────────────────▶            │            │
 │         │           │            │           │            │            │
 │         │           │            │           │─LoadURDF───│            │
 │         │           │            │           │            │            │
 │         │           │            │           │─InitPlanningScene       │
 │         │           │            │           │            │            │
 │         │           │            │           │◀─Ready─────│            │
 │         │           │            │           │            │            │
 │         │─LaunchControl──────────────────────────────────▶            │
 │         │           │            │           │            │            │
 │         │           │            │           │            │─InitHW─────▶
 │         │           │            │           │      (EtherCAT)         │
 │         │           │            │           │            │            │
 │         │           │            │           │            │◀─HwReady───│
 │         │           │            │           │            │            │
 │         │           │            │           │            │─LoadControllers
 │         │           │            │           │            │            │
 │         │           │            │           │            │◀─CtrlReady─│
 │         │           │            │           │            │            │
 │         │─HomeRobot──────────────────────────────────────▶            │
 │         │           │            │           │            │            │
 │         │           │            │           │            │─MoveHome───▶
 │         │           │            │           │            │            │
 │         │           │            │           │            │◀─AtHome────│
 │         │           │            │           │            │            │
 │◀─SystemReady────────│            │           │            │            │
 │         │           │            │           │            │            │
 │─Display─│           │            │           │            │            │
 │ "READY" │           │           │           │            │            │
```

**Startup Time:** ~45 seconds total
- ROS2 daemon: 5s
- Vision nodes: 15s (model loading)
- MoveIt2: 10s (URDF, planning scene)
- ros2_control: 10s (EtherCAT init, homing)
- Final checks: 5s

---

## 10. Shutdown Sequence

### 10.1 Graceful Shutdown

```
User    Task         ros2_control  MoveIt2    Vision      ROS2
        Orchestrator Manager        Nodes      Nodes       Daemon
 │          │            │            │          │           │
 │─Shutdown─▶            │            │          │           │
 │          │            │            │          │           │
 │          │─StopTasks──│            │          │           │
 │          │            │            │          │           │
 │          │─MoveHome───────────────▶            │           │
 │          │            │            │          │           │
 │          │            │─Execute────▶          │           │
 │          │            │            │          │           │
 │          │◀─AtHome────────────────│          │           │
 │          │            │            │          │           │
 │          │─DisableMotors──────────▶            │           │
 │          │            │            │          │           │
 │          │            │─Deactivate─▶          │           │
 │          │            │            │          │           │
 │          │◀─MotorsOff────────────│          │           │
 │          │            │            │          │           │
 │          │─StopVision─────────────────────────▶           │
 │          │            │            │          │           │
 │          │            │            │          │─UnloadModel
 │          │            │            │          │           │
 │          │            │            │          │─CloseCamera
 │          │            │            │          │           │
 │          │◀─VisionStopped──────────────────────│           │
 │          │            │            │          │           │
 │          │─StopMoveIt─────────────────────────▶           │
 │          │            │            │          │           │
 │          │◀─MoveItStopped──────────────────────│           │
 │          │            │            │          │           │
 │          │─StopControl────────────▶            │           │
 │          │            │            │          │           │
 │          │            │─UnloadCtrl─│          │           │
 │          │            │            │          │           │
 │          │            │─CloseHW────▶          │           │
 │          │            │            │          │           │
 │          │◀─ControlStopped─────────│          │           │
 │          │            │            │          │           │
 │          │─ShutdownROS────────────────────────────────────▶│
 │          │            │            │          │           │
 │          │            │            │          │           │─KillNodes
 │          │            │            │          │           │
 │◀─Shutdown─────────────│            │          │           │
 │ Complete │            │            │          │           │
 │          │            │            │          │           │
 │─PowerOff─│            │            │          │           │
```

---

## Summary

This document provides **10 comprehensive sequence diagrams** covering:

1. **End-to-End Pick-Place** - Complete workflow with all subsystems
2. **Vision Pipeline** - Object detection and pose estimation
3. **Grasp Planning** - Synthesis and collision checking
4. **Motion Planning** - MoveIt2 path planning
5. **Trajectory Execution** - ros2_control real-time loop
6. **Error Recovery** - Grasp failure retry logic
7. **Calibration** - Hand-eye calibration procedure
8. **System Startup** - Boot and initialization
9. **System Shutdown** - Graceful shutdown

**Key Insights:**
- Vision pipeline: 150ms (camera→poses)
- Grasp planning: 200ms (pose→grasp)
- Motion planning: 300-500ms (IK→trajectory)
- Control loop: 1ms period @ 1kHz
- Startup time: ~45 seconds
- Total cycle time: ~2 seconds (scan→place)

---

**Document Status:** ✅ Complete
**Last Updated:** 2025-10-18
**Format:** ASCII sequence diagrams (convertible to Mermaid)
**Review Status:** Pending Technical Review
