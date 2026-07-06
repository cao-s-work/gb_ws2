cmake_minimum_required(VERSION 3.8)
project(robots_dog_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/ArcChangeFlag.msg
  msg/ArcModuleCmd.msg
  msg/ArcModuleState.msg
  msg/ArcMultiStageCtrlCmd.msg
  msg/ArcState.msg
  msg/ArcTargetPose.msg
  msg/BmsCmd.msg
  msg/BmsState.msg
  msg/Cartesian.msg
  msg/CmdVelWithTrajectory.msg
  msg/CpuInfo.msg
  msg/CustomMsg.msg
  msg/CustomPoint.msg
  msg/DiskInfo.msg
  msg/DockPoseStamped.msg
  msg/DockState.msg
  msg/ElectronicMap.msg
  msg/ElectronicObstacle.msg
  msg/Ellipsoid.msg
  msg/EllipsoidArray.msg
  msg/FaultInfo.msg
  msg/FaultInfoMultiArray.msg
  msg/Frontier.msg
  msg/HighCmd.msg
  msg/HighLevelCmd.msg
  msg/HighLevelRobotState.msg
  msg/HighState.msg
  msg/IMU.msg
  msg/LED.msg
  msg/LegControlData.msg
  msg/LinktrackAoaNode0.msg
  msg/Localization.msg
  msg/LocalizationState.msg
  msg/LocalMap.msg
  msg/LowCmd.msg
  msg/LowLevelCmd.msg
  msg/LowLevelRobotState.msg
  msg/LowState.msg
  msg/McModeCmd.msg
  msg/McState.msg
  msg/MonitorInfo.msg
  msg/MotorCmd.msg
  msg/MotorState.msg
  msg/NavigationCmd.msg
  msg/NavigationError.msg
  msg/NavigationErrorReport.msg
  msg/NavigationErrorSet.msg
  msg/NavigationState.msg
  msg/NetInfo.msg
  msg/Nmea.msg
  msg/ObstacleInstance2D.msg
  msg/ObstacleScan.msg
  msg/PathScoreBreakdown.msg
  msg/PathSelectionSummary.msg
  msg/PathSelectorWeights.msg
  msg/PathWithScore.msg
  msg/PerceptionPoints.msg
  msg/PerceptionState.msg
  msg/PlannerDebugInfo.msg
  msg/Polyhedron.msg
  msg/PolyhedronArray.msg
  msg/Pose6D.msg
  msg/PredictedPath.msg
  msg/PredictedPathArray.msg
  msg/PredictedPose.msg
  msg/ProcessInfo.msg
  msg/RadarPoint2D.msg
  msg/RadarRaw2D.msg
  msg/RectangleArray.msg
  msg/RectangleWithCentre.msg
  msg/Relocalization.msg
  msg/RslidarPacket.msg
  msg/SlamState.msg
  msg/SrvEvent.msg
  msg/StartArc.msg
  msg/StartNavigation.msg
  msg/StaticMapState.msg
  msg/Trajectory.msg
  msg/TrajectoryArray.msg
  msg/TrajectoryPoint.msg
  msg/UniBestNav.msg
  msg/UniHeading.msg
  msg/UniRtkPvh.msg
  msg/UwbPointStamped.msg
)

set(srv_files
  srv/BuildMap2D.srv
  srv/BuildMap3D.srv
  srv/GetESDFMap.srv
  srv/GetFrontierCentroids.srv
  srv/GetMapState.srv
  srv/GetOptimizedPath.srv
  srv/IsExplorationGoalValid.srv
  srv/IsExplorationGoalsValid.srv
  srv/IsPathValid.srv
  srv/IsTargetInFov.srv
  srv/LoadMap.srv
  srv/LocalizationState.srv
  srv/MappingType.srv
  srv/MapState.srv
  srv/QueryPileResult.srv
  srv/RecordKeyPose.srv
  srv/RecordPath.srv
  srv/SetLocalizationType.srv
  srv/UseMapType.srv
)

set(action_files
  action/BackUp.action
  action/ComputePathThroughPoses.action
  action/ComputePathToPose.action
  action/ComputePathsToPoses.action
  action/ExploreThroughPoses.action
  action/FollowPath.action
  action/FollowWaypoints.action
  action/NavigateThroughPoses.action
  action/NavigateToPose.action
  action/ProcessInfo.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
