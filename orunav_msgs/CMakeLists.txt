cmake_minimum_required(VERSION 2.8.3)
project(orunav_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  std_msgs 
  geometry_msgs
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ActivePolygonConstraints.msg   
   ControllerTrajectoryChunkVec.msg  
   EnvelopeID.msg        
   Path.msg      
   PathParameters.msg           
   TimeEnvelope.msg
   ActiveRobots.msg               
   ControllerTrajectoryStep.msg      
   ForkCommand.msg       
   PolygonConstraint.msg    
   TimeInterval.msg
   BrakeCommand.msg               
   ControllerTrajectoryValue.msg     
   ForkLoadHandling.msg  
   PoseSteering.msg         
   Trajectories.msg
   ControllerCommand.msg          
   ControllerVelocities.msg          
   ForkReport.msg        
   RobotConstraints.msg     
   Trajectory.msg
   ControllerConstraints.msg      
   CoordinatorTime.msg               
   ForkState.msg         
   RobotEnvelopes.msg       
   TrajectoryPoint.msg
   ControllerReport.msg           
   CoordinatorTimeVec.msg            
   Load.msg              
   RobotPathReplanning.msg  
   TwistSteering.msg
   ControllerState.msg            
   DeltaT.msg                        
   Operation.msg         
   RobotReschedule.msg
   ControllerTrajectoryChunk.msg  
   DeltaTVec.msg                     
   ParkingPolygons.msg   
   RobotTarget.msg
   Task.msg
   RobotReport.msg
   Shape.msg
   LevelCommand.msg
   JackReport.msg
   JackState.msg
   ObjectPose.msg
   ObjectType.msg
   VectorMap.msg
   GeoFence.msg
   DrillTarget.msg
   DrillTargetArray.msg
   IliadItem.msg
   IliadItemArray.msg
   EBrake.msg
   ComputeTaskStatus.msg
   ReplanStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPath.srv
  GetParametrizedPath.srv
  SetTask.srv
  ComputeTask.srv
  ExecuteTask.srv
  UpdateTask.srv
  GetPolygonConstraints.srv
  GetSmoothedPath.srv
  GetSmoothedStraightPath.srv
  GetDeltaTVec.srv
  ObjectPoseEstimation.srv
  GetDrillTargets.srv
  GetGeoFence.srv
  GetVectorMap.srv
  GetPolygonFootPrint.srv
  SetDelay.srv
  SetPenetrationRate.srv
  SetBool.srv
  Trigger.srv
  BrakeTask.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(message_test test/message_test.cpp)
add_dependencies(message_test orunav_msgs_generate_messages_cpp)

