# Synchronisation labels

channel                         | EL  | oB  | mTLB | mTLS | mTLM
----------------------------------------------------------------
obstacle                        | no  | yes | no   | no   | no
moveToLocationCall              | no  | no  | no   | yes  | yes
tock                            | yes | no  | no   | no   | no
obstacleTriggered               | yes | yes | no   | no   | no
moveToLocationHappened          | no  | no  | yes  | no   | yes
getRobotPosition                | yes | no  | no   | yes  | no
getRobotVelocity                | yes | no  | no   | no   | no
getRobotAcceleration            | yes | no  | no   | no   | no
getRobotOrientation             | yes | no  | no   | no   | no
getRobotAngularVelocity         | yes | no  | no   | no   | no
getRobotAngularAcceleration     | yes | no  | no   | no   | no
setRobotPosition                | yes | no  | no   | no   | no
setRobotVelocity                | yes | no  | no   | yes  | no
setRobotAcceleration            | yes | no  | no   | no   | no
setRobotOrientation             | yes | no  | no   | yes  | no
setRobotAngularVelocity         | yes | no  | no   | no   | no
setRobotAngularAcceleration     | yes | no  | no   | no   | no
getMaxObstacleID                | yes | no  | no   | no   | no
getObstaclePosition             | yes | no  | no   | no   | no
getObstacleOrientation          | yes   no  | no   | no	  | no
proceed                         | yes   no  | no   | yes  | no

obstacle                        -> obstacle_oB                                  `((EL |[..]| (mtlB ||| oB)) \ {..} |[..]| (mtlS |[..]| mtlM)) \ {..}`
moveToLocationCall              -> moveToLocationCall_mTLS_mTLM                 `((EL |[..]| (mtlB ||| oB)) \ {..} |[..]| (mtlS |[ moveToLocationCall ]| mtlM)) \ {..}`
tock                            -> tock_EL                                      `((EL |[..]| (mtlB ||| oB)) \ {..} |[..]| (mtlS |[..]| mtlM)) \ {..}`
obstacleTriggered               -> obstacleTriggered_EL_oB_hidden               `((EL |[ obstacleTriggered ]| (mtlB ||| oB)) \ { obstacleTriggered } |[..]| (mtlS |[..]| mtlM)) \ {..}`
moveToLocationHappened          -> moveToLocationHappened_mTLB_mTLM_hidden      `((EL |[..]| (mtlB ||| oB)) \ {..} |[ moveToLocationHappended ]| (mtlS |[..]| mtlM)) \ { moveToLocationHappened }`
getRobotPosition                -> getRobotPosition_EL_mTLS_hidden              `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getRobotPosition ]| (mtlS |[..]| mtlM)) \ { getRobotPosition }`
getRobotVelocity                -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getRobotVelocity ]| (mtlS |[..]| mtlM)) \ { getRobotVelocity }`
getRobotAcceleration            -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getRobotAcceleration ]| (mtlS |[..]| mtlM)) \ { getRobotAcceleration }`
getRobotOrientation             -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getRobotOrientation ]| (mtlS |[..]| mtlM)) \ { getRobotOrientation }`
getRobotAngularVelocity         -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getRobotAngularVelocity ]| (mtlS |[..]| mtlM)) \ { getRobotAngularVelocity }`
getRobotAngularAcceleration     -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getRobotAngularAcceleration ]| (mtlS |[..]| mtlM)) \ { getRobotAngularAcceleration }`
setRobotPosition                -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ setRobotPosition ]| (mtlS |[..]| mtlM)) \ { setRobotPosition }`
setRobotVelocity                -> setRobotVelocity_EL_mTLS_hidden              `((EL |[..]| (mtlB ||| oB)) \ {..} |[ setRobotVelocity ]| (mtlS |[..]| mtlM)) \ { setRobotVelocity }`
setRobotAcceleration            -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ setRobotAcceleration ]| (mtlS |[..]| mtlM)) \ { setRobotAcceleration }`
setRobotOrientation             -> setRobotOrientation_EL_mTLS_hidden           `((EL |[..]| (mtlB ||| oB)) \ {..} |[ setRobotOrientation ]| (mtlS |[..]| mtlM)) \ { setRobotOrientation }`
setRobotAngularVelocity         -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ setRobotAngularVelocity ]| (mtlS |[..]| mtlM)) \ { setRobotAngularVelocity }`
setRobotAngularAcceleration     -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ setRobotAngularAcceleration ]| (mtlS |[..]| mtlM)) \ { setRobotAngularAcceleration }`
getMaxObstacleID                -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getMaxObstacleID ]| (mtlS |[..]| mtlM)) \ { getMaxObstacleID }`
getObstaclePosition             -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getObstaclePosition ]| (mtlS |[..]| mtlM)) \ { getObstaclePosition }`
getObstacleOrientation          -> -                                            `((EL |[..]| (mtlB ||| oB)) \ {..} |[ getObstacleOrientation ]| (mtlS |[..]| mtlM)) \ { getObstacleOrientation }`
proceed                         -> proceed_EL_mTLS_hidden                       `((EL |[..]| (mtlB ||| oB)) \ {..} |[ proceed ]| (mtlS |[..]| mtlM)) \ { proceed }`

# EnvironmentLoop

Action                          | start | end | kind                                                                                                   | new locations
----------------------------------------------------------------------------------------------------------------------------------------------------------------------
EnvironmentLoop_1		| 1     | 2   | seq comp (EnvironmentLoop_2, EnvironmentLoop_3)                                                        | 3
EnvironmentLoop_2               | 1     | 3   | discrete schema (EnvironmentLoopStateInit)                                                             | -
EnvironmentLoop_3               | 3     | 2   | seq comp (RobotMovementAction_1, EnvironmentLoop_4)                                                    | 4
RobotMovementAction_1           | 3     | 4   | interrupt by condition (RobotMovement)                                                                 | 5
RobotMovement                   | 3     | 5   | discrete schema                                                                                        | -
EnvironmentLoop_4               | 4     | 2   | seq comp (EnvironmentLoop_5, EnvironmentLoop_3) loop                                                   | -
EnvironmentLoop_5               | 4     | 3   | conditional (HandleCollision, EnvironmentLoop_6)                                                       | 6, 7
HandleCollision                 | 6     | 3   | assignment                                                                                             | -
EnvironmentLoop_6               | 7     | 3   | seq comp (InputTriggers_1, EnvironmentLoop_7)                                                          | 8
InputTriggers_1                 | 7     | 8   | action reference (obstacle_InputEventMapping_1)                                                        | -
obstacle_InputEventMapping_1    | 7     | 8   | conditional (obstacle_InputEventMapping_2, obstacle_InputEventMapping_3, obstacle_InputEventMapping_4) | 9, 10, 11
obstacle_InputEventMapping_2    | 9     | 8   | seq comp (obstacle_InputEventMapping_5, obstacle_InputEventMapping_6)                                  | 12
obstacle_InputEventMapping_5    | 9     | 12  | discrete schema (obstacle_InputEventMapping_DetermineValue)                                            | -
obstacle_InputEventMapping_6    | 12    | 8   | obstacleTriggered output comm, 1 label (obstacle_InputEventMapping_8)                                  | 13, 14
obstacle_InputEventMapping_8    | 14    | 8   | assignment                                                                                             | -
obstacle_InputEventMapping_3    | 10    | 8   | obstacleTriggered mixed comm, 1 label (obstacle_InputEventMapping_7)                                   | 15, 16
obstacle_InputEventMapping_7    | 16    | 8   | Skip                                                                                                   | -
obstacle_InputEventMapping_4    | 11    | 8   | Skip                                                                                                   | -
EnvironmentLoop_7               | 8     | 3   | seq comp (Communication_1, EnvironmentLoop_8)                                                          | 17
Communication_1                 | 8     | 17  | ext choice                                                                                             |
 - comm 1                       | 8     | 16  | getRobotPosition output comm, 1 label (Communication_2)                                                | 18, 19
 - comm 2                       | 8     | 16  | getRobotVelocity output comm, no labels (Communication_3)                                              | -
 - comm 3                       | 8     | 16  | getRobotAcceleration output comm, no labels (Communication_4)                                          | -
 - comm 4                       | 8     | 16  | getRobotOrientation output comm, no labels (Communication_5)                                           | -
 - comm 5                       | 8     | 16  | getRobotAngularVelocity output comm, no labels (Communication_6)                                       | -
 - comm 6                       | 8     | 16  | getRobotAngularAcceleration output comm, no labels (Communication_7)                                   | -
 - comm 7                       | 8     | 16  | getRobotPosition input comm, no labels (Communication_8)                                               | -
 - comm 8                       | 8     | 16  | setRobotVelocity input comm, 1 label (Communication_9)                                                 | 20, 21
 - comm 9                       | 8     | 16  | setRobotAcceleration input comm, no labels (Communication_10)                                          | -
 - comm 10                      | 8     | 16  | setRobotOrientation input comm, 1 label (Communication_11)                                             | 22, 23
 - comm 11                      | 8     | 16  | setRobotAngularVelocity input comm, no labels (Communication_12)                                       | -
 - comm 12                      | 8     | 16  | setRobotAngularAcceleration input comm, no labels (Communication_13)                                   | -
 - comm 13                      | 8     | 16  | getMaxObstacleID ouput comm, no labels (Communication_14)                                              | -
 - comm 14                      | 8     | 16  | getObstaclePosition mixed comm, no labels (Communication_15)                                           | -
 - comm 15                      | 8     | 16  | getObstacleOrientation mixed comm, no labels (Communication_16)                                        | -
 - comm 16                      | 8     | 16  | proceed comm, 1 label (Communication_17)                                                               | 24
Communication_2                 | 19    | 16  | seq comp (Communication_18, Communication_1) loop                                                      | -
Communication_18                | 19    | 8   | Skip                                                                                                   | -
Communication_9                 | 21    | 16  | seq comp (Communication_25, Communication_1) loop                                                      | -
Communication_25                | 21    | 8   | assignment                                                                                             | -
Communication_11                | 23    | 16  | seq comp (Communication_27, Communication_1) loop                                                      | -
Communication_27                | 23    | 8   | assignment                                                                                             | -
Communication_17                | 24    | 16  | Skip                                                                                                   | -
EnvironmentLoop_8               | 17    | 3   | seq comp (CheckTock_1, EnvironmentLoop_9)                                                              | 25
CheckTock_1                     | 17    | 25  | conditional (CheckTock_2, CheckTock_3)                                                                 | 26, 27
CheckTock_2                     | 26    | 25  | tock comm, 1 label (CheckTock_4)                                                                       | 28
CheckTock_4                     | 28    | 25  | assignment                                                                                             | -
CheckTock_3                     | 27    | 25  | Skip                                                                                                   | -
EnvironmentLoop_9               | 25    | 3   | assignment                                                                                             | -

* Loc 1: EnvironmentLoop_1 start, EnvironmentLoop_2 start -> 3
* Loc 2: EnvironmentLoop_1 end (unused)
* Loc 3: EnvironmentLoop_3 start, RobotMovementAction_1 start, RobotMovement start -> 4
* Loc 4: EnvironmentLoop_4 start, EnvironmentLoop_5 start -> 6, 7
* Loc 5: RobotMovement end (unused) -> 4
* Loc 6: HandleCollision start -> 3
* Loc 7: EnvironmentLoop_6 start, InputTriggers_1 start, obstacle_InputEventMapping_1 start -> 9, 10, 11
* Loc 8: EnvironmentLoop_7 start, Communication_1 -> 18, 20, 22, 24
* Loc 9: obstacle_InputEventMapping_2 start, obstacle_InputEventMapping_5 start -> 12
* Loc 10: obstacle_InputEventMapping_3 start -> 15
* Loc 11: obstacle_InputEventMapping_4 start -> 8
* Loc 12: obstacle_InputEventMapping_6 start -> 13
* Loc 13: obstacle_InputEventMapping_6 obstacleTriggered intermediate -> 14
* Loc 14: obstacle_InputEventMapping_8 start -> 8
* Loc 15: obstacle_InputEventMapping_3 obstacleTriggered intermediate -> 16
* Loc 16: obstacle_InputEventMapping_7 start -> 8
* Loc 17: EnvironmentLoop_8 start, CheckTock_1 start -> 26, 27
* Loc 18: Communication_1 getRobotPosition intermediate -> 19
* Loc 19: Communication_2 start, Communication_18 start -> 8
* Loc 20: Communication_1 setRobotVelocity intermediate -> 21
* Loc 21: Communication_9 start, Communication_25 start -> 8
* Loc 22: Communication_1 setRobotOrientation intermediate -> 23
* Loc 23: Communication_11 start, Communication_27 start -> 8
* Loc 24: Communication_17 start -> 16
* Loc 25: EnvironmentLoop_9 start -> 3
* Loc 26: CheckTock_2 start -> 28
* Loc 27: CheckTock_3 start -> 25
* Loc 28: CheckTock_4 start -> 25

# obstacle_Buffer

Action                  | start | end | kind                                                      | new locations
---------------------------------------------------------------------------------------------------------------------
obstacle_Buffer_1       | 1     | 2   | seq comp (obstacle_Buffer_2, obstacle_Buffer_3)           | 3
obstacle_Buffer_2       | 1     | 3   | assignment                                                | -
obstacle_Buffer_3       | 3     | 2   | conditional (obstacle_Buffer_4, obstacle_Buffer_5)        | 4, 5
obstacle_Buffer_4       | 4     | 2   | ext choice                                                |
 - comm 1               | 4     | 2   | obstacleTriggered input comm, 1 label (obstacle_Buffer_6) | 6, 7
 - comm 2               | 4     | 2   | obstacle output comm, 1 label (obstacle_Buffer_7)         | 8, 9
obstacle_Buffer_6       | 7     | 2   | seq comp (obstacle_Buffer_9, obstacle_Buffer_3) loop      | -
obstacle_Buffer_9       | 7     | 3   | assignment                                                | -
obstacle_Buffer_7       | 9     | 2   | seq comp (obstacle_Buffer_10, obstacle_Buffer_3) loop     | -
obstacle_Buffer_10      | 9     | 3   | Skip                                                      | -
obstacle_Buffer_5       | 5     | 2   | obstacleTriggered input comm, 1 label (obstacle_Buffer_8) | 10, 11
obstacle_Buffer_8       | 11    | 2   | seq comp (obstacle_Buffer_11, obstacle_Buffer_3) loop     | -
obstacle_Buffer_11      | 11    | 3   | assignment                                                | -

* Loc 1: obstacle_Buffer_1 start, obstacle_Buffer_2 start -> 3
* Loc 2: obstacle_Buffer_1 end
* Loc 3: obstacle_Buffer_3 start -> 4, 5
* Loc 4: obstacle_Buffer_4 start -> 6, 8
* Loc 5: obstacle_Buffer_5 start -> 10
* Loc 6: obstacle_Buffer_4 obstacleTriggered intermediate -> 7
* Loc 7: obstacle_Buffer_6 start, obstacle_Buffer_9 start -> 3
* Loc 8: obstacle_Buffer_4 obstacle intermediate -> 9
* Loc 9: obstacle_Buffer_7 start, obstacle_Buffer_10 -> 3
* Loc 10 obstacle_Buffer_5 obstacleTriggered intermediate -> 11
* Loc 11 obstacle_Buffer_8 start -> 3

# moveToLocation_Buffer

Action                  | start | end | kind                                                             | new locations
------------------------------------------------------------------------------------------------------------------------
moveToLocation_Buffer_1 | 1     | 2   | seq comp (moveToLocation_Buffer_2, moveToLocation_Buffer_3)      | 3
moveToLocation_Buffer_2 | 1     | 3   | discrete schema                                                  | -
moveToLocation_Buffer_3 | 3     | 2   | moveToLocationHappened comm, 1 label (moveToLocation_Buffer_4)   | 4
moveToLocation_Buffer_4 | 4     | 2   | seq comp (moveToLocation_Buffer_5, moveToLocation_Buffer_1) loop | -
moveToLocation_Buffer_5 | 4     | 1   | assignment                                                       | -

* Loc 1: moveToLocation_Buffer_1 start, moveToLocation_Buffer_2 start -> 3
* Loc 2: moveToLocation_Buffer_1 end
* Loc 3: moveToLocation_Buffer_3 start -> 4
* Loc 4: moveToLocation_Buffer_4 start, moveToLocation_Buffer_5 -> 1

# moveToLocation_Semantics

Action                     | start | end | kind									 | new locations
--------------------------------------------------------------------------------------------------------------------------------------
moveToLocation_Semantics_1 | 1     | 2   | ext choice                                                            |
 - comm 1                  | 1     | 2   | moveToLocationCall input comm, 1 label (moveToLocation_Semantics_2)   | 3, 4
 - comm 2                  | 1     | 2   | proceed	comm, 1 label (moveToLocation_Semantics_1) loop          | -
moveToLocation_Semantics_2 | 4     | 2   | getRobotPosition input comm, 1 label (moveToLocation_Semantics_3)     | 5, 6
moveToLocation_Semantics_3 | 6     | 2   | seq comp (moveToLocation_Semantics_4, moveToLocation_Semantics_1)     | -
moveToLocation_Semantics_4 | 6     | 1   | setRobotOrientation output comm, 1 label (moveToLocation_Semantics_5) | 7, 8
moveToLocation_Semantics_5 | 8     | 1   | seq comp (moveToLocation_Semantics_6, moveToLocation_Semantics_7)     | 9
moveToLocation_Semantics_6 | 8     | 9   | Skip                                                                  | -
moveToLocation_Semantics_7 | 9     | 1   | setRobotVelocity output comm, 1 label (moveToLocation_Semantics_8)    | 10, 11
moveToLocation_Semantics_8 | 11    | 1   | Skip                                                                  | -

* Loc 1: moveToLocation_Semantics_1 start -> 3, 1
* Loc 2: moveToLocation_Semantics_1 end
* Loc 3: moveToLocation_Semantics_1 moveToLocationCall intermediate -> 4
* Loc 4: moveToLocation_Semantics_2 start -> 5
* Loc 5: moveToLocation_Semantics_2 intermediate -> 6
* Loc 6: moveToLocation_Semantics_3 start, moveToLocation_Semantics_4 start -> 7
* Loc 7: moveToLocation_Semantics_4 intermediate -> 8
* Loc 8: moveToLocation_Semantics_5 start, moveToLocation_Semantics_6 start -> 9
* Loc 9: moveToLocation_Semantics_7 start -> 10
* Loc 10: moveToLocation_Semantics_7 intermediate -> 11
* Loc 11: moveToLocation_Semantics_8 -> 1

# moveToLocation_Monitor

Action                   | start | end | kind                                                              | new locations
--------------------------------------------------------------------------------------------------------------------------
moveToLocation_Monitor_1 | 1     | 2   | moveToLocationCall input comm, 1 label (moveToLocation_Monitor_2) | 3, 4
moveToLocation_Monitor_2 | 4     | 2   | moveToLocationHappened comm (moveToLocation_Monitor_1 loop)       | -

* Loc 1: moveToLocation_Monitor_1 start -> 3
* Loc 2: moveToLocation_Monitor_1 end
* Loc 3: moveToLocation_Monitor_1 intermediate -> 4
* Loc 4: moveToLocation_Monitor_2 start -> 1



