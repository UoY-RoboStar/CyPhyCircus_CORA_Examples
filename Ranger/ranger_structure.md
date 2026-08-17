# Synchronisation labels

(((EL |[obstacleTriggered]| (mB ||| oB)) \ (obstacleTriggered) |[...]| (mS |[moveCall]| mM)) \ (...)

channel                         | EnvironmentLoop | obstacle_Buffer | move_Buffer | move_Semantics | move_Monitor
-----------------------------------------------------------------------------------------------------------------
obstacle                        | no              | yes             | no          | no             | no
moveCall                        | no              | no              | no          | yes            | yes
tock                            | yes             | no              | no          | no             | no
obstacleTriggered               | yes             | yes             | no          | no             | no
moveHappened                    | no              | no              | yes         | no             | yes
getRobotPosition                | yes             | no              | no          | no             | no
getRobotVelocity                | yes             | no              | no          | no             | no
getRobotAcceleration            | yes             | no              | no          | no             | no
getRobotOrientation             | yes             | no              | no          | yes            | no
getRobotAngularVelocity         | yes             | no              | no          | no             | no
getRobotAngularAcceleration     | yes             | no              | no          | no             | no
setRobotPosition                | yes             | no              | no          | no             | no
setRobotVelocity                | yes             | no              | no          | yes            | no
setRobotAcceleration            | yes             | no              | no          | no             | no
setRobotOrientation             | yes             | no              | no          | no             | no
setRobotAngularVelocity         | yes             | no              | no          | yes            | no
setRobotAngularAcceleration     | yes             | no              | no          | no             | no
getMaxObstacleID                | yes             | no              | no          | no             | no
getObstaclePosition             | yes             | no              | no          | no             | no
getObstacleOrientation          | yes             | no              | no          | no             | no
proceed                         | yes             | no              | no          | yes            | no

obstacle                        -> obstacle_oB                          `((EL |[..]| (mB ||| oB)) \ {..} |[..]| (mS |[..]| mM)) \ {..}`
moveCall                        -> moveCall_mS_mM                       `((EL |[..]| (mB ||| oB)) \ {..} |[..]| (mS |[ moveCall ]| mM)) \ {..}`
tock                            -> tock_EL                              `((EL |[..]| (mB ||| oB)) \ {..} |[..]| (mS |[..]| mM)) \ {..}`
obstacleTriggered               -> obstacleTriggered_EL_oB_hidden       `((EL |[ obstacleTriggered ]| (mB ||| oB)) \ { obstacleTriggered } |[..]| (mS |[..]| mM)) \ {..}`
moveHappened                    -> moveHappened_mB_mM_hidden            `((EL |[..]| (mB ||| oB)) \ {..} |[ moveHappended ]| (mS |[..]| mM)) \ { moveHappened }`
getRobotPosition                -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getRobotPosition ]| (mS |[..]| mM)) \ { getRobotPosition }`
getRobotVelocity                -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getRobotVelocity ]| (mS |[..]| mM)) \ { getRobotVelocity }`
getRobotAcceleration            -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getRobotAcceleration ]| (mS |[..]| mM)) \ { getRobotAcceleration }`
getRobotOrientation             -> getRobotOrientation_EL_mS_hidden     `((EL |[..]| (mB ||| oB)) \ {..} |[ getRobotOrientation ]| (mS |[..]| mM)) \ { getRobotOrientation }`
getRobotAngularVelocity         -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getRobotAngularVelocity ]| (mS |[..]| mM)) \ { getRobotAngularVelocity }`
getRobotAngularAcceleration     -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getRobotAngularAcceleration ]| (mS |[..]| mM)) \ { getRobotAngularAcceleration }`
setRobotPosition                -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ setRobotPosition ]| (mS |[..]| mM)) \ { setRobotPosition }`
setRobotVelocity                -> setRobotVelocity_EL_mS_hidden        `((EL |[..]| (mB ||| oB)) \ {..} |[ setRobotVelocity ]| (mS |[..]| mM)) \ { setRobotVelocity }`
setRobotAcceleration            -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ setRobotAcceleration ]| (mS |[..]| mM)) \ { setRobotAcceleration }`
setRobotOrientation             -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ setRobotOrientation ]| (mS |[..]| mM)) \ { setRobotOrientation }`
setRobotAngularVelocity         -> setRobotAngularVelocity_EL_mS_hidden `((EL |[..]| (mB ||| oB)) \ {..} |[ setRobotAngularVelocity ]| (mS |[..]| mM)) \ { setRobotAngularVelocity }`
setRobotAngularAcceleration     -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ setRobotAngularAcceleration ]| (mS |[..]| mM)) \ { setRobotAngularAcceleration }`
getMaxObstacleID                -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getMaxObstacleID ]| (mS |[..]| mM)) \ { getMaxObstacleID }`
getObstaclePosition             -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getObstaclePosition ]| (mS |[..]| mM)) \ { getObstaclePosition }`
getObstacleOrientation          -> -                                    `((EL |[..]| (mB ||| oB)) \ {..} |[ getObstacleOrientation ]| (mS |[..]| mM)) \ { getObstacleOrientation }`
proceed                         -> proceed_EL_mS_hidden                 `((EL |[..]| (mB ||| oB)) \ {..} |[ proceed ]| (mS |[..]| mM)) \ { proceed }`

# EnvironmentLoop

Action                       | start | end | kind                                                                                                   | new locations
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
EnvironmentLoop_1            | 1     | 2   | seq comp (EnvironmentLoop_2, EnvironmentLoop_3)                                                        | 3
EnvironmentLoop_2            | 1     | 3   | assignment                                                                                             | -
EnvironmentLoop_3            | 3     | 2   | seq comp (RobotMovementAction_1, EnvironmentLoop_4)                                                    | 4
RobotMovementAction_1        | 3     | 4   | interrupt by condition (RobotMovement)                                                                 | 5
RobotMovement                | 3     | 5   | continuous schema                                                                                      | -
EnvironmentLoop_4            | 4     | 2   | seq comp (EnvironmentLoop_5, EnvironmentLoop_3) loop                                                   | -
EnvironmentLoop_5            | 4     | 3   | conditional (HandleCollision_1, EnvironmentLoop_6)                                                     | 6, 7
HandleCollision_1            | 6     | 3   | assignment                                                                                             |-
EnvironmentLoop_6            | 7     | 3   | seq comp (InputTriggers_1, EnvironmentLoop_7)                                                          | 8
InputTriggers_1              | 7     | 8   | action reference (obstacle\_InputEventMapping\_1)	                                                    | -
obstacle_InputEventMapping_1 | 7     | 8   | conditional (obstacle_InputEventMapping_2, obstacle_InputEventMapping_3, obstacle_InputEventMapping_4) | 9, 10, 11
obstacle_InputEventMapping_2 | 9     | 8   | obstacleTriggered output comm (obstacle_InputEventMapping_5)                                           | 12, 13
obstacle_InputEventMapping_5 | 13    | 8   | assignment                                                                                             | -
obstacle_InputEventMapping_3 | 10    | 8   | obstacleTriggered output comm (obstacle_InputEventMapping_6)                                           | 14, 15
obstacle_InputEventMapping_6 | 15    | 8   | Skip                                                                                                   | -
obstacle_InputEventMapping_4 | 11    | 8   | Skip                                                                                                   | -
EnvironmentLoop_7            | 8     | 3   | seq comp (Communication_1, EnvironmentLoop_8)                                                          | 16
Communication_1              | 8     | 16  | ext choice                                                                                             |
 - comm 1                    | 8     | 16  | getRobotPosition output comm, no labels (Communication_2)                                              | -
 - comm 2                    | 8     | 16  | getRobotVelocity output comm, no labels (Communication_3)                                              | -
 - comm 3                    | 8     | 16  | getRobotAcceleration output comm, no labels (Communication_4)                                          | -
 - comm 4                    | 8     | 16  | getRobotOrientation output comm, 1 label (Communication_5)                                             | 17, 18
 - comm 5                    | 8     | 16  | getRobotAngularVelocity output comm, no labels (Communication_6)                                       | -
 - comm 6                    | 8     | 16  | getRobotAngularAcceleration output comm, no labels (Communication_7)                                   | -
 - comm 7                    | 8     | 16  | getRobotPosition input comm, no labels (Communication_8)                                               | -
 - comm 8                    | 8     | 16  | setRobotVelocity input comm, 1 label (Communication_9)                                                 | 19, 20
 - comm 9                    | 8     | 16  | setRobotAcceleration input comm, no labels (Communication_10)                                          | -
 - comm 10                   | 8     | 16  | setRobotOrientation input comm, no labels (Communication_11)                                           | -
 - comm 11                   | 8     | 16  | setRobotAngularVelocity input comm, 1 label (Communication_12)                                         | 21, 22
 - comm 12                   | 8     | 16  | setRobotAngularAcceleration input comm, no labels (Communication_13)                                   | -
 - comm 13                   | 8     | 16  | getMaxObstacleID ouput comm, no labels (Communication_14)                                              | -
 - comm 14                   | 8     | 16  | getObstaclePosition mixed comm, no labels (Communication_15)                                           | -
 - comm 15                   | 8     | 16  | getObstacleOrientation mixed comm, no labels (Communication_16)                                        | -
 - comm 16                   | 8     | 16  | proceed comm, 1 label (Communication_17)                                                               | 23
Communication_5              | 18    | 16  | seq comp (Communication_21, Communication_1) loop                                                      | -
Communication_21             | 18    | 8   | Skip                                                                                                   | -
Communication_9              | 20    | 16  | seq comp (Communication_25, Communication_1) loop                                                      | -
Communication_25             | 20    | 8   | assignment                                                                                             | -
Communication_12             | 22    | 16  | seq comp (Communication_28, Communication_1) loop                                                      | -
Communication_28             | 22    | 8   | assignment                                                                                             | -
Communication_17             | 23    | 16  | Skip                                                                                                   | -
EnvironmentLoop_8            | 16    | 3   | seq comp (CheckTock_1, EnvironmentLoop_9)                                                              | 24
CheckTock_1                  | 16    | 24  | conditional (CheckTock_2, CheckTock_3)                                                                 | 25, 26
CheckTock_2                  | 25    | 24  | tock comm, 1 label (CheckTock_4)                                                                       | 27
CheckTock_4                  | 27    | 24  | assignment                                                                                             | -
CheckTock_3                  | 26    | 24  | Skip                                                                                                   | -
EnvironmentLoop_9            | 24    | 3   | assignment                                                                                             | -

* Loc 1: EnvironmentLoop_1 start, EnvironmentLoop_2 start -> 3
* Loc 2: EnvironmentLoop_2 end (unused)
* Loc 3: EnvironmentLoop_3 start, RobotMovementAction_1 start, RobotMovement start -> 4
* Loc 4: EnvironmentLoop_4 start, EnvironmentLoop_5 start -> 6, 7
* Loc 5: RobotMovement end (unused) -> 4
* Loc 6: HandleCollision_1 start -> 3
* Loc 7: EnvironmentLoop_6 start, InputTriggers_1 start, obstacle_InputEventMapping_1 start -> 9, 10, 11
* Loc 8: EnvironmentLoop_7 start, Communication_1 start -> 17, 19, 21, 23
* Loc 9: obstacle_InputEventMapping_2 start -> 12
* Loc 10: obstacle_InputEventMapping_3 start -> 14
* Loc 11: obstacle_InputEventMapping_4 start -> 8
* Loc 12: obstacle_InputEventMapping_2 intermediate -> 13
* Loc 13: obstacle_InputEventMapping_5 start -> 8
* Loc 14: obstacle_InputEventMapping_3 intermediate -> 15
* Loc 15: obstacle_InputEventMapping_6 start -> 8
* Loc 16: EnvironmentLoop_8 start, CheckTock_1 start -> 25, 26
* Loc 17: Communication_1 getRobotOrientation intermediate -> 18
* Loc 18: Communication_5 start, Communication_21 start -> 8
* Loc 19: Communication_1 setRobotVelocity intermediate -> 20
* Loc 20: Communication_9 start, Communication_25 start -> 8
* Loc 21: Communication_1 setRobotAngularVelocity intermediate -> 22
* Loc 22: Communication_12 start, Communication_28 start -> 8
* Loc 23: Communication_17 start -> 16
* Loc 24: EnvironmentLoop_9 start -> 3
* Loc 25: CheckTock_2 start -> 27
* Loc 26: CheckTock_3 start -> 24
* Loc 27: CheckTock_4 start -> 24

# obstacle_Buffer

Action             | start | end | kind                                                         | new locations
---------------------------------------------------------------------------------------------------------------
obstacle_Buffer_1  | 1     | 2   | seq comp (obstacle_Buffer_2, obstacle_Buffer_3)              | 3
obstacle_Buffer_2  | 1     | 3   | assignment                                                   | -
obstacle_Buffer_3  | 3     | 2   | conditional (obstacle_Buffer_4, obstacle_Buffer_5)           | 4, 5
obstacle_Buffer_4  | 4     | 2   | ext choice                                                   |
 - comm 1          | 4     | 2   | obstacleTriggered input comm, 1 label (obstacle_Buffer_6)    | 6, 7
 - comm 2          | 4     | 2   | obstacle output comm, 1 label (obstacle_Buffer_7)            | 8, 9
obstacle_Buffer_5  | 5     | 2   | obstacleTriggered input comm, 1 label (obstacle_Buffer_8)    | 10, 11
obstacle_Buffer_6  | 7     | 2   | seq comp (obstacle_Buffer_9, obstacle_Buffer_3) loop         | -
obstacle_Buffer_9  | 7     | 3   | assignment                                                   | -
obstacle_Buffer_7  | 9     | 2   | seq comp (obstacle_Buffer_10, obstacle_Buffer_3) loop        | -
obstacle_Buffer_10 | 9     | 3   | Skip                                                         | -
obstacle_Buffer_8  | 11    | 2   | seq com (obstacle_Buffer_11, obstacle_Buffer_3) loop         | -
obstacle_Buffer_11 | 11    | 3   | assignment                                                   | -

* Loc 1: obstacle_Buffer_1 start, obstacle_Buffer_2 start -> 3
* Loc 2: obstacle_Buffer_1 end (unused)
* Loc 3: obstacle_Buffer_3 start -> 4, 5
* Loc 4: obstacle_Buffer_4 start -> 6, 8
* Loc 5: obstacle_Buffer_5 start -> 10
* Loc 6: obstacle_Buffer_4 obstacleTriggered intermediate -> 7
* Loc 7: obstacle_Buffer_6 start, obstacle_Buffer_9 start -> 3
* Loc 8: obstacle_Buffer_4 obstacle intermediate -> 9
* Loc 9: obstacle_Buffer_7 start, obstacle_Buffer_10 start -> 3
* Loc 10: obstacle_Buffer_5 obstacleTriggered intermediate -> 11
* Loc 11: obstacle_Buffer_8_start, obstacle_Buffer_11 start -> 3

# move_Buffer

Action          | start | end   | kind                                          | new locations
-----------------------------------------------------------------------------------------------
move_Buffer_1   | 1     | 2     | seq comp (move_Buffer_2, move_Buffer_3)       | 3
move_Buffer_2   | 1     | 3     | assignment                                    | -
move_Buffer_3   | 3     | 2     | moveHappened comm, 1 label (move_Buffer_4)    | 4
move_Buffer_4   | 4     | 2     | seq comp (move_Buffer_5, move_Buffer_3) loop  | -
move_Buffer_5   | 4     | 3     | assignment                                    | -

* Loc 1: move_Buffer_1 start, move_Buffer_2 start -> 3
* Loc 2: move_Buffer_1 end (unused)
* Loc 3: move_Buffer_3 start -> 4
* Loc 4: move_Buffer_4 start, move_Buffer_5 start -> 3

# move_Semantics

Action                  |start  | end   | kind                                                          | new locations
-----------------------------------------------------------------------------------------------------------------------
move_Semantics_1        | 1     | 2     | ext choice                                                    | 
 - comm 1               | 1     | 2     | moveCall input comm, 1 label (move_Semantics_2)               | 3, 4
 - comm 2               | 1     | 2     | proceed comm, 1 label (move_Semantics_1) loop                 | -
move_Semantics_2        | 4     | 2     | getRobotOrientation input comm, 1 label (move_Semantics_3)    | 5, 6
move_Semantics_3        | 6     | 2     | seq comp (move_Semantics_4, move_Semantics_5)                 | 7
move_Semantics_4        | 6     | 7     | setRobotVelocity output comm, 1 label (move_Semantics_6)      | 8, 9
move_Semantics_6        | 9     | 7     | Skip                                                          | -
move_Semantics_5        | 7     | 2     | setRobotAngularVelocity output comm (move_Semantics_7)        | 10, 11
move_Semantics_7        | 11    | 2     | seq comp (move_Semantics_8, move_Semantics_1) loop            | -
move_Semantics_8        | 11    | 1     | Skip                                                          | -

* Loc 1: move_Semantics_1 start -> 3, 1
* Loc 2: move_Semantics_1 end
* Loc 3: move_Semantics_1 moveCall intermediate -> 4
* Loc 4: move_Semantics_2 start -> 5
* Loc 5: move_Semantics_2 intermediate -> 6
* Loc 6: move_Semantics_3 start, move_Semantics_4 start -> 8
* Loc 7: move_Semantics_5 start -> 10
* Loc 8: move_Semantics_4 intermediate -> 9
* Loc 9: move_Semantics_6 start -> 7
* Loc 10: move_Semantics_5 intermediate -> 11
* Loc 11: move_Semantics_7 start, move_Semantics_8 start -> 1

# move_Monitor

Action          | start | end | kind                                            | new locations
-----------------------------------------------------------------------------------------------
move_Monitor_1  | 1     | 2   | moveCall input comm, 1 label (move_Monitor_2)   | 3, 4
move_Monitor_2  | 4     | 2   | moveHappened comm, 1 label (move_Monitor_1)     | -

* Loc 1: move_Monitor_1 start -> 3
* Loc 2: move_Monitor_1 end
* Loc 3: move_Monitor_1 intermediate -> 4
* Loc 4: move_Monitor_2 start -> 1

