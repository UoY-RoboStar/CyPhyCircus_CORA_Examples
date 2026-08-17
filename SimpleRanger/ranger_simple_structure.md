# Synchronisation labels

channel		| EnvLoop | EventBuffer | Controller
--------------------------------------------------
target          | no      | yes         | yes
targetTriggered | yes     | yes	        | no
getPos          | yes     | no          | no
getVel          | yes     | no          | no
setPos          | yes     | no          | no
setVel          | yes     | no          | yes
proced          | yes     | no          | yes

target          -> target_EventBuffer_Controller                `(((EnvLoop |[..]|  EventBuffer) \ {..}) |[ target ]| Controller) \ { target }`
targetTriggered -> targetTriggered_EnvLoop_EventBuffer_hidden   `(((EnvLoop |[ targetTriggered ]|  EventBuffer) \ { targetTriggered }) |[..]| Controller) \ {..}`
getPos          -> -                                            `(((EnvLoop |[..]|  EventBuffer) \ {..}) |[ getPos ]| Controller) \ { getPos }`
getVel          -> -                                            `(((EnvLoop |[..]|  EventBuffer) \ {..}) |[ getVel ]| Controller) \ { getVel }`
setPos          -> -                                            `(((EnvLoop |[..]|  EventBuffer) \ {..}) |[ setPos ]| Controller) \ { setPos }`
setVel          -> setVel_EventBuffer_Controller_hidden         `(((EnvLoop |[..]|  EventBuffer) \ {..}) |[ setVel ]| Controller) \ { setVel }`
proceed         -> proceed_EventBuffer_Controller_hidden        `(((EnvLoop |[..]|  EventBuffer) \ {..}) |[ proceed ]| Controller) \ { proceed }`

# EnvLoop

Action	        | start | end | kind                                                            | new locations
---------------------------------------------------------------------------------------------------------------
EnvLoop_1       | 1     | 2   | seq comp (EnvLoop_2, EnvLoop_3)                                 | 3
EnvLoop_2       | 1     | 3   | assignment                                                      | -
EnvLoop_3       | 3     | 2   | seq comp (Movement_1, EnvLoop_4)                                | 4
Movement_1      | 3     | 4   | interrupt by condition (Movement_2)                             | 5
Movement_2      | 3     | 5   | continuous schema                                               | -
EnvLoop_4       | 4     | 2   | seq comp (InputTriggers_1, EnvLoop_5)                           | 6
InputTriggers_1 | 4     | 6   | conditional (InputTriggers_2, InputTriggers_3, InputTriggers_4) | 7, 8, 9
InputTriggers_2 | 7     | 6   | output comm (InputTriggers_5)                                   | 10, 11
InputTriggers_5 | 11    | 6   | Skip                                                            |-
InputTriggers_3 | 8     | 6   | output comm (InputTriggers_6)                                   | 12, 13
InputTriggers_6 | 13    | 6   | Skip                                                            | -
InputTriggers_4 | 9     | 6   | Skip                                                            | -
EnvLoop_5       | 6     | 2   | seq comp (EnvLoop_6, EnvLoop_1) loop                            | -
EnvLoop_6       | 6     | 1   | timeout (QueryUpd_1, EnvLoop_7)                                 | 14, 15, 16
QueryUpd_1      | 14    | 15  | ext choice (5 branches)                                         | -
  - comm 1      | 14    | 15  | getPos output comm, no labels (QueryUpd_2)                      | -
  - comm 2      | 14    | 15  | getVel output comm, no labels (QueryUpd_3)                      | -
  - comm 3      | 14    | 15  | setPos input comm, no label (QueryUpd_4)                        | -
  - comm 4      | 14    | 15  | setVel input comm, 1 label (QueryUpd_5)                         | 17, 18
  - comm 5      | 14    | 15  | proceed simple comm, 1 label (QueryUpd_6)                       | 19
QueryUpd_5      | 18    | 15  | seq comp (QueryUpd_10, QueryUpd_1)                              | -
QueryUpd_10     | 18    | 14  | assignment                                                      | -
QueryUpd_6      | 19    | 15  | Skip                                                            | -
RangerLoop_7    | 16    | 1   | Skip                                                            | -

* Loc 1: EnvLoop_1 start, EnvLoop_2 start -> 3
* Loc 2: EnvLoop_1 end (unused)
* Loc 3: EnvLoop_2 end/Movement_2 start -> 4
* Loc 4: Movement_1 end/InputTriggers_1 start -> 7, 8, 9
* Loc 5: Movement_2 end (unused) -> 4
* Loc 6: InputTriggers_1 end, EnvLoop_6 start (timer init) -> 14
* Loc 7: InputTriggers_2 start -> 10
* Loc 8: InputTriggers_3 start -> 12
* Loc 9: InputTriggers_4 start -> 6
* Loc 10: InputTriggers_2 intermediate -> 11
* Loc 11: InputTriggers_2 end, InputTriggers_5 start -> 6
* Loc 12: InputTriggers_3 intermediate	-> 13
* Loc 13: InputTriggers_3 end, InputTriggers_5 start -> 6
* Loc 14: QueryUpd_1 start -> 17, 19, 16
* Loc 15: QueryUpd_1 end -> 1, 16
* Loc 16: EnvLoop_6 interrupt, EnvLoop_7 start -> 1
* Loc 17: QueryUpd_1 setVel intermediate -> 18, 16
* Loc 18: QueryUpd_5 start, QueryUpd_10 start -> 14, 16
* Loc 19: QueryUpd_6 start -> 15, 16


# EventBuffer

Action          | start | end | kind                                                | new locations
---------------------------------------------------------------------------------------------------
EventBuffer_1   | 1     | 2   | seq comp (EventBuffer_2, EventBuffer_3)             | 3
EventBuffer_2   | 1     | 3   | assignment                                          | -
EventBuffer_3   | 3     | 2   | conditional (EventBuffer_4, EventBuffer_5)          | 4, 5
EventBuffer_4   | 4     | 2   | ext choice                                          |
 - comm 1       | 4     | 2   | targetTriggered input comm, 1 label (EventBuffer_6) | 6, 7
 - comm	2       | 4     | 2   | target comm, 1 label (EventBuffer_7)                | 8
EventBuffer_6   | 7     | 2   | seq comp (EventBuffer_9, EventBuffer_3) loop        | -
EventBuffer_9   | 7     | 3   | assignment                                          | -
EventBuffer_7   | 8     | 2   | seq comp (EventBuffer_10, EventBuffer_3) loop       | -
EventBuffer_10  | 8     | 3   | Skip                                                | -
EventBuffer_5   | 5     | 2   | targetTriggered input comm (EventBuffer_8)          | 9, 10
EventBuffer_8   | 10    | 2   | seq comp (EventBuffer_11, EventBuffer_3) loop       | -
EventBuffer_11  | 10    | 3   | assignment                                          | -


* Loc 1: EventBuffer_1 start, EventBuffer_2 start -> 3
* Loc 2: EventBuffer_1 end (unused)
* Loc 3: EventBuffer_3 start -> 4, 5
* Loc 4: EventBuffer_4 start -> 6, 8
* Loc 5: EventBuffer_5 start -> 9
* Loc 6: EventBuffer_4 targetTriggered intermediate -> 7
* Loc 7: EventBuffer_6 start, EventBuffer_9 start -> 3
* Loc 8: EventBuffer_7 start, EventBuffer_10 start -> 3
* Loc 9: EventBuffer_5 targetTriggered intermediate -> 10
* Loc 10: EventBuffer_8 start, EventBuffer_11 start -> 3

# Controller

Action          | start | end | kind                                       | new locations
Controller_1    | 1     | 2   | seq comp (Controller_2, Controller_3)      | 3
Controller_2    | 1     | 3   | setVel output comm, 1 label (Controller_4) | 4, 5
Controller_4    | 5     | 3   | ext choice                                 |
 - comm 1       | 5     | 3   | target comm, 1 label (Controller_6)        | 6
 - comm 2       | 5     | 3   | proceed comm, 1 label (Controller_4) loop  | -
Controller_6    | 6     | 3   | Skip                                       | -
Controller_3    | 3     | 2   | setVel output comm, 1 label (Controller_5) | 7, 8
Controller_5    | 8     | 2   | proceed comm, 1 label (Controller_5) loop  | -

* Loc 1: Controller_1 start, Controller_2 start -> 4
* Loc 2: Controller_1 end (unused)
* Loc 3: Controller_3 start -> 7
* Loc 4: Controller_2 intermediate -> 5
* Loc 5: Controller_4 start -> 6, 5
* Loc 6: Controller_6 start -> 3
* Loc 7: Controller_3 intermediate -> 8
* Loc 8: Controller_5 start -> 8



