                      IDLE
                        |
                        v
|---------------->NAVIGATE_HOME
|                        |
|                        v                  No
|                    DETECT_OBJECTS------------------------------>MISSION_COMPLETE
|                        | Yes 
|                        v
|                    IDENTIFY_OBJECT
|                        |
|                        v
|                    Is object within pickup range?
|                        |
|                    +--+--+
|                    |     |
|                    Yes    No
|                    |     |
|                    |  APPROACH_OBJECT
|                    |     |
|                    |  Navigate forward
|                    |     |
|                    |  Detect again
|                    |     |
|                    +-----+
|                        |
|                        v
|                    PICK_OBJECT
|                        |
|                        v
|                    NAVIGATE_TO_BAY
|                        |
|                        v
|                    PLACE_OBJECT
|                        |
|                        v
|<-------------------RETURN_HOME