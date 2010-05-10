//#define ROBOT_DASH_MODE

#ifdef ROBOT_DASH_MODE

#define STAND_STRAIGHT_PAGE_NO		2

#define FORWARD_STEP_PAGE_NO		60
//#define FORWARD_STEP_PAGE_NO		78 	/* walk on the spot */
#define BACKWARD_STEP_PAGE_NO		69

#define FORWARD_STEP_COMPLETED_PAGE_NO		65
#define BACKWARD_STEP_COMPLETED_PAGE_NO		73

#define FORWARD_STEP_EXIT_PAGE_NO	67		/* right foot back:exit with 67,left foot back (62) exit with 63 */
#define BACKWARD_STEP_EXIT_PAGE_NO	67		/* CHANGE !*/

#define NO_OF_STEPS_FORWARD			19 /* 18 steps is perfect*/
#define NO_OF_STEPS_BACKWARD		20

extern volatile int Forward_Dash_StepNo;
extern volatile int Backward_Dash_StepNo;
#endif

