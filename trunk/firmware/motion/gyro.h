
#define HIS_MAX		10

#define NOW		(HIS_MAX-1)

#define MOVING_NEUTRAL_POSITON_COMPONENT	0.012

#define A2D_FRONT	0
#define A2D_LEFT	1

#define A2D_CHANNEL_FRONT	1
#define A2D_CHANNEL_SIDE	3


void ResetGyro(void);
void InitGyro(void);
void MeasureGyro(void);

