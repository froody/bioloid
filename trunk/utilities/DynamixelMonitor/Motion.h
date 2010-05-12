#pragma pack(0)
struct pose {
	uint16_t posData[31];
	uint8_t delay;
	uint8_t speed;
};

typedef struct {
	char name[14];
	uint8_t res1[6];
	uint8_t numPoses;
	uint8_t playCode;
	uint8_t pageSpeed;
	uint8_t dxlSetup;
	uint8_t accelTime;
	uint8_t nextPage;
	uint8_t exitPage;
	uint8_t linkedPage1;
	uint8_t linkedPage1PlayCode;
	uint8_t linkedPage2;
	uint8_t linkedPage2PlayCode;
	uint8_t checkSum;
	uint8_t res2[32];
} PageHeader;

struct page {
	PageHeader header;
	struct pose rec[7];
};
#pragma pack()
