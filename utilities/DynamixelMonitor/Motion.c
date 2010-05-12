#include <libc.h>
#include "Motion.h"

struct page *pages = NULL;

#define N_SERVOS 18

#define BUF_SIZE 128*1024

int
main(void)
{
	int fd = open("motion.bin",O_RDONLY);
	int i,j,k;

	if(fd == -1) {
		perror("Couldn't open motion.bin");
		exit(1);
	}

	pages = malloc(BUF_SIZE);

	if(BUF_SIZE != read(fd, pages, BUF_SIZE)) {
		perror("Couldn't read BUF_SIZE from file");
		exit(1);
	}

	for(i=0;i<BUF_SIZE/sizeof(struct page);i++) {
		printf("page %d: %d poses\n", i, pages[i].header.numPoses);
		struct pose *poses = pages[i].rec;
		for(j=0;j<pages[i].header.numPoses;j++) {
			printf("pose %d: delay=%d, speed=%d\n", j, poses[j].delay, poses[j].speed);
			for(k=0;k<N_SERVOS;k++) {
				printf("%03x ", poses[j].posData[k+1]);
			}
			printf("\n");
		}
		i++;
	}

	return 0;
}
