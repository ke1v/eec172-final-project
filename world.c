#include "stdio.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

// BEGIN PLACEHOLD VARIABLES

const unsigned long time;

//   END PLACEHOLD VARIABLES

#define WORLD_SIZE           10
#define WORLD_SECTOR_UNKNOWN 0
#define WORLD_SECTOR_CLEAR   1
#define WORLD_SECTOR_BLOCKED 2

/**
 * @brief Represents the 'world' view of the robot, storing detected obstacles.
 *
 * This struct contains obstacle data detected by the robot. The world is
 * represented by a 2D array of integers, with each value as 0 if unknown, and 1
 * if known clear, and 2 if known blocked.
 *
 * @note The world is dynamic, and obstacles can change or move. Use this data
 * for planning, but check path with real world sensors before moving.
 *
 */
typedef struct World {
	size_t blocked[WORLD_SIZE][WORLD_SIZE];
} World;

/**
 * @brief Represents the robot and its state
 *
 * This struct contains all the data that represent the current state of the
 * robot. The x and y positions range from [0, WORLD_SIZE). Heading ranges
 * from [0, 2*PI) with 0 corresponding to +x, and an increase in heading corresponds to a clockwise rotation.
 *
 */
typedef struct Robot {
	unsigned int x;
	unsigned int y;
	float heading;
} Robot;

typedef struct RawAccelerometerCorrection {
	int x_bias;
	int y_bias;
	int z_bias;
} AccelCorrection;

typedef struct AccelerometerData {
	unsigned int dt;
	unsigned int ax;
	unsigned int ay;
	unsigned int az;
} AccelerometerData;

typedef struct DriveMotionData {
	unsigned int dt;
	unsigned int pl;
	unsigned int pr;
} DriveMotionData;

void dead_reckon();

// https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
// void plotLineHigh(int sx, int sy, int ex, int ey) {
// 	int dx = ex - sx;
// 	int dy = ey - sy;
// 	int xi = 1;
// 	if (dx < 0) {
// 		xi = -1;
// 		dx = -dx;
// 	}
// 	int d = (2 * dx) - dy;
// 	int x = sx;

// 	int y = sy;
// 	for (y = sy; y <= ey; y++) {
// 		plot(x, y);
// 		if (d > 0) {
// 			x = x + xi;
// 			d = d + (2 * (dx - dy));
// 		} else {
// 			d = d + 2 * dx;
// 		}
// 	}
// }

// void plotLineLow(int sx, int sy, int ex, int ey) {
// 	int dx = ex - sx;
// 	int dy = ey - sy;
// 	int yi = 1;
// 	if (dy < 0) {
// 		yi = -1;
// 		dy = -dy;
// 	}
// 	int d = (2 * dy) - dx;
// 	int y = sy;

// 	int x = sx;
// 	for (x = sx; x <= ex; x++) {
// 		plot(x, y);
// 		if (d > 0) {
// 			y = y + yi;
// 			d = d + (2 * (dy - dx));
// 		} else {
// 			d = d + 2 * dy;
// 		}
// 	}
// }

void drawLine(int sx, int sy, float angle, int dist, int blockedEnd, World *world) {
	int ex = sx + (int)(cos(angle) * dist);
	int ey = sy + (int)(sin(angle) * dist);
	printf("SX: %d, EX: %d, SY: %d, EY: %d\n", sx, ex, sy, ey);
	printf("Cos %d Sin %d\n", (int)(cos(angle) * dist), (int)(sin(angle) * dist));

	int dx = abs(ex - sx);
	int xi = sx < ex ? 1 : -1;
	int dy = -abs(ey - sy);
	int yi = sy < ey ? 1 : -1;
	int error = dx + dy;

	while (1) {
		if (sx < 0 || sx >= WORLD_SIZE || sy < 0 || sy >= WORLD_SIZE) {
			break;
		}
		world->blocked[sy][sx] = WORLD_SECTOR_CLEAR;
		int e2 = 2 * error;
		if (e2 >= yi) {
			if (sx == ex) {
				break;
			}
			error = error + dy;
			sx = sx + xi;
		}
		if (e2 <= xi) {
			if (sy == ey) {
				break;
			}
			error = error + dx;
			sy = sy + yi;
		}
	}
	if (blockedEnd) {
		
	}
}

void print_world(World *world) {
	int i, j;
	printf("  ");
	for (i = 0; i < WORLD_SIZE; i++) {
		printf("%d ", i);
	}
	printf("\n");
	for (i = 0; i < WORLD_SIZE; i++) {
		printf("%d ", i);
		for (j = 0; j < WORLD_SIZE; j++) {
			printf("%zu ", world->blocked[i][j]);
		}
		printf("%d ", i);
		printf("\n");
	}
	printf("  ");
	for (i = 0; i < WORLD_SIZE; i++) {
		printf("%d ", i);
	}
	printf("\n");
};

int main() {
	World world = {.blocked = {
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	               }};

	print_world(&world);

	printf("----------\n");

	drawLine(3, 5, 0.3, 10, 0, &world);

	print_world(&world);

	return 0;
}