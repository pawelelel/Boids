#include <math.h>
#include <time.h>

#include "pcl.h"

#define NUMBER_OF_BOIDS 20
#define VIEW_RADIUS 10.0
#define MAX_SPEED 0.01
#define MIN_SPEED 0.0001
#define MIN_DISTANCE 2.0 // separation
#define NOISE 0.01

struct Boid {
	double row, column;
	double direction;
	double speed;
};

double calculateDistance(struct Boid A, struct Boid B) {
	double row = A.row - B.row;
	double column = A.column - B.column;

	row *= row;
	column *= column;

	double sum = row + column;

	double d = sqrt(sum);
	return d;
}

void showBoid(struct Boid b, struct AsciiScreen* ascii) {
	int row = (int)round(b.row);
	int column = (int)round(b.column);

	if (b.direction < 0)
		b.direction += 2 * M_PI;

	int sector = (int)round(b.direction / (M_PI / 2)) % 4;

	char symbols[4] = {'v', '>', '^', '<'};
	char c = symbols[sector];

	setcharcursorascii(ascii, c, row, column);
}

int main(void) {
	struct Console* console = start();
	struct AsciiScreen* ascii = initascii(console);
	hidecursorascii(ascii);
	setinputblock(console, FALSE);

	unsigned int width, height;
	getsize(console, &width, &height);

	struct Boid* boids = malloc(NUMBER_OF_BOIDS * sizeof(struct Boid));

	// initializing boids
	srand(time(NULL));
	for (int i = 0; i < NUMBER_OF_BOIDS; ++i) {
		boids[i].row = rand()/(float)(RAND_MAX/(height));
		boids[i].column = rand()/(float)(RAND_MAX/(width));
		boids[i].direction = rand()/(float)(RAND_MAX/(2*M_PI));
		boids[i].speed = rand()/(float)(RAND_MAX/MAX_SPEED) + MIN_SPEED;
	}

	int input = 0;
	while (input != KEY_ESC) {
		clearascii(ascii);

		int err = getsize(console, &width, &height);

		for (int i = 0; i < NUMBER_OF_BOIDS; i++) {
			boids[i].row += cos(boids[i].direction) * boids[i].speed;
			boids[i].column += sin(boids[i].direction) * boids[i].speed;

			if (boids[i].row < 0) {
				boids[i].row = height - 1;
			}
			if (boids[i].row > height - 1) {
				boids[i].row = 0;
			}
			if (boids[i].column < 0){
				boids[i].row = width - 1;
			}
			if (boids[i].column > width - 1) {
				boids[i].column = 0;
			}

			showBoid(boids[i], ascii);
		}

		double newDirections[NUMBER_OF_BOIDS];
		for (int i = 0; i < NUMBER_OF_BOIDS; i++) {
			struct Boid a = boids[i];
			double moveX = 0;
			double moveY = 0;

			double alignX = 0;
			double alignY = 0;

			double cohX = 0;
			double cohY = 0;

			int neighbors = 0;
			for (int j = 0; j < NUMBER_OF_BOIDS; ++j) {
				if (i != j) {
					struct Boid b = boids[j];
					double distance = calculateDistance(a, b);

					if (distance < VIEW_RADIUS) {
						neighbors++;

						alignX += cos(b.direction);
						alignY += sin(b.direction);

						cohX += b.row;
						cohY += b.column;

						if (distance < MIN_DISTANCE) {
							moveX -= (b.row - a.row);
							moveY -= (b.column - a.column);
						}
					}
				}
			}

			if (neighbors > 0) {

				alignX /= neighbors;
				alignY /= neighbors;

				cohX = (cohX / neighbors) - a.row;
				cohY = (cohY / neighbors) - a.column;
			}

			double vx = cos(a.direction) * a.speed;
			double vy = sin(a.direction) * a.speed;

			vx += 0.05 * moveX;
			vx += 0.05 * alignX;
			vx += 0.01 * cohX;

			vy += 0.05 * moveY;
			vy += 0.05 * alignY;
			vy += 0.01 * cohY;

			double desired = atan2(vy, vx);
			newDirections[i] = (desired - boids[i].direction) * 0.1;
		}

		for (int i = 0; i < NUMBER_OF_BOIDS; ++i) {
			boids[i].direction += newDirections[i];

			double noise = ((double)rand() / RAND_MAX - 0.5) * NOISE;

			boids[i].direction += noise;
		}

		refreshascii(console, ascii);
		input = getcharacter(console);
	}

	free(boids);

	end(console);
	return 0;
}