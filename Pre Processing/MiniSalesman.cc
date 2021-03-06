#include <iostream>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <fstream>
#include "Coordinates.cc"
#include <pthread.h>

using namespace std;
int NUM_THREADS = 8;
int startCount = NUM_THREADS;

pthread_mutex_t exitMutex, startCountMutex;
pthread_cond_t exitCond;
double sum = 1000000;
Coordinates locations[13];
double points[13][13];
Coordinates shortestThreads[8][13];
double sums[8];



double distanceFormula(double x1, double x2, double y1, double y2) {
	double tmp = pow((y2 - y1), 2);
	double tmp2 = pow((x2 - x1), 2);
	return sqrt(tmp2 + tmp);
}

void calcDistances() {
	int rows = 13;
	int cols = 13;

	for (int i = 0; i < rows; i++) {
		for (int j = i; j < cols; j++) {
			if (i == j) {
				points[i][j] = 0;
			}
			else {
				points[i][j] = distanceFormula(locations[i].getX(),
					locations[j].getX(), locations[i].getY(),
					locations[j].getY());
				points[j][i] = points[i][j];
			}
		}

	}
}

void* calcShortest(void* v) {
	// Cast thread number into
	int threadNo = (unsigned long)v;
	sums[threadNo] = 100000;
	shortestThreads[threadNo][0] = locations[0];
	int t, counter = 0;
	double tmpSum;

	for (int a = 0; a < 13; ++a) {
		for (int b = 0; b < 13; ++b) {
			// cout << "Row: " << a  << " and Col " << b << ": " << points[a][b] << endl;
		}
	}

	// For loop from HELL
	for (int i = threadNo + 1; i < 13; i += NUM_THREADS) {
		for (int j = 1; j < 13; ++j) {
			if (j == i) {
				continue;
			}
			for (int k = 1; k < 13; ++k) {
				if (k == i || k == j) {
					continue;
				}
				for (int l = 1; l < 13; ++l) {
					if (l == k || l == j || l == i) {
						continue;
					}
					for (int m = 1; m < 13; ++m) {
						if (m == l || m == k || m == j || m == i) {
							continue;
						}
						for (int n = 1; n < 13; ++n) {
							if (n == l || n == k || n == j || n == i || n == m) {
								continue;
							}
							for (int o = 1; o < 13; ++o) {
								if (o == l || o == k || o == j || o == i || o == m || o == n) {
									continue;
								}
								for (int p = 1; p < 13; ++p) {
									if (p == l || p == k || p == j || p == i || p == m || p == n || p == o) {
										continue;
									}
									for (int q = 1; q < 13; ++q) {
										if (q == l || q == k || q == j || q == i || q == m || q == n || q == o || q == p) {
											continue;
										}
										for (int r = 1; r < 13; ++r) {
											if (r == l || r == k || r == j || r == i || r == m || r == n || r == o || r == p || r == q) {
												continue;
											}
											for (int s = 1; s < 13; ++s) {
												if (s == l || s == k || s == j || s == i || s == m || s == n || s == o || s == p || s == q || s == r) {
													continue;
												}
												// cout << "i " << i << endl;
												// cout << "j " << j << endl;
												// cout << "k " << k << endl;
												// cout << "l " << l << endl;
												// cout << "m " << m << endl;
												// cout << "n " << n << endl;
												// cout << "o " << o << endl;
												// cout << "p " << p << endl;
												// cout << "q " << q << endl;
												// cout << "r " << r << endl;
												// cout << "s " << s << endl;
												// cout << "t " << t << endl;
												t = 78 - (i + j + k + l + m + n + o + p + q + r + s);
												tmpSum = points[0][i] + points[i][j] + points[j][k]
													+ points[k][l] + points[l][m] + points[m][n]
													+ points[n][o] + points[o][p] + points[p][q]
													+ points[q][r] + points[r][s] + points[s][t];
												// cout << tmpSum << " Counting: " << counter << endl;
												counter++;
												if (tmpSum < sums[threadNo]) {
													sums[threadNo] = tmpSum;
													shortestThreads[threadNo][1] = locations[i];
													shortestThreads[threadNo][2] = locations[j];
													shortestThreads[threadNo][3] = locations[k];
													shortestThreads[threadNo][4] = locations[l];
													shortestThreads[threadNo][5] = locations[m];
													shortestThreads[threadNo][6] = locations[n];
													shortestThreads[threadNo][7] = locations[o];
													shortestThreads[threadNo][8] = locations[p];
													shortestThreads[threadNo][9] = locations[q];
													shortestThreads[threadNo][10] = locations[r];
													shortestThreads[threadNo][11] = locations[s];
													shortestThreads[threadNo][12] = locations[t];
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	// cout << "Count " << counter << endl;
	pthread_mutex_lock(&startCountMutex);
	startCount--;
	if (startCount == 0) {
		pthread_mutex_unlock(&startCountMutex);
		pthread_mutex_lock(&exitMutex);
		pthread_cond_broadcast(&exitCond);
		pthread_mutex_unlock(&exitMutex);
	}
	else {
		pthread_mutex_unlock(&startCountMutex);
	}
	return 0;
}

int main(int argc, const char* argv[]) {

	ifstream input(argv[1]);

	string initPoints;
	locations[0] = new Coordinates(0, 0, 0);
	bool negative = false;
	bool onX = true;
	char holder[1];
	int order = 1;
	double x;
	double y;
	getline(input, initPoints);
	for (int i = 0; i < initPoints.length(); i++) {
		const char charNum = initPoints.at(i);
		holder[0] = charNum;
		if (onX) {
			if (charNum == '-') {
				negative = true;
			}
			else {
				if (negative) {
					x = atoi(holder);
					x = x * -1;
					negative = false;
				}
				else {
					x = atoi(holder);
					negative = false;
				}
				onX = false;
			}
		}
		else {
			if (charNum == '-') {
				negative = true;
			}
			else {
				if (negative) {
					y = atoi(holder);
					y = y * -1;
					negative = false;
				}
				else {
					y = atoi(holder);
					negative = false;
				}
				onX = true;
				locations[order] = new Coordinates(x, y, order);
				order++;
			}

		}


	}
	for (int i = 0; i < 13; i++) {
		cout << "Coordinate " << locations[i].getOrder() << ": X = "
			<< locations[i].getX()
			<< " and Y = " << locations[i].getY() << endl;
	}
	cout << "-----------------------------" << endl;


	int distance;
	calcDistances();

	pthread_mutex_init(&exitMutex, 0);
	pthread_mutex_init(&startCountMutex, 0);

	pthread_cond_init(&exitCond, 0);
	pthread_mutex_lock(&exitMutex);
	// cout << "-------------------------------------" << endl;
	pthread_t* pts = (pthread_t*)malloc(NUM_THREADS * sizeof(pthread_t));
	for (int threadNo = 0; threadNo < NUM_THREADS; ++threadNo) {
		pthread_create(pts + threadNo, 0, calcShortest, (void *)threadNo);
	}

	pthread_cond_wait(&exitCond, &exitMutex);

	int smallestIndex = 0;
	double smallVal = 10000;
	for (int j = 0; j < NUM_THREADS; j++) {
		if (sums[j] < smallVal) {
			smallVal = sums[j];
			smallestIndex = j;
		}
	}

	for (int i = 0; i < 13; i++) {
		cout << "Coordinate " << shortestThreads[smallestIndex][i].getOrder() << ": X = "
			<< shortestThreads[smallestIndex][i].getX()
			<< " and Y = " << shortestThreads[smallestIndex][i].getY() << endl;
	}
	for (int i = 1; i < 13; i++) {
		//cout << "Coordinate " << shortest[i].getOrder() << endl;
		cout << "X" << i - 1 << ":      DW " << shortestThreads[smallestIndex][i].getX() * 290 << endl;
		cout << "Y" << i - 1 << ":      DW " << shortestThreads[smallestIndex][i].getY() * 290 << endl;
		cout << "Order" << i - 1 << ":  DW " << shortestThreads[smallestIndex][i].getOrder() << endl;
	}
	free(pts);
}


