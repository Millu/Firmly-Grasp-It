#include <iostream>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <fstream>
#include "Coordinates.cc"
#include <pthread.h>

using namespace std;

pthread_mutex_t shortestMutex;
pthread_mutex_t exitMutex, startCountMutex;
pthread_cond_t exitCond;
double sum = 1000000;
Coordinates locations[13];
double points[13][13];
Coordinates shortest[13];
int startCount = 12;


// This is for doing x and y separately meaning only turn in
// 90 degree increments
// Do the real distance formula for full range of angles
double distanceFormula(double x1, double x2, double y1, double y2) {
    double tmp = (y2 - y1);
    double tmp2 = (x2 - x1);
    return tmp2 + tmp;
}

void calcDistances() {
    int rows = 13;
    int cols = 13;

    for (int i = 0; i < rows; i++) {
        for (int j = i; j < cols; j++) {
            if (i == j) {
                points[i][j] = 0;
            } else {
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
    int i = (unsigned long)v;

    shortest[0] = locations[0];
    int t, counter = 0;
    double tmpSum;

    for (int a = 0; a < 13; ++a) {
        for (int b = 0; b < 13; ++b) {
            // cout << "Row: " << a  << " and Col " << b << ": " << points[a][b] << endl;
        }
    }

    // For loop from HELL
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
                                if (p == l || p == k || p == j || p == i || p == m || p == n || p == o ) {
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
                                            if (tmpSum < sum) {
                                                pthread_mutex_lock(&shortestMutex);
                                                sum = tmpSum;
                                                shortest[1] = locations[i];
                                                shortest[2] = locations[j];
                                                shortest[3] = locations[k];
                                                shortest[4] = locations[l];
                                                shortest[5] = locations[m];
                                                shortest[6] = locations[n];
                                                shortest[7] = locations[o];
                                                shortest[8] = locations[p];
                                                shortest[9] = locations[q];
                                                shortest[10] = locations[r];
                                                shortest[11] = locations[s];
                                                shortest[12] = locations[t];
                                                pthread_mutex_unlock(&shortestMutex);
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
    cout << "End loop " << i << endl;
    // cout << "Count " << counter << endl;
    pthread_mutex_lock(&startCountMutex);
    startCount--;
    cout << "startCount " << startCount << endl;
    if (startCount == 0) {
        pthread_mutex_unlock(&startCountMutex);
        pthread_mutex_lock(&exitMutex);
        pthread_cond_broadcast(&exitCond);
        pthread_mutex_unlock(&exitMutex);
    } else {
        pthread_mutex_unlock(&startCountMutex);
    }
    return 0;
}

int main(int argc, const char* argv[]) {

    ifstream input(argv[1]);

    string xStr;
    string yStr;
    int i = 1;
    locations[0] = new Coordinates(0, 0, 0);

    while (getline(input, xStr, ',')) {
        getline(input, yStr);
        const char* xChar = xStr.c_str();
        const char* yChar = yStr.c_str();

        double x = atoi(xChar);
        double y = atoi(yChar);

        locations[i] = new Coordinates(x, y, i);
        i++;
    }
    for (int i = 0; i < 13; i++) {
        cout << "Coordinate " << locations[i].getOrder() << ": X = "
        << locations[i].getX()
        << " and Y = " << locations[i].getY() << endl;
    }
    cout << "-----------------------------" << endl;


    int distance;
    calcDistances();

    pthread_mutex_init(&shortestMutex, 0);
    pthread_mutex_init(&exitMutex, 0);
    pthread_mutex_init(&startCountMutex, 0);

    pthread_cond_init(&exitCond, 0);
    pthread_mutex_lock(&exitMutex);
    pthread_t* pts = (pthread_t*) malloc (12 * sizeof(pthread_t));
    for (int threadNo = 1; threadNo < 13; ++threadNo) {
        pthread_create(pts+threadNo, 0, calcShortest, (void *)threadNo);
    }

    pthread_cond_wait(&exitCond, &exitMutex);

    for (int i = 0; i < 13; i++) {
        cout << "Coordinate " << shortest[i].getOrder() << ": X = "
        << shortest[i].getX()
        << " and Y = " << shortest[i].getY() << endl;
    }
    cout << "-------------------------------------" << endl;

    for (int i = 1; i < 13; i++) {
        //cout << "Coordinate " << shortest[i].getOrder() << endl;
        cout << "X" << i - 1 << ":      DW " << shortest[i].getX() * 290 << endl;
        cout << "Y" << i - 1 << ":      DW " << shortest[i].getY() * 290 << endl;

    }
    pthread_mutex_destroy(&shortestMutex);
    free(pts);
}


