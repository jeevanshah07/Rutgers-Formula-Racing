#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define NUM_MULTIPLEXERS 3
#define NUM_DATA_POINTS 500 // number of data points
#define NUM_CHANNELS 32     // number of channels

float arr[NUM_CHANNELS][NUM_DATA_POINTS];
float past_temps[10];
int thermal_runaways[10];

static float Calculate_Channel_Mean(uint8_t channel);
static float Get_Channel_Temp(uint8_t channel, uint8_t poll_number);
static float Get_Random_Mean(float arr[], int n);
static bool Is_Thermal_Runaway(float data[], int n, int k);
static void Get_Data();
static void Delay(int ms);
static void Save_Temp(int i);

static void Delay(int ms) { usleep(ms * 1000); }

// get the gradient of an array in units/seconds using least squares
bool Is_Thermal_Runaway(float data[], int n, int k) {
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

  for (int i = 0; i < n; i++) {
    float x = (float)i;
    float y = data[i];

    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }

  float denominator = (n * sumX2 - sumX * sumX) * 4e-6;
  if (denominator == 0)
    return 0;

  if (((n * sumXY - sumX * sumY) / denominator) > 0) {
    thermal_runaways[k % 10] = 1;
  }

  // if 8 of the past 10 gradients are increasing, probably thermal_runaways
  // should probably also have a temp check....
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += thermal_runaways[i];
  }

  if (sum > 8) {
    return true;
  }

  return false;
}

// selects n/4 random values from the array and then takes the average
static float Get_Random_Mean(float arr[], int n) {
  int k = n / 4;
  float sum = 0;

  srand(time(NULL));

  for (int i = 0; i < k; i++) {
    int j = i + rand() % (n - i);

    sum += arr[j];
  }

  return sum / (float)k;
}

static void Get_Data() {
  FILE *f;
  int height, width, i, j;

  if ((f = fopen("data.txt", "r")) == NULL)
    exit(1);

  if (fscanf(f, "%d%d", &height, &width) != 2)
    exit(1);

  if (height < 1 || height > NUM_CHANNELS || width < 1 ||
      width > NUM_DATA_POINTS)
    exit(1);

  for (j = 0; j < height; j++)
    for (i = 0; i < width; i++)
      if (fscanf(f, "%f", &arr[j][i]) != 1)
        exit(1);
  fclose(f);
}

static float Get_Channel_Temp(uint8_t channel, uint8_t poll_number) {
  return arr[channel][poll_number];
}

static float Calculate_Channel_Mean(uint8_t channel) {
  float total = 0.0f;

  for (int i = 0; i < 500; i++) {
    total += Get_Channel_Temp(channel, i);
  }

  return (total / 500.0f);
}

int main(void) {
  struct timeval start, end;
  long seconds, microseconds;
  double time_elapsed;

  gettimeofday(&start, NULL);

  Get_Data();

  // loop to simulate the car running
  for (int k = 0; k < 1000; k++) {
    // take the average temp of the entire system
    float total = 0;

    // in the actual program we'll use j to select the correct multiplexer
    for (int j = 0; j < NUM_MULTIPLEXERS; j++) {
      // take the average temp of each mux
      float mux_total = 0;
      // iterate through the 32 channels on the multiplexer and get the mean of
      // the 500 data points
      for (int i = 0; i < NUM_CHANNELS; i++) {
        float mean = Calculate_Channel_Mean(i);
        mux_total += mean;

        printf("\n");
        printf("%10.1f", mean);
        Delay(3);
      }

      total += (mux_total / (float)32);

      // for each channel, take the random
      printf("\n\n------------------------\n\n");
    }

    // store the past 10 temperatures
    past_temps[k % 10] = (total / (float)3);

    // check for increasing gradient, if increasing toggle flag in array
    if (Is_Thermal_Runaway(past_temps, 10, k)) {
      exit(1);
    }
  }

  gettimeofday(&end, NULL);
  seconds = end.tv_sec - start.tv_sec;
  microseconds = end.tv_usec - start.tv_usec;
  time_elapsed = seconds + microseconds * 1e-6;

  printf("Execution time: %.6f seconds\n", time_elapsed);

  return 0;
}
