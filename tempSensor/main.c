#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#define NUM_MULTIPLEXERS 3
#define NUM_CHANNELS 32
#define MWIDTH 500
#define MHEIGHT 32

float arr[MHEIGHT][MWIDTH];

static float Calculate_Channel_Mean(uint8_t channel);
static float Get_Channel_Temp(uint8_t channel, uint8_t poll_number);
static void Get_Data();
static void Delay(int ms);

static void Delay(int ms) {
  usleep(ms * 1000);
}

static void Get_Data() {
  FILE *f;
  int height, width, i, j;

  if ((f = fopen("data.txt", "r")) == NULL)
    exit(1);

  if (fscanf(f, "%d%d", &height, &width) != 2)
    exit(1);

  if (height < 1 || height > MHEIGHT || width < 1 || width > MWIDTH)
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
  for (int j = 0; j < 3; j++) {
    // in the actual program we'll use j to select the correct multiplexer
    for (int i = 0; i < 32; i++) {
      printf("\n");
      printf("%10.1f", Calculate_Channel_Mean(i));
      Delay(3);
    }
    printf("\n\n------------------------\n\n");
  }

  gettimeofday(&end, NULL);
  seconds = end.tv_sec - start.tv_sec;
  microseconds = end.tv_usec - start.tv_usec;
  time_elapsed = seconds + microseconds*1e-6;

  printf("Execution time: %.6f seconds\n", time_elapsed);

  return 0;
}
