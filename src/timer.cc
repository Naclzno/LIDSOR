#include <methods/timer.hh>
#include <cstdio>
#include <unistd.h>

int main(int argc, char *argv[]) {
  Timer timer;
  printf("Timer Test.\n");
  sleep(1);
  printf("%.2f ms\n", timer.Get() * 1e-6);
  return 0;
}