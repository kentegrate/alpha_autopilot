#include <alpha_drivers/decoder/GPIO_RPI.h>
#include <alpha_drivers/decoder/RCInput_RPI.h>
#include <alpha_drivers/decoder/Scheduler.h>

int main(int argc, char* argv[]){
  Scheduler scheduler;
  GPIO_RPI gpio;
  RCInput_RPI rcin;
  rcin.set_scheduler(&scheduler);
  rcin.set_gpio(&gpio);
  scheduler.setRCInput(&rcin);

  scheduler.init();
  gpio.init();
  rcin.init();
  scheduler.system_initialized();

}
