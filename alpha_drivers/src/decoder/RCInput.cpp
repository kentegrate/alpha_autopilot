/* This code is from the ardupilot project, modifed by Ken Takaki.
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_Linux/RCInput.cpp
*/

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

//#include <AP_HAL/AP_HAL.h>

#include <alpha_drivers/decoder/RCInput.h>
#include <alpha_drivers/decoder/sbus.h>

//extern const AP_HAL::HAL& hal;



RCInput::RCInput() :
  new_rc_input(false)
{
  //  ppm_state._channel_counter = -1;
}

void RCInput::init()
{
}

bool RCInput::new_input()
{
  return new_rc_input;
}

uint8_t RCInput::num_channels()
{
  return _num_channels;
}

uint16_t RCInput::read(uint8_t ch)
{
  new_rc_input = false;
  if (_override[ch]) {
    return _override[ch];
  }
  if (ch >= _num_channels) {
    return 0;
  }
  return _pwm_values[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
  uint8_t i;
  for (i=0; i<len; i++) {
    periods[i] = read(i);
  }
  return len;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len)
{
  bool res = false;
  if(len > LINUX_RC_INPUT_NUM_CHANNELS){
    len = LINUX_RC_INPUT_NUM_CHANNELS;
  }
  for (uint8_t i = 0; i < len; i++) {
    res |= set_override(i, overrides[i]);
  }
  return res;
}

bool RCInput::set_override(uint8_t channel, int16_t override)
{
  if (override < 0) return false; /* -1: no change. */
  if (channel < LINUX_RC_INPUT_NUM_CHANNELS) {
    _override[channel] = override;
    if (override != 0) {
      new_rc_input = true;
      return true;
    }
  }
  return false;
}

void RCInput::clear_overrides()
{
  for (uint8_t i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
    _override[i] = 0;
  }
}


void RCInput::_process_sbus_pulse(uint16_t width_s0, uint16_t width_s1)
{
  // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
  uint16_t bits_s0 = (width_s0+1) / 10;
  uint16_t bits_s1 = (width_s1+1) / 10;
  uint16_t nlow;

  uint8_t byte_ofs = sbus_state.bit_ofs/12;
  uint8_t bit_ofs = sbus_state.bit_ofs%12;

  if (bits_s0 == 0 || bits_s1 == 0) {
    // invalid data
    goto reset;
  }

  if (bits_s0+bit_ofs > 10) {
    // invalid data as last two bits must be stop bits
    goto reset;
  }

  // pull in the high bits
  sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
  sbus_state.bit_ofs += bits_s0;
  bit_ofs += bits_s0;

  // pull in the low bits
  nlow = bits_s1;
  if (nlow + bit_ofs > 12) {
    nlow = 12 - bit_ofs;
  }
  bits_s1 -= nlow;
  sbus_state.bit_ofs += nlow;

  if (sbus_state.bit_ofs == 25*12 && bits_s1 > 12) {
    // we have a full frame
    uint8_t bytes[25];
    uint8_t i;
    for (i=0; i<25; i++) {
      // get inverted data
      uint16_t v = ~sbus_state.bytes[i];
      // check start bit
      if ((v & 1) != 0) {
	goto reset;
      }
      // check stop bits
      if ((v & 0xC00) != 0xC00) {
	goto reset;
      }
      // check parity
      uint8_t parity = 0, j;
      for (j=1; j<=8; j++) {
	parity ^= (v & (1U<<j))?1:0;
      }
      if (parity != (v&0x200)>>9) {
	goto reset;
      }
      bytes[i] = ((v>>1) & 0xFF);
    }
    uint16_t values[LINUX_RC_INPUT_NUM_CHANNELS];
    uint16_t num_values=0;
    bool sbus_failsafe=false, sbus_frame_drop=false;
    if (sbus_decode(bytes, values, &num_values,
		    &sbus_failsafe, &sbus_frame_drop,
		    LINUX_RC_INPUT_NUM_CHANNELS) &&
	num_values >= 5) {
      for (i=0; i<num_values; i++) {
	_pwm_values[i] = values[i];
      }
      _num_channels = num_values;
      new_rc_input = true;
    }
    goto reset;
  } else if (bits_s1 > 12) {
    // break
    goto reset;
  }
  return;
 reset:
  memset(&sbus_state, 0, sizeof(sbus_state));
}


/*
  process a RC input pulse of the given width
*/
void RCInput::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
#if 0
  // useful for debugging
  static FILE *rclog;
  if (rclog == NULL) {
    rclog = fopen("/tmp/rcin.log", "w");
  }
  if (rclog) {
    fprintf(rclog, "%u %u\n", (unsigned)width_s0, (unsigned)width_s1);
  }
#endif

  // treat as SBUS
  _process_sbus_pulse(width_s0, width_s1);

}

/*
 * Update channel values directly
 */
void RCInput::_update_periods(uint16_t *periods, uint8_t len)
{
  if (len > LINUX_RC_INPUT_NUM_CHANNELS) {
    len = LINUX_RC_INPUT_NUM_CHANNELS;
  }
  for (unsigned int i=0; i < len; i++) {
    _pwm_values[i] = periods[i];
  }
  _num_channels = len;
  new_rc_input = true;
}

