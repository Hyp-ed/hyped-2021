#include <array>
#include <chrono>
#include <thread>

#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/timer.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::concurrent::BusyThread;
using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;
using hyped::utils::Timer;

// Set number of pulses to 1-20M for fast pulses and 10-100k for timed ones
constexpr int kNumPulses = 2*1000*1000;  // Multiplication just to count zeros more easily
constexpr int kNumBusyThreads = 8;
constexpr int kNumExtraPins = 3;
constexpr std::array<uint32_t, 8> kPinNums = {66, 67, 69, 68, 45, 44, 23, 26};

void timed_pulses(GPIO& pin, int delay_us, uint64_t t0, Logger& log) {
  uint64_t t;
  for (int i = 0; i < kNumPulses;) {
    t = Timer::getTimeMicros();
    if (t - t0 >= 60) {
      pin.set();
      pin.clear();
      t0 = t;
      ++i;
      // if (i % 10000 == 0)
      //   log.INFO("GPIOTEST", "Reached %d", i);
    }
  }
}

void fast_pulses(GPIO& pin) {
  for (int i = 0; i < kNumPulses; ++i) {
    pin.set();
    pin.clear();
  }
}

class PulseThread : public Thread {
 public:
  PulseThread(int pin_num) : pin_(pin_num, hyped::utils::io::gpio::kOut)
  {
    pin_.clear();
  }
  ~PulseThread()
  {}
  void run() override {
    fast_pulses(pin_);
  }
 private:
  GPIO pin_;
};

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);
  log.INFO("GPIOTEST", "Pulsing %d pins with %d busy threads", kNumExtraPins + 1, kNumBusyThreads);

  GPIO dbg_pin(kPinNums[7], hyped::utils::io::gpio::kOut);
  dbg_pin.set();

  GPIO pin(kPinNums[0], hyped::utils::io::gpio::kOut);
  pin.clear();
  std::array<PulseThread*, kNumExtraPins> pulse_threads;
  for (int i = 0; i < kNumExtraPins; ++i)
    pulse_threads[i] = new PulseThread(kPinNums[i + 1]);

  std::array<BusyThread*, kNumBusyThreads> busy_threads;
  for (BusyThread* t : busy_threads) {
    t = new BusyThread();
    t->start();
  }

  dbg_pin.clear();
  log.INFO("GPIOTEST", "STARTING");

  uint64_t start_time = Timer::getTimeMicros();
  log.INFO("GPIOTEST", "start time %.3fms", start_time/1000.0);
  uint64_t current_time = start_time;
  dbg_pin.set();

  for (PulseThread* t : pulse_threads)
    t->start();
  // Comment/uncomment based on which test you want to run
  // timed_pulses(pin, 60, current_time, log);  // 20000 rpm at 50 pulses per revolution ==> 60us
  fast_pulses(pin);

  dbg_pin.clear();
  current_time = Timer::getTimeMicros();
  log.INFO("GPIOTEST", "end time: %.3fms, duration: %.3fms",
           current_time/1000.0, (current_time - start_time)/1000.0);
  std::this_thread::sleep_for(std::chrono::seconds(4));
  for (BusyThread* t : busy_threads)
    t->running_ = false;
  log.INFO("GPIOTEST", "Waiting for all threads to finish...");
  for (BusyThread* t : busy_threads) {
    t->join();
    delete t;
  }
  for (PulseThread* t : pulse_threads) {
    t->join();
    delete t;
  }
  log.INFO("GPIOTEST", "ENDING");

} // end of main
