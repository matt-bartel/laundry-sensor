#include "esphome.h"
#include "Adafruit_LIS3DH.h"
#include "Adafruit_Sensor.h"

#define TIME_WINDOW 3000 // 3 secs
#define TIME_UNTIL_ON 30000 // 30 secs
#define TIME_UNTIL_DONE 300000 // 5 mins
#define STARTUP_SAMPLES 30 // number of readings to average on startup to determine a baseline
#define ACTIVE_READINGS 2 // number of active readings to require within TIME_WINDOW before becoming active

enum ApplianceState {
  IDLE,
  MAYBE_ON,
  ON,
  MAYBE_DONE,
  DONE,
  STARTUP
};

class VibrationSensor : public PollingComponent, public Sensor {
  private:
    uint8_t startupIndex = 0;
    sensors_vec_t *startupValues;
    const char *name;
    uint8_t address;
    bool error;
    float initialX, initialY, initialZ, threshold;
    uint8_t activeReadings;
    long lastMaybeActive, lastActive, lastIdle;
    ApplianceState state;

    // Average the first STARTUP_SAMPLES readings from the sensor to determine the baseline
    bool buildStartSamples(sensors_event_t *sEvent) {
      startupValues[startupIndex] = sEvent->acceleration;
      startupIndex++;

      bool startupComplete = false;
      if (startupIndex >= STARTUP_SAMPLES) {
        state = IDLE;

        for (uint8_t k = 0; k < STARTUP_SAMPLES; k++) {
          initialX += startupValues[k].x;
          initialY += startupValues[k].y;
          initialZ += startupValues[k].z;
        }

        delete[] startupValues;

        initialX /= STARTUP_SAMPLES;
        initialY /= STARTUP_SAMPLES;
        initialZ /= STARTUP_SAMPLES;

        // make sure we start IDLE
        lastActive = millis() - (TIME_WINDOW + (TIME_WINDOW/2));

        startupComplete = true;
      }

      publish_state(state);
      return startupComplete;
    }

  public:
    Adafruit_LIS3DH lis;

    VibrationSensor(const char *n, uint8_t addr, float thresh) : PollingComponent(200) {
      name = n;
      address = addr;
      threshold = thresh;
      state = STARTUP;
      startupValues = new sensors_vec_t[STARTUP_SAMPLES];
    }

    float get_setup_priority() const override { return esphome::setup_priority::BUS; }

    void setup() override {
      if (!lis.begin(address)) {
        error = true;
        ESP_LOGE(name, "failed starting LIS3DH 0x%x", address);
        return;
      }

      lis.setRange(LIS3DH_RANGE_2_G);

      sensors_event_t sEvent;
      if (!lis.getEvent(&sEvent)) {
        error = true;
        ESP_LOGE(name, "failed reading initial state of 0x%x", address);
        return;
      }
      buildStartSamples(&sEvent);
    }

    void update() override {
      if (error) {
        return;
      }

      sensors_event_t sEvent;
      if (!lis.getEvent(&sEvent)) {
        ESP_LOGE(name, "failed reading state of 0x%x", address);
        return;
      }

      if (state == STARTUP && !buildStartSamples(&sEvent)) {
        return;
      }

      float force = fabs(sEvent.acceleration.x - initialX) +
        fabs(sEvent.acceleration.y - initialY) +
        fabs(sEvent.acceleration.z - initialZ);
      ESP_LOGD(name, "0x%x force: %f", address, force);

      long now = millis();
      if (force > threshold) {
        lastMaybeActive = now;
        if (activeReadings >= ACTIVE_READINGS) {
          lastActive = now;
        }
        else {
          activeReadings++;
        }
      }
      else if (now - lastMaybeActive >= TIME_WINDOW) {
        activeReadings = 0;
      }
      bool recentlyActive = (now - lastActive) < TIME_WINDOW;

      switch (state) {
        case IDLE:
          if (recentlyActive) {
            state = MAYBE_ON;
          } else {
            lastIdle = now;
          }
          break;
        case MAYBE_ON:
          if (recentlyActive && now > (lastIdle + TIME_UNTIL_ON)) {
            state = ON;
          } else if (!recentlyActive) {
            state = IDLE;
          }
          break;
        case ON:
          if (!recentlyActive) {
            state = MAYBE_DONE;
          }
          break;
        case MAYBE_DONE:
          if (recentlyActive) {
            state = ON;
          } else if (now > (lastActive + TIME_UNTIL_DONE)) {
            state = DONE;
          }
          break;
        case DONE:
          state = IDLE;
          break;
      }

      publish_state(state);
    }
};
