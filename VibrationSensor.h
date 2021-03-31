#include "esphome.h"
#include "Adafruit_LIS3DH.h"
#include "Adafruit_Sensor.h"

#define TIME_WINDOW 3000 // 3 secs
#define TIME_UNTIL_ON 30000 // 30 secs
#define TIME_UNTIL_DONE 300000 // 5 mins

enum ApplianceState {
  IDLE,
  MAYBE_ON,
  ON,
  MAYBE_DONE,
  DONE
};

class VibrationSensor : public PollingComponent, public Sensor {
  private:
    uint8_t address;
    bool error;
    float initialX, initialY, initialZ, threshold;
    long lastActive, lastIdle;
    ApplianceState state;

  public:
    Adafruit_LIS3DH lis;

    VibrationSensor(uint8_t addr, float thresh) : PollingComponent(200) {
      address = addr;
      threshold = thresh;
      state = IDLE;
    }

    float get_setup_priority() const override { return esphome::setup_priority::BUS; }

    void setup() override {
      if (!lis.begin(address)) {
        error = true;
        ESP_LOGE("custom", "failed starting LIS3DH %i", address);
        return;
      }

      lis.setRange(LIS3DH_RANGE_2_G);

      sensors_event_t sEvent;
      if (!lis.getEvent(&sEvent)) {
        error = true;
        ESP_LOGE("custom", "failed reading initial state of %i", address);
        return;
      }
      initialX = sEvent.acceleration.x;
      initialY = sEvent.acceleration.y;
      initialZ = sEvent.acceleration.z;

      // make sure we start IDLE
      lastActive = millis() - (TIME_WINDOW + (TIME_WINDOW/2));
    }

    void update() override {
      if (error) {
        return;
      }

      sensors_event_t sEvent;
      if (!lis.getEvent(&sEvent)) {
        ESP_LOGE("custom", "failed reading state of %i", address);
        return;
      }

      float force = fabs(sEvent.acceleration.x - initialX) +
        fabs(sEvent.acceleration.y - initialY) +
        fabs(sEvent.acceleration.z - initialZ);
      ESP_LOGD("custom", "force: %f", force);

      // TODO: probably just want to send the state, might be easier to calculate here rather than in ha
      //       could also pass on the raw value in addition to the state
      // TODO: home assistant dashboard. not very useful.
      // publish_state(force);

      long now = millis();
      if (force > threshold) {
        lastActive = now;
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




      // publish_state(42.0);
    }
};
