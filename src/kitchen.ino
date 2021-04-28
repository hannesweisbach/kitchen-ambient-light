#include <avr/sleep.h>
#include <avr/power.h>

#include <Adafruit_NeoPixel.h>
#include <TaskManagerIO.h>

using Color_t = uint32_t;
void operator delete(void*, unsigned int) {}

static const int NUM_PIXELS = 30;
static const int LED_PIN = 2;

Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRBW + NEO_KHZ800);

class ContinuousRange {
  int8_t begin_;
  int8_t end_;

public:
  class Iterator {
    int8_t i;

  public:
    Iterator(const int8_t begin_) : i(begin_) {}
    Iterator &operator++() {
      ++i;
      return *this;
    }
    Iterator operator++(int) {
      const auto pre = i;
      ++i;
      return Iterator{pre};
    }

    int8_t operator*() { return i; }
    bool operator!=(const auto &rhs) { return i != rhs.i; }
  };

  friend class Iterator;

  ContinuousRange(const int8_t begin, const int8_t end)
      : begin_(begin), end_(end) {}

  Iterator begin() { return Iterator{begin_}; }
  Iterator end() { return Iterator{end_}; }
};

class LEDRange {
  constexpr static const int8_t leds[]{4,  5,  7,  8,  9,  11, 12, 13,
                                       16, 17, 18, 20, 21, 23, 24, 25};
public:
  class Iterator {
    int8_t i_;

  public:
    Iterator(const int i = 0) : i_{i} {};
    Iterator &operator++() {
      ++i_;
      return *this;
    }
    Iterator operator++(int) {
      const auto pre = i_;
      ++i_;
      return Iterator{pre};
    }

    int8_t operator*() { return leds[i_]; }
    bool operator!=(const auto &rhs) { return i_ != rhs.i_; }
  };

  friend class Iterator;

  LEDRange() = default;

  Iterator begin() { return Iterator{0}; }
  Iterator end() { return Iterator{sizeof(leds)}; }
	
};

constexpr static int8_t letters[][3] = {{4, 5, -1},   {7, 8, 9},    {11, 12, 13},
                                       {16, 17, 18}, {20, 21, -1}, {23, 24, 25}};

static const constexpr uint8_t range = 15;
static const constexpr uint8_t mid = 30;
static_assert(range < mid, "Brightness underflow!");
static const constexpr uint8_t low = mid - range;
static const constexpr uint8_t high = mid + range;



constexpr const int8_t LEDRange::leds[];

const auto all_leds = ContinuousRange{0, NUM_PIXELS};
const auto animated_leds = LEDRange{};

static const auto WHITE = Adafruit_NeoPixel::Color(0, 0, 0, 10);

static constexpr double percent_to_8(const double percent) {
  return percent * 255.0 / 100.0;
}

static const constexpr auto BRIGHTNESS = 11.5;

static const constexpr auto BRIGHTNESS_high = percent_to_8(1.5 * BRIGHTNESS);
static const constexpr auto BRIGHTNESS_normal = percent_to_8(BRIGHTNESS);
static const constexpr auto BRIGHTNESS_low = percent_to_8(0.5 * BRIGHTNESS);

uint32_t count = 0;

class LetterFade : public Executable {
  static void set_letter_color(const uint32_t color, const auto &letter_idx) {
    const auto &letter = letters[letter_idx];
#if 0
    Serial.print("Setting letter ");
    Serial.print(letter_idx);
    Serial.print(" color ");
    Serial.print(color >> 24);
    Serial.println();
#endif
    for (const auto &pixel : letter) {
      pixels.setPixelColor(pixel, color);
    }
    pixels.show();
    delay(40);
  }

  static auto intra_letter_sleep_ms() {
#ifndef NDEBUG
    const auto seconds = random(2 * 60, 5 * 60);
    Serial.print("Intra delay: ");
    Serial.print(seconds);
    Serial.println();
#else
    const auto seconds = 2;
#endif
    return seconds * 1000;
  }

  static auto inter_letter_sleep_ms() {
#ifndef NDEBUG
    const auto seconds = random(20 * 60, 45 * 60);
    Serial.print("Inter delay: ");
    Serial.print(seconds);
    Serial.println();
#else
    const auto seconds = 5;
#endif
    return seconds * 1000;
  }

public:
  LetterFade() {}

  void exec() override {
    const auto letter_idx = random(0, 5);

    Serial.print("Letter ");
    Serial.print(letter_idx);
    Serial.println();


    for (auto white = mid; white < high; ++white) {
      const auto color = pixels.Color(0, 0, 0, white);
      set_letter_color(color, letter_idx);
    }

    delay(intra_letter_sleep_ms());

    for (auto white = high; white > low; --white) {
      const auto color = pixels.Color(0, 0, 0, white);
      set_letter_color(color, letter_idx);
    }

    delay(intra_letter_sleep_ms());

    for (auto white = low; white < mid; ++white) {
      const auto color = pixels.Color(0, 0, 0, white);
      set_letter_color(color, letter_idx);
    }

    const auto next = inter_letter_sleep_ms();
    taskManager.scheduleOnce(next, this, TIME_MILLIS);
    Serial.print("Sleeping for ");
    Serial.print(next / 1000);
    Serial.print("s ");
    Serial.print(count);
    Serial.println();
    count = 0;
  }
};

LetterFade letter;


template <typename Range>
class FadeAnim : public Executable {
  enum class State { UP, DOWN, HOLD };

  uint8_t white_;
  uint8_t low_;
  uint8_t high_;
  uint8_t mid_;
  uint8_t highmark_;
  uint32_t step_sleep_;
  uint32_t state_sleep_;

  enum State state_;
  Range leds_;

  void schedule_step(const uint32_t when) {
    Serial.print("Sleeping ");
    Serial.print(when);
    Serial.println("ms");

    taskManager.scheduleOnce(when, this, TIME_MILLIS);
  }

  static uint32_t random_timeout() {
    // 20 to 45 mins in ms
    const auto min = 20 * 60 * 1000;
    const auto max = 45 * 60 * 1000;
    //const auto r =  static_cast<uint32_t>(random(min, max));
    const auto r = 4 * 1000;
    return r;
  }

  static void debug_fade(const enum State new_state, unsigned from,
                         unsigned to) {
    Serial.print("Fade ");
    switch (new_state) {
    case State::UP:
      Serial.print("up ");
      break;
    case State::DOWN:
      Serial.print("down ");
      break;
    case State::HOLD:
      Serial.print("hold ");
      break;
    }
    Serial.print(from);
    Serial.print(" to ");
    Serial.print(to);
    Serial.println();
  }

public:
  FadeAnim(Range leds, const int8_t low, const int8_t high)
      : white_(low), low_(low), high_(high), state_(State::HOLD), leds_{leds} {}

  void begin(const uint8_t mid, const uint8_t low, const uint8_t high,
             const uint32_t step_ms, const uint32_t state_ms) {
    mid_ = mid;
    white_ = mid;
    low_ = low;
    high_ = high;
    step_sleep_ = step_ms;
    state_sleep_ = state_ms;

    state_ = State::HOLD;
    taskManager.execute(this);
  }

  void exec() override {
    auto sleep_ = step_sleep_;
    enum State new_state = state_;

    /* new state */
    switch (state_) {
    case State::UP:
      if (white_ == high_) {
        new_state = State::DOWN;
        debug_fade(new_state, high_, low_);
      } else if (white_ == mid_) {
        new_state = State::HOLD;
      }
      break;
    case State::DOWN:
      if (white_ == low_) {
        new_state = State::UP;
        highmark_ = mid_;
        debug_fade(new_state, low_, highmark_);
      }
      break;

    case State::HOLD:
      highmark_ = high_;
      new_state = State::UP;
      debug_fade(new_state, white_, highmark_);
      break;
    }

    if (state_ != new_state) {
      sleep_ = state_sleep_;
      state_ = new_state;
    }

    Serial.print("Setting color ");
    Serial.println(static_cast<unsigned>(white_));
    const auto color = pixels.Color(0, 0, 0, white_);
    /* set leds */
    for (const auto &led : leds_) {
      pixels.setPixelColor(led, color);
    }
    pixels.show();

    /* new color */
    switch (state_) {
    case State::UP:
      if (white_ < highmark_)
        white_ += 1;
      break;
    case State::DOWN:
      if (white_ > low_)
        white_ -= 1;
      break;
    case State::HOLD:
      break;
    }

    const auto timeout = (state_ == State::HOLD) ? random_timeout() : sleep_;
    schedule_step(timeout);
  }
};

FadeAnim<LEDRange> fade(animated_leds, 0, 12);

ISR(TIMER2_COMPA_vect) {}

void setup() {
  Serial.begin(9600);

  TCCR2A = (1 << WGM21) | (0 << WGM20);
  // prescaler 1024; but timer stopped for now
  TCCR2B = (0 << WGM22) | (0 << CS20);
  TIMSK2 = (1 << OCIE2A);

#ifndef NDEBUG
  randomSeed(analogRead(0));
#endif

  {
    pixels.begin();
    pixels.clear();
    const auto color = pixels.Color(0, 0, 0, mid);
    pixels.fill(color);
    pixels.show();
  }

  //fade.begin(mid, low, high, 60, 2 * 1000);

  //pixels.setBrightness(BRIGHTNESS_high);
//  fade_strip(pixels, 0, 128, 2);

  taskManager.execute(&letter);
}

static void sleep(const uint32_t micros) {
  /* disable ADC */
  ADCSRA &= ~(1 << ADEN);
  /* disable analog comparator */
  ACSR |= (1 << ACD);

  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
//  power_usart0_disable();
#if  0
  power_timer0_disable();
  power_timer1_disable();
#endif

  // clk * prescaler =
  // 1/e8 * 1024 = 128 us
  const auto count = micros / 128;
  OCR2A = count > 255 ? 255 : count;
  TCNT2 = 0;
  /* enable counter */
  TCCR2B |= (0x7 << CS20);

  set_sleep_mode(SLEEP_MODE_IDLE);

  cli();
  sleep_enable();
  sei();
  sleep_cpu();

  sleep_disable();
  /* disable timer */
  TCCR2B &= ~(0x7 << CS20);
}

void loop() {
  ++count;
  const auto delay = taskManager.microsToNextTask();
  sleep(delay);

  taskManager.runLoop();
}

