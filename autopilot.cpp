/*
 * ====================================================================
 *  شبیه‌ساز اتوپایلوت هواپیما (Aircraft Autopilot Simulation)
 * ====================================================================
 *  این برنامه یک سیستم اتوپایلوت ساده را شبیه‌سازی می‌کند که سه پارامتر
 *  اصلی پرواز را به‌صورت خودکار کنترل می‌کند:
 *
 *    1) ارتفاع (Altitude)   - بر حسب فوت (ft)
 *    2) سرعت (Airspeed)     - بر حسب گره دریایی (knots)
 *    3) هدینگ/مسیر (Heading) - بر حسب درجه (0-360)
 *
 *  هسته‌ی کنترل با یک کنترلر PID (Proportional-Integral-Derivative)
 *  پیاده‌سازی شده که خطای بین مقدار هدف (setpoint) و مقدار فعلی را
 *  اندازه می‌گیرد و فرمان کنترلی متناسب تولید می‌کند.
 *
 *  کامپایل:
 *      g++ -std=c++17 -O2 -o autopilot autopilot.cpp
 *  اجرا:
 *      ./autopilot
 * ====================================================================
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>

// --------------------------------------------------------------------
// کلاس کنترلر PID — قلب تپنده هر سیستم اتوپایلوت
// --------------------------------------------------------------------
class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                   double outputMin, double outputMax)
        : kp_(kp), ki_(ki), kd_(kd),
          outMin_(outputMin), outMax_(outputMax),
          integral_(0.0), prevError_(0.0), firstRun_(true) {}

    // محاسبه خروجی کنترلی بر اساس خطا و گام زمانی (dt بر حسب ثانیه)
    double update(double setpoint, double measured, double dt) {
        double error = setpoint - measured;

        // جمله انتگرالی (با محدودسازی برای جلوگیری از Integral Windup)
        integral_ += error * dt;
        integral_ = clamp(integral_, -integralLimit_, integralLimit_);

        // جمله مشتقی (در اولین اجرا صفر در نظر گرفته می‌شود)
        double derivative = firstRun_ ? 0.0 : (error - prevError_) / dt;
        firstRun_ = false;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prevError_ = error;

        return clamp(output, outMin_, outMax_);
    }

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        firstRun_ = true;
    }

private:
    double kp_, ki_, kd_;
    double outMin_, outMax_;
    double integral_;
    double prevError_;
    bool firstRun_;
    const double integralLimit_ = 500.0;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }
};

// --------------------------------------------------------------------
// ساختار وضعیت هواپیما (مدل فیزیکی بسیار ساده‌شده)
// --------------------------------------------------------------------
struct AircraftState {
    double altitude   = 5000.0;   // فوت
    double airspeed   = 180.0;    // گره دریایی (knots)
    double heading     = 90.0;     // درجه (0-360)
    double verticalSpeed = 0.0;   // فوت بر دقیقه (نمایشی)
};

// --------------------------------------------------------------------
// کلاس اصلی اتوپایلوت
// --------------------------------------------------------------------
class Autopilot {
public:
    Autopilot()
        : altitudeHoldPID_(0.35, 0.01, 0.12, -2000.0, 2000.0),  // خروجی: نرخ صعود/نزول هدف (ft/min)
          speedHoldPID_   (1.20, 0.05,  0.10, -15.0, 15.0),       // خروجی: تغییر شتاب سرعت (knots/s)
          headingHoldPID_ (0.50, 0.00,  0.15, -10.0, 10.0),       // خروجی: نرخ چرخش (deg/s)
          engaged_(false) {}

    void engage(double targetAltitude, double targetSpeed, double targetHeading) {
        targetAltitude_ = targetAltitude;
        targetSpeed_    = targetSpeed;
        targetHeading_  = normalizeHeading(targetHeading);
        engaged_ = true;
        altitudeHoldPID_.reset();
        speedHoldPID_.reset();
        headingHoldPID_.reset();

        std::cout << "\n[ AUTOPILOT ENGAGED ]\n";
        std::cout << "  هدف ارتفاع : " << targetAltitude_ << " فوت\n";
        std::cout << "  هدف سرعت   : " << targetSpeed_    << " گره\n";
        std::cout << "  هدف هدینگ  : " << targetHeading_  << " درجه\n\n";
    }

    void disengage() {
        engaged_ = false;
        std::cout << "\n[ AUTOPILOT DISENGAGED ]\n\n";
    }

    bool isEngaged() const { return engaged_; }

    // یک گام شبیه‌سازی: dt بر حسب ثانیه
    void step(AircraftState &state, double dt) {
        if (!engaged_) return;

        // ---- کنترل ارتفاع ----
        double targetVS = altitudeHoldPID_.update(targetAltitude_, state.altitude, dt);
        state.verticalSpeed = targetVS;
        state.altitude += (targetVS / 60.0) * dt;  // تبدیل ft/min به ft/s

        // ---- کنترل سرعت ----
        double speedRate = speedHoldPID_.update(targetSpeed_, state.airspeed, dt);
        state.airspeed += speedRate * dt;
        state.airspeed = std::max(0.0, state.airspeed);

        // ---- کنترل هدینگ (با در نظر گرفتن کوتاه‌ترین مسیر چرخش) ----
        double headingError = shortestHeadingError(targetHeading_, state.heading);
        double turnRate = headingHoldPID_.update(headingError, 0.0, dt);
        state.heading = normalizeHeading(state.heading + turnRate * dt);
    }

    double targetAltitude() const { return targetAltitude_; }
    double targetSpeed()    const { return targetSpeed_; }
    double targetHeading()  const { return targetHeading_; }

private:
    PIDController altitudeHoldPID_;
    PIDController speedHoldPID_;
    PIDController headingHoldPID_;

    bool engaged_;
    double targetAltitude_ = 0.0;
    double targetSpeed_    = 0.0;
    double targetHeading_  = 0.0;

    static double normalizeHeading(double h) {
        while (h < 0)   h += 360.0;
        while (h >= 360) h -= 360.0;
        return h;
    }

    // کوتاه‌ترین فاصله زاویه‌ای بین هدف و مقدار فعلی (بین -180 تا +180)
    static double shortestHeadingError(double target, double current) {
        double diff = std::fmod(target - current + 540.0, 360.0) - 180.0;
        return diff;
    }
};

// --------------------------------------------------------------------
// چاپ وضعیت لحظه‌ای پرواز
// --------------------------------------------------------------------
void printStatus(int t, const AircraftState &s, const Autopilot &ap) {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "t=" << std::setw(3) << t << "s | "
              << "ارتفاع: " << std::setw(7) << s.altitude << " ft (هدف " << ap.targetAltitude() << ") | "
              << "سرعت: " << std::setw(6) << s.airspeed << " kt (هدف " << ap.targetSpeed() << ") | "
              << "هدینگ: " << std::setw(5) << s.heading << " deg (هدف " << ap.targetHeading() << ") | "
              << "VS: " << std::setw(7) << s.verticalSpeed << " ft/min\n";
}

// --------------------------------------------------------------------
// برنامه اصلی: یک سناریوی پروازی نمونه
// --------------------------------------------------------------------
int main() {
    std::cout << "=========================================================\n";
    std::cout << "   شبیه‌ساز اتوپایلوت هواپیما (Altitude / Speed / Heading)\n";
    std::cout << "=========================================================\n";

    AircraftState aircraft;
    aircraft.altitude = 5000.0;
    aircraft.airspeed = 180.0;
    aircraft.heading  = 90.0;

    Autopilot autopilot;

    std::cout << "\n[ وضعیت اولیه پرواز ]\n";
    printStatus(0, aircraft, autopilot);

    // درگیر کردن اتوپایلوت با اهداف جدید پرواز
    autopilot.engage(/*altitude*/ 10000.0, /*speed*/ 250.0, /*heading*/ 270.0);

    const double dt = 1.0;        // گام زمانی شبیه‌سازی: ۱ ثانیه
    const int totalSeconds = 120; // مدت کل شبیه‌سازی: ۱۲۰ ثانیه

    for (int t = 1; t <= totalSeconds; ++t) {
        autopilot.step(aircraft, dt);

        // هر ۵ ثانیه یک‌بار وضعیت را چاپ کن
        if (t % 5 == 0) {
            printStatus(t, aircraft, autopilot);
        }

        // در میانه پرواز، هدف جدیدی به اتوپایلوت بده (تغییر مسیر و ارتفاع)
        if (t == 60) {
            std::cout << "\n>>> دریافت دستور جدید از خلبان: تغییر هدف پرواز <<<\n";
            autopilot.engage(/*altitude*/ 6000.0, /*speed*/ 210.0, /*heading*/ 45.0);
        }

        // برای خوانایی خروجی، یک تأخیر کوتاه (می‌توانید حذف کنید تا سریع‌تر اجرا شود)
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    autopilot.disengage();

    std::cout << "\n[ وضعیت نهایی پرواز ]\n";
    printStatus(totalSeconds, aircraft, autopilot);

    std::cout << "\nشبیه‌سازی پایان یافت.\n";
    return 0;
}
