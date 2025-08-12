//
// Created by tobias on 12/26/24.
//

#pragma once

#include <vector>
#include <functional>

namespace libstp::sensor
{
    class Sensor
    {
    protected:
        int port;

    public:
        virtual ~Sensor() = default;

        explicit Sensor(const int& port);

        virtual int getValue();

        [[nodiscard]] int getPort() const;
    };

    class DigitalSensor : public Sensor
    {
    public:
        explicit DigitalSensor(const int& port);

        int getValue() override;

        bool isClicked();
    };

    class ValueSensor final : public Sensor
    {
        std::function<int()> provider;

    public:
        explicit ValueSensor(const std::function<int()>& provider);

        int getValue() override;
    };

    class AnalogSensor : public Sensor
    {
    public:
        explicit AnalogSensor(const int& port);

        int getValue() override;
    };

    class DistanceSensor final : public AnalogSensor
    {
    public:
        explicit DistanceSensor(const int& port);

        double getDistance();
    };

    class LightSensor : public AnalogSensor
    {
    public:
        int whiteThreshold;
        int blackThreshold;
        float whiteMean = 0.0f;
        float blackMean = 0.0f;
        float whiteStdDev = 1.0f;
        float blackStdDev = 1.0f;
        float calibrationFactor;

        explicit LightSensor(const int& port, float calibrationFactor);

        bool calibrate(const std::vector<int>& whiteValues, const std::vector<int>& blackValues);

        void wait_for_light() const;

        bool isOnWhite();

        virtual bool isOnBlack();

        float probabilityOfBlack();
        float probabilityOfWhite();

        [[nodiscard]] float gaussianProbability(int value, float mean, float stdDev) const;
    };

    void calibrateLightSensors(const std::vector<LightSensor*>& lightSensors);

    bool areOnBlack(LightSensor* leftSensor, LightSensor* rightSensor);

    bool areOnWhite(LightSensor* leftSensor, LightSensor* rightSensor);

    void waitForButtonClick();

    bool isButtonClicked();
}
