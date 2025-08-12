//
// Created by tobias on 12/26/24.
//

#pragma once
#include <functional>


namespace libstp::filter
{
    class Filter
    {
    public:
        unsigned int warmUpCycles;

        explicit Filter(const unsigned int warmUpCycles) : warmUpCycles(warmUpCycles)
        {
        }

        virtual ~Filter() = default;

        virtual double filter(const double dataPoint)
        {
            return dataPoint;
        }

        void warmup(const int warmUpValue)
        {
            for (unsigned int i = 0; i < warmUpCycles; i++)
            {
                filter(warmUpValue);
            }
        }

        virtual double operator()(const double dataPoint)
        {
            return filter(dataPoint);
        }
    };

    class FunctionFilter final : public Filter
    {
        const std::function<double(double)>& filterFunction;

    public:
        explicit FunctionFilter(const std::function<double(double)>& filterFunction) : Filter(1),
            filterFunction(filterFunction)
        {
        }

        double filter(const double dataPoint) override
        {
            return filterFunction(dataPoint);
        }
    };

    class NoFilter final : public Filter
    {
    public:
        NoFilter() : Filter(0)
        {
        }

        double filter(const double dataPoint) override
        {
            return dataPoint;
        }
    };

    class AvgFilter final : public Filter
    {
        unsigned long k;
        double previousAvg;

    public:
        AvgFilter() : Filter(1), k(1), previousAvg(0)
        {
        }

        double filter(const double dataPoint) override
        {
            const double alpha = (static_cast<double>(k) - 1.0) / static_cast<double>(k);
            const double avg = alpha * previousAvg + (1 - alpha) * dataPoint;
            k++;
            previousAvg = avg;
            return avg;
        }
    };

    class MovingAverageFilter final : public Filter
    {
    public:
        const unsigned int n;

        explicit MovingAverageFilter(const unsigned int n) : Filter(n), n(n)
        {
            previousInputs.resize(n, 0);
        }

        double filter(const double dataPoint) override
        {
            sum -= previousInputs[index];
            sum += dataPoint;
            previousInputs[index] = dataPoint;
            index = (index + 1) % n;
            return sum / n;
        }

        [[nodiscard]] double getReading() const
        {
            return sum / n;
        }

    private:
        unsigned long index = 0;
        std::vector<double> previousInputs = {};
        double sum = 0;
    };

    class FirstOrderLowPassFilter final : public Filter
    {
        double alpha;
        double previousOutput;

    public:
        explicit FirstOrderLowPassFilter(const double alpha) : Filter(1), alpha(alpha), previousOutput(0)
        {
        }

        double filter(const double dataPoint) override
        {
            const double output = alpha * dataPoint + (1 - alpha) * previousOutput;
            previousOutput = output;
            return output;
        }
    };

    class MovingFirstOrderLowPassFilter final : public Filter
    {
        const unsigned int n;
        double alpha;
        double previousOutput;
        unsigned long index = 0;
        std::vector<double> previousInputs = {};
        double sum = 0;

    public:
        MovingFirstOrderLowPassFilter(const unsigned int n, const double alpha) : Filter(n), n(n),
            alpha(alpha),
            previousOutput(0)
        {
            previousInputs.resize(n, 0);
        }

        double filter(const double dataPoint) override
        {
            sum -= previousInputs[index];
            sum += dataPoint;
            previousInputs[index] = dataPoint;
            if (++index == n)
                index = 0;
            const double average = sum / n;

            const double output = alpha * average + (1 - alpha) * previousOutput;
            previousOutput = output;
            return output;
        }
    };
}
