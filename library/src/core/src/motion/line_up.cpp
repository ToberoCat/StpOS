//
// Created by tobias on 12/27/24.
//

#include "libstp/motion/line_up.h"


// ToDo Implementations - Compare performance of commented out code with uncommented code
// Story behind discrepancy:
// The difference between them: The commented out code has been written by me (Tobi) and apparently, the line up sucked (according to Anna).
// That's the reason why Anna wrote her own version in python using the api.
// I've implemented the python version in c++ because the python version was used at gcer, but ultimately,
// I want to compare usability between them to check if the python one is actually better

// Make sure to normalize the speed to the range of -1.0 to 1.0, these examples still use absolute mps / dps measurements.
// This method has been optimized for create3 - Therefore use 0.306 m/s for forward speed and 1.9 rad/s for angular speed
// void lineUp(RobotBackend& backend, LightSensor& leftSensor, LightSensor& rightSensor, float speed)
// {
//     backend.setSpeedWhile([&leftSensor, &rightSensor]()
//     {
//         return ConditionResult{0.0f, leftSensor.isOnBlack() && rightSensor.isOnBlack()};
//     }, [&speed, &leftSensor, &rightSensor]
//     {
//         const float leftSpeed = leftSensor.isOnBlack() ? -speed : speed;
//         const float rightSpeed = rightSensor.isOnBlack() ? -speed : speed;
//         msleep(10); // Test if necessary
//         return SpeedType::wheels(leftSpeed, rightSpeed);
//     });
// }
//
// void driveTillBlack(RobotBackend& backend,
//                     LightSensor& leftSensor,
//                     LightSensor& rightSensor,
//                     float speed,
//                     bool invert)
// {
//     backend.setSpeedWhile([&leftSensor, &rightSensor, &invert]()
//                           {
//                               return ConditionResult{
//                                   0.0f, (leftSensor.isOnBlack() && rightSensor.isOnBlack()) == invert
//                               };
//                           }, [&speed]()
//                           {
//                               return SpeedType{speed, 0.0f};
//                           });
// }
//
// void motion::squareup::forwardLineUp(RobotBackend& backend, LightSensor& leftSensor, LightSensor& rightSensor)
// {
//     lineUp(backend, leftSensor, rightSensor, 0.05);
//     driveTillBlack(backend, leftSensor, rightSensor, 0.05, true);
// }
//
// void motion::squareup::backwardLineUp(RobotBackend& backend, LightSensor& leftSensor, LightSensor& rightSensor)
// {
//     lineUp(backend, leftSensor, rightSensor, -0.05);
//     driveTillBlack(backend, leftSensor, rightSensor, -0.05, true);
// }
libstp::async::AsyncAlgorithm<int> line_up(libstp::device::Device& device,
             libstp::sensor::LightSensor& leftSensor,
             libstp::sensor::LightSensor& rightSensor,
             float sign)
{
    using namespace libstp::datatype;

    // Step 1: Move forward until both sensors detect the black line
    // co_await device.setSpeedWhile(
    //     whileFalse([&leftSensor, &rightSensor]() -> bool
    //     {
    //         return leftSensor.isOnBlack() && rightSensor.isOnBlack();
    //     }),
    //     Speed(sign * 0.4f, 0.0f)
    // );
    //
    // // Step 2: Adjust wheels while either sensor detects the black line
    // co_await device.setSpeedWhile(
    //     whileTrue([&leftSensor, &rightSensor]() -> bool
    //     {
    //         return leftSensor.isOnBlack() || rightSensor.isOnBlack();
    //     }),
    //     [&leftSensor, &rightSensor, sign](std::shared_ptr<ConditionalResult>) -> Speed
    //     {
    //         const float leftSpeed = leftSensor.isOnBlack() ? -0.05f * sign : 0.015f * sign;
    //         const float rightSpeed = rightSensor.isOnBlack() ? -0.05f * sign : 0.015f * sign;
    //         return Speed::wheels(leftSpeed, rightSpeed);
    //     }
    // );
    //
    // // Step 3: Move slightly backward until neither sensor detects the black line
    // co_await device.setSpeedWhile(
    //     whileFalse([&leftSensor, &rightSensor]() -> bool
    //     {
    //         return leftSensor.isOnBlack() || rightSensor.isOnBlack();
    //     }),
    //     Speed(-0.01f, 0.0f)
    // );
}


libstp::async::AsyncAlgorithm<int> libstp::motion::forward_line_up(device::Device& device, sensor::LightSensor& left_sensor,
                                     sensor::LightSensor& right_sensor)
{
    return line_up(device, left_sensor, right_sensor, 1.0f);
}

libstp::async::AsyncAlgorithm<int> libstp::motion::backward_line_up(device::Device& device, sensor::LightSensor& left_sensor,
                                      sensor::LightSensor& right_sensor)
{
    return line_up(device, left_sensor, right_sensor, -1.0f);
}
