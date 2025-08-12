#ifndef _KIPR_KIPR_H_
#define _KIPR_KIPR_H_

#include "config.h"

#ifdef KIPR_MODULE_ACCEL
#include "accel/accel.h"
#endif

#ifdef KIPR_MODULE_ANALOG
#include "analog/analog.h"
#endif

#ifdef KIPR_MODULE_AUDIO
#include "audio/audio.h"
#endif

#ifdef KIPR_MODULE_BATTERY
#include "battery/battery.h"
#endif

#ifdef KIPR_MODULE_BOTBALL
#include "botball/botball.h"
#endif

#ifdef KIPR_MODULE_BUTTON
#include "button/button.h"
#endif

#ifdef KIPR_MODULE_CAMERA
#include "camera/camera.h"
#endif

#ifdef KIPR_MODULE_COMPASS
#include "compass/compass.h"
#endif

#ifdef KIPR_MODULE_CONSOLE
#include "console/console.h"
#include "console/display.h"
#endif

#ifdef KIPR_MODULE_CREATE
#include "create/create.h"
#endif

#ifdef KIPR_MODULE_CREATE3
#include "create3/client/client.h"
#endif

#ifdef KIPR_MODULE_DIGITAL
#include "digital/digital.h"
#endif

#ifdef KIPR_MODULE_GRAPHICS
#include "graphics/graphics.h"
#include "graphics/graphics_characters.h"
#include "graphics/graphics_key_code.h"
#endif

#ifdef KIPR_MODULE_GYRO
#include "gyro/gyro.h"
#endif

#ifdef KIPR_MODULE_MAGNETO
#include "magneto/magneto.h"
#endif

#ifdef KIPR_MODULE_MOTOR
#include "motor/motor.h"
#endif

#ifdef KIPR_MODULE_SERVO
#include "servo/servo.h"
#endif

#endif

