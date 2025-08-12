/*!
 * \file magneto.h
 * \copyright KISS Institute for Practical Robotics
 * \defgroup magneto Magnetometer
 */

#ifndef _KIPR_MAGNETO_MAGNETO_H_
#define _KIPR_MAGNETO_MAGNETO_H_

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Gets the sensed x magneto value
 * \return The latest signed x magneto value
 * \ingroup magneto
 */
float magneto_x();

/*!
 * Gets the sensed x magneto value
 * \return The latest signed y magneto value
 * \ingroup magneto
 */
float magneto_y();

/*!
 * Gets the sensed x magneto value
 * \return The latest signed z magneto value
 * \ingroup magneto
 */
float magneto_z();

#ifdef __cplusplus
}
#endif

#endif