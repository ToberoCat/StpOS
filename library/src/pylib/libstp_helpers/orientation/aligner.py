import numpy as np

class Alignment:
    """
    Computes a rotation matrix to transform sensor readings
    from the raw sensor frame into a world frame (e.g., ENU)
    based on the current accelerometer and magnetometer readings.

    Usage:
        from accel import Accel
        from magneto import Magneto
        from alignment import Alignment

        accel_sensor = Accel()
        magneto_sensor = Magneto()
        aligner = Alignment(accel_sensor, magneto_sensor)
        R = aligner.calibrate()  # calibrate when the device is static

        # Now, given a sensor reading (e.g. gyro):
        from gyro import Gyro
        gyro_sensor = Gyro()
        raw_gyro = gyro_sensor.get_reading()
        aligned_gyro = aligner.align(raw_gyro)
    """
    def __init__(self, accel, magneto):
        self.accel = accel
        self.magneto = magneto
        self.alignment_matrix = None

    def calibrate(self):
        """
        Calibrate the alignment matrix using one reading from the
        accelerometer and magnetometer. When the device is static
        (ideally lying flat), the accelerometer gives the gravity vector.
        """
        # Get current sensor readings
        a = self.accel.get_reading()
        m = self.magneto.get_reading()

        # Compute the "up" vector from the accelerometer reading.
        # (Assumes that 'a' is dominated by gravity.)
        up = a / np.linalg.norm(a)

        # Remove the vertical (gravity) component from the magnetometer reading
        m_horizontal = m - np.dot(m, up) * up
        norm_mh = np.linalg.norm(m_horizontal)
        if norm_mh == 0:
            raise ValueError("Invalid magnetometer reading: horizontal component is zero.")
        m_horizontal /= norm_mh

        # Compute east (using cross product: horizontal magnetic field x up)
        east = np.cross(m_horizontal, up)
        east /= np.linalg.norm(east)

        # Compute north as the cross product of up and east.
        north = np.cross(up, east)

        # Build the rotation matrix whose columns are the world frame axes
        # expressed in the sensor frame: [east, north, up].
        self.alignment_matrix = np.column_stack((east, north, up))
        return self.alignment_matrix

    def align(self, sensor_reading):
        """
        Given a sensor reading (a 3-element vector in sensor space),
        return the reading transformed into the aligned world frame.
        The rotation matrix was computed such that:

            sensor_vector = R * world_vector   =>   world_vector = R.T * sensor_vector

        Thus, we transform with the transpose of the alignment matrix.
        """
        if self.alignment_matrix is None:
            return sensor_reading
        return self.alignment_matrix.T @ sensor_reading
