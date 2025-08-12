import math


def lerp(a, b, t):
    return a + (b - a) * t


def ease_in_ease_out(a, b, t):
    return a + (b - a) * (t * t * (3 - 2 * t))

def exponential(a, b, t):
    return a + (b - a) * (math.pow(2, 10 * t))