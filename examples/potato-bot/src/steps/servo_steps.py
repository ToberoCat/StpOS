"""
Arm servo unten
"""
from libstp_helpers.api.steps import slow_servo, servo



def arm_servo_unten_fahrt():
    return slow_servo("arm_servo_unten",85,1.5)
def arm_servo_unten_potatoe_grab():
    return slow_servo("arm_servo_unten", 167, 1.5)
def arm_servo_unten_potatoe_frie():
    return slow_servo("arm_servo_unten", 35, 2)


def arm_servo_oben_setup():
    return slow_servo("arm_servo_oben", 90, 1.5)
def arm_servo_oben_potaoe_grab():
    return servo("arm_servo_oben", 0)
def arm_servo_oben_potatoe_frie():
    return slow_servo("arm_servo_oben", 115, 2)

def arm_mini_servo_setup():
    return slow_servo("arm_mini_servo", 0, 1.5)
def arm_mini_servo_potatoe_zu():
    return servo("arm_mini_servo", 19)
def arm_mini_servo_potatoe_offen():
    return servo("arm_mini_servo", 127)

def flaschen_servo_Setup():
    return slow_servo("flaschen_servo",0, 1.5)

def flaschen_servo_fahrt():
    return slow_servo("flaschen_servo", 82, 1)

def flaschen_servo_fahrt_midpoint():
    return slow_servo("flaschen_servo", 30, 0.8)

def flaschen_servo_grab():
    return slow_servo("flaschen_servo", 13, 1.5)
def flaschen_servo_down():
    return servo("flaschen_servo", 4)
def flaschen_servo_real_setup():
    return slow_servo("flaschen_servo", 148, 0.1)

def flaschen_servo_safe():
    return slow_servo("flaschen_servo", 40, 0.5)

#angel = position*servo_Maxangel(170)/ServoMaxPosition(2048)