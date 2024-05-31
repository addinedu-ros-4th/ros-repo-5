import math
from tf_transformations import quaternion_from_euler

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    return quaternion_from_euler(roll, pitch, yaw)

route_forward = [
    (0.25, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    (0.1, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),
    (0.1 , 0.45, 0.0, *euler_to_quaternion(math.radians(90))),
    (0.1 , 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (0.45, 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (0.3, 0.3, 0.0, *euler_to_quaternion(math.radians(-135))),
    (0.3, 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (-0.05 , 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.1 , 0.3, 0.0, *euler_to_quaternion(math.radians(90))),
    
    (0.1, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (0.1, 0.7, 0.0, *euler_to_quaternion(math.radians(0))),
    
    (1.1, 0.7, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.25, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.55, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(135))),
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.05, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.2, 0.4, 0.0, *euler_to_quaternion(math.radians(-90))),
    
    (1.2, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.45, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.95 ,0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(-45))),
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.25, 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.1, 0.3, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.1, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.1, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    
    (2.35, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.2, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),
    
    (2.2, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (2.2, 0.7, 0.0, *euler_to_quaternion(math.radians(180))),
    (2.2, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    (2.2 ,0.25, 0.0, *euler_to_quaternion(math.radians(-90))),
    (2.2 ,0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.65 ,0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.8 ,0.4, 0.0, *euler_to_quaternion(math.radians(45))),
    (1.8 ,0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.35, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.2, 0.4, 0.0, *euler_to_quaternion(math.radians(-90))),
        ]

route_reverse = [
    (2.2, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (2.2, 0.0, 0.0, *euler_to_quaternion(math.radians(180))),
    
    (1.05, 0.0, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.45, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.95 ,0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(-45))),
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.15, 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.0, 0.3, 0.0, *euler_to_quaternion(math.radians(90))),
    
    (1.0, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.25, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.55, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(135))),
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.5, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.0, 0.4, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.0, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.0, 0.7, 0.0,     *euler_to_quaternion(math.radians(180))),

    (-0.05, 0.7, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.1, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    
    (0.1, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (0.1, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
]

route_forward_segments = [
    [
    (0.25, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    (0.1, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),    
    ],
    [
    (0.1, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (0.1, 0.7, 0.0, *euler_to_quaternion(math.radians(0))),            
    ],
    [
    (1.1, 0.7, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    ],
    [
    (1.2, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(180))),
    ],
    [
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.35, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.2, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),            
    ],
    [
    (2.2, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (2.2, 0.7, 0.0, *euler_to_quaternion(math.radians(180))),
    ],
]

route_reverse_segments = [
    [
    (2.2, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (2.2, 0.0, 0.0, *euler_to_quaternion(math.radians(180))),    
    ],
    [
    (1.05, 0.0, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),           
    ],
    [
    (1.0, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(180))),
    ],
    [
    (-0.05, 0.7, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.1, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    ],
    [
    (0.1, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (0.1, 0.0, 0.0, *euler_to_quaternion(math.radians(0)))
    ],
]

route_forward_description = [
    [
    (0.1 , 0.45, 0.0, *euler_to_quaternion(math.radians(90))),
    (0.1 , 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (0.45, 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (0.3, 0.3, 0.0, *euler_to_quaternion(math.radians(-135))),
    ],
    [
    (1.0, 0.25, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.55, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(135))),
    ],
    [
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.45, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.95 ,0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(-45))),
    ],
    [
    (2.2, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    (2.2 ,0.25, 0.0, *euler_to_quaternion(math.radians(-90))),
    (2.2 ,0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.65 ,0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.8 ,0.4, 0.0, *euler_to_quaternion(math.radians(45))),
    ],
]

route_reverse_description = [
    [
    (1.2, 0.45, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.2, 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.95 ,0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(-45))),
    ],
    [
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.25, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.0, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.55, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(135))),
    ],
]

route_forward_return = [
    [
    (0.3, 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (-0.05 , 0.3, 0.0, *euler_to_quaternion(math.radians(180))),
    (0.1 , 0.3, 0.0, *euler_to_quaternion(math.radians(90))),
    ],
    [
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.05, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.2, 0.4, 0.0, *euler_to_quaternion(math.radians(-90))),
    ],
    [
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.35, 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.2, 0.3, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.2, -0.15, 0.0, *euler_to_quaternion(math.radians(-90))),
    (1.2, 0.0, 0.0, *euler_to_quaternion(math.radians(0))),
    ],
    [
    (1.8 ,0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.35, 0.4, 0.0, *euler_to_quaternion(math.radians(0))),
    (2.2, 0.4, 0.0, *euler_to_quaternion(math.radians(-90))),
    ],
]

route_reverse_return = [
    [
    (0.8 ,0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.15, 0.3, 0.0, *euler_to_quaternion(math.radians(0))),
    (1.0, 0.3, 0.0, *euler_to_quaternion(math.radians(90))),
    ],
    [
    (1.4, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.5, 0.4, 0.0, *euler_to_quaternion(math.radians(180))),
    (1.0, 0.4, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.0, 0.85, 0.0, *euler_to_quaternion(math.radians(90))),
    (1.0, 0.7, 0.0, *euler_to_quaternion(math.radians(180))),
    ],
]