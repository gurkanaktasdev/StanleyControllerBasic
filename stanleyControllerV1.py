
import cv2
import numpy as np
import math


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < math.pi:
        angle += 2*math.pi
    return angle

def find_closet_point(vehicle_pos,path):
    min_dist = float('inf')    
    closet_indx = 0
    for i,pt in enumerate(path):
        dist = math.hypot(vehicle_pos[0]-pt[0],vehicle_pos[1]-pt[1])
        if dist < min_dist:
            min_dist = dist
            closet_indx = i
    return closet_indx,path[closet_indx]

def cross_track_error(vehicle_pos,closest_point,path_heading):
    dx = vehicle_pos[0] - closest_point[0]
    dy = vehicle_pos[1] - closest_point[1]
    return dx * math.sin(path_heading) - dy * math.cos(path_heading)

width,height = 1200,1000
canvas = np.ones((height,width,3),dtype=np.uint8)*255

vehicle = {
    'x'  : 100,
    'y'  : 800,               
    'yaw': 30,            #radian
    'v'  : 5
}

L = 50        # axle distance(pixel based)
k_e = 1.0     # Cross-track gain
k_v = 1.0     # low-speed offset
dt = 1.0      # time stamped

# Create the Path
path = []
for x in range(100,1200,50):
    y = 500 + 100*math.sin(x/200.0)
    path.append((int(x),int(y)))


# simulations number of frames
for _ in range(500):  

    # clear the frame per frame
    canvas.fill(255)
    idx, closest_pt = find_closet_point((vehicle['x'], vehicle['y']), path)
    
    if idx < len(path)-1:
        next_pt = path[idx+1]
    else:
        next_pt = path[idx]
    path_heading = math.atan2(next_pt[1]-closest_pt[1], next_pt[0]-closest_pt[0])

    # Paint the  path
    for i in range(len(path)-1):                            
        cv2.line(canvas, path[i], path[i+1], (0,255,0), 2)  
    
    degree = math.degrees(vehicle['yaw'])
    cv2.putText(canvas,"Stanley Controller Sim.",(100,200),2,2,(0,255,0),2)

    e_heading = normalize_angle(path_heading - vehicle['yaw'])
    e_cte = cross_track_error((vehicle['x'], vehicle['y']), closest_pt, path_heading)

    # Stanley control law
    delta = e_heading + math.atan2(k_e*e_cte, k_v + vehicle['v'])
    
    # Update vehicle pos
    vehicle['x'] += vehicle['v'] * math.cos(vehicle['yaw']) * dt
    vehicle['y'] += vehicle['v'] * math.sin(vehicle['yaw']) * dt
    vehicle['yaw'] += (vehicle['v']/L) * math.tan(delta) * dt
    
    # Vehicle Design
    px = int(vehicle['x'])
    py = int(vehicle['y'])
    yaw = vehicle['yaw']
    triangle = np.array([
        [px + 15*math.cos(yaw), py + 15*math.sin(yaw)],
        [px - 10*math.cos(yaw) + 5*math.sin(yaw), py - 10*math.sin(yaw) - 5*math.cos(yaw)],
        [px - 10*math.cos(yaw) - 5*math.sin(yaw), py - 10*math.sin(yaw) + 5*math.cos(yaw)]
    ], np.int32)
    cv2.fillPoly(canvas, [triangle], (255,0,0))
    
    cv2.imshow("Stanley Simulation", canvas)
    key = cv2.waitKey(20)
    if key == 27:  # Press ESC to quit
        break

cv2.waitKey(0)
cv2.destroyAllWindows()