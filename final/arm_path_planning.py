import cv2
import numpy as np

h = 300
w = 300

# def xyt_to_cv2(xyt):
#     x,y,t = xyt
#     x = 150+x
#     y = 150-y
#     t = 360-t
#     return (x,y,t)

def xy_to_cv2(xy):
    x,y = xy
    x = int(round(150+x))
    y = int(round(150-y))
    return (x,y)

map = np.zeros((h,w,3),dtype=np.uint8)
rect1_p1 = xy_to_cv2((-30,-8))
rect1_p2 = xy_to_cv2((30,-20))
rect2_p1 = xy_to_cv2((-9,-20))
rect2_p2 = xy_to_cv2((9,-34))
rect3_p1 = xy_to_cv2((-30,-34))
rect3_p2 = xy_to_cv2((30,-69))

map = cv2.rectangle(map,rect1_p1,rect1_p2,(255,255,255),-1)
map = cv2.rectangle(map,rect2_p1,rect2_p2,(255,255,255),-1)
map = cv2.rectangle(map,rect3_p1,rect3_p2,(255,255,255),-1)
buffer_map = map.copy()
buffer_map = cv2.cvtColor(buffer_map,cv2.COLOR_BGR2GRAY)
buffer_set = set()
buffer_coord = np.where(buffer_map==255)
for i in range(0,len(buffer_coord[0])):
    buffer_xy = (buffer_coord[1][i],buffer_coord[0][i])
    buffer_set.add(buffer_xy)



xy_origin = (0,0)
cv2_origin = xy_to_cv2(xy_origin)


len_1 = 38
len_2 = 27
len_3 = 27

xy_start = (0,len_1+len_2+len_3)
cv2_start = xy_to_cv2(xy_start)

def euc_dist(p1,p2):
    x1,y1 = p1
    x2,y2 = p2
    dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return int(round(dist))

def move_arm_to(ext,ang_1,ang_2,ang_3):
    p1x = (len_1+ext)*np.cos(np.deg2rad(ang_1))
    p1y = (len_1+ext)*np.sin(np.deg2rad(ang_1))
    p2x = p1x+(len_2*(np.cos(np.deg2rad(ang_1+ang_2))))
    p2y = p1y+(len_2*(np.sin(np.deg2rad(ang_1+ang_2))))
    p3x = p2x+(len_3*(np.cos(np.deg2rad(ang_1+ang_2+ang_3))))
    p3y = p2y+(len_3*(np.sin(np.deg2rad(ang_1+ang_2+ang_3))))
    return (p1x,p1y),(p2x,p2y),(p3x,p3y)

def draw_line_square_ends(img, pt1, pt2, color, thickness):
    x1, y1, x2, y2 = *pt1, *pt2
    theta = np.pi - np.arctan2(y1 - y2, x1 - x2)
    dx = int(np.sin(theta) * thickness / 2)
    dy = int(np.cos(theta) * thickness / 2)
    pts = [
        [x1 + dx, y1 + dy],
        [x1 - dx, y1 - dy],
        [x2 - dx, y2 - dy],
        [x2 + dx, y2 + dy]
    ]
    img = cv2.fillPoly(img, [np.array(pts)], color)
    return img

def line_cross(state1,state2):
    lc_map = np.zeros((h,w,1),dtype=np.uint8)
    ext1,ang1_1,ang2_1,ang3_1 = state1
    ext2,ang1_2,ang2_2,ang3_2 = state2
    d_ext = (ext2-ext1)/10
    d_ang1 = (ang1_2-ang1_1)/10
    d_ang2 = (ang2_2-ang2_1)/10
    d_ang3 = (ang3_2-ang3_1)/10
    for i in range(0,11):
        ext = ext1+(d_ext*i)
        ext = max(min(19,ext),0)

        ang1 = ang1_1+(d_ang1*i)
        ang1 = max(min(ang1,180),0)

        ang2 = ang2_1+(d_ang2*i)
        ang2 = max(min(ang2,90),-90)

        ang3 = ang3_1+(d_ang3*i)
        ang3 = max(min(ang3,90),-90)

        p1,p2,p3 = move_arm_to(ext,ang1,ang2,ang3)
        cv_p1 = xy_to_cv2(p1)
        cv_p2 = xy_to_cv2(p2)
        cv_p3 = xy_to_cv2(p3)
        lc_map = draw_line_square_ends(lc_map,cv2_origin,cv_p1,255,10)
        lc_map = draw_line_square_ends(lc_map,cv_p1,cv_p2,255,10)
        lc_map = draw_line_square_ends(lc_map,cv_p2,cv_p3,255,10)
        cv2.imshow('lcmap',lc_map)
        cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    lc_coords = np.where(lc_map==255)
    for i in range(0,len(lc_coords[0])):
        lxy = (lc_coords[1][i],lc_coords[0][i])
        if lxy in buffer_set:
            return True
    for item in buffer_set:
        lc_map[item[1],item[0]] = 255
    cv2.imshow('lcmap',lc_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return False
        
state_1 = (0,90,0,0)
state_2 = (19,0,-90,-90)
print(line_cross(state_1,state_2))


goal = (30,-27)




