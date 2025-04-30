import cv2
import numpy as np
import heapq

test_set = set()
for a in [-180,-135,-90,-45,0,45,90,135,180]:
    for b in [-180,-135,-90,-45,0,45,90,135,180]:
        for c in [-180,-135,-90,-45,0,45,90,135,180]:
            test_set.add(a+b+c)

print(len(test_set))

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
    x1,y1 = p1[:2]
    x2,y2 = p2[:2]
    dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return int(round(dist))

def move_arm_to(state):
    ext,ang_1,ang_2,ang_3 = state
    p1x = (len_1+ext)*np.cos(np.deg2rad(ang_1))
    p1y = (len_1+ext)*np.sin(np.deg2rad(ang_1))
    p2x = p1x+(len_2*(np.cos(np.deg2rad(ang_1+ang_2))))
    p2y = p1y+(len_2*(np.sin(np.deg2rad(ang_1+ang_2))))
    p3x = p2x+(len_3*(np.cos(np.deg2rad(ang_1+ang_2+ang_3))))
    p3y = p2y+(len_3*(np.sin(np.deg2rad(ang_1+ang_2+ang_3))))
    return (p1x,p1y),(p2x,p2y),(p3x,p3y,ang_1+ang_2+ang_3)

def plot_arm_position(map,state):
    p1,p2,p3 = move_arm_to(state)
    p3 = p3[:2]
    cvp1 = xy_to_cv2(p1)
    cvp2 = xy_to_cv2(p2)
    cvp3 = xy_to_cv2(p3)
    map = cv2.line(map,cv2_origin,cvp1,(255,0,0),1)
    map = cv2.line(map,cvp1,cvp2,(0,255,0),1)
    map = cv2.line(map,cvp2,cvp3,(0,0,255),1)
    return map


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
    d_ext = (ext2-ext1)/5
    d_ang1 = (ang1_2-ang1_1)/3
    d_ang2 = (ang2_2-ang2_1)/3
    d_ang3 = (ang3_2-ang3_1)/3
    for i in range(0,4):
        ext = ext1+(d_ext*i)
        ext = max(min(20,ext),0)

        ang1 = ang1_1+(d_ang1*i)
        ang1 = max(min(ang1,180),0)

        ang2 = ang2_1+(d_ang2*i)
        ang2 = max(min(ang2,90),-90)

        ang3 = ang3_1+(d_ang3*i)
        ang3 = max(min(ang3,90),-90)

        p1,p2,p3 = move_arm_to((ext,ang1,ang2,ang3))
        cv_p1 = xy_to_cv2(p1)
        cv_p2 = xy_to_cv2(p2)
        cv_p3 = xy_to_cv2(p3[:2])
        lc_map = draw_line_square_ends(lc_map,cv2_origin,cv_p1,255,10)
        lc_map = draw_line_square_ends(lc_map,cv_p1,cv_p2,255,10)
        lc_map = draw_line_square_ends(lc_map,cv_p2,cv_p3,255,10)
        #cv2.imshow('lcmap',lc_map)
        #cv2.waitKey(1)
    #cv2.destroyAllWindows()
    
    lc_coords = np.where(lc_map==255)
    for i in range(0,len(lc_coords[0])):
        lxy = (lc_coords[1][i],lc_coords[0][i])
        if lxy in buffer_set:
            return True
    for item in buffer_set:
        lc_map[item[1],item[0]] = 255
    #cv2.imshow('lcmap',lc_map)
    #cv2.waitKey(1)
    #cv2.destroyAllWindows()
    return False


#POSSIBLE ANGLE COMBOS {0, 225, -90, 135, -180, 45, 270, -270, -45, 180, -135, 90, -225}

start_xyt = (31,-27,-180)
start_state = (20,0,-90,-90)

goal_xyt = (-73,46,180)

tc_matrix = np.zeros((h,w,25))
c2c_matrix = np.zeros((h,w,25))
px_matrix = np.zeros((h,w,25))
py_matrix = np.zeros((h,w,25))
pt_matrix = np.zeros((h,w,25))
pext_matrix = np.zeros((h,w,25))
pang1_matrix = np.zeros((h,w,25))
pang2_matrix = np.zeros((h,w,25))
pang3_matrix = np.zeros((h,w,25))

cx1_matrix = np.zeros((h,w,25))
cy1_matrix = np.zeros((h,w,25))
cx2_matrix = np.zeros((h,w,25))
cy2_matrix = np.zeros((h,w,25))
cext_matrix = np.zeros((h,w,25))
cang1_matrix = np.zeros((h,w,25))
cang2_matrix = np.zeros((h,w,25))
cang3_matrix = np.zeros((h,w,25))


open_list = []
closed_list = []
heapq.heapify(open_list)
heapq.heapify(closed_list)


def node_to_index(xyt):
    x,y,t = xyt
    x_idx = int((w/10)-x)
    y_idx = int((h/10)-y)
    t_idx = int(t/45)+6
    return y_idx,x_idx,t_idx


def move_node(node,state_changes):
    p_xyt = node[1]
    p_idx = node_to_index(p_xyt)
    p_state = (pext_matrix[p_idx],pang1_matrix[p_idx],pang2_matrix[p_idx],pang3_matrix[p_idx])
    d_ext,d_ang1,d_ang2,d_ang3 = state_changes
    c_ext = max(min(p_state[0]+d_ext,20),0)
    c_ang1 = max(min(p_state[1]+d_ang1,180),0)
    c_ang2 = max(min(p_state[2]+d_ang2,90),-90)
    c_ang3 = max(min(p_state[3]+d_ang3,90),-90)
    c_state = (c_ext,c_ang1,c_ang2,c_ang3)
    c_1_xy,c_2_xy,c_xyt = move_arm_to(c_state)
    c_idx = node_to_index(c_xyt)
    if line_cross(p_state,c_state):
        return
    current_tc = tc_matrix[c_idx]
    p_c2c = c2c_matrix[p_idx]
    step_cost = abs(state_changes[0])+abs(state_changes[1])+abs(state_changes[2])+abs(state_changes[3])
    c_c2c = p_c2c+step_cost
    c_c2g = euc_dist(goal_xyt,c_xyt)
    c_tc = c_c2c+(c_c2g*2)
    c_node = (c_tc,c_xyt)
    if current_tc == 0:
        tc_matrix[c_idx] = c_tc
        c2c_matrix[c_idx] = c_c2c
        px_matrix[c_idx] = p_xyt[0]
        py_matrix[c_idx] = p_xyt[1]
        pt_matrix[c_idx] = p_xyt[2]
        pext_matrix[c_idx] = p_state[0]
        pang1_matrix[c_idx] = p_state[1]
        pang2_matrix[c_idx] = p_state[2]
        pang3_matrix[c_idx] = p_state[3]
        cx1_matrix[c_idx] = c_1_xy[0]
        cy1_matrix[c_idx] = c_1_xy[1]
        cx2_matrix[c_idx] = c_2_xy[0]
        cy2_matrix[c_idx] = c_2_xy[1]
        cext_matrix[c_idx] = c_ext
        cang1_matrix[c_idx] = c_ang1
        cang2_matrix[c_idx] = c_ang2
        cang3_matrix[c_idx] = c_ang3
        heapq.heappush(open_list,c_node)

    elif current_tc > c_tc:
        old_node = (current_tc,c_xyt)
        open_list.remove(old_node)
        tc_matrix[c_idx] = c_tc
        c2c_matrix[c_idx] = c_c2c
        px_matrix[c_idx] = p_xyt[0]
        py_matrix[c_idx] = p_xyt[1]
        pt_matrix[c_idx] = p_xyt[2]
        pext_matrix[c_idx] = p_state[0]
        pang1_matrix[c_idx] = p_state[1]
        pang2_matrix[c_idx] = p_state[2]
        pang3_matrix[c_idx] = p_state[3]
        cx1_matrix[c_idx] = c_1_xy[0]
        cy1_matrix[c_idx] = c_1_xy[1]
        cx2_matrix[c_idx] = c_2_xy[0]
        cy2_matrix[c_idx] = c_2_xy[1]
        cang1_matrix[c_idx] = c_ang1
        cang2_matrix[c_idx] = c_ang2
        cang3_matrix[c_idx] = c_ang3
        heapq.heappush(open_list,c_node)
    else:
        return

first_ctc = 0
first_ctg = euc_dist(start_xyt[:2],goal_xyt[:2])
first_node = (first_ctg,start_xyt)
heapq.heappush(open_list,first_node)
first_idx = node_to_index(start_xyt)
print(first_idx)
tc_matrix[first_idx] = first_ctg
c2c_matrix[first_idx] = 0
px_matrix[first_idx] = start_xyt[0]
py_matrix[first_idx] = start_xyt[1]
pt_matrix[first_idx] = start_xyt[2]
pext_matrix[first_idx] = start_state[0]
pang1_matrix[first_idx] = start_state[1]
pang2_matrix[first_idx] = start_state[2]
pang3_matrix[first_idx] = start_state[3]
(c1x,c1y),(c2x,c2y),_ = move_arm_to(start_state)
cx1_matrix[first_idx] = c1x
cy1_matrix[first_idx] = c1y
cx2_matrix[first_idx] = c2x
cy2_matrix[first_idx] = c2y
map = plot_arm_position(map,start_state)

cv2.imshow('map',map)
cv2.waitKey(0)
cv2.destroyAllWindows()

while True:
    parent_node = heapq.heappop(open_list)
    heapq.heappush(closed_list,parent_node)

    n_xyt = parent_node[1]
    n_idx = node_to_index(n_xyt)
    if euc_dist(goal_xyt,n_xyt) < 5 and goal_xyt[2] == n_xyt[2]:
        break
    
    n_ext = cext_matrix[n_idx]
    n_ang1 = cang1_matrix[n_idx]
    n_ang2 = cang2_matrix[n_idx]
    n_ang3 = cang3_matrix[n_idx]
    n_state = (n_ext,n_ang1,n_ang2,n_ang3)
    map = plot_arm_position(map,n_state)
    cv2.imshow('map',map)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    for ext_d in [-20,-10,0,10,20]:
        for ang1_d in [-180,-135,-90,-45,0,45,90,135,180]:
            for ang2_d in [-180,-135,-90,-45,0,45,90,135,180]:
                for ang3_d in [-180,-135,-90,-45,0,45,90,135,180]:
                    state_d = (ext_d,ang1_d,ang2_d,ang3_d)
                    move_node(parent_node,state_d)
cv2.waitKey(0)
cv2.destroyAllWindows()

    

