# -*- coding:utf-8 -*-

import time
import zmq
import cv2
import numpy as np
import random
import math

car_length = 4.765
car_width = 1.845
vehicle_base_length = 2.7
front_overhang = 0.99
rear_overhang = 1.075
map_size = 1000


resolution_ = 0.02
ref = [map_size / 2, map_size / 2]

import matplotlib.pyplot as plt
def curvefit(trajectory):

    trajectory = np.array(trajectory)
    trajectory_start = trajectory[:4]
    trajectory_end = trajectory[-4:]
    trajectory = trajectory[::4]
    trajectory_start = np.vstack((trajectory_start, trajectory, trajectory_end))
    trajectory = trajectory_start
    # trajectory_start.extend(trajectory_end)
    trajectory_x = trajectory[:,0]
    trajectory_y = trajectory[:,1]

    poly_mode = np.poly1d(np.polyfit(trajectory_x, trajectory_y, 4))
    poly_line = np.linspace(1, trajectory_x.max(), 100)

    plt.scatter(trajectory_x, trajectory_y)
    plt.plot(poly_line, poly_mode(poly_line))
    plt.show()


def normalizeAngle2Pi(angle):
    while (angle < 0):
        angle += 2.0 * math.pi
    while (angle > 2 * math.pi):
        angle -= 2.0 * math.pi
    return angle

def move_forward(x, y, theta, min_radius):
    # min_radius = 4.19
    cx = x - min_radius * math.sin(theta)
    cy = y + min_radius * math.cos(theta)
    return cx, cy, min_radius


class ClientNode:

    def __init__(self, resolution):
        print("resolution:", resolution)
        self.resolution_ = resolution
        self.count_ = 0
        self.pose_list_ = []
        self.plan_trajectory_ = []
        self.plan_trajectory_whole_ = []
        self.plan_trajectory_pose_ = []
        self.uss_points_ = []
        self.parking_points_list_ = []
        self.simulate_pose_index_ = 0
        self.strategy_label_ = ""
        self.plan_label_ = ""
        self.plan_type_label_ = ""
        self.trajectory_type_label_ = ""
        self.speeds_ = []
        self.debug_mat_ = np.ones((map_size, map_size, 3)) * 255

    def index_x(self, x):
        ret = int(ref[0] + x / self.resolution_)
        if ret < 0:
            return 0
        elif ret >= map_size:
            return map_size - 1
        else:
            return ret

    def index_y(self, y):
        ret = int(ref[1] - y / self.resolution_)
        if ret < 0:
            return 0
        elif ret >= map_size:
            return map_size - 1
        else:
            return ret

    def get_boundary(self, x, y, theta, res):
        front_distance = (vehicle_base_length + front_overhang) / res
        rear_distance = (rear_overhang) / res
        midle_wid = (car_width / 2) / res
        front_center_x = x + front_distance * math.cos(theta)
        front_center_y = y + front_distance * math.sin(theta)
        rear_center_x = x - rear_distance * math.cos(theta)
        rear_center_y = y - rear_distance * math.sin(theta)
        head_cut_wid_ = 0.2 / res
        head_cut_hei_ = 0.2 / res
        tail_cut_wid_ = 0.2 / res
        tail_cut_hei_ = 0.3 / res

        x0 = front_center_x + midle_wid * math.cos(theta + math.pi / 2)
        y0 = front_center_y + midle_wid * math.sin(theta + math.pi / 2)

        x01 = x0 + head_cut_wid_ * math.cos(theta - math.pi / 2)
        y01 = y0 + head_cut_wid_ * math.sin(theta - math.pi / 2)
        x02 = x0 + head_cut_hei_ * math.cos(theta + math.pi)
        y02 = y0 + head_cut_hei_ * math.sin(theta + math.pi)

        x3 = front_center_x + midle_wid * math.cos(theta - math.pi / 2)
        y3 = front_center_y + midle_wid * math.sin(theta - math.pi / 2)

        x31 = x3 + head_cut_hei_ * math.cos(theta + math.pi)
        y31 = y3 + head_cut_hei_ * math.sin(theta + math.pi)
        x32 = x3 + head_cut_wid_ * math.cos(theta + math.pi / 2)
        y32 = y3 + head_cut_wid_ * math.sin(theta + math.pi / 2)

        x1 = rear_center_x + midle_wid * math.cos(theta + math.pi / 2)
        y1 = rear_center_y + midle_wid * math.sin(theta + math.pi / 2)

        x11 = x1 + tail_cut_hei_ * math.cos(theta)
        y11 = y1 + tail_cut_hei_ * math.sin(theta)
        x12 = x1 + tail_cut_wid_ * math.cos(theta - math.pi / 2)
        y12 = y1 + tail_cut_wid_ * math.sin(theta - math.pi / 2)

        x2 = rear_center_x + midle_wid * math.cos(theta - math.pi / 2)
        y2 = rear_center_y + midle_wid * math.sin(theta - math.pi / 2)

        x21 = x2 + tail_cut_wid_ * math.cos(theta + math.pi / 2)
        y21 = y2 + tail_cut_wid_ * math.sin(theta + math.pi / 2)
        x22 = x2 + tail_cut_hei_ * math.cos(theta)
        y22 = y2 + tail_cut_hei_ * math.sin(theta)

        # safe circle
        uss_safe_cicle_head_tail_ = 0.3 / res
        uss_safe_cicle_corner_ = 0.15 / res
        uss_safe_cicle_two_side_ = 0.2 / res

        tiny_theta_offset =  math.pi/4
        x001 = x01 + uss_safe_cicle_head_tail_ * math.cos(theta)
        y001 = y01 + uss_safe_cicle_head_tail_ * math.sin(theta)
        x002 = x02 + uss_safe_cicle_corner_ * math.cos(theta + math.pi / 2 - tiny_theta_offset)
        y002 = y02 + uss_safe_cicle_corner_ * math.sin(theta + math.pi / 2 - tiny_theta_offset)

        # left_side_x = x + (midle_wid + uss_safe_cicle_two_side_) * math.cos(theta + math.pi / 2)
        # left_side_y = y + (midle_wid + uss_safe_cicle_two_side_) * math.sin(theta + math.pi / 2)

        x111 = x11 + uss_safe_cicle_corner_ * math.cos(theta + math.pi / 2 + tiny_theta_offset)
        y111 = y11 + uss_safe_cicle_corner_ * math.sin(theta + math.pi / 2 + tiny_theta_offset)
        x112 = x12 + uss_safe_cicle_head_tail_ * math.cos(theta + math.pi)
        y112 = y12 + uss_safe_cicle_head_tail_ * math.sin(theta + math.pi)

        x221 = x21 + uss_safe_cicle_head_tail_ * math.cos(theta + math.pi)
        y221 = y21 + uss_safe_cicle_head_tail_ * math.sin(theta + math.pi)
        x222 = x22 + uss_safe_cicle_corner_ * math.cos(theta - math.pi / 2 - tiny_theta_offset)
        y222 = y22 + uss_safe_cicle_corner_ * math.sin(theta - math.pi / 2 - tiny_theta_offset)

        # right_side_x = x + (midle_wid + uss_safe_cicle_two_side_) * math.cos(theta - math.pi / 2)
        # right_side_y = y + (midle_wid + uss_safe_cicle_two_side_) * math.sin(theta - math.pi / 2)

        x331 = x31 + uss_safe_cicle_corner_ * math.cos(theta - math.pi / 2 + tiny_theta_offset)
        y331 = y31 + uss_safe_cicle_corner_ * math.sin(theta - math.pi / 2 + tiny_theta_offset)
        x332 = x32 + uss_safe_cicle_head_tail_ * math.cos(theta)
        y332 = y32 + uss_safe_cicle_head_tail_ * math.sin(theta)

        return [[x01, y01], [x02, y02], [x11, y11], [x12, y12], [x21, y21], [x22, y22], [x31, y31], [x32, y32]], \
               [[x001, y001], [x002, y002], [x111, y111], [x112, y112], [x221, y221],
                [x222, y222], [x331, y331], [x332, y332]]

    def draw_trajectory(self, poses, color):
        if len(poses) == 0:
            return
        r, g, b = (0, 0, 255), (0, 255, 0), (255, 0, 0)
        pre_pose = None
        color_list = [r, g, b] * 1000
        # print("color_list", color_list)
        color_index = 0
        local_color = color
        for pose in poses:
            if pre_pose is None or pose == pre_pose or pose == poses[-1]:
                local_color = color_list[color_index]
                color_index += 1
                # print("switch pose:", pose)
            else:
                local_color = color
            x = self.index_x(pose[0])
            y = self.index_y(pose[1])
            theta = -pose[2]
            cv2.circle(self.debug_mat_, (x, y), 5, local_color)
            x2 = int(x + 100*math.cos(theta))
            y2 = int(y + 100*math.sin(theta))
            cv2.line(self.debug_mat_, (x, y), (x2, y2), local_color)
            pre_pose = pose

    def draw_radius(self, pose_list, color, min_radius):
        for pose in pose_list:
            x ,y , theta = pose
            cx, cy, radius = move_forward(x, y, theta, min_radius)
            x = self.index_x(cx)
            y = self.index_y(cy)
            print("draw xy, radius", x, y, radius)
            cv2.circle(self.debug_mat_, (x, y), int(radius/resolution_), color)
    def draw_pots(self, points, color):
        for point in points:
            x = self.index_x(point[0])
            y = self.index_y(point[1])
            cv2.circle(self.debug_mat_, (x, y), 1, color)

    def draw_pose(self, poses, color):
        for pose in poses:
            x = self.index_x(pose[0])
            y = self.index_y(pose[1])
            theta = 0
            if len(pose) == 3:
                theta = -pose[2]
            cv2.circle(self.debug_mat_, (x, y), 5, color)
            x2 = int(x + 50*math.cos(theta))
            y2 = int(y + 50*math.sin(theta))
            cv2.line(self.debug_mat_, (x, y), (x2, y2), color)
    def draw_vehicle(self, poses, color):
        for pose in poses:
            x = self.index_x(pose[0])
            y = self.index_y(pose[1])
            theta = -pose[2]
            cv2.circle(self.debug_mat_, (x, y), 5, color)

            car_circle, safe_circle = self.get_boundary(-26.058495,6.460584,-1.599608, 1.0)
            car_circle, safe_circle = self.get_boundary(0,0,0, 1.0)
            # print("car_circle", car_circle)
            # print("safe_circle", safe_circle)
            self.draw_boundary(x, y, theta, color)
    def draw_boundary(self, x, y, theta, color):

        car_circle, safe_circle = self.get_boundary(x, y, theta, self.resolution_)
        car_circle_pts = np.array(car_circle, np.int32)
        car_circle_pts = car_circle_pts.reshape((-1, 1, 2))
        safe_circle_pts = np.array(safe_circle, np.int32)
        safe_circle_pts = safe_circle_pts.reshape((-1, 1, 2))
        cv2.polylines(self.debug_mat_, [car_circle_pts], True, color, thickness=1)
        cv2.polylines(self.debug_mat_, [safe_circle_pts], True, color, thickness=1)

        x2 = int(x + 100 * math.cos(theta))
        y2 = int(y + 100 * math.sin(theta))
        cv2.arrowedLine(self.debug_mat_, (x, y), (x2, y2), color, thickness=1)

    def draw_lines(self, points):
        pre_x_index = -1
        pre_y_index = -1
        for point in points:
            # point = point.split(",")
            index_x2 = self.index_x(float(point[0]))
            index_y2 = self.index_y(float(point[1]))
            if index_y2 >= map_size or index_x2 >= map_size:
                continue

            self.debug_mat_[index_y2][index_x2] = (0, 0, 0)
            if pre_x_index >= 0:
                cv2.line(self.debug_mat_, (pre_x_index, pre_y_index), (index_x2, index_y2), (0, 255, 0),
                         thickness=2)
            pre_x_index = index_x2
            pre_y_index = index_y2
    def draw_spots(self, parking_points_list_):
        if len(parking_points_list_) == 0:
            return
        self.parking_points_list_ = parking_points_list_
        color_list = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255)]
        color_list = color_list*50
        for index, points in enumerate(self.parking_points_list_):
            # points = points.split(" ")

            pre_x_index = -1
            pre_y_index = -1
            for point in points:
                # point = point.split(",")
                index_x2 = self.index_x(float(point[0]))
                index_y2 = self.index_y(float(point[1]))
                if index_y2 >= map_size or index_x2 >= map_size:
                    continue

                self.debug_mat_[index_y2][index_x2] = (0, 0, 0)
                if pre_x_index >= 0:
                    cv2.line(self.debug_mat_, (pre_x_index, pre_y_index), (index_x2, index_y2), color_list[index],
                             thickness=2)
                pre_x_index = index_x2
                pre_y_index = index_y2
    def show(self):
        cv2.imshow("pose", self.debug_mat_)
        cv2.waitKey(0)

def tf_local(poses, point):
    for index, pose in enumerate(poses):
        pose = poses[index]
        if len(pose) < 2:
            continue
        pose_list = list(pose)
        pose_list[0] -= point[0]
        pose_list[1] -= point[1]
        pose_tuple = tuple(pose_list)
        poses[index] = pose_tuple
#"(1,2,3" to (1,2,3)
def str_to_tuple(str):
    if str.startswith('('):
        str = str[1:]
    if str.endswith(')'):
        str = str[:-1]
    if str.endswith(') ob type '):
        str = str[:-len(') ob type ')]
    items = np.array(str.split(","))
    print("items", items)
    items = items.astype(np.float)
    return items

def angle(p1, p2):
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

def move_point(x, y, alpha, dis):
    ret_x = x + dis * math.cos(alpha);
    ret_y = y + dis * math.sin(alpha);
    return (ret_x, ret_y)

if __name__ == '__main__':

# ------------------------------------------data--------------------
    trajectory = []
    other_pose = []
    vehicle_pose = []
    other_pose2 = []
    vehicle_pose2 = []
    # line_points = []
    spots = []
    spots_frame = []
    steering_frame = []
    dynamic_poses = []
    uss_points = []
    steering = 0
    circle_pose = []
    # load_from_file = False
    load_from_file = True

    if load_from_file:

        # file = open("/home/xhpan/Downloads/dm_upload/pathology/1659599509247_XW-6984/logs/planning.log", "r")
        # file = open("/home/xhpan/Downloads/can_error_by_lam/planning.log", "r")
        # file = open("/home/xhpan/apa/xw/package/build/XuanWu-linux-ubuntu-1804/artifacts/linux-ubuntu-1804/XuanWu/log/planning_tuning.log", "r")
        # file = open("/home/xhpan/apa/planning/planning/build/Planning-linux-ubuntu-1804/debug/log/search_test.log", "r")
        # file = open("/home/xhpan/logs/pp01_-02/strategy.log", "r")#2-363  7-1410
        # file = open("/home/xhpan/Downloads/planning.log", "r")
        # file = open("/home/xhpan/logs/first_time/planning.log", "r")
        file = open("/home/fredric/linux-ubuntu-1804/XuanWu/log/planning.log", "r")
        # file = open("//home/xhpan/Downloads/logs/7976/data/apa/dm_upload/pathology/1667543297040236821/logs/strategy.log", "r")
     
        start_num = 168
        # start_num = 20657

        # start_num = 1722
        end_num = 331
        lines = file.readlines()[start_num:end_num]
        one_frame_spot = []
        publish_spot = False
        last_pose_num = 0
        last_spot_num = 0
        for num, line in enumerate(lines):
            if line.endswith("PlannerController::FinishAPA:kPlanFinishAPA\n") or \
                line.endswith("kPlanExecutingWaiting --> kPlanFail\n") or \
                line.endswith("PlannerController::PlanApaTrajectory plan_type_\n") or \
                line.endswith("IPlannerController::SetStatus:kPlanExecutingWaiting\n") or \
                line.endswith("Planner::WaitingObstacleAway plan status::kPlanExecutingWaiting\n") or \
                line.endswith("PlanningNode::OnStrategyStatus: :kStop\n"):
                print("last number:", start_num + num + 1)
                break
            # publish_spot_flag = line.find("MappingImpl::PublishParkingSpaces:[") > 0
            publish_spot_flag = line.find("MappingImpl::UpdateParkingSpotsData2 input") > 0
            if publish_spot_flag:
                one_frame_spot.clear()
            if (publish_spot_flag) or \
                    (publish_spot and line.find("is_target:") > 0):
                publish_spot = True
                ps_start = line.find("(")
                line = line.strip()
                if line.find("ob types") > 0:
                    line = line[:line.find(") ") + 1]
                parkingspace_str = line[ps_start:-1].split("),(")
                parkingspace = []
                for point_str in parkingspace_str:
                    parkingspace.append(str_to_tuple(point_str))
                one_frame_spot.append(parkingspace)
                last_spot_num = num
            if line.find("is_mechanical") < 0:
                publish_spot = False
                # spots_frame.append(one_frame_spot)
            if line.find("Planner::plan origin_parking_space_") > 0 or \
                    line.find("Planner::PlanApaTrajectory:id") > 0 or \
                    line.find("PlannerController::plan origin_parking_space_") > 0 or \
                    line.find("PlannerController::PlanApaTrajectory:id") > 0:
                ps_start = line.find("(")
                ps_end = line.find(") ")
                parkingspace_str = line[ps_start:ps_end].split("),(")
                parkingspace = []
                for point_str in parkingspace_str:
                    parkingspace.append(str_to_tuple(point_str))
                # offset = 0.15 / 2
                # parkingspace[0] = move_point(parkingspace[0][0],parkingspace[0][1], angle(parkingspace[0],parkingspace[3]), offset)
                # parkingspace[1] = move_point(parkingspace[1][0],parkingspace[1][1], angle(parkingspace[1],parkingspace[2]), offset)
                # parkingspace[2] = move_point(parkingspace[2][0],parkingspace[2][1], angle(parkingspace[2],parkingspace[1]), offset)
                # parkingspace[3] = move_point(parkingspace[3][0],parkingspace[3][1], angle(parkingspace[3],parkingspace[0]), offset)
                spots.append(parkingspace)
            if line.find("PlanningNode::Plan car_pose_:") > 0:
                pose_start = line.find("(")
                pose = line[pose_start:-1]
                vehicle_pose.append(str_to_tuple(pose))
            if line.find("Planner::PublishTrajectory") > 0 or \
                    line.find("PlannerController::PublishTrajectory") > 0:
                # trajectory = []
                trajectory_start = line.find("(")
                tr = line[trajectory_start:-2]
                items = tr.split("),(")
                for item in items:
                    trajectory.append(str_to_tuple(item))
            if line.find("PlannerController::OnUpdate global pose") > 0 or \
                    line.find("Planner::OnUpdate global pose") > 0 or \
                    line.find("PlanningNode::OnUssPoints pose:") > 0 or \
                line.find("Planner::plan start:") > 0 or \
                line.find("PlannerController::plan start:") > 0 or \
                    (line.find("LamNode::UpdatePoseData:") > 0 and (num - last_pose_num) > 100) :
                dynamic_pose_start = line.find("(")
                dynamic_pose = line[dynamic_pose_start:-1]
                if dynamic_pose.find("-0.0878") > 0:
                    pass
                dynamic_poses.append(str_to_tuple(dynamic_pose))
                last_pose_num = num
                spots_frame.append(one_frame_spot[:])
                steering_frame.append(steering)
            if line.find("PlannerController::FillUssPoints uss_data_near") > 0 or \
                    line.find("PlannerController::FillUssPoints uss_data_far") > 0:
                uss_start = line.find("(")
                if uss_start < 0:
                    continue
                uss_points_str = line[uss_start:-2]
                items = uss_points_str.split("),(")
                for item in items:
                    uss_points.append(str_to_tuple(item))

            if line.find("PlanningNode::OnSteering current_steering_") > 0:
                steering_start = line.find("]")
                if steering_start < 0:
                    continue
                steering = float(line[steering_start + 1:-1])
        pass
        # vehicle_pose = [(0,0,0),(0.153452,-0.083993,0.169230)]#19:35:17.608
        # other_pose2 = [(4.057911,-2.851364,1.159144)]
        # vehicle_pose2 = [(3.735971,0.157579,6.483616)]#19:35:17.608
        # trajectory = [(5.456384,11.821624,-0.000000),(5.606384,11.821624,0.000000),(5.756384,11.821624,0.000000),(5.906384,11.821624,0.000000),(6.056384,11.821624,0.000000),(6.206384,11.821624,0.000000),(6.356384,11.821624,0.000000),(6.506384,11.821624,0.000000),(6.656378,11.820593,-0.013749),(6.806312,11.816332,-0.043070),(6.956057,11.807677,-0.072390),(7.105484,11.794637,-0.101711),(7.254464,11.777221,-0.131032),(7.403040,11.756600,-0.144781),(7.551132,11.732786,-0.174102),(7.994329,11.654836,-0.174102),(8.433529,11.557507,-0.262064),(8.865639,11.431993,-0.303311),(9.288637,11.278885,-0.391273),(9.696548,11.089207,-0.479235),(10.086221,10.864428,-0.567198),(10.454639,10.606285,-0.655160),(10.798957,10.316776,-0.743122),(11.116510,9.998138,-0.831084),(11.404844,9.652836,-0.919046),(11.404844,9.652836,-0.919046),(11.147960,10.022133,-1.007008),(11.069672,10.150075,-1.036329),(10.995167,10.280258,-1.065650),(10.924511,10.412569,-1.094971),(10.857765,10.546894,-1.124291),(10.794985,10.683118,-1.153612),(10.735153,10.820667,-1.167361),(10.735153,10.820667,-1.167361),(10.792009,10.681866,-1.196682),(10.844772,10.541458,-1.226003),(10.893395,10.399563,-1.255323),(10.939936,10.256966,-1.255323),(10.984379,10.113707,-1.284644),(11.025728,9.969519,-1.298393),(11.063961,9.824478,-1.327714),(11.097926,9.678380,-1.357035),(11.127594,9.531348,-1.386355),(11.152938,9.383511,-1.415676),(11.175094,9.235156,-1.429425),(11.194048,9.086364,-1.458746),(11.208633,8.937080,-1.488067),(11.218836,8.787433,-1.517387),(11.226844,8.637647,-1.517387),(11.232655,8.487764,-1.546708),(11.236267,8.337808,-1.546708),(11.238849,8.187831,-1.560457),(11.238201,8.037838,-1.589778)]


        # vehicle_pose2 = [(-1.428392,2.502219,0.071309)]
        # uss_points = [(-1.1833,-4.60545),(-1.13521,-4.60318),(-1.09026,-4.6074),(-1.00105,-4.60068),(-0.972255,-4.60878),(-0.853314,-4.60325),(-0.813216,-4.60547),(-0.706822,-4.61184),(-0.616726,-4.60835),(-0.557131,-4.61139),(-0.507788,-4.61681),(-0.459276,-4.63011),(-0.395089,-4.6477),(-0.211563,-4.60716),(-0.199239,-4.60199),(-0.130978,-4.64072),(-0.0789375,-4.64933),(0.0555,-4.6015),(-1.00435,-4.58382),(-0.960143,-4.59802),(-0.906558,-4.59678),(-0.862761,-4.59048),(-0.804087,-4.58376),(-0.752997,-4.5989),(-0.651511,-4.59704),(-0.599403,-4.58297),(-0.412759,-4.59322),(-0.351489,-4.55191),(-0.332168,-4.57845),(-0.283217,-4.5981),(-0.212539,-4.58213),(-0.156365,-4.57277),(-0.119851,-4.57549),(-0.0570006,-4.56502),(0.0384527,-4.58486),(0.0940893,-4.58121),(0.144008,-4.58127),(0.196327,-4.58124),(0.228454,-4.57923),(0.292291,-4.5764),(0.333949,-4.57202),(0.385249,-4.57461),(0.436202,-4.57714),(0.461629,-4.57838),(0.50393,-4.56116),(0.563478,-4.55263),(0.619603,-4.55712),(-1.0889,-4.54555),(-1.01311,-4.50919),(-0.963889,-4.50821),(-0.353977,-4.51675),(-0.311356,-4.51305),(-0.261914,-4.51737),(-0.202145,-4.50627),(-0.159439,-4.52229),(-0.116616,-4.52226),(-0.0640587,-4.51861),(0.0430359,-4.52563),(0.0743628,-4.52398),(0.141863,-4.5205),(0.188685,-4.52463),(0.24698,-4.53233),(0.29239,-4.52471),(0.337255,-4.51434),(0.381408,-4.50127),(0.403075,-4.50113),(0.464422,-4.50035),(0.536751,-4.544),(0.574305,-4.54695),(0.600864,-4.54427),(0.653786,-4.53882),(0.710362,-4.53175),(0.754361,-4.52229),(0.827661,-4.52158),(0.873293,-4.50946),(0.974572,-4.50037),(-1.65661,-4.45121),(-1.17274,-4.45013),(-1.11989,-4.47315),(-1.07454,-4.47034),(-1.01936,-4.491),(-0.902996,-4.46401),(-0.844111,-4.46417),(-0.771212,-4.45606),(-0.728516,-4.474),(-0.321357,-4.49995),(-0.262804,-4.49924),(-0.148383,-4.49239),(-0.0793482,-4.49301),(0.0288167,-4.49703),(0.104328,-4.49722),(0.168844,-4.49321),(0.246668,-4.49677),(0.297422,-4.47903),(0.327521,-4.49922),(0.373597,-4.49344),(0.424689,-4.48555),(0.466937,-4.46724),(0.514135,-4.46575),(0.551326,-4.4575),(0.604002,-4.4522),(0.658617,-4.46081),(0.725942,-4.47488),(0.795188,-4.49421),(0.933763,-4.46139),(1.01129,-4.46593),(1.0712,-4.49686),(1.11681,-4.49038),(-1.78549,-4.40672),(-1.73821,-4.40659),(-1.69367,-4.44532),(-1.22505,-4.42363),(-1.18344,-4.41823),(-1.12928,-4.44613),(-1.08497,-4.43591),(-1.03849,-4.43912),(-0.991226,-4.43595),(-0.852764,-4.41225),(-0.812684,-4.43545),(0.447931,-4.43343),(0.498529,-4.43931),(0.547718,-4.42311),(0.559542,-4.40023),(0.605042,-4.40784),(0.659692,-4.40103),(0.703436,-4.41846),(0.752986,-4.41701),(0.821113,-4.42426),(0.881897,-4.43334),(0.98098,-4.41289),(1.01665,-4.41315),(1.12198,-4.40303),(-1.81289,-4.36806),(-1.76434,-4.35804),(-1.70733,-4.39272),(-1.67815,-4.36862),(-1.27666,-4.39352),(-1.23683,-4.38652),(-0.928112,-4.35846),(-0.891292,-4.38655),(0.585955,-4.39745),(0.604879,-4.36379),(0.650031,-4.35051),(0.702263,-4.35537),(0.754311,-4.36094),(0.809801,-4.37704),(0.864241,-4.37565),(0.915955,-4.37509),(1.17737,-4.35517),(-1.80304,-4.31977),(-1.77646,-4.33208),(-1.71432,-4.33818),(-1.65912,-4.3019),(-1.6203,-4.33612),(-1.55284,-4.3172),(-1.52856,-4.31756),(-1.49907,-4.3148),(-1.44188,-4.30195),(-0.963075,-4.32809),(0.637221,-4.34638),(0.678273,-4.34626),(0.706472,-4.34197),(0.762688,-4.33333),(0.810006,-4.32961),(0.856249,-4.31546),(0.910841,-4.31235),(0.957622,-4.33991),(1.03243,-4.34418),(-1.80799,-4.29858),(-1.78048,-4.27069),(-1.7103,-4.27398),(-1.69567,-4.26526),(-1.62304,-4.25121),(-1.56602,-4.28386),(-1.53763,-4.27987),(-1.48123,-4.26626),(-1.02688,-4.26101),(-0.996041,-4.29556),(0.98968,-4.29049),(1.01419,-4.29339),(1.09951,-4.26825),(1.12888,-4.29434),(1.16188,-4.26022),(-1.7298,-4.22636),(-1.65533,-4.21543),(-1.60873,-4.20395),(-1.57422,-4.24288),(-1.51857,-4.22847),(-1.05546,-4.22457),(1.21884,-4.24762),(1.30988,-4.2392),(-1.76139,-4.18537),(-1.66657,-4.1712),(-1.64106,-4.16318),(-1.55378,-4.18869),(-1.08167,-4.18639),(-1.67111,-4.12071),(-1.61741,-4.10365),(-1.58676,-4.14704),(-1.69878,-4.07665),(-1.64564,-4.05865)]

        # dynamic_poses = []
        #
        #
        line_points = [
            # (-0.356147,-0.691341),(-0.258832,-2.947971),(5.735565,-2.689531),(5.638250,-0.432901)
            # (-3.193548387096774, -4.876623376623376), (-3.193548387096774, -3.8766233766233764), (-2.193548387096774, -3.8766233766233764), (-2.193548387096774, -4.876623376623376),
            # (-3.314516129032258, -8.204545454545455), (-3.314516129032258, -7.204545454545455), (-2.314516129032258, -7.204545454545455), (-2.314516129032258, -8.204545454545455),
            # (-3.45967741935484, -9.90909090909091), (-3.45967741935484, -8.90909090909091), (-2.45967741935484, -8.90909090909091), (-2.45967741935484, -9.90909090909091),
            # (-3.314516129032258, -11.086038961038962), (-3.314516129032258, -10.086038961038962), (-2.314516129032258, -10.086038961038962), (-2.314516129032258, -11.086038961038962)
        ]
        # line_points = [(-0.542944,-2.312431),(-0.606383,-7.732428),(1.595494,-7.702555),(1.654554,-2.345058),(4.030055,-2.352500),(4.086305,-7.695000),(6.396930,-7.635000),(6.338805,-2.337500)]
        # uss_points = []
        # vehicle_pose =[(0,0, 0.78539815)]
        ps_list = [
            # [(2.845161,2.086039),(2.845161,4.486039),(-3.154839,4.486039),(-3.154839,2.086039)]
        ]
        spots.extend(ps_list)

    else:
        vehicle_pose = [(-16.914438,1.062630,-3.067164)]#19:35:17.608
        # circle_pose = vehicle_pose[:]
        # vehicle_pose2 = [(3.422745,-4.230900,3.141593)]
        # other_pose = [(22.165739,-51.495441,0)]
        # other_pose2 = [(-1.748000,-3.700000,1.570796)]

        trajectory = [

        ]


        line_points = [

            (-13.804790,-3.009153),(-11.014851,0.403594),(-12.432714,1.711660),(-15.271524,-1.702979),
            (-13.429570,3.959716),(-10.294057,0.763272),(-9.033575,2.197312),(-12.124523,5.292418)
            # [(21.645599, -8.170734), (16.254025, -8.468280), (16.380756, -10.764691), (21.772331, -10.467146)]
                    ]


        uss_points = [
            # (-1.73334, -1.10108), (-1.72421, -1.09684), (-1.68042, -1.0727), (-1.64575, -1.05282), (-1.63664, -1.04856),
            # (-1.59286, -1.02442), (-1.54907, -1.00027), (-1.35102, -1.02256), (-1.34894, -1.03966),
            # (-1.50529, -0.976128), (-1.4615, -0.951985), (-1.40382, -0.970101), (-1.38697, -0.977031),
            # (-1.41772, -0.927842), (-1.46303, -0.851328), (-1.4429, -0.875585), (-1.46223, -0.820888)

            # (3.25453, -1.19964), (8.23979, -1.12452), (3.21687, 1.30008), (8.13667, 1.37421), (3.25453, -1.19964),
            # (8.18568, -1.12534)

            # (-1.73334, -1.10108), (-1.72421, -1.09684), (-1.68042, -1.0727), (-1.64575, -1.05282), (-1.63664, -1.04856),
            # (-1.59286, -1.02442), (-1.54907, -1.00027), (-1.35102, -1.02256), (-1.34894, -1.03966),
            # (-1.50529, -0.976128), (-1.4615, -0.951985), (-1.40382, -0.970101), (-1.38697, -0.977031),
            # (-1.41772, -0.927842), (-1.46303, -0.851328), (-1.4429, -0.875585), (-1.46223, -0.820888)
            ]


        # circle_pose = other_pose2[:]
        # other_pose = [(-0.7617,-0.26264,0.3485),(-0.744781,-0.256493,0.348712),(-0.727865,-0.250343,0.348924),(-0.710951,-0.244186,0.349136),(-0.694036,-0.238029,0.349347),(-0.677124,-0.231865,0.349559),(-0.660215,-0.2257,0.349771),(-0.643303,-0.219528,0.349983),(-0.626394,-0.213356,0.350195),(-0.609489,-0.207176,0.350407),(-0.592583,-0.200996,0.350618),(-0.57568,-0.194809,0.35083),(-0.558775,-0.188622,0.351042),(-0.541874,-0.182434,0.351254),(-0.524977,-0.176239,0.351466),(-0.508078,-0.170036,0.351678),(-0.491179,-0.163841,0.351889),(-0.474283,-0.157631,0.352101),(-0.457388,-0.151421,0.352313),(-0.440495,-0.14521,0.352525),(-0.423603,-0.138992,0.352737),(-0.40671,-0.132774,0.352949),(-0.38982,-0.126549,0.35316),(-0.372932,-0.120323,0.353372),(-0.356045,-0.11409,0.353584),(-0.339159,-0.107849,0.353796),(-0.322275,-0.101616,0.354008),(-0.305391,-0.0953751,0.35422),(-0.288509,-0.0891266,0.354431),(-0.271627,-0.0828781,0.354643),(-0.254749,-0.0766296,0.354855),(-0.237871,-0.0703735,0.355067),(-0.220995,-0.0641174,0.355279),(-0.204121,-0.0578461,0.355491),(-0.187244,-0.0515823,0.355702),(-0.170374,-0.045311,0.355914),(-0.1535,-0.0390396,0.356126),(-0.136633,-0.0327606,0.356338),(-0.119761,-0.0264816,0.35655),(-0.102896,-0.020195,0.356762),(-0.086031,-0.0139084,0.356973),(-0.0691662,-0.00761414,0.357185),(-0.0523033,-0.00131989,0.357397),(-0.0354404,0.00498199,0.357609),(-0.0185814,0.0112839,0.357821),(-0.00172234,0.0175858,0.358033),(0.0151348,0.0238953,0.358244),(0.0319939,0.0302048,0.358456),(0.0488491,0.0365295,0.358668),(0.0657024,0.0428467,0.35888)]
        # other_pose2 = [(4.899688,6.292188,-0.000000),(4.899688,6.392188,0.000000),(4.899688,6.192188,0.000000),(4.899688,6.492188,0.000000),(4.899688,6.092188,0.000000)]
        parkingspace = [(7.577276,11.297705),(7.467841,15.996309),(9.467248,16.042877),(9.576683,11.344274)]
        # parkingspace = []

        # cut_for_internal_wid = 0.15/2
        # parkingspace[0] = move_point(parkingspace[0][0], parkingspace[0][1], angle(parkingspace[0], parkingspace[3]),
        #                              cut_for_internal_wid)
        # parkingspace[1] = move_point(parkingspace[1][0], parkingspace[1][1], angle(parkingspace[1], parkingspace[2]),
        #                              cut_for_internal_wid)
        # parkingspace[2] = move_point(parkingspace[2][0], parkingspace[2][1], angle(parkingspace[2], parkingspace[1]),
        #                              cut_for_internal_wid)
        # parkingspace[3] = move_point(parkingspace[3][0], parkingspace[3][1], angle(parkingspace[3], parkingspace[0]),
        #                              cut_for_internal_wid)
        # spots.append(parkingspace)

        # spots = [       [(-12.620936,2.166985),(-13.243858,7.530855),(-15.528468,7.265536),(-14.905547,1.901667)],
        #                 [(-8.931681,2.664312),(-9.508537,8.032746),(-11.795093,7.787047),(-11.218234,2.418613)]
        #          ]


        ps_list = [

            [(4.87082,-1.79715),(4.05387,-6.19956),(5.95169,-6.40828),(6.71126,-2.10053)],
            [(-1.05047,-5.49095),(3.34555,-5.8159),(3.60251,-3.90401),(-0.823115,-3.54013)]

                 ]
        spots.extend(ps_list)

        # spots = [[(3.919840,-1.065894),(-1.479233,-1.164721),(-1.435311,-3.564308),(3.963763,-3.465481)],
        #          [(3.928122, 1.297615), (-1.470701, 1.186109), (-1.421143, -1.213368), (3.977680, -1.101862)],
        #          [(3.956915,3.604010),(-1.441977,3.495527),(-1.393763,1.096018),(4.005130,1.204502)]]

    # trajectory = trajectory[7: -6]
    # curvefit(trajectory)
#------------------------------------------scale--------------------

    left = 10000.0
    right = -10000.0
    up = 10000
    down = -10000
    all_point = trajectory[:]
    all_point.extend(line_points)
    all_point.extend(other_pose)
    all_point.extend(other_pose2)
    all_point.extend(circle_pose)
    all_point.extend(vehicle_pose)
    all_point.extend(vehicle_pose2)
    all_point.extend(uss_points)
    all_point.extend(dynamic_poses)
    for ss in spots_frame:
        if len(ss) == 0:
            continue
        ss = np.array(ss)
        all_point.extend(ss.reshape([-1, 2]))
        print("spots_frame size", len(ss))

    spots_points = np.array(spots)
    spots_points = spots_points.reshape(-1, 2)
    all_point.extend(spots_points)
    for p in all_point:
        if p[0] < left:
            left = p[0]
        if p[0] > right:
            right = p[0]
        if p[1] < up:
            up = p[1]
        if p[1] > down:
            down = p[1]
    x_range = right - left
    y_range = down - up
    ref_origin = [left + x_range/2, up + y_range/2]
    print("x_range:", x_range, " y_range:", y_range)
    max_size = x_range if x_range > y_range else y_range
    max_size += 4
    # max_size += 8
    # max_size = 10
    if len(circle_pose) > 0:
        if max_size < 10:
            max_size = 10
    resolution_ = (max_size + 1) / map_size

    for spot in spots:
        tf_local(spot, ref_origin)
    tf_local(uss_points, ref_origin)
    global_poses = dynamic_poses[:]
    tf_local(dynamic_poses, ref_origin)
    tf_local(trajectory, ref_origin)
    tf_local(other_pose, ref_origin)
    tf_local(circle_pose, ref_origin)
    tf_local(vehicle_pose, ref_origin)
    tf_local(other_pose2, ref_origin)
    tf_local(vehicle_pose2, ref_origin)
    tf_local(line_points, ref_origin)

    for one_frame in spots_frame:
        for st in one_frame:
            tf_local(st, ref_origin)

# ------------------------------------------draw meta-----------------
    node = ClientNode(resolution_)
    traj_index = 0
    pose_index = 0
    counter = 0
    save_video = True
    # save_video = False
    finish_save_video = False
    video = cv2.VideoWriter("/home/xhpan/Desktop/simulate.avi", cv2.VideoWriter_fourcc(*'MP42'), 20, (map_size, map_size), True)
    dynamic_pose_finish = False
    Trajectory_pose_finish = False
    while True:
        counter += 1
        node.debug_mat_ = np.ones((map_size, map_size, 3)) * 255
        cv2.line(node.debug_mat_, (node.index_x(-10), node.index_y(0)), (node.index_x(10), node.index_y(0)), (0, 0, 0), thickness=1)
        cv2.line(node.debug_mat_, (node.index_x(0), node.index_y(-10)), (node.index_x(0), node.index_y(10)), (0, 0, 0), thickness=1)

        debug_g_pose = []
        for global_item in global_poses:
            debug_g_pose.append(tuple(global_item))
        # print("debug_g_pose", debug_g_pose)
        # print("steering_frame", steering_frame)

        # node.draw_radius(circle_pose, (0, 0, 0), 4.19)
        # node.draw_radius(circle_pose, (0, 0, 0), 4.8)
        node.draw_spots(spots)
       
        # node.draw_pots(dynamic_poses, (255, 0, 0))
        node.draw_trajectory(trajectory, (0, 0, 0))
       
        # node.draw_pose(other_pose, (0, 0, 255))
        # node.draw_vehicle(vehicle_pose, (0, 0, 0))
        # node.draw_pose(other_pose2, (0, 255, 0))
        # node.draw_pots(uss_points, (0, 0, 255))
        # node.draw_vehicle(vehicle_pose2, (0, 255, 0))
        # node.draw_lines(line_points)

        if len(trajectory) > 0:
            curt_pose = [trajectory[traj_index]]
            if(counter % 5 == 0):
                traj_index += 1
                if traj_index >= len(trajectory):
                    traj_index = 0
                    Trajectory_pose_finish = True
            # node.draw_vehicle(curt_pose, (0, 0, 0))
            if len(dynamic_poses) == 1:
                if traj_index == len(trajectory) - 1:
                    video.release()
                    finish_save_video = True

        if len(dynamic_poses) > 0:
            pose_label = "global pose: {:.5f}, {:.5f}, {:.5f}".format(global_poses[pose_index][0], global_poses[pose_index][1], global_poses[pose_index][2])
            steering_label = "steering: {}".format(steering_frame[pose_index])
            node.debug_mat_ = cv2.putText(node.debug_mat_, pose_label, (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                          1, (0, 0, 0), 1, cv2.LINE_AA)
            node.debug_mat_ = cv2.putText(node.debug_mat_, steering_label, (50, 100), cv2.FONT_HERSHEY_SIMPLEX,
                                          1, (0, 0, 0), 1, cv2.LINE_AA)
            real_pose = [dynamic_poses[pose_index]]
            real_spot = []
            if pose_index < len(spots_frame):
                real_spot = spots_frame[pose_index]
            pose_index += 1
            if pose_index >= len(dynamic_poses):
                pose_index = 0
            node.draw_vehicle(real_pose, (0, 0, 255))
            if len(real_spot) > 0:
                node.draw_spots(real_spot)
        if save_video and not finish_save_video:
            video.write(node.debug_mat_.astype("uint8"))
            if pose_index > 1 and pose_index == len(dynamic_poses) -1:
                dynamic_pose_finish = True
            if dynamic_pose_finish and Trajectory_pose_finish:
                video.release()
                finish_save_video = True
        cv2.imshow("planning_pose", node.debug_mat_)
        cv2.waitKey(10)

    # node.show()