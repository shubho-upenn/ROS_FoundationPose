import numpy as np
from scipy.stats import norm
import cv2
import OPF.utils as utils
import transforms3d as t3d
import pybullet as pb
from math import pi


class OPF_3d:
    def __init__(self, num_particles=5000, name=None, objid=None):
        self.name = name
        self.objid = objid
        self.indexes = None
        self.indexes1 = None
        self.num_particles = num_particles
        self.particles = np.empty((self.num_particles, 3))
        self.particles1 = np.empty((self.num_particles, 3))
        self.trajectory = [np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0])]
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.weights1 = np.ones(self.num_particles) / self.num_particles
        self.R = 0.1
        self.R1 = 0.1
        self.num_plot_particle = 100
        self.particles = np.random.uniform(-1, 1, size=(self.num_particles, 3))
        self.particles1 = np.random.uniform(-3, 3, size=(self.num_particles, 3))
        self.curr_pos = np.average(self.particles[:], weights=self.weights, axis=0)
        self.curr_pos1 = np.average(self.particles1[:], weights=self.weights1, axis=0)
        self.cov = np.cov(self.particles[:, :2].T, aweights=self.weights)

        # occluder to obj transform when occlusion began
        self.occluder_obj_initial_transform = np.empty(0)

        self.dyn_thresh = 1.0 # strong obj static tracking
        # self.dyn_thresh = 0.02 # general tracking
        # self.dyn_thresh = 0.01  # sugar dropping
        self.hidden_factor = 1
        # whether the object is static or moving. Default is static
        self.static = 0
        self.cov_list = []

        self.gt_pos = [np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0])]
        self.future_pred = []
        self.hidden_count = 0

    def predict(self):
        std = np.array([0.01, 0.1])
        vec = np.array(self.trajectory[-1] - self.trajectory[-2]) / 5
        # for both translation and rotation
        for i in range(3):
            self.particles[:, i] += vec[i] + np.random.randn(self.num_particles) * std[1]
            self.particles1[:, i] += vec[i] + np.random.randn(self.num_particles) * std[1]

    def update(self, measurement, hidden=0):
        self.weights.fill(1.)
        distance = np.power((self.particles[:, 0] - measurement[0]) ** 2 +
                            (self.particles[:, 1] - measurement[1]) ** 2 +
                            (self.particles[:, 2] - measurement[2]) ** 2, 0.5)
        distance1 = np.power((self.particles1[:, 0] - measurement[3]) ** 2 +
                             (self.particles1[:, 1] - measurement[4]) ** 2 +
                             (self.particles1[:, 2] - measurement[5]) ** 2, 0.5)
        self.weights *= norm(distance, self.R).pdf(0)
        self.weights += 1.e-300
        self.weights /= sum(self.weights)
        self.weights1 *= norm(distance1, self.R1).pdf(0)
        self.weights1 += 1.e-300
        self.weights1 /= sum(self.weights1)

        self.curr_pos = np.average(self.particles[:], weights=self.weights, axis=0)
        self.curr_pos1 = np.average(self.particles1[:], weights=self.weights1, axis=0)
        self.cov = np.cov(self.particles[:, :2].T, aweights=self.weights)
        # print(">> in update, curr_pos:", self.curr_pos)
        # self.trajectory.append(self.curr_pos)
        self.trajectory.append(np.hstack((self.curr_pos, self.curr_pos1)))

        if not hidden:
            self.hidden_factor = 1
            self.static = 0
            self.hidden_count = 0

    '''
    Get virtual measurement based on 'occluder module' and 'dynamic module', then perform standard update
    '''

    def OP_update(self, occluders, measurement=np.empty(0)):

        if measurement.any():
            self.update(measurement)
            self.occluder_obj_initial_transform = np.empty(0)
        else:
            # first go to 'dynamic module', checking if the object was moving before occluded
            # TODO: take the shape of object into account to decide moving threshold
            if np.linalg.norm(self.trajectory[-10][:3] - self.trajectory[-1][:3]) > self.dyn_thresh and not self.static:
                # mark the object as moving
                self.static = 0
                # model the movement when first occluded
                if self.hidden_factor == 1:
                    print("In OP_update, making future predictions")
                    # empty the future predictions
                    self.future_pred = []
                    self.hidden_count = 0

                    time_steps = np.arange(50)
                    x = np.array(self.trajectory[-50:])[:, 0]
                    y = np.array(self.trajectory[-50:])[:, 1]
                    z = np.array(self.trajectory[-50:])[:, 2]

                    deg = 1
                    new_series_x = np.polynomial.polynomial.Polynomial.fit(time_steps, x, deg)
                    new_series_y = np.polynomial.polynomial.Polynomial.fit(time_steps, y, deg)
                    new_series_z = np.polynomial.polynomial.Polynomial.fit(time_steps, z, deg)
                    x_coef = new_series_x.convert().coef
                    y_coef = new_series_y.convert().coef
                    z_coef = new_series_z.convert().coef

                    euler_angle = np.array(self.trajectory[-50:])[:, 3:]
                    axangle = pb.getAxisAngleFromQuaternion(pb.getQuaternionFromEuler(euler_angle[0]))
                    axis = [axangle[0]]
                    angle = [axangle[1]]
                    prev_axis = axangle[0][0]
                    for eangle in euler_angle[1:]:
                        axangle = pb.getAxisAngleFromQuaternion(pb.getQuaternionFromEuler(eangle))
                        if round(axangle[0][0] - prev_axis) != 0 or round(axangle[0][0] - axis[-1][0]) != 0:
                            axis.append(-np.array(axangle[0]))
                            angle.append(4 * pi - axangle[1])
                        else:
                            axis.append(axangle[0])
                            angle.append(axangle[1])
                        prev_axis = axangle[0][0]
                        # print(axis[-1], angle[-1])
                    axis = np.array(axis)
                    angle = np.array(angle)
                    mean_axis = np.mean(axis, axis=0)
                    mean_angle = np.mean(angle[1:]-angle[:-1])

                    for i in range(51, 1001):
                        px = x_coef[0] + x_coef[1] * i
                        py = y_coef[0] + y_coef[1] * i
                        pz = z_coef[0] + z_coef[1] * i
                        wx, wy, wz = t3d.euler.axangle2euler(mean_axis, mean_angle)
                        self.future_pred.append(np.array([px, py, pz, wx, wy, wz]))

                # Update the hidden_factor according to the moving velocity
                hidden_scale = 2 * np.linalg.norm(self.future_pred[0] - self.future_pred[1])
                self.hidden_factor += hidden_scale
                # print(self.hidden_factor)

                # provide the interpolated point as virtual measurement
                # start from 1 instead of 0 because 0 is stored as velocity
                self.hidden_count += 1

                # self.update(self.future_pred[self.hidden_count], hidden=1)
                # print(">>>", self.future_pred)
                self.update(self.future_pred[self.hidden_count], hidden=1)

            else:
                # mark as static object
                self.static = 1
                dists = []
                for occ in occluders:
                    dist = utils.bhattacharyya(occ.curr_pos[:2], occ.cov, self.trajectory[-1][:2], self.cov)
                    dists.append(dist)
                occluder = occluders[np.argmin(np.array(dists))]
                print(">>> Occluder: {}, dist: {}".format(occluder.name, dists[np.argmin(np.array(dists))]))

                # Use the occluder's position as virtual measurement to update
                # vir_meas = np.hstack((occluder.curr_pos, self.curr_pos1))

                occluder_pose = np.eye(4)
                occluder_pose[:3,3] = occluder.curr_pos
                occluder_pose[:3,:3] = t3d.euler.euler2mat(*occluder.curr_pos1)
                
                if not self.occluder_obj_initial_transform.any():
                    cur_pose = np.eye(4)
                    cur_pose[:3,3] = self.curr_pos
                    cur_pose[:3,:3] = t3d.euler.euler2mat(*self.curr_pos1)

                    self.occluder_obj_initial_transform = np.linalg.inv(occluder_pose) @ cur_pose

                vir_pose = occluder_pose @ self.occluder_obj_initial_transform
                vir_meas = np.concatenate((vir_pose[:3,3], t3d.euler.mat2euler(vir_pose[:3,:3])))

                self.update(vir_meas, hidden=1)

                # Update the hidden_factor (scale the covariance matrix)
                self.hidden_factor *= 1.013

    def systematic_resample(self):
        N = self.num_particles
        positions = (np.arange(N) + np.random.random()) / N

        self.indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N and j < N:
            if positions[i] < cumulative_sum[j]:
                self.indexes[i] = j
                i += 1
            else:
                j += 1

        self.indexes1 = np.zeros(N, 'i')
        cumulative_sum1 = np.cumsum(self.weights1)
        i, j = 0, 0
        while i < N and j < N:
            if positions[i] < cumulative_sum1[j]:
                self.indexes1[i] = j
                i += 1
            else:
                j += 1

    def resample_from_index(self):
        self.particles[:] = self.particles[self.indexes]
        self.weights[:] = self.weights[self.indexes]
        self.weights /= np.sum(self.weights)
        self.particles1[:] = self.particles1[self.indexes1]
        self.weights1[:] = self.weights1[self.indexes1]
        self.weights1 /= np.sum(self.weights1)

    def plot(self, image, projectionMatrix, viewMatrix, plot_gt=False, scale=12):
        if plot_gt and len(self.gt_pos) > 3:
            for i in range(2, len(self.gt_pos) - 1):
                camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.gt_pos[i][:3], scale=scale)
                camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, self.gt_pos[i + 1][:3], scale=scale)
                cv2.line(image, camera_coord, camera_coord1, (255, 0, 255), 1)

        # calculate covariance matrix
        vals, vecs = np.linalg.eigh(5.9915 * self.cov)
        if vals[0] > vals[1]:
            alpha = np.arctan2(vecs[0, 1], vecs[0, 0])
        else:
            alpha = np.arctan2(vecs[1, 1], vecs[1, 0])
        mean = np.average(self.particles, weights=self.weights, axis=0)
        var = np.average((self.particles - mean) ** 2, weights=self.weights, axis=0)
        axs = int(np.exp(self.hidden_factor) * np.sqrt(var[0]) * 2), int(self.hidden_factor * np.sqrt(var[1]) * 2)
        # axs = int(np.sqrt(var[0]) ** self.hidden_factor * 200), int(self.hidden_factor * np.sqrt(var[1]) * 200)
        angle = int(180. * alpha / np.pi)
        if plot_gt:
            self.cov_list.append(np.exp(self.hidden_factor) * np.sqrt(var[0]) * 2)
        # # plot pred (blue or green)
        if self.hidden_factor == 1:
            if plot_gt:
                for i in range(5, len(self.trajectory) - 1):
                    camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i][:3], scale=scale)
                    camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i + 1][:3],
                                                       scale=scale)
                    # cv2.line(image, (trans_coord(self.trajectory[i])).astype(int),
                    #          (trans_coord(self.trajectory[i + 1])).astype(int), (0, 255, 255), 1)
                    cv2.line(image, camera_coord, camera_coord1, (0, 255, 255), 1)
            mean_coord = utils.world2Camera(projectionMatrix, viewMatrix, mean[:3], scale=scale)
            cv2.ellipse(image, tuple(mean_coord[:2]),
                        axs, angle, 0, 360, (0, 255, 255), 1)
            # cv2.ellipse(image, tuple((int(trans_coord(mean[0])), int(trans_coord(mean[1])))),
            #             axs, angle, 0, 360, (0, 255, 255), 1)
        else:
            if plot_gt:
                for i in range(5, len(self.trajectory) - 1):
                    camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i][:3], scale=scale)
                    camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i + 1][:3],
                                                       scale=scale)
                    #     cv2.line(image, (trans_coord(self.trajectory[i])).astype(int),
                    #              (trans_coord(self.trajectory[i + 1])).astype(int), (0, 255, 0), 1)
                    cv2.line(image, camera_coord, camera_coord1, (0, 255, 0), 1)
            mean_coord = utils.world2Camera(projectionMatrix, viewMatrix, mean[:3], scale=scale)
            cv2.ellipse(image, tuple(mean_coord[:2]),
                        axs, angle, 0, 360, (0, 255, 0), 1)
            # cv2.ellipse(image, tuple((int(trans_coord(mean[0])), int(trans_coord(mean[1])))),
            #             axs, angle, 0, 360, (0, 255, 0), 1)

        # Plot the orientation
        orn = self.curr_pos1
        quat = pb.getQuaternionFromEuler(orn)
        rotation_matrix = np.array(pb.getMatrixFromQuaternion(quat)).reshape((3, 3))
        axis_x = np.dot(rotation_matrix, np.array([1, 0, 0]))
        axis_y = np.dot(rotation_matrix, np.array([0, 1, 0]))
        axis_z = np.dot(rotation_matrix, np.array([0, 0, 1]))
        axis_length = 30

        # mean_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.gt_pos[-1][:3], scale=scale)
        # (0, 0, 255): BLUE !!!!
        cv2.line(image, mean_coord, (mean_coord[0] + int(axis_length * axis_x[1]),
                                     mean_coord[1] + int(axis_length * axis_x[0])), (255, 0, 0), 2)
        cv2.line(image, mean_coord, (mean_coord[0] + int(axis_length * axis_y[1]),
                                     mean_coord[1] + int(axis_length * axis_y[0])), (0, 255, 0), 2)
        cv2.line(image, mean_coord, (mean_coord[0] + int(axis_length * axis_z[1]),
                                     mean_coord[1] + int(axis_length * axis_z[0])), (0, 0, 255), 2)

        cv2.imwrite('temp_frame_cv2.png', image)


class PF_3d:
    def __init__(self, num_particles=1000, name=None, objid=None):
        self.name = name
        self.objid = objid
        self.indexes = None
        self.indexes1 = None
        self.num_particles = num_particles
        self.particles = np.empty((self.num_particles, 3))
        self.particles1 = np.empty((self.num_particles, 3))
        self.trajectory = [np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0])]
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.weights1 = np.ones(self.num_particles) / self.num_particles
        self.R = 0.1
        self.R1 = 0.1
        self.num_plot_particle = 100
        self.particles = np.random.uniform(-1, 1, size=(self.num_particles, 3))
        self.particles1 = np.random.uniform(-1, 1, size=(self.num_particles, 3))
        self.curr_pos = np.average(self.particles[:], weights=self.weights, axis=0)
        self.curr_pos1 = np.average(self.particles1[:], weights=self.weights1, axis=0)
        self.cov = np.cov(self.particles.T, aweights=self.weights)

        self.dyn_thresh = 0.03
        self.hidden_factor = 1
        # whether the object is static or moving. Default is static
        self.static = 0
        self.cov_list = []

        self.gt_pos = [np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0])]

    def predict(self):
        std = np.array([0.01, 0.1])
        vec = np.array(self.trajectory[-1] - self.trajectory[-2]) / 5
        for i in range(3):
            self.particles[:, i] += vec[i] + np.random.randn(self.num_particles) * std[1]
            self.particles1[:, i] += vec[i] + np.random.randn(self.num_particles) * std[1]

    def update(self, measurement, hidden=0):
        self.weights.fill(1.)
        distance = np.power((self.particles[:, 0] - measurement[0]) ** 2 +
                            (self.particles[:, 1] - measurement[1]) ** 2 +
                            (self.particles[:, 2] - measurement[2]) ** 2, 0.5)
        distance1 = np.power((self.particles1[:, 0] - measurement[3]) ** 2 +
                             (self.particles1[:, 1] - measurement[4]) ** 2 +
                             (self.particles1[:, 2] - measurement[5]) ** 2, 0.5)
        self.weights *= norm(distance, self.R).pdf(0)
        self.weights += 1.e-300
        self.weights /= sum(self.weights)
        self.weights1 *= norm(distance1, self.R1).pdf(0)
        self.weights1 += 1.e-300
        self.weights1 /= sum(self.weights1)

        self.curr_pos = np.average(self.particles[:], weights=self.weights, axis=0)
        self.curr_pos1 = np.average(self.particles1[:], weights=self.weights1, axis=0)
        self.cov = np.cov(self.particles.T, aweights=self.weights)
        self.trajectory.append(np.hstack((self.curr_pos, self.curr_pos1)))

        if not hidden:
            self.hidden_factor = 1
            self.static = 0

    def OP_update(self, occluders):
        # maintain the original state
        self.trajectory.append(np.hstack((self.curr_pos, self.curr_pos1)))
        pass

    def systematic_resample(self):
        N = self.num_particles
        positions = (np.arange(N) + np.random.random()) / N

        self.indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N and j < N:
            if positions[i] < cumulative_sum[j]:
                self.indexes[i] = j
                i += 1
            else:
                j += 1

        self.indexes1 = np.zeros(N, 'i')
        cumulative_sum1 = np.cumsum(self.weights1)
        i, j = 0, 0
        while i < N and j < N:
            if positions[i] < cumulative_sum1[j]:
                self.indexes1[i] = j
                i += 1
            else:
                j += 1

    def resample_from_index(self):
        self.particles[:] = self.particles[self.indexes]
        self.weights[:] = self.weights[self.indexes]
        self.weights /= np.sum(self.weights)
        self.particles1[:] = self.particles1[self.indexes1]
        self.weights1[:] = self.weights1[self.indexes1]
        self.weights1 /= np.sum(self.weights1)

    def plot(self, image, projectionMatrix, viewMatrix, plot_gt=False, scale=12, orn_gt=None):
        if plot_gt and len(self.gt_pos) > 3:
            for i in range(2, len(self.gt_pos) - 1):
                camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.gt_pos[i][:3], scale=scale)
                camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, self.gt_pos[i + 1][:3], scale=scale)
                cv2.line(image, camera_coord, camera_coord1, (255, 0, 255), 1)

        mean = np.average(self.particles, weights=self.weights, axis=0)
        var = np.average((self.particles - mean) ** 2, weights=self.weights, axis=0)
        axs = int(np.sqrt(var[0]) * 500), int(np.sqrt(var[1]) * 500)

        # # plot pred (blue or green)
        if self.hidden_factor == 1:
            if plot_gt:
                for i in range(5, len(self.trajectory) - 1):
                    camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i][:3], scale=scale)
                    camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i + 1][:3],
                                                       scale=scale)

                    cv2.line(image, camera_coord, camera_coord1, (0, 255, 255), 1)
            mean_coord = utils.world2Camera(projectionMatrix, viewMatrix, mean[:3], scale=scale)

        else:
            if plot_gt:
                for i in range(5, len(self.trajectory) - 1):
                    camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i][:3], scale=scale)
                    camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, self.trajectory[i + 1][:3],
                                                       scale=scale)
                    cv2.line(image, camera_coord, camera_coord1, (0, 255, 0), 1)
            mean_coord = utils.world2Camera(projectionMatrix, viewMatrix, mean[:3], scale=scale)

        # Plot the orientation
        orn = self.curr_pos1
        quat = pb.getQuaternionFromEuler(orn)
        rotation_matrix = np.array(pb.getMatrixFromQuaternion(quat)).reshape((3, 3))
        axis_x = np.dot(rotation_matrix, np.array([1, 0, 0]))
        axis_y = np.dot(rotation_matrix, np.array([0, 1, 0]))
        axis_z = np.dot(rotation_matrix, np.array([0, 0, 1]))
        axis_length = 30

        cv2.line(image, mean_coord, (mean_coord[0] + int(axis_length * axis_x[1]),
                                     mean_coord[1] + int(axis_length * axis_x[0])), (255, 0, 0), 2)
        cv2.line(image, mean_coord, (mean_coord[0] + int(axis_length * axis_y[1]),
                                     mean_coord[1] + int(axis_length * axis_y[0])), (0, 255, 0), 2)
        cv2.line(image, mean_coord, (mean_coord[0] + int(axis_length * axis_z[1]),
                                     mean_coord[1] + int(axis_length * axis_z[0])), (0, 0, 255), 2)

        cv2.imwrite('temp_frame_cv2.png', image)


class OPF_2d:
    def __init__(self, num_particles=5000, name=None, objid=None):
        self.name = name
        self.objid = objid
        self.indexes = None
        self.num_particles = num_particles
        self.particles = np.empty((self.num_particles, 2))
        self.trajectory = [np.array([0, 0]), np.array([0, 0])]
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.R = 0.05
        self.num_plot_particle = 100
        self.particles = np.random.uniform(-1, 1, size=(self.num_particles, 2))
        self.curr_pos = np.average(self.particles, weights=self.weights, axis=0)
        self.cov = np.cov(self.particles.T, aweights=self.weights)

        self.dyn_thresh = 0.03
        self.hidden_factor = 1
        # whether the object is static or moving. Default is static
        self.static = 0

        self.gt_pos = [np.array([0, 0]), np.array([0, 0])]

    def predict(self):
        std = np.array([0.01, 0.1])
        vec = np.array(self.trajectory[-1] - self.trajectory[-2]) / 5

        self.particles[:, 0] += vec[0] + np.random.randn(self.num_particles) * std[1]
        self.particles[:, 1] += vec[1] + np.random.randn(self.num_particles) * std[1]

    def update(self, measurement, hidden=0):
        self.weights.fill(1.)
        distance = np.power((self.particles[:, 0] - measurement[0]) ** 2 +
                            (self.particles[:, 1] - measurement[1]) ** 2, 0.5)
        self.weights *= norm(distance, self.R).pdf(0)
        self.weights += 1.e-300
        self.weights /= sum(self.weights)

        self.curr_pos = np.average(self.particles, weights=self.weights, axis=0)
        self.cov = np.cov(self.particles.T, aweights=self.weights)
        self.trajectory.append(self.curr_pos)

        if not hidden:
            self.hidden_factor = 1
            self.static = 0

    '''
    Get virtual measurement based on 'occluder module' and 'dynamic module', then perform standard update
    '''

    def OP_update(self, occluders):
        # first go to 'dynamic module', checking if the object was moving before occluded
        # TODO: when in 6DoF, take the shape of object into account to decide moving threshold
        if np.linalg.norm(self.trajectory[-10] - self.trajectory[-1]) > self.dyn_thresh and not self.static:
            # mark the object as moving
            self.static = 0
            # model the movement
            time_steps = np.arange(50)
            x = np.array(self.trajectory[-50:])[:, 0]
            y = np.array(self.trajectory[-50:])[:, 1]
            deg = 1

            new_series_x = np.polynomial.polynomial.Polynomial.fit(time_steps, x, deg)
            new_series_y = np.polynomial.polynomial.Polynomial.fit(time_steps, y, deg)
            x_coef = new_series_x.convert().coef
            y_coef = new_series_y.convert().coef
            px = x_coef[0] + x_coef[1] * 51
            py = y_coef[0] + y_coef[1] * 51
            print(">>> In OP_update, next x and y:", px, py)

            # Update the hidden_factor according to the moving velocity
            hidden_scale = 7 * np.linalg.norm(np.array([x_coef[1], y_coef[1]]))
            self.hidden_factor *= (1 + hidden_scale)

            # provide the interpolated point as virtual measurement
            self.update(np.array([px, py]), hidden=1)

        else:
            # mark as static object
            self.static = 1
            dists = []
            for occ in occluders:
                dist = utils.bhattacharyya(occ.curr_pos, occ.cov, self.trajectory[-1], self.cov)
                dists.append(dist)
            occluder = occluders[np.argmin(np.array(dists))]
            print(">>> Occluder: {}, dist: {}".format(occluder.name, dists[np.argmin(np.array(dists))]))
            # Use the occluder's position as virtual measurement to update
            self.update(occluder.curr_pos, hidden=1)

            # Update the hidden_factor (scale the covariance matrix)
            self.hidden_factor *= 1.02

    def systematic_resample(self):
        N = self.num_particles
        positions = (np.arange(N) + np.random.random()) / N

        self.indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N and j < N:
            if positions[i] < cumulative_sum[j]:
                self.indexes[i] = j
                i += 1
            else:
                j += 1

    def resample_from_index(self):
        self.particles[:] = self.particles[self.indexes]
        self.weights[:] = self.weights[self.indexes]
        self.weights /= np.sum(self.weights)

    def plot(self, image, projectionMatrix, viewMatrix, plot_gt=False):
        # change from world coordinate to image coordinate
        trans_coord = lambda a: 200 * a + 200
        # plot gt (purple)
        if plot_gt and len(self.gt_pos) > 3:
            for i in range(2, len(self.gt_pos) - 1):
                camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, np.append(self.gt_pos[i], 0.08))
                camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix, np.append(self.gt_pos[i + 1], 0.08))
                # cv2.line(image, (trans_coord(self.gt_pos[i])).astype(int),
                #          (trans_coord(self.gt_pos[i + 1])).astype(int),
                #          (255, 0, 255), 1)
                cv2.line(image, camera_coord, camera_coord1, (255, 0, 255), 1)

        # calculate covariance matrix
        vals, vecs = np.linalg.eigh(5.9915 * self.cov)
        if vals[0] > vals[1]:
            alpha = np.arctan2(vecs[0, 1], vecs[0, 0])
        else:
            alpha = np.arctan2(vecs[1, 1], vecs[1, 0])
        mean = np.average(self.particles, weights=self.weights, axis=0)
        var = np.average((self.particles - mean) ** 2, weights=self.weights, axis=0)
        axs = int(self.hidden_factor * np.sqrt(var[0]) * 500), int(self.hidden_factor * np.sqrt(var[1]) * 500)
        angle = int(180. * alpha / np.pi)

        # plot pred (blue or green)
        if self.hidden_factor == 1:
            for i in range(2, len(self.trajectory) - 1):
                camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, np.append(self.trajectory[i], 0.08))
                camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix,
                                                   np.append(self.trajectory[i + 1], 0.08))
                # cv2.line(image, (trans_coord(self.trajectory[i])).astype(int),
                #          (trans_coord(self.trajectory[i + 1])).astype(int), (0, 255, 255), 1)
                cv2.line(image, camera_coord, camera_coord1, (0, 255, 255), 1)
            camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, np.append(mean, 0.08))
            cv2.ellipse(image, tuple(camera_coord),
                        axs, angle, 0, 360, (0, 255, 255), 1)
            # cv2.ellipse(image, tuple((int(trans_coord(mean[0])), int(trans_coord(mean[1])))),
            #             axs, angle, 0, 360, (0, 255, 255), 1)
        else:
            for i in range(2, len(self.trajectory) - 1):
                camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, np.append(self.trajectory[i], 0.08))
                camera_coord1 = utils.world2Camera(projectionMatrix, viewMatrix,
                                                   np.append(self.trajectory[i + 1], 0.08))
                #     cv2.line(image, (trans_coord(self.trajectory[i])).astype(int),
                #              (trans_coord(self.trajectory[i + 1])).astype(int), (0, 255, 0), 1)
                cv2.line(image, camera_coord, camera_coord1, (0, 255, 0), 1)
            camera_coord = utils.world2Camera(projectionMatrix, viewMatrix, np.append(mean, 0.08))
            cv2.ellipse(image, tuple(camera_coord),
                        axs, angle, 0, 360, (0, 255, 0), 1)
            # cv2.ellipse(image, tuple((int(trans_coord(mean[0])), int(trans_coord(mean[1])))),
            #             axs, angle, 0, 360, (0, 255, 0), 1)

        cv2.imwrite('temp_frame_cv2.png', image)
