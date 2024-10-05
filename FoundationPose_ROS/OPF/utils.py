import numpy as np
import pybullet as pb
import cv2


def bhattacharyya(mean1, cov1, mean2, cov2):
    mean1 = np.array(mean1)
    mean2 = np.array(mean2)
    cov = (1/2)*(cov1+cov2)
    t1 = (1/8)*np.sqrt((mean1-mean2)@np.linalg.inv(cov)@(mean1-mean2).T)
    t2 = (1/2)*np.log(np.linalg.det(cov)/np.sqrt(np.linalg.det(cov1)*np.linalg.det(cov2)))
    return t1 + t2


def setupCamera(x=0, y=0, z=0.49):
    camPos = [x, y, z]
    tgtPos = [x, y, 0]
    camUpVec = [1, 0, 0]
    viewMatrix = pb.computeViewMatrix(
        cameraEyePosition=camPos,
        cameraTargetPosition=tgtPos,
        cameraUpVector=camUpVec
    )
    projectionMatrix = pb.computeProjectionMatrixFOV(
        fov=80.0,
        aspect=1.0,
        nearVal=0.05,
        farVal=2
    )
    print(">> ProjectionMatrix:", projectionMatrix)
    print(">> viewMatrix:", viewMatrix)

    return viewMatrix, projectionMatrix


def getCamera(img_width, img_height, viewMatrix, projectionMatrix):
    imgs = pb.getCameraImage(
        width=img_width, height=img_height,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix,
        renderer=pb.ER_TINY_RENDERER
    )
    return imgs


def getCoord(img_width, img_height, point, depth_buffer, viewMatrix, projectionMatrix):
    u, v = point[0], point[1]
    x_ndc = (2.0 * u / img_width) - 1.0
    y_ndc = -((2.0 * v / img_height) - 1.0)
    depth_value = depth_buffer[v, u]
    depth_value = 0.8
    point_camera_homogeneous = np.dot(np.linalg.inv(projectionMatrix), [x_ndc, y_ndc, depth_value, 1.0])
    point_world_homogeneous = np.dot(np.linalg.inv(viewMatrix), point_camera_homogeneous)
    point_world = point_world_homogeneous[:3] / point_world_homogeneous[3]
    return point_world


def detectOcclusion(seg, num_obj_original):
    return len(np.unique(seg)) == num_obj_original + 1


def findCenter(img_width, img_height, seg, nums):
    centers = []
    img = np.zeros((img_width, img_height, 3))
    img[:,:,0] = seg
    img[:,:,1] = seg
    img[:,:,2] = seg
    normalizedImg = np.zeros((img_width,img_height,3))
    normalizedImg = cv2.normalize(img, normalizedImg, 0, 255, cv2.NORM_MINMAX)
    for i in range(nums):
        seg_ = seg.copy()
        seg_[seg == (i+2)] = 1
        seg_[seg != (i+2)] = 0
        M = cv2.moments(np.float32(seg_))
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centers.append([cx,cy])
            cv2.circle(normalizedImg, (cx, cy), 1, (0, 0, 255), 1)
        else:
            centers.append(None)
    cv2.imwrite('centers.png', normalizedImg)
    return centers


def world2Camera(projectionMatrix, viewMatrix, coord, scale=10.2):
    proj_matrix = np.asarray(projectionMatrix).reshape([4, 4])
    view_matrix = np.asarray(viewMatrix).reshape([4, 4], order="F")
    # print(proj_matrix, '\n', view_matrix)
    camera_clip_space = np.dot(view_matrix, np.append(coord, 1))

    normalized_device_coord = camera_clip_space

    image_coords = np.dot(proj_matrix, normalized_device_coord)
    # print(image_coords)
    image_coords = image_coords[:2] / image_coords[3]

    pixel_x = 122 + image_coords[0] * scale
    pixel_y = 122 - image_coords[1] * scale

    return np.array([pixel_x, pixel_y]).astype(np.uint8)
