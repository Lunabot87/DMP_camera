import numpy as np
import cv2 as cv


def pretty_depth(depth):
    """Converts depth into a 'nicer' format for display

    This is abstracted to allow for experimentation with normalization

    Args:
        depth: A numpy array with 2 bytes per pixel

    Returns:
        A numpy array that has been processed whos datatype is unspecified
    """
    np.clip(depth, 0, 2**11 - 1, depth)
    depth >>= 3
    depth = depth.astype(np.uint8)
    return depth


""" Note incoming pixels are linear in parallax shift, with dark
    being nearer (but any shadows or too close yields full-white as
    error value)
"""
def full_depth_cv (depth):
    # take reciprocal to get linear depth information rather than parallax
    # this means further is darker, decreasing pixel values
    # incoming min pixel is about 250, so numerator set to 16M
    import cv
    depth = depth.astype (np.uint16)

    # depth = 15000000 / depth   # reciprocal

    depth = depth * 32  # scaled


    # map 1024 +/- 33 to larger range
    #depth = np.clip (depth, 1024-33, 1024+33)
    
    #depth = (depth - 1024) * 1000 + 0x8000


    image = cv.CreateImageHeader ((depth.shape[1], depth.shape[0]),
                                  cv.IPL_DEPTH_16U,
                                  1)
    cv.SetData (image, depth.tostring(), depth.dtype.itemsize * depth.shape [1])
    return image


def raw_depth_cv (depth):
    import cv
    depth = depth.astype (np.uint16)
    image = cv.CreateImageHeader ((depth.shape[1], depth.shape[0]), cv.IPL_DEPTH_16U, 1)
    cv.SetData (image, depth.tostring(), depth.dtype.itemsize * depth.shape [1])
    return image



def pretty_depth_cv(depth):
    """Converts depth into a 'nicer' format for display

    This is abstracted to allow for experimentation with normalization

    Args:
        depth: A numpy array with 2 bytes per pixel

    Returns:
        An opencv image who's datatype is unspecified
    """
    import cv
    depth = pretty_depth(depth)
    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 1)
    cv.SetData(image, depth.tostring(),
               depth.dtype.itemsize * depth.shape[1])
    return image


def video_cv(video):
    """Converts video into a BGR format for opencv

    This is abstracted out to allow for experimentation

    Args:
        video: A numpy array with 1 byte per pixel, 3 channels RGB

    Returns:
        An opencv image who's datatype is 1 byte, 3 channel BGR
    """
    import cv
    video = video[:, :, ::-1]  # RGB -> BGR
    image = cv.CreateImageHeader((video.shape[1], video.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 3)
    cv.SetData(image, video.tostring(),
               video.dtype.itemsize * 3 * video.shape[1])
    return image