import math
import json

class VisionPostProcessing:
    """Class VisionPostprocessing enables one to extract the spatial
    properties of the found objects, i.e. mutual location of the
    goal posts and another parts of the goal to filter out the
    inconsistent measurements and extract additional information
    from the scene, particularly to improve the one goal post case
    handling.
    """
    def __init__(self):
        pass

    def process(self, cameraDataRaw, posts, support):
        posts_num   = len (cameraDataRaw [posts])
        support_num = len (cameraDataRaw [support])

        left_post  = []
        right_post = []

        if (posts_num == 2):
            post1 = cameraDataRaw [posts] [0]
            post2 = cameraDataRaw [posts] [1]

            if (post1.x () < post2.x ()):
                left_post  = [post1]
                right_post = [post2]

            else:
                left_post  = [post2]
                right_post = [post1]

            cameraDataRaw.update ({"left_"  + posts  : left_post})
            cameraDataRaw.update ({"right_" + posts : right_post})

        elif (posts_num == 1):
            post = cameraDataRaw [posts] [0]

            if (support_num == 1):
                support = cameraDataRaw [support] [0]

                if (post.x() > support.x()):
                    cameraDataRaw.update ({"left_" + posts : []})
                    cameraDataRaw.update ({"right_" + posts : [post]})
                    print ("right post")

                else:
                    cameraDataRaw.update ({"left_" + posts : [post]})
                    cameraDataRaw.update ({"right_" + posts : []})
                    print ("left post")

            if (support_num == 0):
                cameraDataRaw.update ({"left_" + posts : [post]})
                cameraDataRaw.update ({"right_" + posts : []})

        cameraDataRaw.update ({"left_"  + posts : left_post})
        cameraDataRaw.update ({"right_" + posts : right_post})

        return cameraDataRaw