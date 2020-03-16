import math, time

def compute_leg_ik(target, orientation, model):

    # get model leg sizes (distances between joints)
    a5 = model.sizes['a5']
    b5 = model.sizes['b5']
    c5 = model.sizes['c5']
    a6 = model.sizes['a6']
    a7 = model.sizes['a7']
    a8 = model.sizes['a8']
    a9 = model.sizes['a9']
    a10 = model.sizes['a10']
    b10 = model.sizes['b10']
    c10 = model.sizes['c10']

    if c5 == 1:
        pass
    
    # Normalize orientation quaternion
    orientation = orientation.normalize_vector()
    
    # angle5 is hip_yaw. Turn the leg first
    angle5 = orientation.w 
    cos5 = math.cos(angle5)
    sin5 = math.sin(angle5)
    target.x = target.x * cos5 + (target.y + a5) * sin5
    tmp = target.y
    target.y = (tmp + a5) * cos5 - target.x * sin5
    target.z = target.z

    orientation.x =  orientation.x * cos5 + orientation.y * sin5
    orientation.y = orientation.y * cos5 - orientation.x * sin5
    orientation.z = orientation.z
    
    angles = {}
    solutions = [] 

    # get servos' limits. limitsX for servo X    
    angle6_limits = model.servos[(6,1)]['limits']
    for i in range(len(angle6_limits)):
        angle6_limits[i] = angle6_limits[i] / 180. * math.pi
    angle7_limits = model.servos[(7,1)]['limits']
    for i in range(len(angle7_limits)):
        angle7_limits[i] = angle7_limits[i] / 180. * math.pi 
    angle8_limits = model.servos[(8,1)]['limits']
    for i in range(len(angle8_limits)):
        angle8_limits[i] = angle8_limits[i] / 180. * math.pi 
    angle9_limits = model.servos[(9,1)]['limits']
    for i in range(len(angle9_limits)):
        angle9_limits[i] = angle9_limits[i] / 180. * math.pi 
    angle10_limits = model.servos[(10,1)]['limits']
    for i in range(len(angle10_limits)):
        angle10_limits[i] = angle10_limits[i] / 180. * math.pi 

    # calculate angle6 (hip_roll) with numerical method
    # first attempt to find angle6
    # divide limits segment into 10 parts to get 11 probable points
    calculation_step = (angle6_limits[1] - angle6_limits[0]) / 10
    node_points = []
    # calculate the value of F function for each probable point
    for i in range(11): 
        angle6 = angle6_limits[0] + i * calculation_step
        cos = math.cos(angle6)
        sin = math.sin(angle6)
        tmp1 = orientation.y * cos + orientation.z * sin
        tmp2 = orientation.z * cos - orientation.y * sin
        # F functon looks like
        node_points.append(((target.y + b5) * cos + target.z * sin - c10) * (tmp1**2 -
            tmp2**2 - orientation.x**2) - a10 - b10 * tmp1 / math.sqrt(tmp2**2 + orientation.x**2))

    # find all segments with zero solutions for the equation F(angle6)=0
    solution_segments = []
    for i in range(10):
        if (node_points[i] > 0 and node_points[i+1] < 0) or \
        (node_points[i] < 0 and node_points[i+1] > 0): 
            solution_segments.append(i)
    # if there is no zero solution segments
    if len(solution_segments) == 0:
        k = 0
        for i in range(11):
            if math.fabs(node_points[i]) < math.fabs(node_points[k]): 
                k = i
        # if function decreases monotonically, add the last segment
        # else add the segment with min point 
        if k == 10: 
            solution_segments.append(9)
        else:
            if math.fabs(node_points[k-1]) < math.fabs(node_points[k+1]): 
                solution_segments.append(k-1)
            else: solution_segments.append(k)
    
    # look into each solution_segment in the same way to find more precised solution
    angle6_solutions = []

    for segment in solution_segments:
        # define the boundaries of the segment
        bound1 =  angle6_limits[0] + segment * calculation_step
        bound2 = bound1 + calculation_step
        step = calculation_step
        while step > 0.00025:
            step /= 10
            node_points = []
            for i in range(11):
                angle6 = bound1 + i * step
                cos = math.cos(angle6)
                sin = math.sin(angle6)
                tmp1 = orientation.y * cos + orientation.z * sin
                tmp2 = orientation.z * cos - orientation.y * sin
                
                node_points.append(((target.y + b5) * cos + target.z * sin - c10) * (tmp1**2 - 
                    tmp2**2 - orientation.x**2) - a10 - b10 * tmp1 / math.sqrt(tmp2**2 + orientation.x**2))
            k = 0
                
            for i in range(11):
                if math.fabs(node_points[i]) < math.fabs(node_points[k]): 
                    k = i
            # k is one bound point of new solution segment and k2 is another.
            if k == 0: 
                k2 = 1
            elif k == 10: 
                k2 = 9
            else:
                if math.fabs(node_points[k-1]) < math.fabs(node_points[k+1]):
                    k2 = k - 1
                else: 
                    k2 = k + 1
            # calculate more precised value of angle6
            angle6 = bound1 + k * step
            # narrow the boundaries
            if k > k2:
                bound1 += k2 * step
                bound2 = bound1 + step
            else:
                bound1 += k * step
                bound2 = bound1 + step
        # add calculated solution
        angle6_solutions.append(angle6)
    # calculate angle10 (ankle_roll) for each angle6 as it depends on angle6
    angle10_solutions = []
    k = 0
    for i in range(len(angle6_solutions)):
        tan6 = math.tan(angle6_solutions[i-k])
        alpha10 = math.atan((-orientation.y - orientation.z * tan6) /  \
            math.sqrt((orientation.z - orientation.y * tan6)**2 + orientation.x**2 * (1 + tan6**2)))

        # check if got angle10 fits the limits. If not, angle6 solution is not a solution 
        if angle10_limits[0] < alpha10 and alpha10 < angle10_limits[1]: 
            angle10_solutions.append(alpha10)
        else:
            angle6_solutions.pop(i-k)
            k += 1
    # now time to calculate angle7 (hip_pitch), angle8 (knee), angle9 (ankle_pitch)
    k = 0
    for i in range(len(angle6_solutions)):
        
        angle9_solutions = []
        angle8_solutions = []
        angle7_solutions = []

        cos6 = math.cos(angle6_solutions[i-k])
        sin6 = math.sin(angle6_solutions[i-k])
        # angle987 is sum of angle7, angle8, angle9
        angle987 = math.atan(-orientation.x / (orientation.z * cos6 - orientation.y * sin6))
        sin987 = math.sin(angle987)
        cos987 = math.cos(angle987)
        # calculate angle9
        k1 = a6 * sin987 + target.x * cos987 + (target.z * cos6 - (target.y + b5) * sin6) * sin987
        k2 = a9 + a6 * cos987 + (target.z * cos6 - (target.y + b5) * sin6) * cos987 - \
            target.x * sin987 + b10 / math.cos(angle10_solutions[i-k]) + ((target.y + b5) * cos6 + \
            target.z * sin6 - c10) * math.tan(angle10_solutions[i-k])
        m = (k1**2 + k2**2 + a8**2 - a7**2) / (a8 * 2)
        temp1 = k1**2 * m**2 - (k1**2 + k2**2) * (m**2 - k2**2)
        if temp1 >= 0:
            temp2 = (-k1 * m + math.sqrt(temp1)) / (k1**2 + k2**2)
            temp3 = (-k1 * m - math.sqrt(temp1)) / (k1**2 + k2**2)
            if math.fabs(temp2) <= 1 and math.fabs(temp3) <= 1:
                angle9_solutions.append(math.asin(temp2))
                angle9_solutions.append(math.asin(temp3))
            else:
                angle6_solutions.pop(i-k)
                angle10_solutions.pop(i-k)
                k += 1
                continue
        else:
            angle6_solutions.pop(i-k)
            angle10_solutions.pop(i-k)
            k += 1
            continue
        # calculate angle8
        angle8_solutions.append(
            math.atan((k1 + a8 * math.sin(angle9_solutions[0])) / \
            (k2 + a8 * math.cos(angle9_solutions[0]))) - angle9_solutions[0])
        angle8_solutions.append(
            math.atan((k1 + a8 * math.sin(angle9_solutions[1])) / \
            (k2 + a8 * math.cos(angle9_solutions[1]))) - angle9_solutions[1])
        # calculate angle7
        angle7_solutions.append(angle9_solutions[0] + angle8_solutions[0] - angle987)
        angle7_solutions.append(angle9_solutions[1] + angle8_solutions[1] - angle987)
        # check each solution to fit the limits
        temp71 = angle7_solutions[0] < angle7_limits[0] or angle7_solutions[0] > angle7_limits[1]
        temp72 = angle7_solutions[1] < angle7_limits[0] or angle7_solutions[1] > angle7_limits[1]
        temp81 = angle8_solutions[0] < angle8_limits[0] or angle8_solutions[0] > angle8_limits[1]
        temp82 = angle8_solutions[1] < angle8_limits[0] or angle8_solutions[1] > angle8_limits[1]
        temp91 = angle9_solutions[0] < angle9_limits[0] or angle9_solutions[0] > angle9_limits[1]
        temp92 = angle9_solutions[1] < angle9_limits[0] or angle9_solutions[1] > angle9_limits[1]
        # check if both solutions for any angle don't fit the limits, 
        # solutions for angle6 and angle10 don't fit too
        if (temp71 and temp72) or (temp81 and temp82) or (temp91 and temp92) or \
        ((temp71 or temp81 or temp91) and (temp72 or temp82 or temp92)):
            angle6_solutions.pop(i-k)
            angle10_solutions.pop(i-k)
            k += 1
            continue
        else:
            # add suitable solution. Can return 0, 1 or 2 packs of angles
            if not (temp71 or temp81 or temp91):
                angles['hip_yaw'] = angle5
                angles['hip_roll'] = angle6_solutions[i-k]
                angles['hip_pitch'] = angle7_solutions[0]
                angles['knee'] = angle8_solutions[0]
                angles['ankle_pitch'] = angle9_solutions[0]
                angles['ankle_roll'] = angle10_solutions[i-k]
                solutions.append(angles)
            if not (temp72 or temp82 or temp92):
                angles['hip_yaw'] = angle5
                angles['hip_roll'] = angle6_solutions[i-k]
                angles['hip_pitch'] = angle7_solutions[1]
                angles['knee'] = angle8_solutions[1]
                angles['ankle_pitch'] = angle9_solutions[1]
                angles['ankle_roll'] = angle10_solutions[i-k]
                solutions.append(angles)
    return solutions

if __name__ == "__main__":
    import sys
    from geometry.Vector import Vector
    from geometry.Quaternion import Quaternion
    sys.path.append('model')
    from KondoMVModel import KondoMVModel
<<<<<<< HEAD
    print(compute_leg_ik(Vector(0, -0.0534, -0.223), Quaternion(0,0,-1, 0), KondoMVModel()))
=======
    print(compute_leg_ik(Vector(0, -0.0319, -0.200), Quaternion(0,0,-1, 0), KondoMVModel()))
>>>>>>> 72ecda77b4f566e3b8ea885e8d52ee86a7b9bbfb
