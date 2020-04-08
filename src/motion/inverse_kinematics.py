import math
import time

def compute_leg_ik(foot_target, foot_orientation, model):
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
    orientation = foot_orientation
    orientation = orientation.normalize_vector()
    target = foot_target
    # Turn the leg first
    hip_yaw = orientation.w 
    cos5 = math.cos(hip_yaw)
    sin5 = math.sin(hip_yaw)
    target.x = foot_target.x * cos5 + (foot_target.y + a5) * sin5
    tmp = target.y
    target.y = (tmp + a5) * cos5 - foot_target.x * sin5
    target.z = foot_target.z

    orientation.x =  foot_orientation.x * cos5 + orientation.y * sin5
    orientation.y = orientation.y * cos5 - orientation.x * sin5
    orientation.z = orientation.z
    
    angles = {}
    solutions = [] 

    # get servos' limits. angleX_limits for servo X    
    hip_roll_limits = [model.servos[(6,1)]['limits'][0] / 180. * math.pi, 
                    model.servos[(6,1)]['limits'][1] / 180. * math.pi]

    hip_pitch_limits = [model.servos[(7,1)]['limits'][0] / 180. * math.pi, 
                    model.servos[(7,1)]['limits'][1] / 180. * math.pi]

    knee_limits = [model.servos[(8,1)]['limits'][0] / 180. * math.pi, 
                    model.servos[(8,1)]['limits'][1] / 180. * math.pi]

    ankle_pitch_limits = [model.servos[(9,1)]['limits'][0] / 180. * math.pi, 
                    model.servos[(9,1)]['limits'][1] / 180. * math.pi]

    ankle_roll_limits = [model.servos[(10,1)]['limits'][0] / 180. * math.pi, 
                    model.servos[(10,1)]['limits'][1] / 180. * math.pi]

    # calculate hip_roll with numerical method
    # first attempt to find hip_roll
    # divide limits segment into 10 parts to get 11 probable points
    calculation_step = (hip_roll_limits[1] - hip_roll_limits[0]) / 10
    node_points = []
    # calculate the value of F function for each probable point
    for i in range(11): 
        hip_roll = hip_roll_limits[0] + i * calculation_step
        cos = math.cos(hip_roll)
        sin = math.sin(hip_roll)
        tmp1 = orientation.y * cos + orientation.z * sin
        tmp2 = orientation.z * cos - orientation.y * sin
        # F functon looks like
        node_points.append(((target.y + b5) * cos + target.z * sin - c10) * (tmp1**2 -
            tmp2**2 - orientation.x**2) - a10 - b10 * tmp1 / math.sqrt(tmp2**2 + orientation.x**2))
    # find all segments with zero solutions for the equation F(hip_roll)=0
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
    hip_roll_solutions = []

    for segment in solution_segments:
        # define the boundaries of the segment
        bound1 =  hip_roll_limits[0] + segment * calculation_step
        bound2 = bound1 + calculation_step
        while bound2 - bound1 > 0.00025:
            step = (bound2 - bound1) / 10
            node_points = []
            for i in range(11):
                hip_roll = bound1 + i * step
                cos = math.cos(hip_roll)
                sin = math.sin(hip_roll)
                tmp1 = orientation.y * cos + orientation.z * sin
                tmp2 = orientation.z * cos - orientation.y * sin
                
                node_points.append(((target.y + b5) * cos + target.z * sin - c10) * (tmp1**2 - 
                    tmp2**2 - orientation.x**2) - a10 - b10 * tmp1 / math.sqrt(tmp2**2 + 
                    orientation.x**2))
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
            # calculate more precised value of hip_roll
            hip_roll = bound1 + k * step
            # narrow the boundaries
            if k > k2:
                bound1 += k2 * step
                bound2 = bound1 + step
            else:
                bound1 += k * step
                bound2 = bound1 + step
        # add calculated solution
        hip_roll_solutions.append(hip_roll)
    # calculate ankle_roll for each hip_roll as it depends on hip_roll
    ankle_roll_solutions = []
    k = 0
    for i in range(len(hip_roll_solutions)):
        tan6 = math.tan(hip_roll_solutions[i-k])
        ankle_roll = math.atan((-orientation.y - orientation.z * tan6) /  \
            math.sqrt((orientation.z - orientation.y * tan6)**2 + orientation.x**2 * (1 + tan6**2)))

        # check if got ankle_roll fits the limits. If not, hip_roll solution is not a solution 
        if ankle_roll_limits[0] < ankle_roll and ankle_roll < ankle_roll_limits[1]: 
            ankle_roll_solutions.append(ankle_roll)
        else:
            hip_roll_solutions.pop(i-k)
            k += 1
    # now time to calculate hip_pitch, knee, ankle_pitch
    k = 0
    for i in range(len(hip_roll_solutions)):
        
        ankle_pitch_solutions = []
        knee_solutions = []
        hip_pitch_solutions = []

        cos6 = math.cos(hip_roll_solutions[i-k])
        sin6 = math.sin(hip_roll_solutions[i-k])
        # angle987 is sum of hip_pitch, knee, ankle_pitch
        angle987 = math.atan(-orientation.x / (orientation.z * cos6 - orientation.y * sin6))
        sin987 = math.sin(angle987)
        cos987 = math.cos(angle987)
        # calculate ankle_pitch
        k1 = a6 * sin987 + target.x * cos987 + (target.z * cos6 - (target.y + b5) * sin6) * sin987
        k2 = a9 + a6 * cos987 + (target.z * cos6 - (target.y + b5) * sin6) * cos987 - \
            target.x * sin987 + b10 / math.cos(ankle_roll_solutions[i-k]) + ((target.y + b5) * cos6 + \
            target.z * sin6 - c10) * math.tan(ankle_roll_solutions[i-k])
        m = (k1**2 + k2**2 + a8**2 - a7**2) / (a8 * 2)
        temp1 = k1**2 * m**2 - (k1**2 + k2**2) * (m**2 - k2**2)
        if temp1 >= 0:
            temp2 = (-k1 * m + math.sqrt(temp1)) / (k1**2 + k2**2)
            temp3 = (-k1 * m - math.sqrt(temp1)) / (k1**2 + k2**2)
            if math.fabs(temp2) <= 1 and math.fabs(temp3) <= 1:
                ankle_pitch_solutions.append(math.asin(temp2))
                ankle_pitch_solutions.append(math.asin(temp3))
            else:
                hip_roll_solutions.pop(i-k)
                ankle_roll_solutions.pop(i-k)
                k += 1
                continue
        else:
            hip_roll_solutions.pop(i-k)
            ankle_roll_solutions.pop(i-k)
            k += 1
            continue
        # calculate knee
        knee_solutions.append(
            math.atan((k1 + a8 * math.sin(ankle_pitch_solutions[0])) / \
            (k2 + a8 * math.cos(ankle_pitch_solutions[0]))) - ankle_pitch_solutions[0])
        knee_solutions.append(
            math.atan((k1 + a8 * math.sin(ankle_pitch_solutions[1])) / \
            (k2 + a8 * math.cos(ankle_pitch_solutions[1]))) - ankle_pitch_solutions[1])
        # calculate hip_pitch
        hip_pitch_solutions.append(ankle_pitch_solutions[0] + knee_solutions[0] - angle987)
        hip_pitch_solutions.append(ankle_pitch_solutions[1] + knee_solutions[1] - angle987)
        # check each solution to fit the limits
        temp71 = hip_pitch_solutions[0] < hip_pitch_limits[0] or \
            hip_pitch_solutions[0] > hip_pitch_limits[1]
        temp72 = hip_pitch_solutions[1] < hip_pitch_limits[0] or \
            hip_pitch_solutions[1] > hip_pitch_limits[1]
        temp81 = knee_solutions[0] < knee_limits[0] or knee_solutions[0] > knee_limits[1]
        temp82 = knee_solutions[1] < knee_limits[0] or knee_solutions[1] > knee_limits[1]
        temp91 = ankle_pitch_solutions[0] < ankle_pitch_limits[0] or \
            ankle_pitch_solutions[0] > ankle_pitch_limits[1]
        temp92 = ankle_pitch_solutions[1] < ankle_pitch_limits[0] or \
            ankle_pitch_solutions[1] > ankle_pitch_limits[1]
        # check if both solutions for any angle don't fit the limits, 
        # solutions for hip_roll and ankle_roll don't fit too
        if (temp71 and temp72) or (temp81 and temp82) or (temp91 and temp92) or \
        ((temp71 or temp81 or temp91) and (temp72 or temp82 or temp92)):
            hip_roll_solutions.pop(i-k)
            ankle_roll_solutions.pop(i-k)
            k += 1
            continue
        else:
            # add suitable solution. Can return 0, 1 or 2 packs of angles
            if not (temp71 or temp81 or temp91):
                angles['hip_yaw'] = hip_yaw
                angles['hip_roll'] = hip_roll_solutions[i-k]
                angles['hip_pitch'] = hip_pitch_solutions[0]
                angles['knee'] = knee_solutions[0]
                angles['ankle_pitch'] = ankle_pitch_solutions[0]
                angles['ankle_roll'] = ankle_roll_solutions[i-k]
                solutions.append(angles)
            if not (temp72 or temp82 or temp92):
                angles['hip_yaw'] = hip_yaw
                angles['hip_roll'] = hip_roll_solutions[i-k]
                angles['hip_pitch'] = hip_pitch_solutions[1]
                angles['knee'] = knee_solutions[1]
                angles['ankle_pitch'] = ankle_pitch_solutions[1]
                angles['ankle_roll'] = ankle_roll_solutions[i-k]
                solutions.append(angles)
    return solutions