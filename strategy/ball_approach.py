import math
import json


class BallApproach:

	dist = 1
	turn_ang = 0

	def get_data(self, xr, yr, xb, yb, yaw, half_ang):
		self.xr = xr
		self.yr = yr
		self.xb = xb
		self.yb = yb
		self.yaw = yaw
		self.half_ang = half_ang


	def set_constants(self, consts):
		self.max_step = consts["max_step"]
		self.min_dist = consts["min_dist"]
		self.ang_thres1 = consts["ang_thres1"]
		self.ang_thres2 = consts["ang_thres2"]
		self.CIRCLE_RADIUS = consts["CIRCLE_RADIUS"]
		self.GOAL_LEN = consts["GOAL_LEN"]
		self.WIND_X = consts["WIND_X"]
		self.GOAL_POS = consts["GOAL_POS"]
		self.step_before_strike = consts["step_before_strike"]


	def get_diff(self):
		return self.xr - self.xb

	def find_trajectory (self):
		xr = self.xr
		yr = self.yr
		xb = self.xb
		yb = self.yb
		#max_step = self.max_step
		CIRCLE_RADIUS = self.CIRCLE_RADIUS
		GOAL_LEN = self.GOAL_LEN
		WIND_X = self.WIND_X
		GOAL_POS = self.GOAL_POS

		traj = []
    	
		traj.append ((xr, yr))
		
    	#-----------------------------------------------------------
    	#find starting point on the circle
		
		xbr = xb - xr #x ball relative
		ybr = yb - yr #y ball relative
    	
		r    = CIRCLE_RADIUS
		leng = math.sqrt (xbr**2 + ybr**2)
		
		beta  = math.asin (ybr / leng)
		alpha = math.asin (r / leng)
		
		if (xb < xr):# + CIRCLE_RADIUS):
			sx = 0
			sy = 0

			if (yr + (yr - GOAL_POS - int (GOAL_LEN / 2)) * (xr - xb) / (WIND_X - xb) > yb):
				sx = - leng * math.cos (alpha + beta) * math.cos (alpha) + xr
				sy = leng * math.sin (alpha + beta) * math.cos (- alpha) + yr
		
			else:
				alpha = - alpha
				
				sx = - leng * math.cos (alpha + beta) * math.cos (alpha) + xr
				sy = leng * math.sin (alpha + beta) * math.cos (- alpha) + yr
		
			traj.append ((sx, sy))
	
		#-----------------------------------------------------------
    	#find kick point on the circle
		gbx = xb - WIND_X                        #goal-ball x
		gby = yb - GOAL_POS - int (GOAL_LEN / 2) #goal-ball y
		
		length_gb =  math.sqrt (gbx**2 + gby**2)
		
		kpx = xb + CIRCLE_RADIUS * gbx / length_gb #kick point x
		kpy = yb + CIRCLE_RADIUS * gby / length_gb #kick point y
		
		traj.append ((kpx, kpy))
	
    	#move with proper steps on the circle
    	
    	#if (xr < xb):
    	#    traj.append ((xr, yr))
    	#    traj.append ((xb - CIRCLE_RADIUS, yb))
    	
    	#else:
    	#    if (yr < yb):
    	#    else:
    	
		traj.append ((WIND_X, GOAL_POS + GOAL_LEN / 2))

		self.wtraj = traj
		
		return traj

	def lin_trans(self, W, shift, vec):
		res = [vec[i] - shift[i] for i in range(2)]
		res = [W[i][0] * res[0] + W[i][1] * res[1] for i in range(2)]
		return res

	def convert_trajectory(self):
		traj = self.wtraj
		yaw = self.yaw

		# shift,rotmat - coefficients of the rotation transformation
		shift = traj[0]
		rotmat = [[math.cos(yaw), math.sin(yaw)], [-1 * math.sin(yaw), math.cos(yaw)]]
	
		rtraj = [self.lin_trans(rotmat, shift, point) for point in traj]
		self.rtraj = rtraj
		return rtraj


	def make_decision(self):

		rtraj = self.rtraj
		min_dist = self.min_dist
		ang_thres1 = self.ang_thres1 # - minimum allowed angle between robot walk direction and robot-to-ball direction
		ang_thres2 = self.ang_thres2 # - minimum allowed angle between the parts of the trajectory


		# changing the strike point in rtraj to the point that is better for right foot kick
		# targvec - the vector from  center of the goal to the ball
		# tv_ln - length of the targvec
		# norm - the vector normal to the targvec with the length taken from data.json
		targvec = (rtraj[1][0] - rtraj[2][0], rtraj[1][1] - rtraj[2][1])
		tv_ln = math.sqrt(targvec[0] ** 2 + targvec[1] ** 2)
		norm = (self.step_before_strike * targvec[1] * 1.0 / tv_ln, -self.step_before_strike * targvec[0] * 1.0 / tv_ln)
		rtraj[1][0] += norm[0]
		rtraj[1][1] += norm[1]
        
		# path - vector from robot to ball
		# vec1,vec2 - vectors, representing the first and the second parts of the trajectory
		path = rtraj[1]
		pth_ln = math.sqrt(path[0] ** 2 + path[1] ** 2)
		self.dist = pth_ln
        # checking if the robot is too close to the ball
		if pth_ln == 0:
			raise Exception('robot too close to the kick point')
            
		ang1 = math.acos(path[0] / pth_ln)
		ang2 = math.pi - math.acos((targvec[0] * path[0] + targvec[1] * path[1]) / tv_ln / pth_ln)
		vec_prod = targvec[0] * path[1] - targvec[1] * path[0]

        # making the decision, based on the distance and angles
		if pth_ln < min_dist:            
			if ang2 > ang_thres2:
				if vec_prod < 0:
					return "step left"
				elif vec_prod > 0:
					return "step right"
                
			if ang2 < ang_thres2:
				if ang1 > ang_thres1:
					if path[1] > 0:
						return "turn left"
					else:
						return "turn right"
				else:
					return "strike"

		else:
			if ang2 < ang_thres2:
				if ang1 > ang_thres1:
					if path[1] > 0:
						self.turn_ang = ang1
						return "turn left"
					else:
						self.turn_ang = ang1
						return "turn right"
				else:
					return "step forward"
                
			if vec_prod > 0:
				if path[1] < 0:
					self.turn_ang = ang1 + self.half_ang / 2.0
					return "turn right"
				else:
					if abs(ang1 - self.half_ang / 2.0) < ang_thres1:
						return "step forward"
					elif ang1 < self.half_ang / 2.0 - ang_thres1:
						self.turn_ang = self.half_ang - ang1
						return "turn right"
					else:
						self.turn_ang = -self.half_ang + ang1
						return "turn left"
			elif vec_prod < 0:
				if path[1] > 0:
					self.turn_ang = ang1 + self.half_ang / 2.0
					return "turn left"
				else:
					if abs(ang1 - self.half_ang / 2.0) < ang_thres1:
						return "step forward"
					elif ang1 < self.half_ang / 2.0 - ang_thres1:
						self.turn_ang = self.half_ang - ang1
						return "turn left"
					else:
						self.turn_ang = -self.half_ang + ang1
						return "turn right"