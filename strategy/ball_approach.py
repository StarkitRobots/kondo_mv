import math
import json


class ball_approach:

	def get_data(self, xr, yr, xb, yb, yaw):
		self.xr = xr
		self.yr = yr
		self.xb = xb
		self.yb = yb
		self.yaw = yaw

	def set_constants(self, consts):
		self.max_step = consts[0]
		self.min_dist = consts[1]
		self.ang_thres1 = consts[2]
		self.ang_thres2 = consts[3]
		self.CIRCLE_RADIUS = consts[4]
		self.GOAL_LEN = consts[5]
		self.WIND_X = consts[6]
		self.GOAL_POS = consts[7]


	def get_diff(self):
		return self.xr - self.xb

	def find_trajectory (self):
		xr = self.xr
		yr = self.yr
		xb = self.xb
		yb = self.yb
		max_step = self.max_step
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
    	
		traj.append ((WIND_X, GOAL_POS + int (GOAL_LEN / 2)))

		self.wtraj = traj
		
		return traj

	def lin_trans(self, W, shift, vec):
		res = [vec[i] - shift[i] for i in range(2)]
		res = [W[i][0] * res[0] + W[i][1] * res[1] for i in range(2)]
		return res

	def convert_trajectory(self):
		traj = self.wtraj
		yaw = self.yaw   
		shift = traj[0]
		rotmat = [[math.cos(yaw), math.sin(yaw)], [-1 * math.sin(yaw), math.cos(yaw)]]
	
		rtraj = [self.lin_trans(rotmat, shift, point) for point in traj]
		self.rtraj = rtraj
		return rtraj

	def cross_dot(self, v1, v2):
		return -v1[0] * v2[1] + v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1]

	def make_decision(self):

		rtraj = self.rtraj
		min_dist = self.min_dist
		ang_thres1 = self.ang_thres1
		ang_thres2 = self.ang_thres2

		with open('data.json') as f:
			d = json.load(f)
		dv = list(d.values())

		targvec = (rtraj[1][0] - rtraj[2][0], rtraj[1][1] - rtraj[2][1])
		tv_ln = math.sqrt(targvec[0] ** 2 + targvec[1] ** 2)
		norm = (int(dv[-1] * targvec[1] / tv_ln), int(-dv[-1] * targvec[0] / tv_ln))
		rtraj[1][0] -= norm[0]
		rtraj[1][1] -= norm[1]
	
		path = rtraj[1]
	
		vec1 = path
		vec1_ln = math.sqrt(vec1[0] ** 2 + vec1[1] ** 2)
		vec2 = (rtraj[2][0] - vec1[0], rtraj[2][1] - vec1[1])
		vec2_ln = math.sqrt(vec2[0] ** 2 + vec2[1] ** 2)
	
		prod = self.cross_dot(vec1, vec2)
		ang2 = prod[0] / vec1_ln / vec2_ln
	
		if prod[1] > 0:
			ang2 = (math.pi - abs(ang2)) * prod[0] / abs(prod[0])


		if vec1_ln < min_dist:
	
			if path[0] <= 0:
				return "problem"
		
			if ang2 > 0 and (math.pi - abs(ang2)) > ang_thres2:
				return "step right"

			elif ang2 < 0 and (math.pi - abs(ang2)) > ang_thres2:
				return "step left"
		
			if path[0] > 0:
				if abs(math.atan(path[1] / path[0])) < ang_thres1:
					return "strike"
				elif path[1] < 0:
					return "turn left"
				else:
					return "turn right"
			else:
				if path[1] < 0:
					return "turn left"
				else:
					return "turn right"

		elif path[0] > 0:
			if abs(math.atan(path[1] / path[0])) < ang_thres1:
				return "step forward"
			elif path[1] < 0:
				return "turn left"
			else:
				return "turn right"
		else:
			if path[1] < 0:
				return "turn left"
			else:
				return "turn right"

