import math

class Model:

	def setparams(self, A, h):
		self.A = A
		self.h = h

	def r2pic(self, xb, yb, pol, azim):
		xbi = xb * math.cos(azim) + yb * math.sin(azim)
		ybi = yb * math.cos(azim) - xb * math.sin(azim)

		if xbi <= 0:
			return "problem"

		alp = math.atan(self.h / xbi) + pol
		x = ybi / math.cos(alp) / math.sqrt(xbi**2 + self.h ** 2)
		y = math.tan(alp)
		u = x * self.A[0][0] + self.A[0][2]
		v = y * self.A[1][1] + self.A[1][2]
		return (int(u), int(v))

	def pic2r(self, u, v, pol, azim):
		x = (u - self.A[0][2]) / 1.0 / self.A[0][0]
		y = (v - self.A[1][2]) / 1.0 / self.A[1][1]
		alp = math.atan(y)
		bet = math.atan(x)
		balp = alp - pol
		bbet = bet + azim

		if math.tan(balp) == 0:
			return "problem"

		xb = self.h / math.tan(balp) * math.cos(bbet)
		yb = self.h / math.tan(balp) * math.sin(bbet)

		return (xb, yb)

