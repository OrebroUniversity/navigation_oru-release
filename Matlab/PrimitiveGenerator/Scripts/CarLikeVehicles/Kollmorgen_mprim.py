import shutil

from math import cos, sin, tan, atan, atan2, pi, sqrt, radians
import os

from numpy import array, linalg, mat, linspace, zeros, float64

epsilon = 1e-12
EXPORT_STEER_ANGLE=True


def wrap(x):
    '''
    return a value between -pi and pi
    '''
    if -pi <= x < pi:
        return x
    if 0.0 < x < 3.0 * pi:
        return x - 2.0 * pi
    if -3.0 * pi <= x < 0.0:
        return x + 2.0 * pi
    y = x % (2.0 * pi)
    if y > pi:
        return y - 2.0 * pi
    return y


class Parameter:
    def __init__(self, name):
        self.name = name
        self.fileformat = 0
        self.car_width = 0.550
        self.car_length_back = 0.200
        self.car_length_front = 1.600
        self.wheelbase = 1.190
        self.max_steer_angle = 1.220
        self.numberofangles = 16
        self.steeringanglepartitions = 8
        self.steeringanglecardinality = 1
        self.distance = 2.0
        self.cell_size = 0.1
        self.max_steer_rate = 1
        self.curve_min_speed = 0.3
        self.forwardcost = 1.0
        self.forwardturncost = 1.2
        self.backwardcost = 1.4
        self.backwardturncost = 1.6
        self.step_max_distance = self.cell_size / 2
        self.step_max_angle = radians(1.0)
        self.step_max_steer = radians(2.5)
        self.min_speed = 0.1
        self.max_speed = 1.0
        self.acceleration = 0.5
        self.deceleration = 0.5
        self.sideways_acceleration = 0.2

    def set_file_par(self, key, value):
        parmap = dict(fileformat=('fileformat', int),
                      car_width=('car_width', float),
                      car_length_front=('car_length_front', float),
                      car_length_back=('car_length_back', float),
                      max_steering_radians=('max_steer_angle', float),
                      distance_between_axes=('wheelbase', float),
                      resolution_m=('cell_size', float),
                      numberofangles=('numberofangles', int),
                      steeringanglepartitions=('steeringanglepartitions',
                                               int),
                      steeringanglecardinality=('steeringanglecardinality',
                                                int),
                      max_speed=('max_speed', float),
                      min_speed=('min_speed', float),
                      acceleration=('acceleration', float),
                      deceleration=('deceleration', float),
                      sideways_acceleration=('sideways_acceleration', float),
                      wheel_angular_velocity=('max_steer_rate',
                                              float),
                      step_max_distance=('step_max_distance', float),
                      step_max_angle=('step_max_angle', float),
                      step_max_steer=('step_max_steer', float)
                      )
        try:
            attr, type_ = parmap[key]
            setattr(self, attr, type_(value))
        except KeyError:
            if (self.fileformat == 0 and
                key in ('startangle_c', 'startsteer_c')):
                pass
            else:
                raise

    def write_file_header(self, file):
        file.write('fileformat: %d\n' % self.fileformat)
        file.write('resolution_m: %.6f\n' % self.cell_size)
        file.write('numberofangles: %d\n' % self.numberofangles)
        file.write('steeringanglepartitions: %d\n' %
                   self.steeringanglepartitions)
        file.write('steeringanglecardinality: %d\n' %
                   self.steeringanglecardinality)
        file.write('car_width: %.6f\n' % self.car_width)
        file.write('car_length_front: %.6f\n' % self.car_length_front)
        file.write('car_length_back: %.6f\n' % self.car_length_back)
        file.write('max_steering_radians: %.6f\n' %
                   self.max_steer_angle)
        file.write('distance_between_axes: %.6f\n' %
                   self.wheelbase)
        file.write('max_speed: %.3f\n' % self.max_speed)
        file.write('min_speed: %.3f\n' % self.min_speed)
        file.write('acceleration: %.3f\n' % self.acceleration)
        file.write('deceleration: %.3f\n' % self.deceleration)
        file.write('sideways_acceleration: %.3f\n' %
                   self.sideways_acceleration)
        file.write('wheel_angular_velocity: %.3f\n' %
                   self.max_steer_rate)
        file.write('step_max_distance: %.3f\n' % self.step_max_distance)
        file.write('step_max_angle: %.3f\n' % self.step_max_angle)
        file.write('step_max_steer: %.3f\n' % self.step_max_steer)

    def canonical_name(self):
        return ('%s_%d_%d_%.1f_%.1f' %
                (self.name,
                 self.numberofangles,
                 self.steeringanglecardinality,
                 self.distance,
                 self.cell_size))

    def x_range(self):
        return linspace(-self.distance, self.distance,
                        int(round(2*self.distance / self.cell_size)) + 1)

    def y_range(self):
        return self.x_range()

    def th_range(self):
        return linspace(0, 2*pi, self.numberofangles, endpoint=False)

    def steer_range(self):
        steer_step = 2 * pi / self.steeringanglepartitions
        max_steer = steer_step * ((self.steeringanglecardinality - 1) // 2)
        return linspace(-max_steer, max_steer,
                        self.steeringanglecardinality)

    def steer_id_range(self):
        limit = self.steeringanglecardinality // 2
        return [steer % self.steeringanglepartitions
                for steer in range(-limit, limit + 1)]

    def symmetry_steer_id(self, steer_id):
        return ((self.steeringanglepartitions - steer_id) %
                self.steeringanglepartitions)

    def angle_id(self, th):
        if th < 0.0:
            th += 2.0 * pi
        return (int(round(th / (2.0 * pi) * self.numberofangles)) %
                self.numberofangles)

    def steer(self, curvature, forward):
        angle = atan(curvature * self.wheelbase)
        if not forward:
            return -angle
        return angle

    def steer_id(self, th):
        if th < 0.0:
            th += 2.0 * pi
        return (int(round(th / (2.0 * pi) * self.steeringanglepartitions)) %
                self.steeringanglepartitions)


class Poly5:

    # y(x) = a5*x**5 + a4*x**4 + a3*x**3 + a2*x**2 + a1*x + a0
    # Use formulas orientation, th
    # dy = dy/dx = tan(th)
    # and curvature, k
    # k = ddy/(1 + dy)**(3/2)
    # ddy = k * (1 + dy**2)**(3/2)
    # y(0) = 0
    # dy(0) = tan(th0)
    # ddy(0) = k0 * 1 + (1 + tan(th0)**2)**(3/2)
    # y(length) = 0
    # dy(length) = tan(th1)
    # ddy(length) = k1 * 1 + (1 + tan(th1)**2)**(3/2)

    def __init__(self, length, th0, k0, th1, k1):
        self._coeff = a = [0.0] * 6  # coefficients for the polynom

        # The condition for the starting points gives the first three
        # coeefficients directly.
        a[0] = 0.0
        a[1] = tan(th0)
        a[2] = k0 * (1.0 + a[1] * a[1])**(3.0 / 2.0) / 2.0

        # To get the remaining three solve the linear system given by
        # the condition for the end point.
        x = length
        x2 = x * x
        x3 = x2 * x
        x4 = x3 * x
        x5 = x4 * x
        A = mat([[x5, x4, x3],
                 [5.0 * x4, 4.0 * x3, 3.0 * x2],
                 [20.0 * x3, 12.0 * x2, 6.0 * x]])

        tan_th1 = tan(th1)
        B = mat([[-a[2] * x2 - a[1] * x],
                 [tan_th1 - 2.0 * a[2] * x - a[1]],
                 [k1 * (1.0 + tan_th1 * tan_th1)**(3.0 / 2.0) - 2.0 * a[2]]])
        u = linalg.solve(A, B)
        a[5] = u[0, 0]
        a[4] = u[1, 0]
        a[3] = u[2, 0]
        self._coeff = a

    def state(self, x):
        '''returns y, th and curvature for given x coordinate.'''
        x2 = x * x
        x3 = x2 * x
        x4 = x3 * x
        x5 = x4 * x
        a = self._coeff
        y = a[5] * x5 + a[4] * x4 + a[3] * x3 + a[2] * x2 + a[1] * x
        dy = (5 * a[5] * x4 + 4 * a[4] * x3 + 3 * a[3] * x2 +
              2 * a[2] * x + a[1])
        ddy = (20 * a[5] * x3 + 12 * a[4] * x2 + 6 * a[3] * x +
               2 * a[2])
        return y, atan(dy), ddy/(1 + dy * dy)**(3.0 / 2.0)

    def step_dist(self, x_0, y_0, dist):
        a = self._coeff
        x = x_0 + dist
        while True:
            x2 = x * x
            x3 = x2 * x
            x4 = x3 * x
            x5 = x4 * x
            y = a[5] * x5 + a[4] * x4 + a[3] * x3 + a[2] * x2 + a[1] * x
            d2 = (x - x_0) * (x - x_0) + (y - y_0) * (y - y_0)
            if abs(d2 - dist * dist) < 1.0e-6:
                return x
            dy = (5 * a[5] * x4 + 4 * a[4] * x3 + 3 * a[3] * x2 +
                  2 * a[2] * x + a[1])
            step = (dist - sqrt(d2)) / sqrt(1.0 + dy * dy)
            if x + step < x_0:
                step = (x_0 - x) * 0.99
            x += step

    def step_angle(self, x_0, th_0, x, max_dth):
        th_goal = None
        a = self._coeff
        iteration = 0
        while iteration < 100:
            x2 = x * x
            x3 = x2 * x
            x4 = x3 * x
            dy = (5 * a[5] * x4 + 4 * a[4] * x3 + 3 * a[3] * x2 +
                  2 * a[2] * x + a[1])
            th = atan(dy)
            if th_goal is None:
                if th > th_0 + max_dth:
                    th_goal = th_0 + max_dth
                elif th < th_0 - max_dth:
                    th_goal = th_0 - max_dth
                else:
                    return x
            elif abs(th - th_goal) < 1.0e-6:
                return x
            step = (th_goal - th) / (th - th_0) * (x - x_0)
            x_0 = x
            th_0 = th
            x += step
            iteration += 1
        raise Exception('step_angle failed')

    def step_steer(self, x_0, th_0, x, max_dth, wheelbase):
        a = self._coeff
        iteration = 0
        x_start = x_0
        th_start = th_0
        while iteration < 100:
            x2 = x * x
            x3 = x2 * x
            x4 = x3 * x
            dy = (5 * a[5] * x4 + 4 * a[4] * x3 + 3 * a[3] * x2 +
                  2 * a[2] * x + a[1])
            ddy = (20 * a[5] * x3 + 12 * a[4] * x2 + 6 * a[3] * x +
                   2 * a[2])
            k = ddy/(1 + dy * dy)**(3.0 / 2.0)
            th = atan(k * wheelbase)
            if abs(th - th_start) < max_dth:
                if iteration == 0:
                    return x
                else:
                    x_0 = x
            else:
                x_1 = x
            if abs(abs(th - th_start) - max_dth) < 1e-3:
                return x
            x = (x_0 + x_1) / 2.0
            iteration += 1
        raise Exception('step_steer failed')

def gen_path(par, start, goal):
    # We make a coordinate transform such that the polynom curve
    # starts at origo and ends on the positive x-axis.
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    a = atan2(dy, dx)

    # If we are moving forward from the starting point we must enter
    # the goal point forward or vice verse. Otherwise we can not find
    # a polynom solution.
    if cos(start[2] - a) * cos(goal[2] - a) <= 0.00001:
        return None

    distance = sqrt(dx * dx + dy * dy)
    wheelbase = par.wheelbase
    poly = Poly5(distance,
                 start[2] - a,
                 tan(start[3]) / wheelbase,
                 goal[2] - a,
                 tan(goal[3]) / wheelbase)
    c = cos(a)
    s = sin(a)
    path = []
    if cos(start[2] - a) > 0.0:
        # forward
        direction = 1.0
        orientation = 0.0
    else:
        # reverse
        direction = -1.0
        orientation = pi
    d = 0.0
    prev = None
    path = []
    x = 0.0
    done = False
    while not done:
        if x > distance - par.step_max_distance / 10:
            done = True
            x = distance
        y, th, k = poly.state(x)
        steer = atan(k * wheelbase)
        if abs(steer) > par.max_steer_angle:
            return None
        if prev:
            dx = x - prev[0]
            dy = y - prev[1]
            ds = sqrt(dx * dx + dy *dy)
            dt = ds / par.curve_min_speed
            d += ds
            if d > 2 * distance:
                return None
            if abs(steer - prev[2]) / dt > par.max_steer_rate:
                return None
        prev = (x, y, steer)
        if EXPORT_STEER_ANGLE:
            path.append((start[0] + c * x - s *y,
                         start[1] + s * x + c *y,
                         wrap(th + a + orientation),
                         steer))
        else:
            path.append((start[0] + c * x - s *y,
                         start[1] + s * x + c *y,
                         wrap(th + a + orientation),
                         direction * k))
        x1 = poly.step_dist(x, y, par.step_max_distance)
        x1 = poly.step_angle(x, th, x1, par.step_max_angle)
        x = poly.step_steer(x, steer, x1, par.step_max_steer, wheelbase)
    return array(path)


class Motion:
    def __init__(self, tag, par, path=None, file=None):
        self.tag = tag
        self.par = par
        self.angle = None
        self.steer = None
        if file:
            self.read(file)
        else:
            self.path = path
            self.cost = self.calc_cost()
            self.size = 0 if path is None else path.shape[0]
        self.angle = self.start_angle_index()
        self.steer = self.start_steer_index()

    def forward(self):
        x = self.path[1, 0] - self.path[0, 0]
        y = self.path[1, 1] - self.path[0, 1]
        th = self.path[0, 2]
        return x * cos(th) + y * sin(th) > 0.0

    def straigth(self):
        x = self.path[-1, 0] - self.path[0, 0]
        y = self.path[-1, 1] - self.path[0, 1]
        th = self.path[0, 2]
        if self.forward():
            return abs(wrap(th - atan2(y, x))) < 1.0e-6
        else:
            return abs(wrap(th - atan2(-y, -x))) < 1.0e-6

    def calc_cost(self):
        if self.path is None:
            return None
        if self.forward():
            if self.straigth():
                return self.par.forwardcost
            else:
                return self.par.forwardturncost
        else:
            if self.straigth():
                return self.par.backwardcost
            else:
                return self.par.backwardturncost

    def start_angle_index(self):
        if self.path is None:
            return None
        return self.par.angle_id(self.path[0, 2])

    def start_steer_index(self):
        if self.path is None:
            return None
        th = self.par.steer(self.path[0,3], self.forward())
        return self.par.steer_id(th)

    def read(self, file):
        for line in file:
            if not line.strip():
                continue
            data = line.split(':')
            if len(data) == 2:
                self._setattr(data[0].strip(), data[1].strip())
            elif not (self.par.fileformat == 0 and
                      line.startswith('startangle_c')):
                break

        self.path = zeros((self.size, 4), dtype=float64)
        self.path[0, :] = [float(x) for x in line.split()]
        for i in range(1, self.size):
            data = file.next().split()
            self.path[i, :] = [float(x) for x in data]
        if self.par.fileformat == 0:
            self._steer2curvature()

    def write(self, file):
        file.write('primID: %d\n' % self.tag)
        file.write('startangle_c: %d\n' % self.angle)
        file.write('startsteer_c: %d\n' % self.steer)
        file.write('additionalactioncostmult: %.2f\n' % self.cost)
        direction = '1' if self.forward() else '-1'
        file.write('motiondirection: %s\n' % direction)
        file.write('intermediateposes: %d\n' % self.size)

        # Don't write -0.000 to file
        def pretty(x):
            if abs(x) < epsilon:
                return 0.0
            else:
                return x

        for i in range(self.size):
            file.write('%.4f %.4f %.4f %.4f\n' %
                       tuple(pretty(x) for x in self.path[i, :]))

    def _setattr(self, key, value):
        attrmap = {'startangle_c': ('angle', int),
                   'startsteer_c': ('steer', int),
                   'additionalactioncostmult': ('cost', float),
                   'motiondirection': ('direction', int),
                   'intermediateposes': ('size', int)}
        if key in attrmap:
            attr, type_ = attrmap[key]
            try:
                setattr(self, attr, type_(value))
            except ValueError:
                if self.par.fileformat == 0 and key == 'startangle_c':
                    pass
                else:
                    print self.par.fileformat, key, value
                    raise

    def symmetry_pi_4(self):
        '''return symmetry motion around pi/4'''
        if not 0.0 <= self.path[0, 2] <= pi/4.0:
            print self.path[0, 2]
        assert 0.0 - epsilon <= self.path[0, 2] <= pi/4.0 + epsilon
        sym_path = zeros(self.path.shape, dtype=float64)
        sym_path[:, 0] = self.path[:, 1]
        sym_path[:, 1] = self.path[:, 0]
        for i in range(self.path.shape[0]):
            sym_path[i, 2] = wrap(pi / 2.0 - self.path[i, 2])
        sym_path[:, 3] = -self.path[:, 3]
        return Motion(self.tag, self.par, path=sym_path)

    def symmetry_pi_2(self):
        '''return symmetry motion around pi/2'''
        assert 0.0 - epsilon <= self.path[0, 2] <= pi / 2.0 + epsilon
        sym_path = zeros(self.path.shape, dtype=float64)
        sym_path[:, 0] = -self.path[:, 0]
        sym_path[:, 1] = self.path[:, 1]
        for i in range(self.path.shape[0]):
            sym_path[i, 2] = wrap(pi - self.path[i, 2])
        sym_path[:, 3] = -self.path[:, 3]
        return Motion(self.tag, self.par, path=sym_path)

    def symmetry_pi(self):
        '''return symmetry motion around pi'''
        assert 0.0 - epsilon <= self.path[0, 2] <= pi + epsilon
        sym_path = zeros(self.path.shape, dtype=float64)
        sym_path[:, 0] = self.path[:, 0]
        sym_path[:, 1] = -self.path[:, 1]
        for i in range(self.path.shape[0]):
            sym_path[i, 2] = -self.path[i, 2]
        sym_path[:, 3] = -self.path[:, 3]
        return Motion(self.tag, self.par, path=sym_path)

    def _steer2curvature(self):
        for i in range(self.size):
            steer = self.path[i, 3]
            self.path[i, 3] = tan(steer) / self.par.wheelbase


class MotionPrimitives:
    def __init__(self, par=Parameter('default'), filename=None):
        self.par = par
        self.motion = {}
        if filename:
            self.load(filename)

    def load(self, filename):
        self.motion = {}
        f = open(filename)
        self.par.fileformat = 0
        for line in f:
            data = line.split(':')
            if len(data) == 2:
                key, value = [x.strip() for x in data]
                if key == 'primID':
                    prim = Motion(int(value), self.par, file=f)
                    if prim.angle not in self.motion:
                        self.motion[prim.angle] = {}
                    if prim.steer not in self.motion[prim.angle]:
                        self.motion[prim.angle][prim.steer] = [prim]
                    else:
                        self.motion[prim.angle][prim.steer].append(prim)
                else:
                    self.par.set_file_par(key, value)

    def save(self, filename):
        '''write primitives to file'''
        f = open(filename, 'w')
        self.par.fileformat = 1
        self.par.write_file_header(f)
        for angle in sorted(self.motion.keys()):
            for steer in sorted(self.motion[angle].keys()):
                for motion in self.motion[angle][steer]:
                    motion.write(f)

    def generate_primitives(self):
        self._generate_base_primitives()
        self._apply_symmetry_pi_4()
        self._apply_symmetry_pi_2()
        self._apply_symmetry_pi()

    def _generate_base_primitives(self):
        # orientation and steering granularity + the initial steering ID
        steering_angle_granularity = 2 * pi / self.par.steeringanglepartitions
        orientation_angle_granularity = 2 * pi / self.par.numberofangles

        # generate the base primitives for this model, that is, the ones
        # in the first sector [0 pi/4]
        x = 0
        y = 0
        # orientation and steering
        for orient_id in range(self.par.numberofangles / 8 + 1):
            if not orient_id in self.motion:
                self.motion[orient_id] = {}
            for steer_id in self.par.steer_id_range():
                th = orient_id * orientation_angle_granularity
                steer = wrap(steer_id * steering_angle_granularity)
                paths = self._generate_primitives_from((x, y, th, steer))
                self.motion[orient_id][steer_id] = paths

    def _generate_primitives_from(self, start):
        paths = []
        tag = 0
        for x in self.par.x_range():
            for y in self.par.y_range():
                if (x == 0.0 and y == 0.0):
                    continue
                dx = x - start[0]
                dy = y - start[1]
                if sqrt(dx * dx + dy * dy) > self.par.distance + 1e-6:
                    continue
                for th in self.par.th_range():
                    for steer in self.par.steer_range():
                        path = gen_path(self.par, start, (x, y, th, steer))
                        if path is not None:
                            paths.append(Motion(tag, self.par, path=path))
                            tag += 1
                print 'target', start[2], start[3], x, y, len(paths)
        return paths

    def _apply_symmetry_pi_4(self):
        syms = {}
        for angle in range(self.par.numberofangles // 8):
            sym_angle = self.par.numberofangles // 4 - angle
            syms[sym_angle] = {}
            if angle not in self.motion:
                continue
            for steer in self.par.steer_id_range():
                if steer not in self.motion[angle]:
                    continue
                sym_paths = []
                for motion in self.motion[angle][steer]:
                    sym_paths.append(motion.symmetry_pi_4())
                sym_steer = self.par.symmetry_steer_id(steer)
                syms[sym_angle][sym_steer] = sym_paths
        self.motion.update(syms)

    def _apply_symmetry_pi_2(self):
        syms = {}
        for angle in range(self.par.numberofangles // 4):
            sym_angle = self.par.numberofangles // 2 - angle
            syms[sym_angle] = {}
            if angle not in self.motion:
                continue
            for steer in self.par.steer_id_range():
                if steer not in self.motion[angle]:
                    continue
                sym_paths = []
                for motion in self.motion[angle][steer]:
                    sym_paths.append(motion.symmetry_pi_2())
                sym_steer = self.par.symmetry_steer_id(steer)
                syms[sym_angle][sym_steer] = sym_paths
        self.motion.update(syms)

    def _apply_symmetry_pi(self):
        syms = {}
        for angle in range(1, self.par.numberofangles // 2):
            sym_angle = self.par.numberofangles - angle
            syms[sym_angle] = {}
            if angle not in self.motion:
                continue
            for steer in self.par.steer_id_range():
                if steer not in self.motion[angle]:
                    continue
                sym_paths = []
                for motion in self.motion[angle][steer]:
                    sym_paths.append(motion.symmetry_pi())
                sym_steer = self.par.symmetry_steer_id(steer)
                syms[sym_angle][sym_steer] = sym_paths
        self.motion.update(syms)


def plot_primitives(filename):
    prim = MotionPrimitives(filename=filename)
    angles = sorted(prim.motion.keys())
    import pylab
    from matplotlib.collections import LineCollection

    def plot_angle(prim, angle):
        forward = []
        backward = []
        for paths in prim.motion[angle].values():
            for motion in paths:
                if motion.forward():
                    forward.append(zip(motion.path[:,0], motion.path[:,1]))
                else:
                    backward.append(zip(motion.path[:,0], motion.path[:,1]))
        pylab.title('Angle = %d' % angle)
        pylab.gca().add_collection(LineCollection(forward, color='blue'))
        pylab.gca().add_collection(LineCollection(backward, color='red'))
        pylab.axis('equal')
        pylab.grid(1)

    pylab.figure('Angle 0')
    plot_angle(prim, 0)

    pylab.figure('Angle 1')
    plot_angle(prim, 1)

    pylab.figure('Angle 2')
    plot_angle(prim, 2)

    pylab.figure('All Angles')
    for angle in angles:
        pylab.subplot(4, 4, angle + 1, aspect='equal')
        plot_angle(prim, angle)
    pylab.show()


def generate(par, config_dir=None):
    prims = MotionPrimitives(par)
    prims.generate_primitives()
    if config_dir is None:
        config_dir = os.path.dirname(os.path.abspath(__file__))
    primdir = os.path.join(config_dir, 'Primitives')
    hstdir = os.path.join(config_dir, 'LookupTables')
    for filepath in (primdir, hstdir):
        try:
            os.mkdir(filepath)
        except OSError:
            pass
    filename = os.path.join(primdir, par.canonical_name() + '.mprim')
    prims.save(filename)


def gen_jh():
    par = Parameter('JH')
    par.distance = 2.5
    par.car_width = 1.100
    par.car_length_front = 2.400
    par.car_length_back = 0.8
    par.wheelbase = 1.370
    par.max_steer_angle = radians(70.0)
    par.max_steer_rate = radians(30.0)
    generate(par)


def gen_rocla():
    par = Parameter('Rocla')
    par.distance = 2.0
    par.car_width = 1.000
    par.car_length_front = 2.500
    par.car_length_back = 0.760
    par.wheelbase = 1.307
    par.max_steer_angle = radians(70.0)
    par.max_steer_rate = radians(75.0)
    generate(par)


def gen_bt():
    par = Parameter('BT')
    par.distance = 2.0
    par.car_width = 0.9
    par.car_length_front = 2.500
    par.car_length_back = 0.40
    par.wheelbase = 1.604000;
    par.max_steer_angle = radians(75.0)
    par.max_steer_rate = radians(45.0)
    generate(par)


def gen_citi2():
    par = Parameter('Citi2')
    par.distance = 2.0
    par.car_width = 0.550
    par.car_length_front = 1.600
    par.car_length_back = 0.20
    par.wheelbase = 1.190
    par.max_steer_angle = radians(75.0)
    par.max_steer_rate = radians(50.0)
    generate(par)


def upgrade(filename):
    shutil.copy2(filename, filename + '.org')
    prims = MotionPrimitives(filename=filename)
    prims.save(filename)


if __name__ == '__main__':
    gen_citi2()
    plot_primitives('Primitives/Citi2_16_1_2.0_0.1.mprim')
