# SPDX-FileCopyrightText: 2011-2024 Blender Foundation
#
# SPDX-License-Identifier: Apache-2.0
#
# Code for computing the Lookup Tables used by solid-angle ellipse sampling.
# The computed values go into scene/shader.tables.
#
# The fundamental idea to precompute the CDF comes from
# "Area-preserving parameterizations for spherical ellipses" by Ibón Guillén et al.,
# but the ellipse_S fit and LUT as well as the parametrizations are new.

import numpy
from mpmath import mp, mpf
import random
import multiprocessing


ACCURATE_PREC = 192
FLOAT_PREC = 24


def ellipse_params(alpha, beta):
    a = mp.sin(alpha)
    b = mp.sin(beta)
    a2 = a * a
    b2 = b * b
    a2m = 1.0 - a2
    b2m = 1.0 - b2
    c = (b * a2m) / (a * mp.sqrt(b2m))
    m = (a2 - b2) / b2m
    n = m / a2
    return n, m, c


# Compute the solid angle of a spherical ellipse.
def ellipse_S(alpha, beta):
    n, m, c = ellipse_params(alpha, beta)
    return mp.pi / 2 - c * mp.ellippi(n, m)


# Numerical approximation to ellipse_S, we tabulate the residual factor between them.
def ellipse_S_fit(x, y, A=0.988812, B=0.328265, C=1.64709):
    return A * (x * x * y) * (C + B * mp.cos(x * mp.pi / 2))


# Compute the fraction of solid angle of a spherical ellipse up to internal angle phi.
# Since spherical ellipses are symmetric w.r.t both axes, this only considers one
# quarter, so 0 <= phi <= pi/2.
def ellipse_S_partial(alpha, beta, phi):
    n, m, c = ellipse_params(alpha, beta)
    t = mp.atan((mp.tan(alpha) / mp.tan(beta)) * mp.tan(phi))

    full = mp.pi / 2 - c * mp.ellippi(n, m)
    partial = phi - c * mp.ellippi(n, t, m)
    return partial / full


class EllipseTable:
    def __init__(self, N):
        self.N = N
        self.table = numpy.empty((N, N), dtype=numpy.float32)

    # Implementation of the particular table
    def coord_from_lut(self, x, y):
        raise NotImplementedError

    def coord_to_lut(self, x, y):
        raise NotImplementedError

    def compute_value(self, x, y):
        raise NotImplementedError

    def approximate_value(self, x, y):
        raise NotImplementedError

    # Compute the value that belongs at the given LUT coords
    def compute_lut_value(self, coords):
        with mp.workprec(ACCURATE_PREC):
            x, y = self.coord_from_lut(mpf(coords[0]), mpf(coords[1]))
            return self.compute_value(x, y)

    # Compute the LUT in parallel
    def build_lut(self, pool):
        coords = [(xi, yi) for xi in range(self.N) for yi in range(self.N)]
        data = pool.map(self.compute_lut_value, coords)

        for (xi, yi), val in zip(coords, data):
            self.table[yi, xi] = val

    # Lookup a value in the computed LUT
    def lookup_lut(self, x, y):
        x, y = self.coord_to_lut(x, y)

        x0 = int(x)
        x1 = min(x0 + 1, self.N - 1)
        y0 = int(y)
        y1 = min(y0 + 1, self.N - 1)

        fx = x - x0
        fy = y - y0

        def mix(a, b, t): return a + (b - a) * t
        low = mix(self.table[y0, x0], self.table[y0, x1], fx)
        high = mix(self.table[y1, x0], self.table[y1, x1], fx)
        return mix(low, high, fy)

    # Transpose the computed LUT
    def transpose(self):
        self.table = numpy.transpose(self.table)

    # Print table in C syntax
    def print_table(self):
        print(f"static const float {self.NAME}[{self.N}*{self.N}] = {{")
        for row in self.table:
            print("  " + " ".join(f"{float(p):.8f}f," for p in row))
        print("};")

    # Save table as an OpenEXR file
    def save_table(self):
        import OpenImageIO
        name = f'{self.NAME}.exr'
        out = OpenImageIO.ImageOutput.create(name)
        buf = OpenImageIO.ImageBuf(self.table)
        out.open(name, buf.spec())
        buf.write(out)
        out.close()

    # Compute a chunk of errors for random positions
    def compute_errors(self, seed, num=1000):
        random.seed(seed)
        res = []
        with mp.workprec(FLOAT_PREC):
            for _ in range(num):
                x = mpf(random.random())
                y = mpf(random.random())

                with mp.workprec(ACCURATE_PREC):
                    ref = self.compute_value(x, y)
                lut = self.lookup_lut(x, y)

                res.append((x, y, abs(lut - ref)))
        return res

    # Evaluate the error statistics for the LUT approximation
    def eval_error(self, pool, num=32):
        result = []
        for sub_result in pool.map(self.compute_errors, range(num)):
            result += sub_result

        values = [v[2] for v in result]
        print(f"// Max: {max(values)}")
        print(f"// Avg: {sum(values)/len(values)}")
        print(f"// 10th-largest: {sorted(values)[-10]}")

        with open(f'{self.NAME}.tsv', 'w') as f:
            for x, y, v in result:
                f.write(f"{x}\t{y}\t{v}\n")


class EllipseS(EllipseTable):
    NAME = "table_ellipse_S"

    def coord_from_lut(self, x, y):
        x = mp.sqrt(x / (self.N - 1))
        y = mp.sqrt(y / (self.N - 1))
        return x, y

    def coord_to_lut(self, x, y):
        x = x * x * (self.N - 1)
        y = y * y * (self.N - 1)
        return x, y

    def compute_value(self, x, y):
        # Convert to angles
        alpha = max(1e-10, min(1.0 - 1e-10, x) * mp.pi / 2)
        beta = max(1e-10, y * alpha)

        # Convert back for fit evaluation to account for the clamping
        x = alpha * 2 / mp.pi
        y = beta / alpha

        # Tabulate residual factor
        return ellipse_S(alpha, beta) / ellipse_S_fit(x, y)


class EllipseCDF(EllipseTable):
    NAME = "table_ellipse_CDF"

    def coord_from_lut(self, x, y):
        x = (x / (self.N - 1)) ** 4
        y = (y / (self.N - 1)) ** 4
        return x, y

    def coord_to_lut(self, x, y):
        x = x**(1 / 4) * (self.N - 1)
        y = y**(1 / 4) * (self.N - 1)
        return x, y

    def compute_value(self, x, y):
        # Convert to angles
        alpha = 0.5
        beta = max(1e-10, y * alpha)
        phi = x * mp.pi / 2

        return ellipse_S_partial(alpha, beta, phi)


if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    pool = multiprocessing.Pool()

    cdf = EllipseCDF(64)
    cdf.build_lut(pool)
    cdf.save_table()
    cdf.eval_error(pool)
    cdf.transpose()
    cdf.print_table()

    s = EllipseS(32)
    s.build_lut(pool)
    s.save_table()
    s.eval_error(pool)
    s.print_table()
