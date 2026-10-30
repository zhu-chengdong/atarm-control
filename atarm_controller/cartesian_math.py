"""Small dense algebra for the offline position controller (no device imports)."""
import math


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def norm(x):
    return math.sqrt(dot(x, x))


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def mv(a, x):
    return tuple(dot(row, x) for row in a)


def gram(a):
    n = len(a[0])
    return [[sum(row[i]*row[j] for row in a) for j in range(n)] for i in range(n)]


def solve(a, b):
    n = len(b)
    a = [list(row)+[v] for row, v in zip(a, b)]
    for i in range(n):
        pivot = max(range(i, n), key=lambda j: abs(a[j][i]))
        a[i], a[pivot] = a[pivot], a[i]
        if abs(a[i][i]) < 1e-14:
            raise ValueError('CARTESIAN_SINGULAR_SYSTEM')
        value = a[i][i]
        a[i] = [x/value for x in a[i]]
        for j in range(n):
            if j != i:
                value = a[j][i]
                a[j] = [x-value*y for x, y in zip(a[j], a[i])]
    return tuple(row[-1] for row in a)


def least_squares(a, y, damping=0.):
    g = gram(a)
    for i in range(len(g)):
        g[i][i] += damping*damping
    return solve(g, [sum(row[i]*v for row, v in zip(a, y)) for i in range(len(g))])


def singular_values(a):
    """Jacobi diagonalization of J^T J, sufficient for these 1--3 column matrices."""
    g = gram(a)
    n = len(g)
    for _ in range(60):
        if n == 1:
            break
        p, q = max(((i, j) for i in range(n) for j in range(i+1, n)), key=lambda ij: abs(g[ij[0]][ij[1]]))
        if abs(g[p][q]) < 1e-14:
            break
        angle = .5*math.atan2(2*g[p][q], g[q][q]-g[p][p])
        c, s = math.cos(angle), math.sin(angle)
        pp, qq, pq = g[p][p], g[q][q], g[p][q]
        for i in range(n):
            if i not in (p, q):
                ip, iq = g[i][p], g[i][q]
                g[i][p] = g[p][i] = c*ip-s*iq
                g[i][q] = g[q][i] = s*ip+c*iq
        g[p][p] = c*c*pp-2*c*s*pq+s*s*qq
        g[q][q] = s*s*pp+2*c*s*pq+c*c*qq
        g[p][q] = g[q][p] = 0.
    return tuple(sorted(math.sqrt(max(0., g[i][i])) for i in range(n)))


def mix_matrix(bend):
    return tuple(tuple(row)+(0.,) for row in bend)+((0., 0., 1.),)


def quaternion_angle(a, b):
    ratio = abs(dot(a, b))/math.sqrt(dot(a, a)*dot(b, b))
    return 2*math.acos(min(1., ratio))
