import math


def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def median(inp, dim=2):
    res = []
    l = len(inp)

    for i in range(dim):
        arr = []

        for el in inp:
            arr.append(el[i])

        sort = sorted(arr)
        res_i = sort[l//2]
        res.append(res_i)

    return res
