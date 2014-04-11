def sift(arr, key, val):
    r = []
    for a in arr:
        if a[key] == val:
            r.append(a)
    return r


def pluck(arr, key):
    r = []
    for a in arr:
        r.append(a[key])
    return r


def findKV(arr, key, val):
    for a in arr:
        if a[key] == val:
            return a
    return None
