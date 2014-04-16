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


def findKVindex(arr, key, val):
    for i in range(len(arr)):
        if arr[i][key] == val:
            return i
    return -1


def wait(msg=''):
    print msg
    a = raw_input("press <enter> to continue...")


def yesno(msg):
    print msg
    a = raw_input('y/n: ')
    if a == 'y':
        return True
    elif a == 'n':
        return False
    else:
        return yesno(msg)
