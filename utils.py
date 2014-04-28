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


def yesno(msg):
    print msg
    a = raw_input('y/n: ')
    if a == 'y':
        return True
    elif a == 'n':
        return False
    else:
        return yesno(msg)


class ex(Exception):

    def __init__(self, msg):
        Exception.__init__(self, msg)


def wait(msg=""):
    if msg != "":
        print ""
        print msg
    else:
        print ""
    _ = raw_input("Press <enter> to continue...")
    print ""


import pprint


def pr(level, *args, **kwargs):
    '''
    @summary: helper function for printing to terminal
    @param level: defines how to tabulate the print statement
    @param *args: numbers, strings, anything
    @result: nothing
    '''

    # if "pretty" in kwargs:

    string = " ".join([pprint.pformat(arg) for arg in args])
    string = string.replace("'", "").replace('"', '')
    if level == 0:
        sym = "="
        w = 78
        string = " " + string + " "
        n = len(string)
        d = w - n
        l = ''
        r = ''
        if d > 0:
            if d % 2 == 0:
                l = sym * int(d / 2.)
                r = l
            else:
                l = sym * int(d / 2.)
                r = sym * int(d / 2. + 1)
            print ''
            print sym * w
            print l + string + r
            print sym * w
            print ''
        else:
            print ''
            print sym * w
            print string
            print sym * w
            print ''
    elif level == 1:
        print " - " + string
    elif level == 2:
        print "   * " + string
    elif level == 3:
        lines = string.split("\n")
        for line in lines:
            print "      " + line
    else:
        print "? " + string