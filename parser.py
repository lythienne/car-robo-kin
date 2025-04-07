import re


# this is definitely awful
def isNumber(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def parse(filename):
    text = ""
    stepsize = 0.5  # default values
    rate = 100
    points = []
    splines = []
    paths = []
    face_one_way = False

    # sets text to the text contents of the file
    with open(filename, encoding="utf8") as file:
        text = file.read().replace('\n', ' ')

    # puts all the points in the text into a list
    for point in re.findall(r"\((.*?)\)", text):
        points.append(tuple(float(x.strip()) for x in point.split(",")))

    # puts all the spline component lists in the text into a list
    for point in re.findall(r"\[(.*?)\]", text):
        splines.append(list(float(x.strip()) for x in point.split(",")))

    # search through the rest of the text to find paths, step size, and rate
    words = text.split(" ")
    temp = 0  # if a number appears before its keyword it is stored here
    found_curve = False
    semicircle = False

    for word in words:
        word = word.strip(".,")
        if isNumber(word):
            if stepsize == -1:  # if step size keyword has been found
                stepsize = float(word)
            elif rate == -1:
                rate = float(word)
            elif found_curve:  # if "curve" appears before the radius in text
                paths.append(("curve", float(word)))
                found_curve = False
            else:
                temp = float(word)
        else:
            if word == "line":
                paths.append(("line",))
            elif word == "spline":
                paths.append(("spline", splines[0], splines[1]))
                splines = splines[2:]
            elif word == "steps":
                stepsize = temp if temp != 0 else -1  # ready to accept val
                temp = 0
            elif word == "steps/sec":
                rate = temp if temp != 0 else -1
                temp = 0
            elif word == "curve":
                if temp != 0:
                    paths.append(("curve", temp))
                    temp = 0
                else:
                    found_curve = True
            elif word == "semicircle":
                if temp != 0:
                    paths.append(("semicircle", temp > 0))
                    temp = 0
                else:
                    semicircle = True
            elif word == "counterclockwise":
                if semicircle:
                    paths.append(("semicircle", True))
                    semicircle = False
                else:
                    temp = 1
            elif word == "clockwise":
                if semicircle:
                    paths.append(("semicircle", False))
                    semicircle = False
                else:
                    temp = -1
            elif word == "face_one_way":
                face_one_way = True
    return (points, paths, face_one_way, stepsize, rate)
