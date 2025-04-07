from parser import parse

debug = True
if (debug):
    import pathDebug
else:
    import path
    import motorControl


# def plotAmogusAndHors():
#     RATE = 100  # steps/sec

#     # draws an among us
#     body_pts = [(5, 0), (5, 4), (0, 4),
#                 (0, 0), (2, 0), (2, 1), (3, 1), (3, 0)]
#     body_paths = [("line"), ("curve", 3), ("line"),
#                   ("curve", 1), ("line"), ("line"), ("line"),
#                   ("curve", 1)]
#     path.traceShape(body_pts, body_paths, 0.5, RATE)
#     visor_pts = [(4, 3), (1, 3)]
#     visor_paths = [("curve", 2), ("curve", 2)]
#     path.traceShape(visor_pts, visor_paths, 0.5, RATE)

#     # draws a hors
#     hb_pts = [(-4, 0), (-6.5, 0),
#               (-6.5, -8),
#               (-4, -8), (-4, -6), (-1.5, -6),
#               (-1.5, -8),
#               (1, -8), (1, -4.5),
#               (0.3, -3.5), (-4, -3.5)]
#     hb_paths = [("semicircle", True), ("line"),
#                 ("curve", 1.3),
#                 ("line"), ("line"), ("line"),
#                 ("curve", 1.3),
#                 ("line"), ("curve", 1),
#                 ("line"), ("line")]
#     path.traceShape(hb_pts, hb_paths, 0.5, RATE)
#     path.plotPoint((-5, 0), "ro")
#     path.plotPoint((-6, 0), "ro")
#     path.traceShape([(-5, -1), (-6, -1)],
#                     [("curve", -2)],
#                     0.3, RATE)
#     input()


if __name__ == "__main__":
    try:
        shape = parse("./instruction.txt")
        print(shape)
        if (debug):
            print("doing debug")
            pathDebug.traceShape(shape[0], shape[1],
                                 shape[2], shape[3], shape[4])
        else:
            path.traceShape(shape[0], shape[1],
                            shape[2], shape[3], shape[4])
            motorControl.stop_car()  # stop movement
            motorControl.destroy()   # clean up GPIO
        print("\nFinished path and cleanup done")
    except KeyboardInterrupt:
        if (not debug):
            motorControl.stop_car()  # stop movement
            motorControl.destroy()   # clean up GPIO
        print("Stopped and cleanup done")
