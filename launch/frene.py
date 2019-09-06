desiredSpeed = 1.9
desiredSpeedPlus = 0.0
desiredSpeedMinus = -0.5
desiredSpeedDelta = 0.25

print "desiredSpeed: ", desiredSpeed

print (desiredSpeed+ desiredSpeedPlus) - (desiredSpeed+ desiredSpeedMinus) , ( (desiredSpeed+ desiredSpeedPlus) - (desiredSpeed+ desiredSpeedMinus) ) / 2, "\n"


def my_range(start, end, step):
    while start <= end:
        yield start
        start += step

#for x in my_range(desiredSpeed + desiredSpeedMinus, desiredSpeed + desiredSpeedPlus, desiredSpeedDelta):
#    print x


computed_Delta = ( (desiredSpeed+ desiredSpeedPlus) - (desiredSpeed+ desiredSpeedMinus) ) / 2
print "\n For 3 steps, use Delta: ", computed_Delta
for x in my_range(desiredSpeed + desiredSpeedMinus, desiredSpeed + desiredSpeedPlus, computed_Delta):
    print x

#for( double sf = desiredLongPosition + desiredLongPositionMinus; isSmallerThanOrApproxEqual( sf, desiredLongPosition + desiredLongPositionPlus, 1e-04); sf += desiredLongPositionDelta )
