import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from numpy import floor, array, ceil
from numpy.linalg import norm
from numpy import inf
# ________________________________________________________________________________

def gridValue(mapData, Xp):
    #Create a method to wait on map...
    
    #rclpy.node.get_logger('node name').info('Res' + str(mapData) + ' Res  ' + str(mapData.info.resolution))
    resolution = round(mapData.info.resolution, 2)
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    #If data is bad sometimes get a division by zero error
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + (floor((Xp[0]-Xstartx)/resolution))

    #rclpy.node.get_logger('node name').info('Index ' + str(index))
    if int(index) < len(Data):
        #rclpy.node.get_logger('node name').info('I am a wall ' + str(Data[int(index)]))
        return Data[int(index)]
    else:
        return 100
        
        
# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int((floor((Xp[1]-Xstarty)/resolution) * width) + (floor((Xp[0]-Xstartx)/resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + (i/mapData.info.width)*mapData.info.resolution #+ 0.006
    x = (i - (floor((y - mapData.info.origin.position.y)/mapData.info.resolution) * mapData.info.width)) * mapData.info.resolution + mapData.info.origin.position.x
    
    if y > 0:
        y = y - 0.006
    else:
        y = y - 0.002
        
    '''
    mapData.info.origin.position.x + (i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
    '''
    return array([x, y])
# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point) #Something liek 20,000 probs
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    
    points = point_of_index(mapData, index)
    
    #rclpy.node.get_logger('node name').info('Point Index then Point from index' + str(point) + "  " + str(index) + "   " + str(points))
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                #rclpy.node.get_logger('node name').info('Point ' + str(norm(array(point)-point_of_index(mapData, i))))
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain += 1
    #rclpy.node.get_logger('node name').info('Info gain ' + str(infoGain) + 'Real gain ' + str(infoGain*(mapData.info.resolution**2)))
    return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________

