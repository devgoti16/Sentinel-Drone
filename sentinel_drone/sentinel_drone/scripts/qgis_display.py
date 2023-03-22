#!/usr/bin/env python3
from qgis.utils import iface
import rospy
from std_msgs.msg import String
from sentinel_drone.msg import Geolocation

def longlatcallback(data):
    objid = data.objectid 
    long = data.long
    lat = data.lat
    pnt = QgsPointXY(lon, lat)
    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('Black'))
    m.setIconType(QgsVertexMarker.ICON_CIRCLE)
    m.setIconSize(12)
    m.setPenWidth(1)
    m.setFillColor(QColor(0, 200, 0))
    
canvas = iface.mapCanvas()
rospy.init_node('OpenMapPlotting',anonymous = True)
rospy.Subscriber('geolocation',Geolocation,longlatcallback)
rospy.spin()

    