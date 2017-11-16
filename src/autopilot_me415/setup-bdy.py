import gmplot
import numpy as np

# -- set waypoints ---
# 1508 elevation
wp0 = [40.26778432216462, -111.63506331014253]
wp1 = [40.26702297859316, -111.63508476781465]
wp2 = [40.267641059243935, -111.63587333726502]
wp3 = [40.26658909078882, -111.6358518795929]


# ---- plot waypoints ----
gmap = gmplot.GoogleMapPlotter(wp2[0], wp2[1], 19)
gmap.circle(wp0[0], wp0[1], 10.0)
gmap.circle(wp1[0], wp1[1], 10.0)
gmap.circle(wp2[0], wp2[1], 10.0)
gmap.circle(wp3[0], wp3[1], 10.0)


# --- compute distance in meters between two lat/lon pairs ---
def distance(pt1, pt2):
    """pt = [lat, long]"""
    EARTH_RADIUS = 6371000.0
    distN = EARTH_RADIUS*(pt2[0] - pt1[0])*np.pi/180.0
    distE = EARTH_RADIUS*np.cos(pt1[0]*np.pi/180.0)*(pt2[1] - pt1[1])*np.pi/180.0
    return distE, distN, np.linalg.norm([distN, distE])


# define boundary ellipse
ellipsemajor1 = np.array([40.268316, -111.635040])
ellipsemajor2 = np.array([40.266145, -111.636008])
ellipseminor = [40.266994, -111.634407]

# compute center
center = 0.5*(ellipsemajor1 + ellipsemajor2)

# gmap.circle(ellipsemajor1[0], ellipsemajor1[1], 1.0)
# gmap.circle(ellipsemajor2[0], ellipsemajor2[1], 1.0)
# gmap.circle(ellipseminor[0], ellipseminor[1], 1.0)
# gmap.circle(center[0], center[1], 1.0)

# compute semimajor/minor distances
_, _, R1 = distance(center, ellipsemajor1)
_, _, R2 = distance(center, ellipseminor)
R = [R2, R1]
# gmap.circle(center[0], center[1], R1)
# gmap.circle(center[0], center[1], R2)

# rotation angle
theta = -15*np.pi/180.0

# print center, R, theta

# compute if in/on/out of boundary
def ellipse(pt, center, R, theta):

    dx, dy, _ = distance(center, pt)
    ct = np.cos(theta)
    st = np.sin(theta)
    ell = ((dx*ct + dy*st)/R[0])**2 + ((dx*st - dy*ct)/R[1])**2

    return ell

# # check results
# print ellipse(ellipsemajor1, center, R, theta)
# print ellipse(ellipsemajor2, center, R, theta)
# print ellipse(ellipseminor, center, R, theta)

# compute a bunch of points on boundary
na = 90
nd = 40
alphavec = np.linspace(0.0, 2*np.pi, na)
distvec = np.linspace(0, 0.0013, nd)
lats = np.zeros(na)
lons = np.zeros(na)
lats_in = np.zeros(na)
lons_in = np.zeros(na)
for i in range(na):
    ell = np.zeros(nd)
    for j in range(nd):
        pt = center + [distvec[j]*np.cos(alphavec[i]), distvec[j]*np.sin(alphavec[i])]
        ell[j] = ellipse(pt, center, R, theta)
    lats[i] = np.interp(1.0, ell, center[0] + distvec*np.cos(alphavec[i]))
    lons[i] = np.interp(1.0, ell, center[1] + distvec*np.sin(alphavec[i]))
    lats_in[i] = np.interp(0.7, ell, center[0] + distvec*np.cos(alphavec[i]))
    lons_in[i] = np.interp(0.7, ell, center[1] + distvec*np.sin(alphavec[i]))


# plot boundaries
gmap.plot(lats, lons, edge_width=10)
gmap.plot(lats_in, lons_in, edge_width=10)



gmap.draw("mymap.html")