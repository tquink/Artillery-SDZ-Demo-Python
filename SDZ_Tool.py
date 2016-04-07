import arcpy
from math import radians, sin, cos, tan
import re

arcpy.env.overwriteOutput = True

def arcAngles(def1, def2):
	if def1 >= def2:
		arcpy.AddMessage("Angles are not valid...") 
	count = .1
	yield def1
	while (def1 + count) < def2:
		yield def1 + count
		count += .1
	yield def2

def validAngle(deflection):
	angle = deflection/(6400.0/360.0)
	if angle > 360:
		return angle - 360.0
	else:
		return angle

def pointTransformation(distance, angle, start_x, start_y):
	(disp_x, disp_y) = (distance * sin(radians(angle)), distance * cos(radians(angle)))
	(end_x, end_y) = (start_x + disp_x, start_y + disp_y)
	return (end_x, end_y)

def createArcLines(leftDef,rightDef, dis, pointX, pointY, fc):
	array = arcpy.Array()
	for i in arcAngles(leftDef,rightDef):
		x, y = pointTransformation(dis, validAngle(i), pointX, pointY)
		array.append(arcpy.Point(x,y))

	polyline = arcpy.Polyline(array)

	with arcpy.da.InsertCursor(fc, "SHAPE@") as insertCursor:
		insertCursor.insertRow([polyline])

def createAreas(dis, ang, x_point, y_point, FC_in, FC_out):
	for curveDis in dis:
		createArcLines(ang[0], ang[1], curveDis, x_point, y_point, FC_in)

	vertxArray = arcpy.Array()
	for sideAngle in ang:
		minx, miny = pointTransformation(dis[0], validAngle(sideAngle), x_point, y_point)
		vertxArray.append(arcpy.Point(minx,miny))
		maxx, maxy = pointTransformation(dis[1] - dis[0], validAngle(sideAngle), minx, miny)
		vertxArray.append(arcpy.Point(maxx,maxy))
		sideLine = arcpy.Polyline(vertxArray)
		
		with arcpy.da.InsertCursor(FC_in, "SHAPE@") as insertCursor:
			insertCursor.insertRow([sideLine])
		vertxArray = arcpy.Array()


def areaB(maxDis, minDis, startPoints, areaALen, deflections, plusAngle, memoryFC, outFC):
	minAngle = math.degrees(math.acos(((minDis**2 + minDis**2) - areaALen**2)/(2*minDis*minDis)))
	minLeftDeflection = deflections[0] - ((minAngle + plusAngle) * (6400.0/360.0))
	minRightDeflection = deflections[1] + ((minAngle +  plusAngle) * (6400.0/360.0))
	createArcLines(minLeftDeflection, minRightDeflection, minDis, startPoints[0], startPoints[1], memoryFC)

	maxAngle = math.degrees(math.acos(((maxDis**2 + maxDis**2) - areaALen**2)/(2*maxDis*maxDis)))
	maxLeftDeflection = deflections[0] - ((maxAngle + plusAngle) * (6400.0/360.0))
	maxRightDeflection = deflections[1] + ((maxAngle +  plusAngle) * (6400.0/360.0))
	createArcLines(maxLeftDeflection, maxRightDeflection, maxDis, startPoints[0], startPoints[1], memoryFC)
	
	vertArray = arcpy.Array()
	for mindis, mindirect, maxdis, maxdirect in [[minDis,minLeftDeflection, maxDis, maxLeftDeflection],[minDis,minRightDeflection, maxDis, maxRightDeflection]]:
		minx, miny = pointTransformation(mindis, validAngle(mindirect), startPoints[0], startPoints[1])
		vertArray.append(arcpy.Point(minx,miny))
		maxx, maxy = pointTransformation(maxdis, validAngle(maxdirect), startPoints[0], startPoints[1])
		vertArray.append(arcpy.Point(maxx,maxy))
		sideLine = arcpy.Polyline(vertArray)
		
		with arcpy.da.InsertCursor(memoryFC, "SHAPE@") as insertCursor:
			insertCursor.insertRow([sideLine])
		vertArray = arcpy.Array()


def areaC(maxD, minD, originPoints, areaALength, allDefs, littleAngle, inFC, outFeatures):
	createArcLines(allDefs[0] - (littleAngle * (6400.0/360.0)), allDefs[1] + (littleAngle * (6400.0/360.0)), minD, originPoints[0], originPoints[1], inFC)

	maxAng = math.degrees(math.acos(((maxD**2 + maxD**2) - areaALength**2)/(2*maxD*maxD)))
	maxLeftDef = allDefs[0] - ((maxAng + littleAngle) * (6400.0/360.0))
	maxRightDef = allDefs[1] + ((maxAng +  littleAngle) * (6400.0/360.0))
	createArcLines(maxLeftDef, maxRightDef, maxD, originPoints[0], originPoints[1], inFC)
	
	ptArray = arcpy.Array()
	for mindis, mindirect, maxdis, maxdirect in [[minD,allDefs[0] - (littleAngle * (6400.0/360.0)), maxD, maxLeftDef],[minD,allDefs[1] + (littleAngle * (6400.0/360.0)), maxD, maxRightDef]]:
		minx, miny = pointTransformation(mindis, validAngle(mindirect), originPoints[0], originPoints[1])
		ptArray.append(arcpy.Point(minx,miny))
		maxx, maxy = pointTransformation(maxdis, validAngle(maxdirect), originPoints[0], originPoints[1])
		ptArray.append(arcpy.Point(maxx,maxy))
		sideLine = arcpy.Polyline(ptArray)
		
		with arcpy.da.InsertCursor(inFC, "SHAPE@") as insertCursor:
			insertCursor.insertRow([sideLine])
		ptArray = arcpy.Array()


def areaD(distances, l_r_def, pointStarts, FCinputs, FCoutputs, smallAngle, lengthSide, shortLength, areaAL):
	createArcLines(l_r_def[0] - ((25.0 + smallAngle) * (6400.0/360.0)),l_r_def[1] + ((25.0 + smallAngle) * (6400.0/360.0)), distances[0], pointStarts[0], pointStarts[1], FCinputs)
	createArcLines(l_r_def[0],l_r_def[1], distances[1], pointStarts[0], pointStarts[1], FCinputs)
	
	maxAng = math.degrees(math.acos(((shortLength**2 + shortLength**2) - areaAL**2)/(2*shortLength*shortLength)))
	maxLeftDef = l_r_def[0] - ((maxAng + smallAngle) * (6400.0/360.0))
	maxRightDef = l_r_def[1] + ((maxAng + smallAngle) * (6400.0/360.0))

	ptArray = arcpy.Array()
	xl1, yl1 = pointTransformation(distances[0], validAngle(l_r_def[0]) - (smallAngle + 25.0), pointStarts[0], pointStarts[1])
	xl2, yl2 = pointTransformation(lengthSide, validAngle(l_r_def[0]) - (smallAngle + 25.0), pointStarts[0], pointStarts[1])
	xl3, yl3 = pointTransformation(shortLength, validAngle(maxLeftDef), pointStarts[0], pointStarts[1])
	xl4, yl4 = pointTransformation(distances[1], validAngle(l_r_def[0]) - smallAngle, pointStarts[0], pointStarts[1])
	points1 = [arcpy.Point(xl1, yl1),arcpy.Point(xl2, yl2),arcpy.Point(xl3, yl3),arcpy.Point(xl4, yl4)]
	arrayLine = arcpy.Array()
	for pt in points1:
		arrayLine.append(pt)
	line = arcpy.Polyline(arrayLine)
	with arcpy.da.InsertCursor(FCinputs, "SHAPE@") as insertCursor:
			insertCursor.insertRow([line])
	arrayLine = arcpy.Array()

	xr1, yr1 = pointTransformation(distances[0], validAngle(l_r_def[1]) + (smallAngle + 25.0), pointStarts[0], pointStarts[1])
	xr2, yr2 = pointTransformation(lengthSide, validAngle(l_r_def[1]) + (smallAngle + 25.0), pointStarts[0], pointStarts[1])
	xr3, yr3 = pointTransformation(shortLength, validAngle(maxRightDef), pointStarts[0], pointStarts[1])
	xr4, yr4 = pointTransformation(distances[1], validAngle(l_r_def[1]) + smallAngle, pointStarts[0], pointStarts[1])
	points2 = [arcpy.Point(xr1, yr1),arcpy.Point(xr2, yr2),arcpy.Point(xr3, yr3),arcpy.Point(xr4, yr4)]
	arrayLine = arcpy.Array()
	for pt in points2:
		arrayLine.append(pt)
	line = arcpy.Polyline(arrayLine)
	with arcpy.da.InsertCursor(FCinputs, "SHAPE@") as insertCursor:
			insertCursor.insertRow([line])
	arrayLine = arcpy.Array()

def areaE(areaEL, sideLengths, points, defAll, eAngle, inputFC, outputFC):
	createAreas([0,areaEL], [defAll[0], defAll[1]], points[0], points[1], inputFC, outputFC)

def dashedLines(minLMinus, minL, leftD, rightD, origins, xAng, outPath, fpName):
	splitLength = outPath.rfind("\\")
	if arcpy.Exists('Dashed_Lines_' + outPath[splitLength+1:]) == False:
		fc_input = arcpy.CreateFeatureclass_management(outPath[:splitLength], 'Dashed_Lines_' + outPath[splitLength+1:],\
	                                               "POLYLINE",'','','', arcpy.SpatialReference(3857))
		arcpy.AddField_management(fc_input, "firing_point", "TEXT")
	else:
		fc_input = outPath[:splitLength] + '\\Dashed_Lines_' + outPath[splitLength+1:]
		with arcpy.da.UpdateCursor(fc_input, "firing_point") as fpCursor:
			for fp in fpCursor:
				if fp[0] == fpName:
					fpCursor.deleteRow()
	
	lx1, ly1 = pointTransformation(minL, validAngle(leftD), origins[0], origins[1])
	lx2, ly2 = pointTransformation(minLMinus, validAngle(leftD) - xAng, origins[0], origins[1])

	lineA = arcpy.Array()
	lineA.append(arcpy.Point(origins[0], origins[1]))
	lineA.append(arcpy.Point(lx1, ly1))
	line_A = arcpy.Polyline(lineA)

	lineB = arcpy.Array()
	lineB.append(arcpy.Point(origins[0], origins[1]))
	lineB.append(arcpy.Point(lx2, ly2))
	line_B = arcpy.Polyline(lineB)

	rx1, ry1 = pointTransformation(minL, validAngle(rightD), origins[0], origins[1])
	rx2, ry2 = pointTransformation(minLMinus, validAngle(rightD) + xAng, origins[0], origins[1])

	lineC = arcpy.Array()
	lineC.append(arcpy.Point(origins[0], origins[1]))
	lineC.append(arcpy.Point(rx1, ry1))
	line_C = arcpy.Polyline(lineC)

	lineD = arcpy.Array()
	lineD.append(arcpy.Point(origins[0], origins[1]))
	lineD.append(arcpy.Point(rx2, ry2))
	line_D = arcpy.Polyline(lineD)

	poly_lines = [line_A, line_B, line_C, line_D]

	for l in poly_lines:
		with arcpy.da.InsertCursor(fc_input, ("firing_point", "status", "SHAPE@")) as insertCursor:
			insertCursor.insertRow([fpName, "Inactive",l])

def main(*args):
	xIn, yIn = args[2].split(',')
	origin_x, origin_y = float(xIn), float(yIn)

	artillery = {105:{'peD':8.0*float(args[9]), 'peR_UP':12.0*float(args[8]), 'peR_Dow':8.0*float(args[8]), 'AreaA':550.0, 'AreaB':550.0, 'AreaC':300.0, "AreaE": 550.0},
				 155:{'peD':8.0*float(args[9]), 'peR_UP':12.0*float(args[8]), 'peR_Dow':8.0*float(args[8]), 'AreaA':725.0, 'AreaB':725.0, 'AreaC':350.0, "AreaE": 725.0}}

	fc_input = arcpy.CreateFeatureclass_management('in_memory', 'target_area', "POLYLINE",'','','', arcpy.SpatialReference(3857))

	createAreas([float(args[5]), float(args[4])], [float(args[6]), float(args[7])], origin_x, origin_y, fc_input, args[10])
	arcpy.AddMessage("Target area complete...")

	extraAngle = math.degrees(math.asin(artillery[float(args[3])]['peD'] /((float(args[4])-float(args[5]))/2 + float(args[5]))))
	minLength = (float(args[5]) - artillery[float(args[3])]['peR_UP'])

	createAreas([minLength,\
		         float(args[4]) + artillery[float(args[3])]['peR_Dow']],\
		        [(6400.0/360.0) * (validAngle(float(args[6])) - (extraAngle)),\
	             (6400.0/360.0) * (extraAngle + validAngle(float(args[7])))],\
	             origin_x, origin_y,\
	             fc_input, args[10])
	arcpy.AddMessage("Impact area complete...")

	maxLength = (float(args[4]) + artillery[float(args[3])]['peR_Dow'] + artillery[float(args[3])]['AreaB'])

	areaB(maxLength, minLength, [origin_x, origin_y], artillery[float(args[3])]['AreaA'],\
	     [float(args[6]), float(args[7])], extraAngle, fc_input, args[10])
	arcpy.AddMessage("Areas A & B complete...")

	areaC(minLength, minLength - artillery[float(args[3])]['AreaC'], [origin_x, origin_y], artillery[float(args[3])]['AreaA'],\
		  [float(args[6]), float(args[7])], extraAngle, fc_input, args[10])
	arcpy.AddMessage("Area C complete...")

	areaD([artillery[float(args[3])]['AreaE'],\
		   minLength - artillery[float(args[3])]['AreaC']],\
		  [float(args[6]), float(args[7])],\
	      [origin_x, origin_y],\
	       fc_input, args[10], extraAngle,\
	       artillery[float(args[3])]['AreaA']/sin(radians(25.0)),\
	       minLength, artillery[float(args[3])]['AreaA'])
	arcpy.AddMessage("Area D complete...")

	areaE(artillery[float(args[3])]['AreaE'], artillery[float(args[3])]['AreaA']/sin(radians(25.0)),\
	     [origin_x, origin_y],\
	     [(6400.0/360.0) * (validAngle(float(args[6])) - (extraAngle + 25.0)),\
	      (6400.0/360.0) * (25.0 + extraAngle + validAngle(float(args[7])))],\
	       extraAngle, fc_input, args[10])
	arcpy.AddMessage("Area E complete...")

	if args[12] == "New":
		dashedLines(minLength, float(args[5]), float(args[6]), float(args[7]), [origin_x, origin_y], extraAngle, args[10], args[13])
		arcpy.FeatureToPolygon_management(fc_input, args[10])
		arcpy.AddField_management(args[10], "area_name", "TEXT")
		arcpy.AddField_management(args[10], "firing_point", "TEXT")

		areas = {1:"Target Area", 2:"Impact Area", 3:"Area A/B", 4:"Area C", 5:"Area D", 6:"Area E"}
		areaCount = 1		
		with arcpy.da.UpdateCursor(args[10], ("area_name","firing_point", "status")) as UpdateCur:
			for update in UpdateCur:
				update[0] = areas[areaCount]
				update[1] = args[13]
				update[2] = "Inactive"
				UpdateCur.updateRow(update)
				areaCount += 1
	else:
		if arcpy.Exists(args[11]) == False:
			arcpy.AddMessage("No Feature Class to insert SDZ into.")
			return False
		else:
			dashedLines(minLength, float(args[5]), float(args[6]), float(args[7]), [origin_x, origin_y], extraAngle, args[11], args[13])
			arcpy.FeatureToPolygon_management(fc_input, "in_memory/tempPolys")

			with arcpy.da.UpdateCursor(args[11], "firing_point") as fpCur:
				for fp in fpCur:
					if fp[0] == args[13]:
						fpCur.deleteRow()

			areas = {1:"Target Area", 2:"Impact Area", 3:"Area A/B", 4:"Area C", 5:"Area D", 6:"Area E"}
			areaCount = 1
			with arcpy.da.SearchCursor("in_memory/tempPolys", "SHAPE@") as tempCur:
				for temp in tempCur:
					with arcpy.da.InsertCursor(args[11], ("area_name","firing_point", "status", "SHAPE@")) as insertCur:
						insertCur.insertRow((areas[areaCount], args[13], "Inactive", temp[0]))
						areaCount += 1

	try:
		mxd = arcpy.mapping.MapDocument("CURRENT")
		df = arcpy.mapping.ListDataFrames(mxd)[0]
		splitLength = args[10].rfind("\\")
		layer = args[10][:splitLength] + '\\dashed_lines_' + args[10][splitLength+1:]
		lyr = arcpy.mapping.Layer(layer)
		arcpy.mapping.AddLayer(df, lyr, "AUTO_ARRANGE")
	except:
		return True

	arcpy.AddMessage("Surface Danger Zone complete.")

	arcpy.AddMessage("This is for demo purposes only and not to be used for real SDZ creation")
	
if __name__ == "__main__":

	argv = tuple(arcpy.GetParameterAsText(i) for i in range(arcpy.GetArgumentCount()))
	main(*argv)

	
