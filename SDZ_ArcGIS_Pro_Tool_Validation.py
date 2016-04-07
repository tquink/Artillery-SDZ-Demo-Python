import arcpy
class ToolValidator(object):
  """Class for validating a tool's parameter values and controlling
  the behavior of the tool's dialog."""

  def __init__(self):
    """Setup arcpy and the list of tool parameters."""
    self.params = arcpy.GetParameterInfo()

  def initializeParameters(self):
    """Refine the properties of a tool's parameters.  This method is
    called when the tool is opened."""
         
    self.params[11].enabled = False
    self.params[10].enabled = True
    self.params[3].filter.list = ["155","105"]
    self.params[3].value = '155'
    
  def updateParameters(self):
    """Modify the values and properties of parameters before internal
    validation is performed.  This method is called whenever a parameter
    has been changed."""
    
    if self.params[0].altered:
        fp = self.params[0].valueAsText
        rows = arcpy.SearchCursor(fp)
        self.params[1].filter.list = sorted(list(set(r.getValue('firing_point') for r in rows)))
        del rows

    if self.params[1].value != None:
        if self.params[1].hasBeenValidated == False:
            with arcpy.da.SearchCursor(fp, ('min_range','max_range', 'left_deflection','right_deflection','SHAPE@X','SHAPE@Y'), "firing_point = " + "'" + self.params[1].valueAsText + "'") as searchCursor:
                for row in searchCursor:
                    self.params[2].value = str(row[4]) + "," + str(row[5])
                    self.params[2].enabled = False
                    self.params[4].value = row[1] 
                    self.params[5].value = row[0] 
                    self.params[6].value = row[2] 
                    self.params[7].value = row[3] 
                    self.params[8].value = 25
                    self.params[9].value = 5

    if self.params[12].value == "New":
        self.params[11].enabled = False
        self.params[11].value = ""
        self.params[10].enabled = True
        self.params[13].value = ""
    else:
        self.params[11].enabled = True
        self.params[10].enabled = False
        self.params[10].value = ""
        self.params[13].value = self.params[1].value
    
    return

  def updateMessages(self):
    """Modify the messages created by internal validation for each tool
    parameter.  This method is called after internal validation."""
    return