class Controller:
    def __init__(self, workspace, configspace):
        self.workspace = workspace
        self.configspace= configspace
        self.configspace.setDimensions(self.workspace.envArray.shape[1]-round(self.workspace.robotArray.shape[1]/2)
        ,self.workspace.envArray.shape[0]-round(self.workspace.robotArray.shape[0]/2))

    def setCurrentPosAsInit(self):
        self.configspace.initConfig=(self.workspace.currentPos[0],self.workspace.currentPos[1])
        self.configspace.drawSpace()

    def setCurrentPosAsGoal(self):
        self.configspace.goalConfig=(self.workspace.currentPos[0],self.workspace.currentPos[1])
        self.configspace.setIntialSolutionPath()
        self.configspace.isInitialize = True
        self.workspace.isInitialize = True
        self.configspace.drawSpace()

    def drawMouseOffSet(self,mouseX,mouseY):
        self.workspace.drawAll(mouseX-round(0.5*self.workspace.robotImage.width),
                               mouseY-round(0.5*self.workspace.robotImage.height),
                               self.configspace.initConfig[0],self.configspace.initConfig[1],
                               self.configspace.goalConfig[0],self.configspace.goalConfig[1])

    def drawCurrentPos(self):
        self.workspace.drawAll(self.workspace.currentPos[0]-round(0.5*self.workspace.robotImage.width),
                               self.workspace.currentPos[1]-round(0.5*self.workspace.robotImage.height),
                               self.configspace.initConfig[0],self.configspace.initConfig[1],
                               self.configspace.goalConfig[0],self.configspace.goalConfig[1])

    def isInCollision(self, x=None,y=None):
        if x is None: x= self.workspace.currentPos[0]+round(0.5*self.workspace.robotImage.width)
        if y is None: y= self.workspace.currentPos[1]+round(0.5*self.workspace.robotImage.height)
        return self.workspace.is_in_collision(x,y)

    def isAllInitialized(self):
        if self.configspace.isInitialize and self.workspace.isInitialize: return True
        return False
    
    def setSolutionPathOnCurrentPos(self, index):
        self.workspace.currentPos = self.configspace.solutionPath[index]
        
    def display_c_space(self):
        self.workspace.computer_c_space()
        self.workspace.display_c_space()
        
    def compute_path_with_sPRM(self, radius, samples):
        self.workspace.compute_path_with_sPRM(radius, samples, self.workspace, self.configspace)