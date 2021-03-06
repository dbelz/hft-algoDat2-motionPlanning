from utils import is_in_collision


class Controller:
    def __init__(self, workspace, configspace):
        self.workspace = workspace
        self.configspace= configspace
        self.configspace.setDimensions(
            self.workspace.envArray.shape[1] - workspace.robotRadius,
            self.workspace.envArray.shape[0] - workspace.robotRadius)

    def setCurrentPosAsInit(self):
        self.configspace.initConfig = (self.workspace.currentPos[0], self.workspace.currentPos[1])
        print("[POS] Initial configuration set to ", self.configspace.initConfig)
        self.configspace.drawSpace()

    def setCurrentPosAsGoal(self):
        self.configspace.goalConfig = (self.workspace.currentPos[0], self.workspace.currentPos[1])
        print("[POS] Goal configuration set to ", self.configspace.goalConfig)
        self.configspace.set_initial_solution_path()
        self.configspace.isInitialize = True
        self.workspace.isInitialize = True
        self.configspace.drawSpace()

    def drawMouseOffSet(self,mouseX,mouseY):
        self.workspace.drawAll(mouseX, mouseY,
                               self.configspace.initConfig[0],self.configspace.initConfig[1],
                               self.configspace.goalConfig[0],self.configspace.goalConfig[1])

    def drawCurrentPos(self):
        self.workspace.drawAll(self.workspace.currentPos[0],
                               self.workspace.currentPos[1],
                               self.configspace.initConfig[0],self.configspace.initConfig[1],
                               self.configspace.goalConfig[0],self.configspace.goalConfig[1])

    def isInCollision(self, x=None,y=None):
        if x is None: x = self.workspace.currentPos[0]
        if y is None: y = self.workspace.currentPos[1]
        return is_in_collision(self.workspace, x, y)

    def isAllInitialized(self):
        if self.configspace.isInitialize and self.workspace.isInitialize: return True
        return False
    
    def setSolutionPathOnCurrentPos(self, index):
        self.workspace.currentPos = self.configspace.solutionPath[index]
        
    # -------------------------------------------------------------------------
    def display_c_space(self):
        
        self.workspace.computer_c_space()
        self.workspace.display_c_space()

    # -------------------------------------------------------------------------
    def construct_roadmap_with_sPRM(self, radius, samples):
        
        self.workspace.construct_roadmap_with_sPRM(radius, samples, self.workspace, self.configspace)
        
    # -------------------------------------------------------------------------
    def find_path_with_sprm(self):
        
        solution_path = self.workspace.find_path_with_sprm(self.configspace.initConfig,
                                                           self.configspace.goalConfig)
        if (solution_path):
            self.configspace.solutionPath = solution_path
        self.configspace.drawSpace()
        
    # -------------------------------------------------------------------------
    def find_path_with_rrt(self, range, iterations):
        
        solution_path = self.workspace.find_path_with_rrt(self.workspace,
                                                          self.configspace,
                                                          self.configspace.initConfig,
                                                          self.configspace.goalConfig,
                                                          range,
                                                          iterations)
        if (solution_path):
            self.configspace.solutionPath = solution_path
        self.configspace.drawSpace()