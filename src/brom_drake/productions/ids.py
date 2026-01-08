from enum import Enum, unique

@unique
class ProductionID(Enum):
    """
    **Description**

    A unique ID assigned to each production type.
    """

    #: Undefined Production ID (Used to Detect Errors in Production Construction)
    kNotDefined = "NotDefined" 

    #: Show Me This Model Production
    kShowMeThisModel = "ShowMeThisModel" 

    #: Shelf Planning 1 Production, a simple kinematic motion planning task
    kShelfPlanning1 = "ShelfPlanning1" 
    
    #: Chem Lab Planning 1 Production, a simple kinematic motion planning task
    kChemLab1 = "ChemLab-Planning1" 

    #: Chem Lab Planning 2 Production, a more complex dynamic motion planning task
    kChemLab2 = "ChemLab-Planning2" 
    
    """
    Chem Lab 3, a Pick and Place task in the Chem Lab environment

    .. warning::
        
        This production is currently not functional due to issues with the Chem Lab environment model
    """
    kChemLab3 = "ChemLab-Planning3"
    
    #: Demonstrate Static Grasp, visualizes a static scene of the gripper in a grasping pose with an object
    #: Useful for debugging grasp poses
    kDemonstrateStaticGrasp = "Demonstrate-StaticGrasp" 

    #: Attempt Grasp with Puppeteer, uses the Puppeteer system to attempt a grasp
    #: on a free object that is lying on a movable floor
    kAttemptGraspWithPuppeteer = "Attempt-Grasp-WithPuppeteer"

    #: Attempt Grasp with Static Wrist, attempts a grasp with a static wrist pose
    #: on a free object that is lying on a movable floor
    kAttemptGraspWithStaticWrist = "Attempt-Grasp-WithStaticWrist"